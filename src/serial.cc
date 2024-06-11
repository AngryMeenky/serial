/* Copyright 2012 William Woodall and John Harrison */
#include <algorithm>

#if !defined(_WIN32) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
# include <alloca.h>
#endif

#if defined (__MINGW32__)
# define alloca __builtin_alloca
#endif

#include "serial/serial.h"

#ifdef _WIN32
#include "serial/impl/win.h"
#else
#include "serial/impl/unix.h"
#endif

using std::invalid_argument;
using std::min;
using std::numeric_limits;
using std::vector;
using std::size_t;
using std::string;

using serial::Serial;
using serial::bytesize_t;
using serial::parity_t;
using serial::stopbits_t;
using serial::flowcontrol_t;
using serial::serialerror_t;

class Serial::ScopedReadLock {
public:
  ScopedReadLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->readLock();
  }
  ~ScopedReadLock() {
    this->pimpl_->readUnlock();
  }
private:
  // Disable copy constructors
  ScopedReadLock(const ScopedReadLock&);
  const ScopedReadLock& operator=(ScopedReadLock);

  SerialImpl *pimpl_;
};

class Serial::ScopedWriteLock {
public:
  ScopedWriteLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->writeLock();
  }
  ~ScopedWriteLock() {
    this->pimpl_->writeUnlock();
  }
private:
  // Disable copy constructors
  ScopedWriteLock(const ScopedWriteLock&);
  const ScopedWriteLock& operator=(ScopedWriteLock);
  SerialImpl *pimpl_;
};

Serial::Serial (const string &port, uint32_t baudrate, serial::Timeout timeout,
                bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
                flowcontrol_t flowcontrol, serialerror_t *serialerror)
 : pimpl_(new SerialImpl (port, baudrate, bytesize, parity,
                          stopbits, flowcontrol, serialerror))
{
  pimpl_->setTimeout(timeout);
}

Serial::~Serial ()
{
  delete pimpl_;
}


const std::string &
Serial::getLastError() const
{
  return pimpl_->getLastError();
}


serialerror_t
Serial::open ()
{
  return pimpl_->open ();
}


serialerror_t
Serial::close ()
{
  return pimpl_->close ();
}


bool
Serial::isOpen () const
{
  return pimpl_->isOpen ();
}

size_t
Serial::available (serialerror_t *serialerror)
{
  return pimpl_->available (serialerror);
}

bool
Serial::waitReadable (serialerror_t *serialerror)
{
  serial::Timeout timeout(pimpl_->getTimeout ());
  return pimpl_->waitReadable(timeout.read_timeout_constant, serialerror);
}

void
Serial::waitByteTimes (size_t count)
{
  pimpl_->waitByteTimes(count);
}

size_t
Serial::read_ (uint8_t *buffer, size_t size, serialerror_t *serialerror)
{
  return this->pimpl_->read (buffer, size, serialerror);
}

size_t
Serial::read (uint8_t *buffer, size_t size, serialerror_t *serialerror)
{
  ScopedReadLock lock(this->pimpl_);
  return this->pimpl_->read (buffer, size, serialerror);
}

size_t
Serial::read (std::vector<uint8_t> &buffer, size_t size, serialerror_t *serialerror)
{
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = 0;
  serialerror_t err;

  bytes_read = this->pimpl_->read (buffer_, size, &err);

  if(err == serialerror_success) {
    buffer.insert (buffer.end (), buffer_, buffer_+bytes_read);
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
 
  delete[] buffer_;
  return bytes_read;
}

size_t
Serial::read (std::string &buffer, size_t size, serialerror_t *serialerror)
{
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = 0;
  serialerror_t err;
  
  bytes_read = this->pimpl_->read (buffer_, size, &err);
  if(err == serialerror_success) {
    buffer.append (reinterpret_cast<const char*>(buffer_), bytes_read);
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }

  delete[] buffer_;
  return bytes_read;
}

string
Serial::read (size_t size, serialerror_t *serialerror)
{
  std::string buffer;
  this->read (buffer, size, serialerror);
  return buffer;
}

size_t
Serial::readline (string &buffer, size_t size, string eol, serialerror_t *serialerror)
{
  ScopedReadLock lock(this->pimpl_);
  size_t eol_len = eol.length ();
  uint8_t *buffer_ = static_cast<uint8_t*>
                              (alloca (size * sizeof (uint8_t)));
  size_t read_so_far = 0;
  serialerror_t err =  serialerror_success;
  while (err == serialerror_success)
  {
    size_t bytes_read = this->read_ (buffer_ + read_so_far, 1, &err);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break; // Timeout occured on reading 1 byte
    }
    if(read_so_far < eol_len) continue;
    if (string (reinterpret_cast<const char*>
         (buffer_ + read_so_far - eol_len), eol_len) == eol) {
      break; // EOL found
    }
    if (read_so_far == size) {
      break; // Reached the maximum read length
    }
  }
  if(serialerror != nullptr) {
    *serialerror = err;
  }
  buffer.append(reinterpret_cast<const char*> (buffer_), read_so_far);
  return read_so_far;
}

string
Serial::readline (size_t size, string eol, serialerror_t *serialerror)
{
  std::string buffer;
  this->readline (buffer, size, eol, serialerror);
  return buffer;
}

vector<string>
Serial::readlines (size_t size, string eol, serialerror_t *serialerror)
{
  ScopedReadLock lock(this->pimpl_);
  std::vector<std::string> lines;
  size_t eol_len = eol.length ();
  uint8_t *buffer_ = static_cast<uint8_t*>
    (alloca (size * sizeof (uint8_t)));
  size_t read_so_far = 0;
  size_t start_of_line = 0;
  serialerror_t err = serialerror_success;
 
  while (err == serialerror_success && read_so_far < size) {
    size_t bytes_read = this->read_ (buffer_+read_so_far, 1, &err);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      if (start_of_line != read_so_far) {
        lines.push_back (
          string (reinterpret_cast<const char*> (buffer_ + start_of_line),
            read_so_far - start_of_line));
      }
      break; // Timeout occured on reading 1 byte
    }
    if(read_so_far < eol_len) continue;
    if (string (reinterpret_cast<const char*>
         (buffer_ + read_so_far - eol_len), eol_len) == eol) {
      // EOL found
      lines.push_back(
        string(reinterpret_cast<const char*> (buffer_ + start_of_line),
          read_so_far - start_of_line));
      start_of_line = read_so_far;
    }
    if (read_so_far == size) {
      if (start_of_line != read_so_far) {
        lines.push_back(
          string(reinterpret_cast<const char*> (buffer_ + start_of_line),
            read_so_far - start_of_line));
      }
      break; // Reached the maximum read length
    }
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
  return lines;
}

size_t
Serial::write (const string &data, serialerror_t *serialerror)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_ (reinterpret_cast<const uint8_t*>(data.c_str()),
                       data.length(), serialerror);
}

size_t
Serial::write (const std::vector<uint8_t> &data, serialerror_t *serialerror)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_ (&data[0], data.size(), serialerror);
}

size_t
Serial::write (const uint8_t *data, size_t size, serialerror_t *serialerror)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(data, size, serialerror);
}

size_t
Serial::write_ (const uint8_t *data, size_t length, serialerror_t *serialerror)
{
  return pimpl_->write (data, length, serialerror);
}

void
Serial::setPort (const string &port, serialerror_t *serialerror)
{
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  serialerror_t err = serialerror_success;
  bool was_open = pimpl_->isOpen();
 
  if (was_open) close();
  pimpl_->setPort(port);
  if (was_open) err = open();

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}

string
Serial::getPort (serialerror_t *serialerror) const
{
  return pimpl_->getPort (serialerror);
}

void
Serial::setTimeout (serial::Timeout &timeout)
{
  pimpl_->setTimeout (timeout);
}

serial::Timeout
Serial::getTimeout () const {
  return pimpl_->getTimeout ();
}

void
Serial::setBaudrate (uint32_t baudrate, serialerror_t *serialerror)
{
  pimpl_->setBaudrate (baudrate, serialerror);
}

uint32_t
Serial::getBaudrate (serialerror_t *serialerror) const
{
  return uint32_t(pimpl_->getBaudrate (serialerror));
}

void
Serial::setBytesize (bytesize_t bytesize, serialerror_t *serialerror)
{
  pimpl_->setBytesize (bytesize, serialerror);
}

bytesize_t
Serial::getBytesize (serialerror_t *serialerror) const
{
  return pimpl_->getBytesize (serialerror);
}

void
Serial::setParity (parity_t parity, serialerror_t *serialerror)
{
  pimpl_->setParity (parity, serialerror);
}

parity_t
Serial::getParity (serialerror_t *serialerror) const
{
  return pimpl_->getParity (serialerror);
}

void
Serial::setStopbits (stopbits_t stopbits, serialerror_t *serialerror)
{
  pimpl_->setStopbits (stopbits, serialerror);
}

stopbits_t
Serial::getStopbits (serialerror_t *serialerror) const
{
  return pimpl_->getStopbits (serialerror);
}

void
Serial::setFlowcontrol (flowcontrol_t flowcontrol, serialerror_t *serialerror)
{
  pimpl_->setFlowcontrol (flowcontrol, serialerror);
}

flowcontrol_t
Serial::getFlowcontrol (serialerror_t *serialerror) const
{
  return pimpl_->getFlowcontrol (serialerror);
}

serialerror_t Serial::flush ()
{
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  return pimpl_->flush ();
}

serialerror_t Serial::flushInput ()
{
  ScopedReadLock lock(this->pimpl_);
  return pimpl_->flushInput ();
}

serialerror_t Serial::flushOutput ()
{
  ScopedWriteLock lock(this->pimpl_);
  return pimpl_->flushOutput ();
}

serialerror_t Serial::sendBreak (int duration)
{
  return pimpl_->sendBreak (duration);
}

serialerror_t Serial::setBreak (bool level)
{
  return pimpl_->setBreak (level);
}

serialerror_t Serial::setRTS (bool level)
{
  return pimpl_->setRTS (level);
}

serialerror_t Serial::setDTR (bool level)
{
  return pimpl_->setDTR (level);
}

bool Serial::waitForChange(serialerror_t *serialerror)
{
  return pimpl_->waitForChange(serialerror);
}

bool Serial::getCTS (serialerror_t *serialerror)
{
  return pimpl_->getCTS (serialerror);
}

bool Serial::getDSR (serialerror_t *serialerror)
{
  return pimpl_->getDSR (serialerror);
}

bool Serial::getRI (serialerror_t *serialerror)
{
  return pimpl_->getRI (serialerror);
}

bool Serial::getCD (serialerror_t *serialerror)
{
  return pimpl_->getCD (serialerror);
}
