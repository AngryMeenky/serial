/* Copyright 2012 William Woodall and John Harrison
 *
 * Additional Contributors: Christopher Baker @bakercp
 */

#if !defined(_WIN32)

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <paths.h>
#include <sysexits.h>
#include <termios.h>
#include <sys/param.h>
#include <pthread.h>

#if defined(__linux__)
# include <linux/serial.h>
#endif

#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#ifdef __MACH__
#include <AvailabilityMacros.h>
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "serial/impl/unix.h"

#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif

#if defined(MAC_OS_X_VERSION_10_3) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_3)
#include <IOKit/serial/ioss.h>
#endif

using std::string;
using std::stringstream;
using serial::MillisecondTimer;
using serial::Serial;
using serial::serialerror_t;


MillisecondTimer::MillisecondTimer (const uint32_t millis)
  : expiry(timespec_now())
{
  int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
  if (tv_nsec >= 1e9) {
    int64_t sec_diff = tv_nsec / static_cast<int> (1e9);
    expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
    expiry.tv_sec += sec_diff;
  } else {
    expiry.tv_nsec = tv_nsec;
  }
}

int64_t
MillisecondTimer::remaining ()
{
  timespec now(timespec_now());
  int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
  millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
  return millis;
}

timespec
MillisecondTimer::timespec_now ()
{
  timespec time;
# ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  time.tv_sec = mts.tv_sec;
  time.tv_nsec = mts.tv_nsec;
# else
  clock_gettime(CLOCK_MONOTONIC, &time);
# endif
  return time;
}

timespec
timespec_from_ms (const uint32_t millis)
{
  timespec time;
  time.tv_sec = millis / 1e3;
  time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
  return time;
}

Serial::SerialImpl::SerialImpl (const string &port, unsigned long baudrate,
                                bytesize_t bytesize,
                                parity_t parity, stopbits_t stopbits,
                                flowcontrol_t flowcontrol, serialerror_t *serialerror)
  : error_(), port_ (port), fd_ (-1),
    is_open_ (false), xonxoff_ (false), rtscts_ (false),
    baudrate_ (baudrate), parity_ (parity),
    bytesize_ (bytesize), stopbits_ (stopbits), flowcontrol_ (flowcontrol) {
  pthread_mutex_init(&this->read_mutex, NULL);
  pthread_mutex_init(&this->write_mutex, NULL);
  serialerror_t err = serialerror_success;

  if(port_.empty() == false) {
    err = open();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


Serial::SerialImpl::~SerialImpl() {
  close();
  pthread_mutex_destroy(&this->read_mutex);
  pthread_mutex_destroy(&this->write_mutex);
}


const std::string &
Serial::SerialImpl::getLastError() const {
  return error_;
}


serialerror_t
Serial::SerialImpl::open() {
  if (port_.empty ()) {
    error_.assign("No port");
    return serialerror_argument;
  }
  if (is_open_ == true) {
    error_.assign("Already open");
    return serialerror_serial;
  }

  fd_ = ::open (port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ == -1) {
    error_.assign(strerror(errno));
    switch (errno) {
    case EINTR:
      // Recurse because this is a recoverable error.
      return open ();
    case ENFILE:
    case EMFILE:
      return serialerror_open_failed;
    default:
      return serialerror_open_failed;
    }
  }

  serialerror_t err = reconfigurePort();
  is_open_ = err == serialerror_success;
  return err;
}


serialerror_t
Serial::SerialImpl::reconfigurePort ()
{
  if (fd_ == -1) {
    error_.assign("Port not open");
    // Can only operate on a valid file descriptor
    return serialerror_not_opened;
  }

  struct termios options; // The options for the file descriptor

  if (tcgetattr(fd_, &options) == -1) {
    error_.assign(strerror(errno));
    return serialerror_io_failed;
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                       ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

  // setup baud rate
  bool custom_baud = false;
  speed_t baud;
  switch (baudrate_) {
#ifdef B0
  case 0: baud = B0; break;
#endif
#ifdef B50
  case 50: baud = B50; break;
#endif
#ifdef B75
  case 75: baud = B75; break;
#endif
#ifdef B110
  case 110: baud = B110; break;
#endif
#ifdef B134
  case 134: baud = B134; break;
#endif
#ifdef B150
  case 150: baud = B150; break;
#endif
#ifdef B200
  case 200: baud = B200; break;
#endif
#ifdef B300
  case 300: baud = B300; break;
#endif
#ifdef B600
  case 600: baud = B600; break;
#endif
#ifdef B1200
  case 1200: baud = B1200; break;
#endif
#ifdef B1800
  case 1800: baud = B1800; break;
#endif
#ifdef B2400
  case 2400: baud = B2400; break;
#endif
#ifdef B4800
  case 4800: baud = B4800; break;
#endif
#ifdef B7200
  case 7200: baud = B7200; break;
#endif
#ifdef B9600
  case 9600: baud = B9600; break;
#endif
#ifdef B14400
  case 14400: baud = B14400; break;
#endif
#ifdef B19200
  case 19200: baud = B19200; break;
#endif
#ifdef B28800
  case 28800: baud = B28800; break;
#endif
#ifdef B57600
  case 57600: baud = B57600; break;
#endif
#ifdef B76800
  case 76800: baud = B76800; break;
#endif
#ifdef B38400
  case 38400: baud = B38400; break;
#endif
#ifdef B115200
  case 115200: baud = B115200; break;
#endif
#ifdef B128000
  case 128000: baud = B128000; break;
#endif
#ifdef B153600
  case 153600: baud = B153600; break;
#endif
#ifdef B230400
  case 230400: baud = B230400; break;
#endif
#ifdef B256000
  case 256000: baud = B256000; break;
#endif
#ifdef B460800
  case 460800: baud = B460800; break;
#endif
#ifdef B500000
  case 500000: baud = B500000; break;
#endif
#ifdef B576000
  case 576000: baud = B576000; break;
#endif
#ifdef B921600
  case 921600: baud = B921600; break;
#endif
#ifdef B1000000
  case 1000000: baud = B1000000; break;
#endif
#ifdef B1152000
  case 1152000: baud = B1152000; break;
#endif
#ifdef B1500000
  case 1500000: baud = B1500000; break;
#endif
#ifdef B2000000
  case 2000000: baud = B2000000; break;
#endif
#ifdef B2500000
  case 2500000: baud = B2500000; break;
#endif
#ifdef B3000000
  case 3000000: baud = B3000000; break;
#endif
#ifdef B3500000
  case 3500000: baud = B3500000; break;
#endif
#ifdef B4000000
  case 4000000: baud = B4000000; break;
#endif
  default:
    custom_baud = true;
  }
  if (custom_baud == false) {
#ifdef _BSD_SOURCE
    ::cfsetspeed(&options, baud);
#else
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);
#endif
  }

  // setup char len
  options.c_cflag &= (tcflag_t) ~CSIZE;
  if (bytesize_ == eightbits) {
    options.c_cflag |= CS8;
  }
  else if (bytesize_ == sevenbits) {
    options.c_cflag |= CS7;
  }
  else if (bytesize_ == sixbits) {
    options.c_cflag |= CS6;
  }
  else if (bytesize_ == fivebits) {
    options.c_cflag |= CS5;
  }
  else {
    error_.assign("Invalid bytesize");
    return serialerror_argument;
  }

  // setup stopbits
  if (stopbits_ == stopbits_one) {
    options.c_cflag &= (tcflag_t) ~(CSTOPB);
  }
  else if (stopbits_ == stopbits_one_point_five) {
    // ONE POINT FIVE same as TWO.. there is no POSIX support for 1.5
    options.c_cflag |=  (CSTOPB);
  }
  else if (stopbits_ == stopbits_two) {
    options.c_cflag |=  (CSTOPB);
  }
  else {
    error_.assign("Invalid stopbits");
    return serialerror_argument;
  }

  // setup parity
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  if (parity_ == parity_none) {
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
  } else if (parity_ == parity_even) {
    options.c_cflag &= (tcflag_t) ~(PARODD);
    options.c_cflag |=  (PARENB);
  } else if (parity_ == parity_odd) {
    options.c_cflag |=  (PARENB | PARODD);
  }
#ifdef CMSPAR
  else if (parity_ == parity_mark) {
    options.c_cflag |=  (PARENB | CMSPAR | PARODD);
  }
  else if (parity_ == parity_space) {
    options.c_cflag |=  (PARENB | CMSPAR);
    options.c_cflag &= (tcflag_t) ~(PARODD);
  }
#else
  // CMSPAR is not defined on OSX. So do not support mark or space parity.
  else if (parity_ == parity_mark || parity_ == parity_space) {
    error_.assign("Invalid parity");
    return serialerror_argument;
  }
#endif  // ifdef CMSPAR
  else {
    error_.assign("Invalid parity");
    return serialerror_argument;
  }

  // setup flow control
  if (flowcontrol_ == flowcontrol_none) {
    xonxoff_ = false;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_software) {
    xonxoff_ = true;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_hardware) {
    xonxoff_ = false;
    rtscts_ = true;
  }
  // xonxoff
#ifdef IXANY
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF); //|IXANY)
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF);
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
  // rtscts
#ifdef CRTSCTS
  if (rtscts_)
    options.c_cflag |=  (CRTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CRTSCTS);
#elif defined CNEW_RTSCTS
  if (rtscts_)
    options.c_cflag |=  (CNEW_RTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // this basically sets the read call up to be a polling read,
  // but we are using select to ensure there is data available
  // to read before each call, so we should never needlessly poll
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr (fd_, TCSANOW, &options);

  // apply custom baud rate, if any
  if (custom_baud == true) {
    // OS X support
#if defined(MAC_OS_X_VERSION_10_4) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
    // Starting with Tiger, the IOSSIOSPEED ioctl can be used to set arbitrary baud rates
    // other than those specified by POSIX. The driver for the underlying serial hardware
    // ultimately determines which baud rates can be used. This ioctl sets both the input
    // and output speed.
    speed_t new_baud = static_cast<speed_t> (baudrate_);
    // PySerial uses IOSSIOSPEED=0x80045402
    if (-1 == ioctl (fd_, IOSSIOSPEED, &new_baud, 1)) {
      error_.assign(strerror(errno));
      return serialerror_io_failed;
    }
    // Linux Support
#elif defined(__linux__) && defined (TIOCSSERIAL)
    struct serial_struct ser;

    if (-1 == ioctl (fd_, TIOCGSERIAL, &ser)) {
      error_.assign(strerror(errno));
      return serialerror_io_failed;
    }

    // set custom divisor
    ser.custom_divisor = ser.baud_base / static_cast<int> (baudrate_);
    // update flags
    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;

    if (-1 == ioctl (fd_, TIOCSSERIAL, &ser)) {
      error_.assign(strerror(errno));
      return serialerror_io_failed;
    }
#else
    error_.assign("Custom baud rate unsupported");
    return serialerror_argument;
#endif
  }

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_ns = 1e9 / baudrate_;
  byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

  // Compensate for the stopbits_one_point_five enum being equal to int 3,
  // and not 1.5.
  if (stopbits_ == stopbits_one_point_five) {
    byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
  }
 
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::close() {
  if (is_open_ == true) {
    if (fd_ != -1) {
      int ret;
      ret = ::close (fd_);
      if (ret == 0) {
        fd_ = -1;
      } else {
        error_.assign(strerror(errno));
        return serialerror_io_failed;
      }
    }
    is_open_ = false;
  }

  return serialerror_success;
}


bool
Serial::SerialImpl::isOpen() const {
  return is_open_;
}

size_t
Serial::SerialImpl::available(serialerror_t *serialerror) {
  if (!is_open_) {
    return 0;
  }
  int count = 0;
  if (-1 == ioctl (fd_, TIOCINQ, &count)) {
    error_.assign(strerror(errno));
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
  }

  return static_cast<size_t> (count);
}

bool
Serial::SerialImpl::waitReadable (uint32_t timeout, serialerror_t *serialerror)
{
  // Setup a select call to block for serial data or a timeout
  fd_set readfds;
  FD_ZERO (&readfds);
  FD_SET (fd_, &readfds);
  timespec timeout_ts (timespec_from_ms (timeout));
  int r = pselect (fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

  if (r < 0) {
    // Select was interrupted
    if (errno == EINTR) {
      if(serialerror != nullptr) {
        *serialerror = serialerror_success;
      }
    }
    // Otherwise there was some error
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
    error_.assign(strerror(errno));
    return false;
  }

  // Timeout occurred
  if (r == 0) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_success;
    }
    return false;
  }

  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET (fd_, &readfds)) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
    error_.assign(strerror(errno));
    return false;
  }

  // Data available to read.
  return true;
}


void
Serial::SerialImpl::waitByteTimes(size_t count) {
  timespec wait_time = { 0, static_cast<long>(byte_time_ns_ * count)};
  pselect (0, NULL, NULL, NULL, &wait_time, NULL);
}


size_t
Serial::SerialImpl::read (uint8_t *buf, size_t size, serialerror_t *serialerror) {
  size_t bytes_read = 0;
  // If the port is not open, throw
  if (!is_open_) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }
    error_.assign("Port not open");
    return bytes_read;
  }

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.read_timeout_constant;
  total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long> (size);
  MillisecondTimer total_timeout(total_timeout_ms);

  // Pre-fill buffer with available bytes
  {
    ssize_t bytes_read_now = ::read (fd_, buf, size);
    if (bytes_read_now > 0) {
      bytes_read = bytes_read_now;
    }
  }

  serialerror_t err = serialerror_success;
  while(err == serialerror_success && bytes_read < size) {
    int64_t timeout_remaining_ms = total_timeout.remaining();
    if (timeout_remaining_ms <= 0) {
      // Timed out
      break;
    }
    // Timeout for the next select is whichever is less of the remaining
    // total read timeout and the inter-byte timeout.
    uint32_t timeout = std::min(static_cast<uint32_t> (timeout_remaining_ms),
                                timeout_.inter_byte_timeout);
    // Wait for the device to be readable, and then attempt to read.
    if (waitReadable(timeout, &err)) {
      // If it's a fixed-length multi-byte read, insert a wait here so that
      // we can attempt to grab the whole thing in a single IO call. Skip
      // this wait if a non-max inter_byte_timeout is specified.
      if (size > 1 && timeout_.inter_byte_timeout == Timeout::max()) {
        size_t bytes_available = available();
        if (bytes_available + bytes_read < size) {
          waitByteTimes(size - (bytes_available + bytes_read));
        }
      }
      // This should be non-blocking returning only what is available now
      //  Then returning so that select can block again.
      ssize_t bytes_read_now =
        ::read (fd_, buf + bytes_read, size - bytes_read);
      // read should always return some data as select reported it was
      // ready to read when we get to this point.
      if (bytes_read_now < 1) {
        // Disconnected devices, at least on Linux, show the
        // behavior that they are always ready to read immediately
        // but reading returns nothing.
	err = serialerror_serial;
        error_.assign(strerror(errno));
	break;
      }
      // Update bytes_read
      bytes_read += static_cast<size_t> (bytes_read_now);
      // If bytes_read == size then we have read everything we need
      if (bytes_read == size) {
        break;
      }
      // If bytes_read < size then we have more to read
      if (bytes_read < size) {
        continue;
      }
      // If bytes_read > size then we have over read, which shouldn't happen
      if (bytes_read > size) {
	err = serialerror_serial;
        error_.assign("Impossible error: read more that request");
	break;
      }
    }
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }

  return bytes_read;
}


size_t
Serial::SerialImpl::write(const uint8_t *data, size_t length, serialerror_t *serialerror) {
  size_t bytes_written = 0;
  if (is_open_ == false) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }
    error_.assign("Port not open");
    return bytes_written;
  }
  fd_set writefds;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.write_timeout_constant;
  total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long> (length);
  MillisecondTimer total_timeout(total_timeout_ms);

  serialerror_t err = serialerror_success;
  bool first_iteration = true;
  while (err == serialerror_success && bytes_written < length) {
    int64_t timeout_remaining_ms = total_timeout.remaining();
    // Only consider the timeout if it's not the first iteration of the loop
    // otherwise a timeout of 0 won't be allowed through
    if (!first_iteration && (timeout_remaining_ms <= 0)) {
      // Timed out
      break;
    }
    first_iteration = false;

    timespec timeout(timespec_from_ms(timeout_remaining_ms));

    FD_ZERO (&writefds);
    FD_SET (fd_, &writefds);

    // Do the select
    int r = pselect (fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

    // Figure out what happened by looking at select's response 'r'
    /** Error **/
    if (r < 0) {
      // Select was interrupted, try again
      if (errno == EINTR) {
        continue;
      }

      err = serialerror_io_failed;
      error_.assign(strerror(errno));
      break;
    }

    /** Timeout **/
    if (r == 0) {
      break;
    }

    /** Port ready to write **/
    if (r > 0) {
      // Make sure our file descriptor is in the ready to write list
      if (FD_ISSET (fd_, &writefds)) {
        // This will write some
        ssize_t bytes_written_now =
          ::write (fd_, data + bytes_written, length - bytes_written);

        // even though pselect returned readiness the call might still be 
        // interrupted. In that case simply retry.
        if (bytes_written_now == -1 && errno == EINTR) {
          continue;
        }

        // write should always return some data as select reported it was
        // ready to write when we get to this point.
        if (bytes_written_now < 1) {
          // Disconnected devices, at least on Linux, show the
          // behavior that they are always ready to write immediately
          // but writing returns nothing.
          err = serialerror_io_failed;
          error_.assign(strerror(errno));
          break;
        }
        // Update bytes_written
        bytes_written += static_cast<size_t> (bytes_written_now);
        // If bytes_written == size then we have written everything we need to
        if (bytes_written == length) {
          break;
        }
        // If bytes_written < size then we have more to write
        if (bytes_written < length) {
          continue;
        }
        // If bytes_written > size then we have over written, which shouldn't happen
        if (bytes_written > length) {
          error_.assign("Impossible error: read more that request");
          err = serialerror_serial;
          break;
        }
      }
      err = serialerror_io_failed;
    }
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }

  return bytes_written;
}


void
Serial::SerialImpl::setPort(const string &port, serialerror_t *serialerror) {
  port_ = port;
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }
}


string
Serial::SerialImpl::getPort(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }
  return port_;
}


void
Serial::SerialImpl::setTimeout(serial::Timeout &timeout) {
  timeout_ = timeout;
}


serial::Timeout
Serial::SerialImpl::getTimeout() const {
  return timeout_;
}


void
Serial::SerialImpl::setBaudrate(unsigned long baudrate, serialerror_t *serialerror) {
  baudrate_ = baudrate;
  serialerror_t err = serialerror_success;
  if(is_open_) {
    err = reconfigurePort();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


unsigned long
Serial::SerialImpl::getBaudrate(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return baudrate_;
}


void
Serial::SerialImpl::setBytesize(serial::bytesize_t bytesize, serialerror_t *serialerror) {
  bytesize_ = bytesize;
  serialerror_t err = serialerror_success;
  if(is_open_) {
    err = reconfigurePort();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


serial::bytesize_t
Serial::SerialImpl::getBytesize(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return bytesize_;
}


void
Serial::SerialImpl::setParity(serial::parity_t parity, serialerror_t *serialerror) {
  parity_ = parity;
  serialerror_t err = serialerror_success;
  if(is_open_) {
    err = reconfigurePort();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


serial::parity_t
Serial::SerialImpl::getParity(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return parity_;
}


void
Serial::SerialImpl::setStopbits(serial::stopbits_t stopbits, serialerror_t *serialerror) {
  stopbits_ = stopbits;
  serialerror_t err = serialerror_success;
  if(is_open_) {
    err = reconfigurePort();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


serial::stopbits_t
Serial::SerialImpl::getStopbits(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return stopbits_;
}


void
Serial::SerialImpl::setFlowcontrol(serial::flowcontrol_t flowcontrol, serialerror_t *serialerror) {
  flowcontrol_ = flowcontrol;
  serialerror_t err = serialerror_success;
  if(is_open_) {
    err = reconfigurePort();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


serial::flowcontrol_t
Serial::SerialImpl::getFlowcontrol(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return flowcontrol_;
}


serialerror_t
Serial::SerialImpl::flush() {
  if(is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  tcdrain(fd_);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::flushInput() {
  if(is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  tcflush (fd_, TCIFLUSH);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::flushOutput() {
  if (is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  tcflush (fd_, TCOFLUSH);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::sendBreak(int duration) {
  if(is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  tcsendbreak (fd_, static_cast<int> (duration / 4));
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::setBreak(bool level) {
  if(is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  if (-1 == ioctl (fd_, level ? TIOCSBRK : TIOCCBRK)) {
    error_.assign(strerror(errno));
    return serialerror_serial;
  }

  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::setRTS(bool level) {
  if(is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  int command = TIOCM_RTS;
  if (-1 == ioctl (fd_, level ? TIOCMBIS : TIOCMBIC)) {
    error_.assign(strerror(errno));
    return serialerror_serial;
  }

  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::setDTR(bool level) {
  if(is_open_ == false) {
    error_.assign("Port not open");
    return serialerror_not_opened;
  }

  int command = TIOCM_DTR;
  if (-1 == ioctl (fd_, level ? TIOCMBIS : TIOCMBIC)) {
    error_.assign(strerror(errno));
    return serialerror_serial;
  }

  return serialerror_success;
}


bool
Serial::SerialImpl::waitForChange(serialerror_t *serialerror) {
#ifndef TIOCMIWAIT

while (is_open_ == true) {

    int status;

    if(-1 == ioctl (fd_, TIOCMGET, &status)) {
      if(serialerror != nullptr) {
        *serialerror = serialerror_serial;
      }

    error_.assign(strerror(errno));
      return false;
    }
    else {
        if (0 != (status & TIOCM_CTS)
         || 0 != (status & TIOCM_DSR)
         || 0 != (status & TIOCM_RI)
         || 0 != (status & TIOCM_CD)) {
          if(serialerror != nullptr) {
            *serialerror = serialerror_success;
          }
          return true;
        }
    }

    usleep(1000);
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }
  return false;
#else
  int command = (TIOCM_CD | TIOCM_DSR | TIOCM_RI | TIOCM_CTS);

  if (-1 == ioctl (fd_, TIOCMIWAIT, &command)) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    error_.assign(strerror(errno));
    return false;
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }
  return true;
#endif
}


bool
Serial::SerialImpl::getCTS(serialerror_t *serialerror) {
  if (is_open_ == false) {
    error_.assign("Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }

    return false;
  }

  int status;

  if(-1 == ioctl (fd_, TIOCMGET, &status)) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    error_.assign(strerror(errno));
    return false;
  }
  else {
    if(serialerror != nullptr) {
      *serialerror = serialerror_success;
    }

    return 0 != (status & TIOCM_CTS);
  }
}


bool
Serial::SerialImpl::getDSR(serialerror_t *serialerror) {
  if (is_open_ == false) {
    error_.assign("Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }

    return false;
  }

  int status;

  if(-1 == ioctl (fd_, TIOCMGET, &status)) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    error_.assign(strerror(errno));
    return false;
  }
  else {
    if(serialerror != nullptr) {
      *serialerror = serialerror_success;
    }

    return 0 != (status & TIOCM_DSR);
  }
}


bool
Serial::SerialImpl::getRI(serialerror_t *serialerror) {
  if (is_open_ == false) {
    error_.assign("Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }

    return false;
  }

  int status;

  if(-1 == ioctl (fd_, TIOCMGET, &status)) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    error_.assign(strerror(errno));
    return false;
  }
  else {
    if(serialerror != nullptr) {
      *serialerror = serialerror_success;
    }

    return 0 != (status & TIOCM_RI);
  }
}


bool
Serial::SerialImpl::getCD(serialerror_t *serialerror) {
  if (is_open_ == false) {
    error_.assign("Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }

    return false;
  }

  int status;

  if(-1 == ioctl (fd_, TIOCMGET, &status)) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    error_.assign(strerror(errno));
    return false;
  }
  else {
    if(serialerror != nullptr) {
      *serialerror = serialerror_success;
    }

    return 0 != (status & TIOCM_CD);
  }
}


serialerror_t
Serial::SerialImpl::readLock() {
  return pthread_mutex_lock(&this->read_mutex) ? serialerror_mutex : serialerror_success;
}


serialerror_t
Serial::SerialImpl::readUnlock() {
  return pthread_mutex_unlock(&this->read_mutex) ? serialerror_mutex : serialerror_success;
}


serialerror_t
Serial::SerialImpl::writeLock() {
  return pthread_mutex_lock(&this->write_mutex) ? serialerror_mutex : serialerror_success;
}


serialerror_t
Serial::SerialImpl::writeUnlock() {
  return pthread_mutex_unlock(&this->write_mutex) ? serialerror_mutex : serialerror_success;
}

#endif // !defined(_WIN32)
