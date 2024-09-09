#if defined(_WIN32)

/* Copyright 2012 William Woodall and John Harrison */

#include <codecvt>
#include <sstream>

#include "serial/impl/win.h"

using std::string;
using std::wstring;
using std::stringstream;
using serial::Serial;
using serial::Timeout;
using serial::bytesize_t;
using serial::parity_t;
using serial::stopbits_t;
using serial::flowcontrol_t;
using serial::serialerror_t;

inline wstring
_prefix_port_if_needed(const wstring &input)
{
  static wstring windows_com_port_prefix = L"\\\\.\\";
  if (input.compare(0, windows_com_port_prefix.size(), windows_com_port_prefix) != 0)
  {
    return windows_com_port_prefix + input;
  }
  return input;
}

Serial::SerialImpl::SerialImpl (const string &port, unsigned long baudrate,
                                bytesize_t bytesize,
                                parity_t parity, stopbits_t stopbits,
                                flowcontrol_t flowcontrol, serialerror_t *serialerror)
  : port_ (port.begin(), port.end()), fd_ (INVALID_HANDLE_VALUE), is_open_ (false),
    baudrate_ (baudrate), parity_ (parity),
    bytesize_ (bytesize), stopbits_ (stopbits), flowcontrol_ (flowcontrol)
{
  read_mutex = CreateMutex(NULL, false, NULL);
  write_mutex = CreateMutex(NULL, false, NULL);
  serialerror_t err = serialerror_success;

  if(port_.empty() == false) {
    err = open();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


Serial::SerialImpl::~SerialImpl ()
{
  this->close();
  CloseHandle(read_mutex);
  CloseHandle(write_mutex);
}


const std::string &
Serial::SerialImpl::getLastError() const {
  return error_;
}


serialerror_t
Serial::SerialImpl::open() {
  if (port_.empty ()) {
    error_.assign("Empty port is invalid.");
    return serialerror_argument;
  }
  if (is_open_ == true) {
    error_.assign("Serial port already open.");
    return serialerror_serial;
  }

  // See: https://github.com/wjwwood/serial/issues/84
  wstring port_with_prefix = _prefix_port_if_needed(port_);
  LPCWSTR lp_port = port_with_prefix.c_str();
  fd_ = CreateFileW(lp_port,
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    0,
                    OPEN_EXISTING,
                    FILE_ATTRIBUTE_NORMAL,
                    0);

  if (fd_ == INVALID_HANDLE_VALUE) {
    DWORD create_file_err = GetLastError();
    stringstream ss;
    switch (create_file_err) {
      case ERROR_FILE_NOT_FOUND:
        // Use this->getPort to convert to a std::string
        ss << "Specified port, " << this->getPort() << ", does not exist.";
        error_.assign(ss.str().c_str());
        return serialerror_open_failed;
      default:
        ss << "Unknown error opening the serial port: " << create_file_err;
        error_.assign(ss.str().c_str());
        return serialerror_open_failed;
    }
  }

  serialerror_t err = reconfigurePort();
  is_open_ = err == serialerror_success;
  return err;
}


serialerror_t
Serial::SerialImpl::reconfigurePort() {
  if (fd_ == INVALID_HANDLE_VALUE) {
    // Can only operate on a valid file descriptor
    error_.assign("Invalid file descriptor, is the serial port open?");
    return serialerror_not_opened;
  }

  DCB dcbSerialParams = {0};

  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

  if(!GetCommState(fd_, &dcbSerialParams)) {
    //error getting state
    error_.assign("Error getting the serial port state.");
    return serialerror_io_failed;
  }

  // setup baud rate
  switch (baudrate_) {
#ifdef CBR_0
  case 0: dcbSerialParams.BaudRate = CBR_0; break;
#endif
#ifdef CBR_50
  case 50: dcbSerialParams.BaudRate = CBR_50; break;
#endif
#ifdef CBR_75
  case 75: dcbSerialParams.BaudRate = CBR_75; break;
#endif
#ifdef CBR_110
  case 110: dcbSerialParams.BaudRate = CBR_110; break;
#endif
#ifdef CBR_134
  case 134: dcbSerialParams.BaudRate = CBR_134; break;
#endif
#ifdef CBR_150
  case 150: dcbSerialParams.BaudRate = CBR_150; break;
#endif
#ifdef CBR_200
  case 200: dcbSerialParams.BaudRate = CBR_200; break;
#endif
#ifdef CBR_300
  case 300: dcbSerialParams.BaudRate = CBR_300; break;
#endif
#ifdef CBR_600
  case 600: dcbSerialParams.BaudRate = CBR_600; break;
#endif
#ifdef CBR_1200
  case 1200: dcbSerialParams.BaudRate = CBR_1200; break;
#endif
#ifdef CBR_1800
  case 1800: dcbSerialParams.BaudRate = CBR_1800; break;
#endif
#ifdef CBR_2400
  case 2400: dcbSerialParams.BaudRate = CBR_2400; break;
#endif
#ifdef CBR_4800
  case 4800: dcbSerialParams.BaudRate = CBR_4800; break;
#endif
#ifdef CBR_7200
  case 7200: dcbSerialParams.BaudRate = CBR_7200; break;
#endif
#ifdef CBR_9600
  case 9600: dcbSerialParams.BaudRate = CBR_9600; break;
#endif
#ifdef CBR_14400
  case 14400: dcbSerialParams.BaudRate = CBR_14400; break;
#endif
#ifdef CBR_19200
  case 19200: dcbSerialParams.BaudRate = CBR_19200; break;
#endif
#ifdef CBR_28800
  case 28800: dcbSerialParams.BaudRate = CBR_28800; break;
#endif
#ifdef CBR_57600
  case 57600: dcbSerialParams.BaudRate = CBR_57600; break;
#endif
#ifdef CBR_76800
  case 76800: dcbSerialParams.BaudRate = CBR_76800; break;
#endif
#ifdef CBR_38400
  case 38400: dcbSerialParams.BaudRate = CBR_38400; break;
#endif
#ifdef CBR_115200
  case 115200: dcbSerialParams.BaudRate = CBR_115200; break;
#endif
#ifdef CBR_128000
  case 128000: dcbSerialParams.BaudRate = CBR_128000; break;
#endif
#ifdef CBR_153600
  case 153600: dcbSerialParams.BaudRate = CBR_153600; break;
#endif
#ifdef CBR_230400
  case 230400: dcbSerialParams.BaudRate = CBR_230400; break;
#endif
#ifdef CBR_256000
  case 256000: dcbSerialParams.BaudRate = CBR_256000; break;
#endif
#ifdef CBR_460800
  case 460800: dcbSerialParams.BaudRate = CBR_460800; break;
#endif
#ifdef CBR_921600
  case 921600: dcbSerialParams.BaudRate = CBR_921600; break;
#endif
  default:
    // Try to blindly assign it
    dcbSerialParams.BaudRate = baudrate_;
  }

  // setup char len
  if(bytesize_ == eightbits) {
    dcbSerialParams.ByteSize = 8;
  }
  else if(bytesize_ == sevenbits) {
    dcbSerialParams.ByteSize = 7;
  }
  else if(bytesize_ == sixbits) {
    dcbSerialParams.ByteSize = 6;
  }
  else if(bytesize_ == fivebits) {
    dcbSerialParams.ByteSize = 5;
  }
  else {
    error_.assign("Invalid bytesize");
    return serialerror_argument;
  }

  // setup stopbits
  if(stopbits_ == stopbits_one) {
    dcbSerialParams.StopBits = ONESTOPBIT;
  }
  else if(stopbits_ == stopbits_one_point_five) {
    dcbSerialParams.StopBits = ONE5STOPBITS;
  }
  else if(stopbits_ == stopbits_two) {
    dcbSerialParams.StopBits = TWOSTOPBITS;
  }
  else {
    error_.assign("Invalid stopbits");
    return serialerror_argument;
  }

  // setup parity
  if(parity_ == parity_none) {
    dcbSerialParams.Parity = NOPARITY;
  }
  else if(parity_ == parity_even) {
    dcbSerialParams.Parity = EVENPARITY;
  }
  else if(parity_ == parity_odd) {
    dcbSerialParams.Parity = ODDPARITY;
  }
  else if(parity_ == parity_mark) {
    dcbSerialParams.Parity = MARKPARITY;
  }
  else if(parity_ == parity_space) {
    dcbSerialParams.Parity = SPACEPARITY;
  }
  else {
    error_.assign("Invalid parity");
    return serialerror_argument;
  }

  // setup flowcontrol
  if(flowcontrol_ == flowcontrol_none) {
    dcbSerialParams.fOutxCtsFlow = false;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutX = false;
    dcbSerialParams.fInX = false;
  }
  else if(flowcontrol_ == flowcontrol_software) {
    dcbSerialParams.fOutxCtsFlow = false;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutX = true;
    dcbSerialParams.fInX = true;
  }
  else if(flowcontrol_ == flowcontrol_hardware) {
    dcbSerialParams.fOutxCtsFlow = true;
    dcbSerialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
    dcbSerialParams.fOutX = false;
    dcbSerialParams.fInX = false;
  }

  // activate settings
  if(!SetCommState(fd_, &dcbSerialParams)){
    CloseHandle(fd_);
    error_.assign("Error setting serial port settings.");
    return serialerror_io_failed;
  }

  // Setup timeouts
  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = timeout_.inter_byte_timeout;
  timeouts.ReadTotalTimeoutConstant = timeout_.read_timeout_constant;
  timeouts.ReadTotalTimeoutMultiplier = timeout_.read_timeout_multiplier;
  timeouts.WriteTotalTimeoutConstant = timeout_.write_timeout_constant;
  timeouts.WriteTotalTimeoutMultiplier = timeout_.write_timeout_multiplier;
  if (!SetCommTimeouts(fd_, &timeouts)) {
    error_.assign("Error setting timeouts.");
    return serialerror_io_failed;
  }

  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::close() {
  if(is_open_ == true) {
    if(fd_ != INVALID_HANDLE_VALUE) {
      if(CloseHandle(fd_) == 0) {
        stringstream ss;
        ss << "Error while closing serial port: " << GetLastError();
        error_.assign(ss.str().c_str());
        return serialerror_io_failed;
      }
      else {
        fd_ = INVALID_HANDLE_VALUE;
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
  if(!is_open_) {
    return 0;
  }

  COMSTAT cs;
  if(!ClearCommError(fd_, NULL, &cs)) {
    stringstream ss;
    ss << "Error while checking status of the serial port: " << GetLastError();
    error_.assign(ss.str().c_str());
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
  }

  return static_cast<size_t>(cs.cbInQue);
}


bool
Serial::SerialImpl::waitReadable(uint32_t /*timeout*/, serialerror_t *serialerror) {
  error_.assign("waitReadable is not implemented on Windows.");
  if(serialerror != nullptr) {
    *serialerror = serialerror_io_failed;
  }

  return false;
}


void
Serial::SerialImpl::waitByteTimes(size_t /*count*/) {
  error_.assign("waitByteTimes is not implemented on Windows.");
}


size_t
Serial::SerialImpl::read(uint8_t *buf, size_t size, serialerror_t *serialerror) {
  if(!is_open_) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }
    error_.assign("Serial::read: Port not open");
    return 0;
  }

  DWORD bytes_read;
  if(!ReadFile(fd_, buf, static_cast<DWORD>(size), &bytes_read, NULL)) {
    stringstream ss;
    ss << "Error while reading from the serial port: " << GetLastError();
    error_.assign(ss.str().c_str());
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
  }

  return (size_t) (bytes_read);
}


size_t
Serial::SerialImpl::write (const uint8_t *data, size_t length, serialerror_t *serialerror) {
  if(is_open_ == false) {
    if(serialerror != nullptr) {
      *serialerror = serialerror_not_opened;
    }
    error_.assign("Serial::write: Port not open");
    return 0;
  }

  DWORD bytes_written;
  if(!WriteFile(fd_, data, static_cast<DWORD>(length), &bytes_written, NULL)) {
    stringstream ss;
    ss << "Error while writing to the serial port: " << GetLastError();
    error_.assign(ss.str().c_str());
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
  }

  return (size_t) (bytes_written);
}


void
Serial::SerialImpl::setPort(const string &port, serialerror_t *serialerror) {
  port_ = wstring(port.begin(), port.end());
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }
}


string
Serial::SerialImpl::getPort(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(port_);
}


void
Serial::SerialImpl::setTimeout(serial::Timeout &timeout, serialerror_t *serialerror) {
  timeout_ = timeout;
  serialerror_t err = serialerror_success;
  if(is_open_) {
    err = reconfigurePort();
  }

  if(serialerror != nullptr) {
    *serialerror = err;
  }
}


serial::Timeout
Serial::SerialImpl::getTimeout(serialerror_t *serialerror) const {
  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

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
    error_.assign("Serial::flush: Port not open");
    return serialerror_not_opened;
  }

  FlushFileBuffers(fd_);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::flushInput() {
  if(is_open_ == false) {
    error_.assign("Serial::flushInput: Port not open");
    return serialerror_not_opened;
  }

  PurgeComm(fd_, PURGE_RXCLEAR);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::flushOutput() {
  if(is_open_ == false) {
    error_.assign("Serial::flushInput: Port not open");
    return serialerror_not_opened;
  }

  PurgeComm(fd_, PURGE_TXCLEAR);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::sendBreak (int /*duration*/) {
  if(is_open_ == false) {
    error_.assign("Serial::sendBreak: Port not open");
    return serialerror_not_opened;
  }

  error_.assign("sendBreak is not supported on Windows.");
  return serialerror_serial;
}


serialerror_t
Serial::SerialImpl::setBreak(bool level) {
  if(is_open_ == false) {
    error_.assign("Serial::setBreak: Port not open");
    return serialerror_not_opened;
  }

  EscapeCommFunction(fd_, level ? SETBREAK : CLRBREAK);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::setRTS(bool level) {
  if(is_open_ == false) {
    error_.assign("Serial::setRTS: Port not open");
    return serialerror_not_opened;
  }

  EscapeCommFunction(fd_, level ? SETRTS : CLRRTS);
  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::setDTR(bool level) {
  if(is_open_ == false) {
    error_.assign("Serial::setDTS: Port not open");
    return serialerror_not_opened;
  }

  EscapeCommFunction(fd_, level ? SETDTR : CLRDTR);
  return serialerror_success;
}


bool
Serial::SerialImpl::waitForChange(serialerror_t *serialerror) {
  if(is_open_ == false) {
    error_.assign("Serial::waitForChange: Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    return false;
  }
  DWORD dwCommEvent;
 
  if(!SetCommMask(fd_, EV_CTS | EV_DSR | EV_RING | EV_RLSD)) {
    // Error setting communications mask
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }
    return false;
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return !!WaitCommEvent(fd_, &dwCommEvent, NULL);
}


bool
Serial::SerialImpl::getCTS(serialerror_t *serialerror) {
  if(is_open_ == false) {
    error_.assign("Serial::getCTS: Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    return false;
  }

  DWORD dwModemStatus;
  if(!GetCommModemStatus(fd_, &dwModemStatus)) {
    error_.assign("Error getting the status of the CTS line.");
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }

    return false;
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return (MS_CTS_ON & dwModemStatus) != 0;
}


bool
Serial::SerialImpl::getDSR(serialerror_t *serialerror) {
  if(is_open_ == false) {
    error_.assign("Serial::getDTR: Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    return false;
  }

  DWORD dwModemStatus;
  if(!GetCommModemStatus(fd_, &dwModemStatus)) {
    error_.assign("Error getting the status of the DSR line.");
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }

    return false;
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return (MS_DSR_ON & dwModemStatus) != 0;
}


bool
Serial::SerialImpl::getRI(serialerror_t *serialerror) {
  if(is_open_ == false) {
    error_.assign("Serial::getRI: Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    return false;
  }

  DWORD dwModemStatus;
  if(!GetCommModemStatus(fd_, &dwModemStatus)) {
    error_.assign("Error getting the status of the RI line.");
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }

    return false;
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return (MS_RING_ON & dwModemStatus) != 0;
}


bool
Serial::SerialImpl::getCD(serialerror_t *serialerror) {
  if(is_open_ == false) {
    error_.assign("Serial::getCD: Port not open");
    if(serialerror != nullptr) {
      *serialerror = serialerror_serial;
    }

    return false;
  }

  DWORD dwModemStatus;
  if(!GetCommModemStatus(fd_, &dwModemStatus)) {
    // Error in GetCommModemStatus;
    error_.assign("Error getting the status of the CD line.");
    if(serialerror != nullptr) {
      *serialerror = serialerror_io_failed;
    }

    return false;
  }

  if(serialerror != nullptr) {
    *serialerror = serialerror_success;
  }

  return (MS_RLSD_ON & dwModemStatus) != 0;
}


serialerror_t
Serial::SerialImpl::readLock() {
  if(WaitForSingleObject(read_mutex, INFINITE) != WAIT_OBJECT_0) {
    error_.assign("Error claiming read mutex.");
    return serialerror_mutex;
  }

  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::readUnlock() {
  if(!ReleaseMutex(read_mutex)) {
    error_.assign("Error releasing read mutex.");
    return serialerror_mutex;
  }

  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::writeLock() {
  if(WaitForSingleObject(write_mutex, INFINITE) != WAIT_OBJECT_0) {
    error_.assign("Error claiming write mutex.");
    return serialerror_mutex;
  }

  return serialerror_success;
}


serialerror_t
Serial::SerialImpl::writeUnlock() {
  if(!ReleaseMutex(write_mutex)) {
    error_.assign("Error releasing write mutex.");
    return serialerror_mutex;
  }

  return serialerror_success;
}

#endif // #if defined(_WIN32)

