/*!
 * \file serial/impl/unix.h
 * \author  William Woodall <wjwwood@gmail.com>
 * \author  John Harrison <ash@greaterthaninfinity.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 William Woodall, John Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a unix based pimpl for the Serial class. This implementation is
 * based off termios.h and uses select for multiplexing the IO ports.
 *
 */

#if !defined(_WIN32)

#ifndef SERIAL_IMPL_UNIX_H
#define SERIAL_IMPL_UNIX_H

#include "serial/serial.h"

#include <pthread.h>

namespace serial {

using std::size_t;
using std::string;


class MillisecondTimer {
public:
  MillisecondTimer(const uint32_t millis);         
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

class serial::Serial::SerialImpl {
public:
  SerialImpl (const string &port,
              unsigned long baudrate,
              bytesize_t bytesize,
              parity_t parity,
              stopbits_t stopbits,
              flowcontrol_t flowcontrol,
	      serialerror_t *serialerror);

  virtual ~SerialImpl ();

  const std::string &
  getLastError() const;

  serialerror_t
  open();

  serialerror_t
  close();

  bool
  isOpen() const;

  size_t
  available(serialerror_t *serialerror = nullptr);

  bool
  waitReadable(uint32_t timeout, serialerror_t *serialerror = nullptr);

  void
  waitByteTimes(size_t count);

  size_t
  read(uint8_t *buf, size_t size = 1, serialerror_t *serialerror = nullptr);

  size_t
  write(const uint8_t *data, size_t length, serialerror_t *serialerror = nullptr);

  serialerror_t
  flush();

  serialerror_t
  flushInput();

  serialerror_t
  flushOutput();

  serialerror_t
  sendBreak(int duration);

  serialerror_t
  setBreak(bool level);

  serialerror_t
  setRTS(bool level);

  serialerror_t
  setDTR(bool level);

  bool
  waitForChange(serialerror_t *serialerror = nullptr);

  bool
  getCTS(serialerror_t *serialerror = nullptr);

  bool
  getDSR(serialerror_t *serialerror = nullptr);

  bool
  getRI(serialerror_t *serialerror = nullptr);

  bool
  getCD(serialerror_t *serialerror = nullptr);

  void
  setPort(const string &port, serialerror_t *serialerror = nullptr);

  string
  getPort(serialerror_t *serialerror = nullptr) const;

  void
  setTimeout(Timeout &timeout);

  Timeout
  getTimeout() const;

  void
  setBaudrate(unsigned long baudrate, serialerror_t *serialerror = nullptr);

  unsigned long
  getBaudrate(serialerror_t *serialerror = nullptr) const;

  void
  setBytesize(bytesize_t bytesize, serialerror_t *serialerror = nullptr);

  bytesize_t
  getBytesize(serialerror_t *serialerror = nullptr) const;

  void
  setParity(parity_t parity, serialerror_t *serialerror = nullptr);

  parity_t
  getParity(serialerror_t *serialerror = nullptr) const;

  void
  setStopbits(stopbits_t stopbits, serialerror_t *serialerror = nullptr);

  stopbits_t
  getStopbits(serialerror_t *serialerror = nullptr) const;

  void
  setFlowcontrol(flowcontrol_t flowcontrol, serialerror_t *serialerror = nullptr);

  flowcontrol_t
  getFlowcontrol(serialerror_t *serialerror = nullptr) const;

  serialerror_t
  readLock();

  serialerror_t
  readUnlock();

  serialerror_t
  writeLock();

  serialerror_t
  writeUnlock();

protected:
  serialerror_t reconfigurePort();

private:
  string error_;
  string port_;               // Path to the file descriptor
  int fd_;                    // The current file descriptor

  bool is_open_;
  bool xonxoff_;
  bool rtscts_;

  Timeout timeout_;           // Timeout for read operations
  unsigned long baudrate_;    // Baudrate
  uint32_t byte_time_ns_;     // Nanoseconds to transmit/receive a single byte

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

  // Mutex used to lock the read functions
  pthread_mutex_t read_mutex;
  // Mutex used to lock the write functions
  pthread_mutex_t write_mutex;
};

}

#endif // SERIAL_IMPL_UNIX_H

#endif // !defined(_WIN32)
