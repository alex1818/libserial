/*
 * serial.c:
 *	Handle a serial port
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "serial.h"

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

int serialOpen (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;

  switch (baud)
  {
    case     50:	myBaud =     B50 ; break ;
    case     75:	myBaud =     B75 ; break ;
    case    110:	myBaud =    B110 ; break ;
    case    134:	myBaud =    B134 ; break ;
    case    150:	myBaud =    B150 ; break ;
    case    200:	myBaud =    B200 ; break ;
    case    300:	myBaud =    B300 ; break ;
    case    600:	myBaud =    B600 ; break ;
    case   1200:	myBaud =   B1200 ; break ;
    case   1800:	myBaud =   B1800 ; break ;
    case   2400:	myBaud =   B2400 ; break ;
    case   4800:	myBaud =   B4800 ; break ;
    case   9600:	myBaud =   B9600 ; break ;
    case  19200:	myBaud =  B19200 ; break ;
    case  38400:	myBaud =  B38400 ; break ;
    case  57600:	myBaud =  B57600 ; break ;
    case 115200:	myBaud = B115200 ; break ;
    case 230400:	myBaud = B230400 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  return fd ;
}


/*
 * serialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

void serialFlush (const int fd)
{
  tcflush (fd, TCIOFLUSH) ;
}

/*
 * serialFlushIn:
 *	Flush the serial tx buffer
 *********************************************************************************
 */

void serialFlushIn (const int fd)
{
  tcflush (fd, TCIFLUSH) ;
}

/*
 * serialFlushOut:
 *	Flush the serial rx buffer
 *********************************************************************************
 */

void serialFlushOut (const int fd)
{
  tcflush (fd, TCOFLUSH) ;
}


/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void serialClose (const int fd)
{
  close (fd) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

void serialPutchar (const int fd, const unsigned char c)
{
  write (fd, &c, 1) ;
}


/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

void serialPuts (const int fd, const char *s)
{
  write (fd, s, strlen (s)) ;
}

/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf (const int fd, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (fd, buffer) ;
}


/*
 * serialDataToRead:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int serialDataToRead (const int fd)
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}

/*
 * serialDataToWrite:
 *	Return the number of bytes of data avalable to write to the serial port
 *********************************************************************************
 */

int serialDataToWrite(const int fd)
{
    int result;

    if (ioctl(fd, TIOCOUTQ, &result) < 0)
        return -1;

    return result;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int serialGetchar (const int fd)
{
  uint8_t x ;

  if (read (fd, &x, 1) != 1)
    return -1 ;

  return ((int)x) & 0xFF ;
}

/*
 * serialSetDataBit:
 *	Sets the DataBit
 *********************************************************************************
 */

int serialSetDataBit(const int fd, const int dataBit)
{
    struct termios options;

    if (dataBit < 5 || dataBit > 8)
        return -1;

    tcgetattr(fd, &options);

    options.c_cflag &= ~CSIZE;

    switch (dataBit)
    {
        case 5: options.c_cflag |= CS5; break;
        case 6: options.c_cflag |= CS6; break;
        case 7: options.c_cflag |= CS7; break;
        case 8:
        default: options.c_cflag |= CS8; break;
    }

    if (tcsetattr(fd, TCSANOW | TCSAFLUSH, &options) < 0)
        return -1;

    return 1;

}

/*
 * serialSetStopbits:
 *	Sets the Stopbits
 *********************************************************************************
 */

int serialSetStopits(const int fd, const int sb)
{
    struct termios options;

    if (sb < 0 || sb > 3)
        return -1;

    tcgetattr(fd, &options);

    switch(sb)
    {
        case 0 : break; //Not used
        case 1 : options.c_cflag &= ~CSTOPB; break;
        case 2 : options.c_cflag |= CSTOPB; break;
        case 3 : break; //Not used
    }

    if (tcsetattr(fd, TCSANOW | TCSAFLUSH, &options) < 0)
        return -1;

    return 1;
}

/*
 * serialSetParity:
 *	Sets the Parity
 *********************************************************************************
 */

 int serialSetParity(const int fd, const int parity)
 {
    struct termios options;

    if (parity < 0 || parity > 4)
        return -1;

    tcgetattr(fd, &options);

    options.c_iflag &= ~(INPCK | ISTRIP);

    switch(parity)
    {
        case 0 :    // No parity
            options.c_cflag &= ~(PARENB | PARODD);
            break;
        case 1 :    // Odd parity
            options.c_cflag |= PARENB | PARODD;
            break;
        case 2 :    // Even parity
            options.c_cflag &= ~PARODD;
            options.c_cflag |= PARENB;
            break;
        case 3 :    // Not implemented
        case 4 :    // Not implemented
            break;
    }

    if (tcsetattr(fd, TCSANOW | TCSAFLUSH, &options) < 0)
        return -1;

    return 1;
 }

/*
 * serialSetHandshake:
 *	Sets the Handshake
 *********************************************************************************
 */

 int serialSetHandshake(const int fd, const int handshake)
 {
    struct termios options;

    if (handshake < 0 || handshake > 3)
        return -1;

    tcgetattr(fd, &options);

    // The following was derived from mono serial.c
    options.c_iflag &= ~(IXOFF | IXON);

#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif // CRTSCTS

    switch(handshake)
    {
        case 0 : break; // None - Do nothing
        case 1 :        // RequestToSend (RTS)
#ifdef CRTSCTS
            options.c_cflag |= CRTSCTS;
#endif // CRTSCTS
            break;
        case 2 :        // RequestToSendXOnXOff (RTS + XON/XOFF)
#ifdef CRTSCTS
            options.c_cflag |= CRTSCTS;
#endif // CRTSCTS
            // Fall through
        case 3 :        // XOn/Xoff
            options.c_iflag |= IXOFF | IXON;
            break;
    }

    if (tcsetattr(fd, TCSANOW | TCSAFLUSH, &options) < 0)
        return -1;

    return 1;
 }

/*
 * serialSetBaud:
 *  Sets the desired baud rate
 *********************************************************************************
 */

int serialSetBaud(const int fd, const int baud)
{
    struct termios options;
    speed_t myBaud;

    switch (baud)
    {
        case     50:	myBaud =     B50 ; break ;
        case     75:	myBaud =     B75 ; break ;
        case    110:	myBaud =    B110 ; break ;
        case    134:	myBaud =    B134 ; break ;
        case    150:	myBaud =    B150 ; break ;
        case    200:	myBaud =    B200 ; break ;
        case    300:	myBaud =    B300 ; break ;
        case    600:	myBaud =    B600 ; break ;
        case   1200:	myBaud =   B1200 ; break ;
        case   1800:	myBaud =   B1800 ; break ;
        case   2400:	myBaud =   B2400 ; break ;
        case   4800:	myBaud =   B4800 ; break ;
        case   9600:	myBaud =   B9600 ; break ;
        case  19200:	myBaud =  B19200 ; break ;
        case  38400:	myBaud =  B38400 ; break ;
        case  57600:	myBaud =  B57600 ; break ;
        case 115200:	myBaud = B115200 ; break ;
        case 230400:	myBaud = B230400 ; break ;

        default:
          return -2 ;
    }

    tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    if (tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) < 0)
        return -1;

    return 1;
}

/*
 * serialEnableRTS:
 *  Enables RTS returns 1 on success
 *********************************************************************************
 */

int serialEnableRTS(const int fd)
{
    int rtsFlag;

    rtsFlag = TIOCM_RTS;

    if (ioctl(fd, TIOCMBIS, &rtsFlag) == -1)
        return -1;

    return 1;
}

/*
 * serialDisableRTS:
 *  Disables RTS returns 1 on success
 *********************************************************************************
 */

int serialDisableRTS(const int fd)
{
    int rtsFlag;

    rtsFlag = TIOCM_RTS;

    if (ioctl(fd, TIOCMBIC, &rtsFlag) == -1)
        return -1;

    return 1;
}

/*
 * serialEnableDTR:
 *  Enables DTR returns 1 on success
 *********************************************************************************
 */

int serialEnableDTR(const int fd)
{
    int dtrFlag;

    dtrFlag = TIOCM_DTR;

    if (ioctl(fd, TIOCMBIS, &dtrFlag) == -1)
        return -1;

    return 1;
}

/*
 * serialDisableDTR:
 *  Disables DTR returns 1 on success
 *********************************************************************************
 */

int serialDisableDTR(const int fd)
{
    int dtrFlag;

    dtrFlag = TIOCM_DTR;

    if (ioctl(fd, TIOCMBIC, &dtrFlag) == -1)
        return -1;

    return 1;
}
