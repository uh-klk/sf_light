#ifndef __ARDUINOSERIALCOM_H
#define __ARDUINOSERIALCOM_H

#include <iostream>

#include<fstream>
#include<cstdlib>

#include <csignal>      //for ctrl+c signal handling

#include <vector>

//Serial port interface program
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <string.h>     // string function definitions
#include <fcntl.h>      // File control definitions

using namespace std;
/* The values for speed are B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800, etc.
   * The values for parity are 0 (meaning no parity), PARENB|PARODD (enable parity and use odd),
   * PARENB (enable parity and use even), PARENB|PARODD|CMSPAR (mark parity), and PARENB|CMSPAR (space parity).
   *
   * Blocking" sets whether a read() on the port waits for the specified number of characters to arrive.
   * Setting no blocking means that a read() returns however many characters are available without waiting
   * for more, up to the buffer limit.
   */
   
class ArduinoSerialCommunication
{
public:

    ArduinoSerialCommunication(char* portname, int baudrate) : portname_(portname), baudrate_(baudrate)
    {
        fh_ = -1;
    };
	
    ~ArduinoSerialCommunication()
    {
        closePort();
    }
    void setPort(char* portname);
    void setBaud(int baudrate);
    int openPort();
    void closePort();
    void signalHandler( int signum );
    int set_interface_attribs (int fh, int speed, int parity);
    void set_blocking (int fh, int should_block);
    void sendData(unsigned char *data1, int arraySize);
    unsigned char calChkSum(unsigned char *payload, int payloadSize);

protected:
    int fh_; //file handle
    char *portname_; //i.e. "/dev/ttyACM0";
    int baudrate_;
};


#endif
