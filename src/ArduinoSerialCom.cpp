#include "ArduinoSerialCom.h"

void ArduinoSerialCommunication::setPort(char* portname)
{
    portname_ = portname;
}

void ArduinoSerialCommunication::setBaud(int baudrate)
{
    baudrate_ = baudrate;
}

int ArduinoSerialCommunication::openPort()
{
    fh_ = open (portname_, O_RDWR | O_NOCTTY | O_SYNC);
    if (fh_ < 0)
    {
        printf("error %d opening %s: %s", errno, portname_, strerror (errno));
        return 0;
    }

    set_interface_attribs (fh_, baudrate_, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fh_, 0);	// set no blocking}

    return 1;
}

unsigned char ArduinoSerialCommunication::calChkSum(unsigned char *payload, int payloadSize)
{
    int sum = 0;
    unsigned char chkSum;
	
    for(int i =0; i< payloadSize; i++)
    {
        sum += payload[i];
    }

    printf("\n Payload sum : %d", sum);
    chkSum = (unsigned char) (sum & 0xFF);
	
    return chkSum;
}

void ArduinoSerialCommunication::sendData(unsigned char *payload, int payloadSize)
{
    unsigned char header [] = {255,255,255,15};
    unsigned int bytesOfRelevantData = 0; //to stored the size of relevant data for arduino to read the payload and checksum
    unsigned char checksum [] = {0};
    unsigned char footer [] = {255,255};
	
    int headerSize = sizeof(header) / sizeof(unsigned char);
    int relevantDataSize = sizeof(unsigned char); //store the size of payload + checksum
    int checksumSize = sizeof(checksum)/sizeof(unsigned char);
    int footerSize = sizeof(footer) / sizeof(unsigned char);
  
    unsigned char package [headerSize + relevantDataSize + payloadSize + checksumSize + footerSize] ;
	
    //pack header
    printf("\n Header : ");
    for (int i = 0; i < headerSize; ++i) {
        package[i] = header[i];
        printf(" %d",header[i]);
    }

    //pack payload
    printf("\n Payload : ");
    for (int i = headerSize + relevantDataSize; i < (headerSize + relevantDataSize + payloadSize); ++i)
    {
        package[i] = payload[i - (headerSize + relevantDataSize)];
        bytesOfRelevantData += 1;
        printf(" [%d]=%d", i - (headerSize + relevantDataSize), payload[i - (headerSize + relevantDataSize)]);
    }

    //pack checksum
    package[headerSize + relevantDataSize + payloadSize] = calChkSum(payload, payloadSize);
    printf("\n Checksum : %d",package[headerSize + relevantDataSize + payloadSize]);
    bytesOfRelevantData += 1;
  			
    //pack the num of relevant data bytes just after the header
    printf("\n Bytes of Relevant Data : ");
    package[headerSize] = bytesOfRelevantData;
    printf(" %d ", bytesOfRelevantData);
  			
    //pack fooder
    printf("\n Fooder : ");
    for (int i = headerSize + relevantDataSize + payloadSize + checksumSize; i < (headerSize + relevantDataSize + payloadSize + checksumSize + footerSize); ++i)
    {
        package[i] = footer[i - (headerSize + relevantDataSize + payloadSize + checksumSize)];
        printf(" %d", footer[i - (headerSize + relevantDataSize + payloadSize + checksumSize)]);
    }
		
	
    printf("\n ------------------------------------------");
    printf("\n Package is: ");

    for (int i = 0; i < (headerSize + relevantDataSize + payloadSize + checksumSize + footerSize); i++) {
        printf("%d, ",package[i]);
    }
	
    printf("\n ------------------------------------------");
    write(fh_, package, headerSize + relevantDataSize + payloadSize + checksumSize + footerSize);
}

void ArduinoSerialCommunication::closePort()
{
    close(fh_);
}

void ArduinoSerialCommunication::signalHandler( int signum )
{
    cout << "\nInterrupt signal (" << signum << ") received.\n";
    close(fh_); //close port when ctrl+c signal is received
    exit(signum);
}

int ArduinoSerialCommunication::set_interface_attribs (int fh, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fh, &tty) != 0)
    {
        printf("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    //  no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                        //   enable reading
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fh, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr", errno);
        return -1;
    }

    return 0;
}

void ArduinoSerialCommunication::set_blocking (int fh, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fh, &tty) != 0)
    {
        printf("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;    // 0.5 seconds read timeout

    if (tcsetattr (fh, TCSANOW, &tty) != 0)
        printf("error %d setting term attributes", errno);
}
