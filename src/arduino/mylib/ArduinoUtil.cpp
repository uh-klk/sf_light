#include "ArduinoUtil.h"

float convertbytes2float(unsigned char byte0, unsigned char byte1, unsigned char byte2, unsigned char byte3 )
{
	static bytes2float convertBytes2Float;

 	convertBytes2Float.b[0] = byte0;
 	convertBytes2Float.b[1] = byte1;
 	convertBytes2Float.b[2] = byte2;
 	convertBytes2Float.b[3] = byte3;

  return convertBytes2Float.f;  
}

long convertbytes2long(unsigned char byte0, unsigned char byte1, unsigned char byte2, unsigned char byte3) 
{
	long value = 0;
  value = ((unsigned long) byte0) << 24;
  value |= ((unsigned long) byte1) << 16;
  value |= ((unsigned long) byte2) << 8;
  value |= ((unsigned long) byte3);
  
  return value;
}
