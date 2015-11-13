#ifndef __ARDUINOUTIL_H
#define __ARDUINOUTIL_H

#include <FastLED.h>

//for converting bites back to float
typedef union {
    float f;
    unsigned char b[4];
} bytes2float;

//expected data format from driver
struct requestdataseq {                       
	unsigned int mode;
	float frequency;
	CRGB ledRGBColor;
	CHSV ledHSVColor;
	unsigned char ledIntencity;
	float on_pct;
};


float convertbytes2float(unsigned char byte0, unsigned char byte1, unsigned char byte2, unsigned char byte3 );

long convertbytes2long(unsigned char byte0, unsigned char byte1, unsigned char byte2, unsigned char byte3);

#endif
