

/*
 -- Author: Osman Ali (o.ali@tum.de) 
 
 ============================================
This code is placed under the MIT license
Copyright (c) 2013 Osman Ali

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


#include <Arduino.h>

#define  PID_XBUF_ENABLE 0

class PID{


public:

PID( float *ncf, float ng, int8_t nl,float nh);

void setCoeffs(float * cf);

void setIerr(float);
//void setPval(float);
//void setDval(float);

void setGain(float);
float getGain(void);

void setH(float);
float getH(void);

#if PID_XBUF_ENABLE
float getX(int8_t idx);
#endif
float getY(int8_t idx);
float getE(int8_t idx);

void reset(void);

void setOutputLimits(float,float);

#if PID_XBUF_ENABLE
float *x;
#endif

float *y;
float *e;
float step(float,float);


private:
float gain; //overall gain
float* cf;
float h ; // time between samples

float ierr; // integrated error

// vars to adress the arrays x y e

int8_t ai;

#if PID_XBUF_ENABLE
int8_t bi;
#endif

int8_t ci;

#if PID_XBUF_ENABLE
int8_t xlen;
#endif
int8_t ylen;
int8_t elen;

float ymin;
float ymax;

};
