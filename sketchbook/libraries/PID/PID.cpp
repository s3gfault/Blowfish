
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




#include <PID.h>

PID::PID(float *ncf, float ng, int8_t nl,float nh){
cf = ncf;
h = nh;
#if PID_XBUF_ENABLE
xlen = nl +1;
#endif

ylen = nl +1;
elen = nl +1;
gain = ng;
ierr = 0.0f;

ymin = 0.0f;
ymax = 0.0f;

#if PID_XBUF_ENABLE
x = (float*) calloc(xlen,sizeof(float));
#endif

y = (float*) calloc(ylen,sizeof(float));
e = (float*) calloc(elen,sizeof(float));
}//PID


void PID::setCoeffs(float *ncf){

cf = ncf;
}//setCoeffs



void PID::setGain(float ng){

gain = ng;

}//setGain



float PID::getGain(void){
return gain;

}//getGain

void PID::setH(float nh){

h = nh;

}//setH



float PID::getH(void){
return h;

}//getH

void PID::setIerr(float nerr){
ierr = nerr;
}//setIerr


float PID::step(float w,float x0){

float yn = 0.0f;
#if PID_XBUF_ENABLE
x[bi] = x0;
#endif

// err
e[ci] = w - x0;

#if PID_XBUF_ENABLE
bi = (bi +1) % xlen;
#endif

// P
yn += cf[0] * e[ci] ;

// I
ierr += (h/2.0f)*(e[(int)(((ci - 1)<0)?(elen + (ci - 1)):(ci - 1))] + e[ci]);
yn += cf[1] * ierr;

// D 
yn += cf[2] * (e[ci] - e[(int)(((ci - 1)<0)?(elen + (ci - 1)):(ci - 1))]);

ci = (ci+1) % elen;


yn *= gain;

if(!((ymin == 0.0f) && (ymax == 0.0f))){

	if(yn <= ymin){

		yn = ymin;

	}else if(yn >= ymax){
		yn = ymax;
	}

}

y[ai] = yn;
ai = (ai + 1) % ylen;
return yn;

}//step


void PID::reset(void){

 ierr = 0.0f;
#if PID_XBUF_ENABLE
for(int i= 0; i < xlen;i++){
x[i] = 0.0f;
}
#endif

for(int i= 0; i < elen;i++){
e[i] = 0.0f;
}

for(int i= 0; i < ylen;i++){
y[i] = 0.0f;
}

}

#if PID_XBUF_ENABLE
float PID::getX(int8_t idx){
	if(idx >= xlen){
	idx = xlen-1;
	}

return x[(int)(((bi - idx - 1)<0)?(xlen + (bi - idx -1)):(bi - idx -1))];
}//getX
#endif 

float PID::getY(int8_t idx){
if(idx >= ylen){
idx = ylen-1;
}

return y[(int)(((ai - idx - 1)<0)?(ylen + (ai - idx -1)):(ai - idx -1))];
}//getY


float PID::getE(int8_t idx){
if(idx >= elen){
idx = elen-1;
}

return e[(int)(((ci - idx - 1)<0)?(elen + (ci - idx -1)):(ci - idx -1))];
}//getE

void PID::setOutputLimits(float min , float max){ // min==max==0 -> no limit
if(max < min)return;

ymin = min;
ymax = max;


}


