




#include <PID.h>

PID::PID(float *ncf, float ng, int8_t nl,float nh){
cf = ncf;
h = nh;
xlen = nl +1;
ylen = nl +1;
elen = nl +1;
gain = ng;
ierr = 0.0f;

x = (float*) calloc(xlen,sizeof(float));
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

}//geGain

void PID::setIerr(float nerr){
ierr = nerr;
}//setIerr


float PID::step(float w,float x0){

float yn = 0.0f;
x[bi] = x0;


// err
e[ci] = w - x[bi];
bi = (bi +1) % xlen;

// P
yn += cf[0] * e[ci] ;

// I
ierr += (h/2.0f)*(e[(int)(((ci - 1)<0)?(elen + (ci - 1)):(ci - 1))] + e[ci]);
yn += cf[1] * ierr;

// D 
yn += cf[2] * (e[ci] - e[(int)(((ci - 1)<0)?(elen + (ci - 1)):(ci - 1))]);

ci = (ci+1) % elen;


yn *= gain;

y[ai] = yn;
ai = (ai + 1) % ylen;
return yn;

}//step

float PID::getX(int8_t idx){
	if(idx >= xlen){
	idx = xlen-1;
	}

return x[(int)(((bi - idx - 1)<0)?(xlen + (bi - idx -1)):(bi - idx -1))];
}//getX

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

