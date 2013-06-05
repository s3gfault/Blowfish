



#include <Arduino.h>

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

float getX(int8_t idx);
float getY(int8_t idx);
float getE(int8_t idx);

void reset(void);

void setOutputLimits(float,float);

float *x;
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
int8_t bi;
int8_t ci;

int8_t xlen;
int8_t ylen;
int8_t elen;

float ymin;
float ymax;

};
