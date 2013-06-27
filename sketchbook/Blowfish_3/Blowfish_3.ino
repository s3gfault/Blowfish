/*
-- BLOWFISH AIRSHIP CONTROL SOFTWARE
 |
 -- Author: Osman Ali (o.ali@tum.de) 
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 
 */

/*

 How this program works:
 The program is divided into four parts: Declaration,Setup, Loop, and the ISR
 
 1. Declaration:
     In this area all necessary data structure are defined and declared. At the beginning there's a small
     config section, which allows to en-/ disable and/or change how key parts of the sofware work
 
 2. Setup
     Initializes all used internal and external Hardware
     
 3. Loop    
     The loop consists of several if conditionals. They're jumped into when the ISR sets the corresponding flags.
     If no flags are set, nothing will happen until a flag is set.
     
 4. ISR
     This interrupt handler catches the timer overflow interrupt that occurs every millisecond.
 
 // TODO: finish
 
 */
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <SRF02.h>

#include <IIR.h>

#include <PID.h>
//#include <MadgwickAHRS.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here


// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
#define DBGOUTMODE 8
#define DBG_ISR 1

#if  DBG_ISR
     byte blinkstate =0;
#endif
float dbg1f=0.0f,dbg2f=0.0f,dbg3f=0.0f;
int dbg1 = 0,dbg2 = 0,dbg3 = 0;

// config
#define SERIALBAUD 19200

#define TRAPEZ 1
#define SIMPSON 2
#define INTEGRATION TRAPEZ // set integration method

// self explanationary
#define MAGN_ENABLE 1  
#define ULTS_ENABLE 1
#define ACCEL_ENABLE 0
#define GYRO_ENABLE 1
#define IPS_TX_ENABLE 1
#define DROP_ENABLE 0

// a moving avg for debugging
#define MOVAVG 0

// select how you want to comunicate with the baloon
#define ATOF 1  // type float directly in terminal
#define HEX  2 // hex needs our base station
#define SERIALCONV ATOF 

#define HIGHPASS 1
#define LOWPASS 2
#define GYRO_FILT LOWPASS   // filter type for the Gyroscope

// select how the motors shoudl be stopped
#define MOT_SHORT 0
#define MOT_FREERUN 1
#define MOT_HALTMODE MOT_FREERUN

#define MC_GYRO_OMEGA 1
#define MC_GYRO_PHI 2
#define MC_MAGN 3
#define MC_GYRO_MAGN 4
#define MOT_CONT_METH MC_MAGN

#define INITMSG 0

#define DBGPIN 13

MPU6050 accelgyro(0x69);

#if MAGN_ENABLE 
HMC5883L compass;
#endif

#if ULTS_ENABLE 

SRF02 ultrasonic(0x70,SRF02_CENTIMETERS);

#endif
//prototypes
//void setStruct( IMUFilt *imu, float xval,float yval,float zval);

String inString;
String cmd;

// volatile var for the interrupt routine 

#if ((GYRO_ENABLE ) || (ACCEL_ENABLE))
volatile byte imuready = 0;
#endif 

#if MAGN_ENABLE 
volatile byte magnready = 0;
#endif

volatile byte motcont = 0;

#if ULTS_ENABLE 
volatile byte ultsready = 0;
#endif

#if IPS_TX_ENABLE
volatile byte ips_ready = 0;
byte ips_read = 0;

byte ips_tx_on = 1;

#endif

volatile byte serialready = 0;

volatile uint16_t ovfcnt = 0;
volatile uint8_t ovfcnt2 = 0;
volatile uint8_t ovfcnt3 = 0;


volatile byte serialstrbegin = 0;

byte imudataready = 0;

#if MAGN_ENABLE 
byte magndataready = 0;
byte get_magn_offset = 0;
#endif

#if ULTS_ENABLE 
byte ultsread = 0;
#endif



byte main_enable = 0;

byte mot_rl_cont_auto = 1;
byte mot_alt_cont_auto = 1;

byte mot_rl_cont_gyro = 0;

#define LPAORDER 1
#define LPBORDER 1
#define LPGAIN 0.02008f

// define the filter constants for the sensors

//general:

#define ACCEL_GYRO_SAMPLERATE_HZ 50
//ACCELEROMETE

#if ACCEL_ENABLE

#define LP_ACCEL_ORDER 1
#define LP_ACCEL_CUTOFF_HZ 12.5f
#define LP_ACCEL_RC (1.0f/LP_ACCEL_CUTOFF_HZ)

float lp_accel_alpha = ((1.0f/ACCEL_GYRO_SAMPLERATE_HZ)/(LP_ACCEL_RC+(1.0f/ACCEL_GYRO_SAMPLERATE_HZ)));

float lp_accel_b[LP_ACCEL_ORDER+1] = {
  lp_accel_alpha ,0.0f};
float lp_accel_a[LP_ACCEL_ORDER+1] = {
  1.0f,(1.0f-lp_accel_alpha)};

IIR lowpass_accel_x(lp_accel_a,lp_accel_b,LP_ACCEL_ORDER,LP_ACCEL_ORDER);
IIR lowpass_accel_y(lp_accel_a,lp_accel_b,LP_ACCEL_ORDER,LP_ACCEL_ORDER);
IIR lowpass_accel_z(lp_accel_a,lp_accel_b,LP_ACCEL_ORDER,LP_ACCEL_ORDER);

#endif
//GYROSCOPE

#define GYRO_SCALE_LSB 65.5f

#define GYROOFFSET_X 1.3624f // found out experimentally, trough a moving average while gyro is static
#define GYROOFFSET_Y -1.5140f
#define GYROOFFSET_Z -1.3335f

#if GYRO_FILT == HIGHPASS

#define HP_GYRO_ORDER 2
#define HP_GYRO_CUTOFF_HZ 12.5f
#define HP_GYRO_RC (1.0f/HP_GYRO_CUTOFF_HZ)

float hp_gyro_alpha = ((HP_GYRO_RC)/(HP_GYRO_RC+(1.0f/ACCEL_GYRO_SAMPLERATE_HZ)));

float hp_gyro_b[HP_GYRO_ORDER+1] = {
  hp_gyro_alpha ,0.0f,0.0f};
float hp_gyro_a[HP_GYRO_ORDER+1] = {
  1.0f,(1.0f-hp_gyro_alpha),0.0f};

IIR gyro_filt_x(hp_gyro_a,hp_gyro_b,HP_GYRO_ORDER,HP_GYRO_ORDER);
IIR gyro_filt_y(hp_gyro_a,hp_gyro_b,HP_GYRO_ORDER,HP_GYRO_ORDER);
IIR gyro_filt_z(hp_gyro_a,hp_gyro_b,HP_GYRO_ORDER,HP_GYRO_ORDER);

#elif GYRO_FILT == LOWPASS

#define LP_GYRO_ORDER 2
#define LP_GYRO_CUTOFF_HZ 6.0f
#define LP_GYRO_RC (1.0f/LP_GYRO_CUTOFF_HZ)

float lp_gyro_alpha = ((1.0f/ACCEL_GYRO_SAMPLERATE_HZ)/(LP_GYRO_RC+(1.0f/ACCEL_GYRO_SAMPLERATE_HZ)));

float lp_gyro_b[LP_GYRO_ORDER+1] = {
  lp_gyro_alpha ,0.0f,0.0f};
float lp_gyro_a[LP_GYRO_ORDER+1] = {
  1.0f,(1.0f-lp_gyro_alpha),0.0f};


IIR gyro_filt_x(lp_gyro_a,lp_gyro_b,LP_GYRO_ORDER,LP_GYRO_ORDER);
IIR gyro_filt_y(lp_gyro_a,lp_gyro_b,LP_GYRO_ORDER,LP_GYRO_ORDER);
IIR gyro_filt_z(lp_gyro_a,lp_gyro_b,LP_GYRO_ORDER,LP_GYRO_ORDER);

#endif


#if INTEGRATION == SIMPSON
byte simpcnt = 0;

#endif

//magnetometer
#if MAGN_ENABLE

#define MAGN_SAMPLERATE_HZ 50.0f

#define LP_MAGN_ORDER 1
#define LP_MAGN_CUTOFF_HZ 2.0f
#define LP_MAGN_RC (1.0f/LP_MAGN_CUTOFF_HZ)

float lp_magn_alpha = ((1.0f/MAGN_SAMPLERATE_HZ)/(LP_MAGN_RC+(1.0f/MAGN_SAMPLERATE_HZ)));

float lp_magn_b[LP_MAGN_ORDER+1] = {
  lp_magn_alpha ,0.0f};
float lp_magn_a[LP_MAGN_ORDER+1] = {
  1.0f,(1.0f-lp_magn_alpha)};

IIR lowpass_magn_x(lp_magn_a,lp_magn_b,LP_MAGN_ORDER,LP_MAGN_ORDER);
IIR lowpass_magn_y(lp_magn_a,lp_magn_b,LP_MAGN_ORDER,LP_MAGN_ORDER);
IIR lowpass_magn_z(lp_magn_a,lp_magn_b,LP_MAGN_ORDER,LP_MAGN_ORDER);

float magn_head_rad = 0.0f;
float magn_head_deg = 0.0f;
float magn_head_rad_avg = 0.0f;
float magn_head_deg_avg = 0.0f;
float magn_offset_deg = 0.0f;

int magn_head_rad_avg_cnt = 1;
//int magn_head_deg_avg_cnt = 1;


#endif
//ultrasonic



#define ULTS_SAMPLERATE_HZ 10.0f
#define ULTS_INTERVAL 100
#define LP_ULTS_ORDER 1
#define LP_ULTS_CUTOFF_HZ 4.0f
#define LP_ULTS_RC (1.0f/LP_ULTS_CUTOFF_HZ)

float lp_ults_alpha = ((1.0f/ULTS_SAMPLERATE_HZ)/(LP_ULTS_RC+(1.0f/ULTS_SAMPLERATE_HZ)));

float lp_ults_b[LP_ULTS_ORDER+1] = {
  lp_ults_alpha ,0.0f};
float lp_ults_a[LP_ULTS_ORDER+1] = {
  1.0f,(1.0f-lp_ults_alpha)};

IIR lowpass_ults_h(lp_ults_a,lp_ults_b,LP_ULTS_ORDER,LP_ULTS_ORDER);

float ults_h_raw = 0.0f;
float ults_h = 0.0f;

//structs

typedef struct {
  int16_t x; 
  int16_t y;
  int16_t z;
}
IMURaw;

struct IMUFilt;

typedef struct IMUFilt {
  float x; 
  float y;
  float z;
}
IMUFilt;

//ips Transmitter

#if IPS_TX_ENABLE

#define IPS_PIN 2
unsigned long ipstime = 0;

#endif

//package drop

#if DROP_ENABLE

#define DROP_PIN 3


#endif


//controller
#define PID_KP 1.0f
#define PID_KI 1.0f
#define PID_KD 1.0f
#define PID_GAIN 1.0f
#define PID_H (1.0f/ACCEL_GYRO_SAMPLERATE_HZ)


#define PID_MOT_RL_KP 1.3f
#define PID_MOT_RL_KI 0.0f//0.13f
#define PID_MOT_RL_KD 0.17f

#define PID_MOT_RL_GAIN PID_GAIN
#define PID_MOT_RL_H PID_H

#define PID_MOT_RL_LIM_MIN -255.0f
#define PID_MOT_RL_LIM_MAX 255.0f


float pid_mot_rl[3] = {
  PID_MOT_RL_KP,PID_MOT_RL_KI,PID_MOT_RL_KD};

float pid_mot_rl_aggro[3] = {
  PID_MOT_RL_KP,PID_MOT_RL_KI,PID_MOT_RL_KD};


float pid_mot_rl_conservative[3] = {
  PID_MOT_RL_KP,PID_MOT_RL_KI,PID_MOT_RL_KD};


PID pid_rl(pid_mot_rl,PID_MOT_RL_GAIN,3,PID_MOT_RL_H);


#define PID_MOT_ALT_KP 0.4f
#define PID_MOT_ALT_KI 0.0f//0.015f
#define PID_MOT_ALT_KD 0.0f//0.2f

#define PID_MOT_ALT_AGGRO_KP 1.0f
#define PID_MOT_ALT_AGGRO_KI 0.0f//0.015f
#define PID_MOT_ALT_AGGRO_KD 0.0f//0.2f

#define PID_MOT_ALT_CONSERVATIVE_KP 0.2f
#define PID_MOT_ALT_CONSERVATIVE_KI 0.0f //0.015f
#define PID_MOT_ALT_CONSERVATIVE_KD 0.0f//0.2f

#define PID_MOT_AGGRO_LIM 25.0f
#define PID_MOT_NORMAL_LIM 5.0f
#define PID_MOT_CONSERVATIVE_LIM 0.0f


#define PID_MOT_ALT_GAIN PID_GAIN
#define PID_MOT_ALT_H PID_H

#define PID_MOT_ALT_LIM_MIN -255.0f
#define PID_MOT_ALT_LIM_MAX 255.0f


float  pid_mot_alt[3] ={
  PID_MOT_ALT_KP,PID_MOT_ALT_KI,PID_MOT_ALT_KD};

float pid_mot_alt_aggro[3] = { 
  PID_MOT_ALT_AGGRO_KP,PID_MOT_ALT_AGGRO_KI,PID_MOT_ALT_AGGRO_KD};

float  pid_mot_alt_conservative[3] ={
  PID_MOT_ALT_CONSERVATIVE_KP,PID_MOT_ALT_CONSERVATIVE_KI,PID_MOT_ALT_CONSERVATIVE_KD};

PID pid_alt(pid_mot_alt,PID_MOT_ALT_GAIN,3,PID_MOT_ALT_H);

//control parameters

float reg_set_rl_ang = 0.0f;
int reg_set_speed = 0;
float reg_set_h = 130.0f;


//motor controls

#define MOT_R_PWM_PIN 9 //9
#define MOT_L_PWM_PIN 10//10
#define MOT_ALT_PWM_PIN 6//6

#define MOT_R_IN1_PIN 8//7
#define MOT_R_IN2_PIN 7//8

#define MOT_L_IN1_PIN 11//11
#define MOT_L_IN2_PIN 12//12

#define MOT_ALT_IN1_PIN 5//5
#define MOT_ALT_IN2_PIN 4//4

#define MOT_DIR 0
#define MOT_ALT 1

/* in1 in2 
 0    0    stopp
 1    0     r1
 0     1    r2
 1     1  SHORT
 
 
 */
IMURaw gr,ar;
IMUFilt gf,af,gyAng;


#if MAGN_ENABLE 
MagnetometerScaled mr,mf; 

#endif

volatile byte dpin = 0;

//long t1 = 0,t2 = 0,t3 = 0,t4 = 0,t5=0;

long t[7] ={
  0,0,0,0,0,0,0}; // store time for debug

byte prready = 0;

#if MOVAVG
float avg_x = 0.0f; // vals for moving average calculation, needed for gyro calibration
float avg_y = 0.0f;
float avg_z = 0.0f;

unsigned long avgcnt = 1;
#endif

int error = 0;

void setup() {

#if DBGOUTMODE == 9
  t[0] = micros();

#endif 
  TIMSK2 |= (1<<TOIE2);

  Wire.begin();

  Serial.begin(SERIALBAUD);
  //Serial.begin(19200);

  // initialize device

  accelgyro.initialize();

  accelgyro.setMultiMasterEnabled(false);  
  accelgyro.setI2CBypassEnabled(true);

  accelgyro.setFullScaleGyroRange(1);
  /*
  S_SEL  Full Scale Range  LSB Sensitivity
   0      ± 250 °/s      131 LSB/°/s
   1      ± 500 °/s      65.5 LSB/°/s
   2      ± 1000 °/s      32.8 LSB/°/s
   3      ± 2000 °/s    16.4 LSB/°/s
   
   */
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  /*
AFS_SEL Full Scale Range LSB Sensitivity
   0           ±2g                 16384 LSB/g
   1           ±4g                 8192 LSB/g
   2           ±8g                 4096 LSB/g
   3           ±16g                2048 LSB/g
   
   
   */
  // verify connection
#if INITMSG
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif

#if MAGN_ENABLE
  compass = HMC5883L(); // Construct a new HMC5883 compass.

  error = compass.SetScale(2.5f); // Set the scale of the compass.
  compass.Write(0,0x78); // 8Avg 75Hz
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));


  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));

  // configure Arduino LED for
#endif
  pinMode(DBGPIN, OUTPUT);
  digitalWrite(DBGPIN,LOW);
#if ULTS_ENABLE 
  ultrasonic.setInterval(ULTS_INTERVAL);
#endif

#if IPS_TX_ENABLE
  pinMode(IPS_PIN,OUTPUT);
  digitalWrite(IPS_PIN,HIGH);
#endif

#if DROP_ENABLE
  pinMode(DROP_PIN,OUTPUT);
  digitalWrite(DROP_PIN,LOW);
#endif


  setImuStruct(&ar,(uint16_t)0,(uint16_t)0,(uint16_t)0);
  setImuStruct(&gr,(uint16_t)0,(uint16_t)0,(uint16_t)0);

  setImuStruct(&gyAng,0.0f,0.0f,0.0f);
  setImuStruct(&gf,0.0f,0.0f,0.0f);
  setImuStruct(&af,0.0f,0.0f,0.0f);


  pid_alt.setOutputLimits(PID_MOT_ALT_LIM_MIN,PID_MOT_ALT_LIM_MAX);
  pid_rl.setOutputLimits(PID_MOT_RL_LIM_MIN,PID_MOT_RL_LIM_MAX);
#if DBGOUTMODE == 9
  t[0] =  micros() - t[3];
#endif
}



void loop() {
#if DBGOUTMODE == 9
  t[1] = micros();
#endif

  if(imuready){

#if DBGOUTMODE == 9
    t[2] = micros();
#endif
    //digitalWrite(DBGPIN,dpin^1);
    //  digitalWrite(DBGPIN,1);
    accelgyro.getMotion6(&ar.x, &ar.y, &ar.z, &gr.x, &gr.y, &gr.z);

    //  digitalWrite(DBGPIN,0);

    imuready = 0;
    imudataready = 1;

  }//imuready

  if(imudataready){




    //  digitalWrite(DBGPIN,1);
    //filtering
#if ACCEL_ENABLE == 1    
    af.x = lowpass_accel_x.step(((float)ar.x/(float)(1<<14)));
    af.y = lowpass_accel_y.step(((float)ar.y/(float)(1<<14)));
    af.z = lowpass_accel_z.step(((float)ar.z/(float)(1<<14)));
#endif

    gf.x = gyro_filt_x.step(((float)(gr.x)/GYRO_SCALE_LSB)-GYROOFFSET_X);
    gf.y = gyro_filt_y.step(((float)(gr.y)/GYRO_SCALE_LSB)-GYROOFFSET_Y);
    gf.z = gyro_filt_z.step(((float)(gr.z)/GYRO_SCALE_LSB)-GYROOFFSET_Z);

    //num itegration of gyro vals
#define INT_GYRO_H_2  (1.0f/(ACCEL_GYRO_SAMPLERATE_HZ*2.0f))
#define INT_GYRO_H    (1.0f/(ACCEL_GYRO_SAMPLERATE_HZ))

    //calc moving average if enabled:
#if MOVAVG
    avg_x = avg_x + (( ( (float)(gr.x)/GYRO_SCALE_LSB) - avg_x)/(float)(avgcnt+1));
    avg_y = avg_y + ((((float)(gr.y)/GYRO_SCALE_LSB)- avg_y)/(float)(avgcnt+1));
    avg_z = avg_z + ((((float)(gr.z)/GYRO_SCALE_LSB)- avg_z)/(float)(avgcnt+1));

    avgcnt++;

    if(avgcnt == 0){ //reset on overflow
      avg_x = 0.0f;
      avg_y = 0.0f;
      avg_z = 0.0f;

      avgcnt =1;


    }
#endif

    //integrate
#define GYRO_INT_THRES_X 0.5f
#define GYRO_INT_THRES_Y 0.5f
#define GYRO_INT_THRES_Z 0.5f


#if INTEGRATION == SIMPSON

    if(simpcnt == 1){

      if(abs(gyro_filt_x.getY(0))  >  GYRO_INT_THRES_X){

        gyAng.x += ((INT_GYRO_H)/3)*(gyro_filt_x.getY(2) + (4* gyro_filt_x.getY(1)) + gyro_filt_x.getY(0) );

      }
      if(abs(gyro_filt_y.getY(0))  > GYRO_INT_THRES_Y){
        gyAng.y += ((INT_GYRO_H)/3)*(gyro_filt_y.getY(2) + (4*gyro_filt_y.getY(1)) + gyro_filt_y.getY(0) );
      }
      if(abs(gyro_filt_z.getY(0))  > GYRO_INT_THRES_Z){
        gyAng.z += ((INT_GYRO_H)/3)*(gyro_filt_z.getY(2) + (4*gyro_filt_z.getY(1)) + gyro_filt_z.getY(0) );
      }

      simpcnt = 0;

    } 
    else{

      simpcnt++;
    }



#elif INTEGRATION == TRAPEZ
    if(abs(gyro_filt_x.getY(0))  > GYRO_INT_THRES_X){
      gyAng.x += ((INT_GYRO_H_2)*(gyro_filt_x.getY(1) + gyro_filt_x.getY(0)));
    }
    if(abs(gyro_filt_y.getY(0))  > GYRO_INT_THRES_Y){
      gyAng.y += ((INT_GYRO_H_2)*(gyro_filt_y.getY(1) + gyro_filt_y.getY(0)));
    }
    if(abs(gyro_filt_z.getY(0))  > GYRO_INT_THRES_Z){
      gyAng.z += ((INT_GYRO_H_2)*(gyro_filt_z.getY(1) + gyro_filt_z.getY(0)));
    }



#endif

    //wrap
    if(abs(gyAng.x) > 180.0f){
      gyAng.x += ( ((float)((  ((long)gyAng.x) & 0x80000000L ) >> 31))? 360.0f:-360.0f);
    }

    if(abs(gyAng.y) > 180.0f){
      gyAng.y += ( ((float)((  ((long)gyAng.y) & 0x80000000L ) >> 31))? 360.0f:-360.0f);
    }

    if(abs(gyAng.z) > 180.0f){
      gyAng.z += ( ((float)((  ((long)gyAng.z) & 0x80000000L ) >> 31))? 360.0f:-360.0f);
    }

    //digitalWrite(DBGPIN,0);
    imudataready = 0;

    prready = 1;

#if DBGOUTMODE == 9
    t[2] = micros() -t[2];
#endif

  }//imudata 
  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);


  /*----------------------------------------------------------------------------------------------------------------------------*/

#if MAGN_ENABLE 
  if(magnready){


    // Retrive the raw values from the compass (not scaled).
    //     MagnetometerRaw raw = compass.ReadRawAxis();
    // Retrived the scaled values from the compass (scaled to the configured scale).

    //   digitalWrite(DBGPIN,1);
    mr = compass.ReadScaledAxis();


    magndataready = 1;
    magnready = 0;
    //   digitalWrite(DBGPIN,0);  
  }//magnready

  if(magndataready){

#if DBGOUTMODE == 9
    t[3] = micros();
#endif

    if(abs(mr.XAxis - lowpass_magn_x.getX(0)) < 1000.0f){
      mf.XAxis =  lowpass_magn_x.step(mr.XAxis);
    }
    else{
      mf.XAxis =  lowpass_magn_x.step(mf.XAxis);

    }  


    if(abs(mr.YAxis - lowpass_magn_y.getX(0)) < 1000.0f){
      mf.YAxis =  lowpass_magn_y.step(mr.YAxis);
    }
    else{
      mf.YAxis =  lowpass_magn_y.step(mf.YAxis);

    }  

    if(abs(mr.ZAxis - lowpass_magn_z.getX(0)) < 1000.0f){
      mf.ZAxis =  lowpass_magn_z.step(mr.ZAxis);
    }
    else{
      mf.ZAxis =  lowpass_magn_z.step(mf.ZAxis);

    }  


    // mf.YAxis =  lowpass_magn_y.step(mr.YAxis);


    // mf.ZAxis =  lowpass_magn_z.step(mr.ZAxis);

    //     int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

    // Calculate magn_head_rad when the magnetometer is level, then correct for signs of axis.
    magn_head_rad = atan2(mf.YAxis, mf.XAxis);

    // Once you have your magn_head_rad, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in yo

    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.


    // Muenchen: 2°27' -> 0.04276056
    // declinationangle  
    magn_head_rad += 0.04276;

    // Correct for when signs are reversed.
    if(magn_head_rad < -PI)
      magn_head_rad += 2*PI;

    // Check for wrap due to addition of declination.
    if(magn_head_rad > PI)
      magn_head_rad -= 2*PI;

    // Convert radians to degrees for readability.
    magn_head_deg = rad2deg(magn_head_rad);// * 180/M_PI; 

    //moving avg

    magn_head_rad_avg += (magn_head_rad - magn_head_rad_avg)/((float)(magn_head_rad_avg_cnt + 1));

    magn_head_rad_avg_cnt++;

    if(magn_head_rad_avg_cnt == 50){
      if(get_magn_offset){

        magn_offset_deg = rad2deg(magn_head_rad_avg );

        get_magn_offset = 0;

      }
      magn_head_rad_avg = 0.0f;
      magn_head_rad_avg_cnt = 0;
    }

    magn_head_deg_avg = rad2deg(magn_head_rad_avg);
    magndataready = 0;


#if DBGOUTMODE == 9
    t[3] = micros() -t[3] ;
#endif

  }//magndata

#endif
  /*----------------------------------------------------------------------------------------------------------------------------*/

#if ULTS_ENABLE 
  if(
#if IPS_TX_ENABLE

  ultsready == 1 && ips_ready == 0
#else    

  ultsready    

#endif      
    ){
#if DBGOUTMODE == 9
    t[4] = micros();
#endif
    ultsready=0;
    if(ultsread){
      ults_h_raw = (float) ultrasonic.read();
      ultrasonic.update();
      ults_h = lowpass_ults_h.step(ults_h_raw);

      ultsread = 0;
    }
    else{

      ultrasonic.update();

      ultsread = 1;

    }

#if DBGOUTMODE == 9
    t[4] = micros() - t[4];
#endif

  }//ultsready
#endif

  /*----------------------------------------------------------------------------------------------------------------------------*/
#if IPS_TX_ENABLE
 if(ips_tx_on){
  if(ips_ready){

    if(ips_read ){

      if(((micros() - ipstime) >= 3000L)){
      digitalWrite(IPS_PIN,HIGH);
      ips_read = 0;
      ips_ready = 0;
      }
    }
    else{
      digitalWrite(IPS_PIN,LOW);
      ipstime = micros();
      ips_read = 1;
    }

  }//ipsready
 }// tx on
#endif
  /*----------------------------------------------------------------------------------------------------------------------------*/

  if(serialready){

#if DBGOUTMODE == 9

    t[5] = micros();
#endif

    Serial.print(inString);
    Serial.print("len: ");
    Serial.println(inString.length());

    cmd = inString;
    inString = "";
    Serial.print(cmd);
    //cmd.trim();


    float *pidf = (float*) calloc(3,sizeof(float));

#if SERIALCONV == HEX
    uint8_t * buf = (uint8_t*) calloc(cmd.length()+1,sizeof(uint8_t));
    int16_t test_i16;    
    float test_f;
    //Serial.println((char*)buf);
    Serial.println();


    for(byte i = 0; i < cmd.length();i++){
      buf[i] = (char) cmd.charAt(i);

    }

    for(int i = 0; i<cmd.length();i++){
      Serial.print((char)buf[i]);

    }
    int k = 0;
    if(cmd.length() >6){

      for(struct {byte i ;byte  k;} fl = {5,0} ; fl.i < (((cmd.length() -5-1 )/2)+5) ;fl.i++, fl.k++){

        buf[fl.i] = (char) ( ( ( ((buf[fl.i+fl.k] - 48)>9)?(buf[fl.i+fl.k]-48-7):(buf[fl.i+fl.k]-48))<<4) + (((buf[fl.i+fl.k+1] - 48)>9)?(buf[fl.i+fl.k+1]-48-7):(buf[fl.i+fl.k+1]-48)) );

      }
    }


    for(int i = 0; i<cmd.length();i++){
      Serial.print(buf[i]);

    }

    if((buf[k+1] == 'e' && buf[k+2] == 'n') || (buf[k+1] == 'E' && buf[k+2] == 'N')){

      main_enable = 1;
      setImuStruct(&gyAng,0.0f,0.0f,0.0f);
      //  pid_rl.reset();
      //   pid_alt.reset();

    } 
    else if((buf[k+1] == 'd' && buf[k+2] == 'i' && buf[k+3] == 's' ) || (buf[k+1] == 'D' && buf[k+2] == 'I' && buf[k+3] == 'S')){

      main_enable = 0;
      setImuStruct(&gyAng,0.0f,0.0f,0.0f);
    } 
    else if(buf[k+1] == 'p' || buf[k+1] == 'P' ){


      pidf =  ((float*) (buf+5));




      switch(buf[k+2]){ // hoehe oder richtung

      case 'h':
      case 'H':
        {

          switch(buf[k+3]){ // dyn. regelungs stufe (normal conservativ, agressiv)

          case '1':
          case 'n':
            {
              for(byte i = 0 ; i<3;i++){
                pid_mot_alt[i] = pidf[i];
              }
              pid_alt.reset();
              break;
            }

          case '2':
          case 'c':
            {

              for(byte i = 0 ; i<3;i++){
                pid_mot_alt_conservative[i] = pidf[i];
              }
              pid_alt.reset();
              break;
            }

          case '3':
          case 'a':
            {

              for(byte i = 0 ; i<3;i++){
                pid_mot_alt_aggro[i] = pidf[i];
              }
              pid_alt.reset();
              break;
            }
          default:
            break;
          }
          break;
        }
      case 'r':
      case 'R':
        {
          switch(buf[k+3]){

          case '1':
          case 'n':
            {
              for(byte i = 0 ; i<3;i++){
                pid_mot_rl[i] = pidf[i];
              }
              pid_rl.reset();
              break;
            }

          case '2':
          case 'c':
            {


              break;
            }

          case '3':
          case 'a':
            {


              break;
            }
          default:
            break;  
          }
          break;

        }

      default:
        break;
      }//sw
    }
    else if(buf[k+1] == 'w' && buf[k+2] == 'i'){


      test_f = *((float*) (buf+5));

      reg_set_rl_ang = constrain(test_f,-90.0f,90.0f);


    }

    else if(buf[k+1] == 's' && buf[k+2] == 'c'){

      //    buf_3[0] = buf[k+5];
      //      buf_3[1] = buf[k+7];

      test_i16 = *((int16_t*) (buf+5));


      reg_set_speed = constrain(test_i16,-255,255);
      //      Serial.print(schub);
      //   Serial.print(" ");

    }

    else if(buf[k+1] == 'w' && buf[k+2] == 'h'){

      //      buf_3[0] = buf[k+5];
      //      buf_3[1] = buf[k+7];
      test_f = *((float*) (buf+5));
      //   test_i16 = *((int16_t*) (buf+5));

      //  if(test_i16 > 30 && test_i16 < 300){

      reg_set_h = constrain(test_f,30.0f,300.0f);
      // Serial.print(wunschhoehe);
      //  Serial.print(" ");
      // }

    }
    else if(buf[k+1] == 'r' || buf[k+1] == 'R'){
      switch(buf[k+2]){
      case 'g':
      case 'G':
        {

          switch(buf[k+3]){

          case 'X':
          case 'x':
            {
              setImuStruct(&gyAng,0.0f,gyAng.y,gyAng.z);

              break;
            }

          case 'Y':
          case 'y':
            {
              setImuStruct(&gyAng,gyAng.x,0.0f,gyAng.z);

              break;
            }

          case 'Z':
          case 'z':
            {
              setImuStruct(&gyAng,gyAng.x,gyAng.y,0.0f);

              break;
            }
          default:
            {


              setImuStruct(&gyAng,0.0f,0.0f,0.0f);

              break;
            }
          }

          break;
        }
      case 'S':
      case 's':
        {
          pid_alt.reset();
          pid_rl.reset();
          setImuStruct(&gyAng,0.0f,0.0f,0.0f);

          break;
        }

      default:
        break;
      }
    } 


#elif SERIALCONV == ATOF
    int16_t *ibuf = (int16_t*) calloc(2,sizeof(int16_t));

    cmd.trim();
    if((cmd.charAt(0) == 'b') || (cmd.charAt(0) == 'B' ) ){
#define CHRBUFSIZE 24
      char* xbuf = (char*) calloc( CHRBUFSIZE, sizeof(char));
      char* ybuf = (char*) calloc( CHRBUFSIZE, sizeof(char));
      char* zbuf = (char*) calloc( CHRBUFSIZE, sizeof(char));
      int xstart = 0;
      int ystart = 0;
      int zstart = 0;

      if( cmd.substring(1).equals("en") || cmd.substring(1).equals("EN")  ){
        // ben

        main_enable = 1;
#if MAGN_ENABLE
        get_magn_offset = 1;
        magn_head_rad_avg_cnt = 0;
#endif
        Serial.println("###");
        Serial.println(!-5);
        setImuStruct(&gyAng,0.0f,0.0f,0.0f);

      }
      else if(cmd.substring(1).equals("dis") || cmd.substring(1).equals("DIS")){

        main_enable = 0;
        setImuStruct(&gyAng,0.0f,0.0f,0.0f);


      }
      else if(cmd.substring(1).equals("tog") || cmd.substring(1).equals("TOG")){
        main_enable ^= 1;
        setImuStruct(&gyAng,0.0f,0.0f,0.0f);

      }
#if DROP_ENABLE   
      else if(cmd.substring(1).equals("drop") || cmd.substring(1).equals("DROP")){

        digitalWrite(DROP_PIN,HIGH); 

      }
#endif       
      else if(cmd.substring(1).equals("che") || cmd.substring(1).equals("CHE")){
        mot_alt_cont_auto = 1;
      }
      else if(cmd.substring(1).equals("chd") || cmd.substring(1).equals("CHD")){
        mot_alt_cont_auto = 0;
      }
      else if(cmd.substring(1).equals("cre") || cmd.substring(1).equals("CRE")){
        mot_rl_cont_auto = 1;
      }
      else if(cmd.substring(1).equals("crd") || cmd.substring(1).equals("CRD")){
        mot_rl_cont_auto = 0;
      }
      else if(cmd.substring(1).equals("cmg") || cmd.substring(1).equals("CMG")){
        mot_rl_cont_gyro = 1;
      }
      else if(cmd.substring(1).equals("cmm") || cmd.substring(1).equals("CMM")){
        mot_rl_cont_gyro = 0;
      }
      else
      {

        xstart = cmd.indexOf(" ");
        ystart = cmd.indexOf(" ", xstart+1);
        zstart = cmd.indexOf(" ", ystart+1);


        cmd.substring(xstart + 1,ystart ).toCharArray(xbuf,CHRBUFSIZE);
        cmd.substring(ystart +1,zstart ).toCharArray(ybuf,CHRBUFSIZE);
        cmd.substring(zstart+1).toCharArray(zbuf,CHRBUFSIZE);

        switch((char)cmd.charAt(1)){
        case 'P':
        case 'p':
          {


            pidf[0] = atof(xbuf);
            pidf[1] = atof(ybuf);
            pidf[2] = atof(zbuf);

            switch((char)cmd.charAt(2)){
            case 'r':
            case 'R':
              {

                switch((char)cmd.charAt(3)){


                case '1':
                case 'n':
                case 'N':
                case '2':
                case 'c':
                case 'C':
                case '3':
                case 'a':
                case 'A':
                  {

                    for(byte i = 0 ; i<3;i++){
                      pid_mot_rl[i] = pidf[i];
                    }
                    pid_rl.reset();
                    break;

                  }
                default:
                  break;

                  break;
                }
                break;
              }
            case 'H':
            case 'h':
              {
                for(byte i = 0 ; i<3;i++){
                  switch((char)cmd.charAt(3)){

                  case '1':
                  case 'n':
                  case 'N':
                    {

                      pid_mot_alt[i] = pidf[i];
                      break;
                    }
                  case '2':
                  case 'c':
                  case 'C':
                    {

                      pid_mot_alt_conservative[i] = pidf[i];
                      break;
                    }
                  case '3':
                  case 'a':
                  case 'A':
                    {

                      pid_mot_alt[i] = pidf[i];
                      break;
                    }

                  default:
                    break;


                  }//sw

                }//for

                pid_alt.reset();
                break;
              }


            default:
              break;

            }
            break;
          }//case p
        case 'w':
        case 'W':
          {

            pidf[0] = atof(xbuf);

            switch((char)cmd.charAt(2)){

            case 'i':
            case 'I':
              {
                reg_set_rl_ang = constrain(pidf[0],-90.0f,90.0f);          
                setImuStruct(&gyAng,gyAng.x,gyAng.y,0.0f);
                break;

              }
            case 'h':
            case 'H':
              {

                reg_set_h = constrain(pidf[0],30.0f,300.0f);
                break;
              }
            default:
              break;

            }
            break;
          }// case w
        case 's':
        case 'S':
          {

            switch((char)cmd.charAt(2)){

            case 'c':
            case 'C':
              {
                ibuf[0] = atoi(xbuf);    

                reg_set_speed = constrain(ibuf[0],-255,255);
                break;
              }

            default:
              break;
            }
            break;
          }//case s
        case 'r':
        case 'R':
          {
            switch((char)cmd.charAt(2)){
            case 'G':
            case 'g':
              {
                switch((char)cmd.charAt(3)){
                case 'x':
                case 'X':
                  {
                    setImuStruct(&gyAng,0.0f,gyAng.y,gyAng.z);

                    break;
                  }
                case 'y':
                case 'Y':
                  {
                    setImuStruct(&gyAng,gyAng.x,0.0f,gyAng.z);

                    break;
                  }
                case 'z':
                case 'Z':
                  {

                    setImuStruct(&gyAng,gyAng.x,gyAng.y,0.0f);

                    break;
                  }

                default:
                  {
                    break;
                    setImuStruct(&gyAng,0.0f,0.0f,0.0f);
                  }
                }// sw c3
              }
            case 'S':
            case 's':
              {
                pid_alt.reset();
                pid_rl.reset();
                setImuStruct(&gyAng,0.0f,0.0f,0.0f);
                break;
              }
#if MAGN_ENABLE 
            case 'm':
            case 'M':
              {
                get_magn_offset = 1;
                magn_head_rad_avg_cnt = 0;  

                break;
              }
#endif
            default:
              break;
            }
            break;
          }


        default:
          break;


        }//switch char 1


      }

      free(xbuf);
      free(ybuf);
      free(zbuf);  

    }// if startswith b

#endif 

      cmd = "";

    free(pidf);
    free(ibuf);
#if SERIALCONV == HEX
    free(buf);
#endif


#if DBGOUTMODE == 8
      Serial.print("phn:\t");
    Serial.print(pid_mot_alt[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt[1],6);
    Serial.print(" ");
    Serial.println(pid_mot_alt[2],6);

    Serial.print("phn:\t");
    Serial.print(pid_mot_alt_conservative[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt_conservative[1],6);
    Serial.print(" ");
    Serial.println(pid_mot_alt_conservative[2],6);

    Serial.print("pha:\t");
    Serial.print(pid_mot_alt_aggro[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt_aggro[1],6);
    Serial.print(" ");
    Serial.println(pid_mot_alt_aggro[2],6);

    Serial.print("prn:\t");
    Serial.print(pid_mot_rl[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_rl[1],6);
    Serial.print(" ");
    Serial.println(pid_mot_rl[2],6);
    /*
    Serial.print("\tpr2:\t");
     Serial.print(pid_mot_rl_conservative[0],6);
     Serial.print(" ");
     Serial.print(pid_mot_rl_conservative[1],6);
     Serial.print(" ");
     Serial.print(pid_mot_rl_conservative[2],6);
     
     Serial.print("\tpr3:\t");
     Serial.print(pid_mot_rl_aggro[0],6);
     Serial.print(" ");
     Serial.print(pid_mot_rl_aggro[1],6);
     Serial.print(" ");
     Serial.print(pid_mot_rl_aggro[2],6);
     */
    Serial.print("wi:\t");
    Serial.println(reg_set_rl_ang,6);

    Serial.print("wh:\t");
    Serial.println(reg_set_h,6);

    Serial.print("sc:\t");
    Serial.println(reg_set_speed);
#if MAGN_ENABLE
    Serial.print("mg_offs:\t");
    Serial.println(magn_offset_deg);
#endif
    Serial.print("Main:\t");
    Serial.println(main_enable);

    Serial.print("dirC:\t");
    Serial.println(mot_rl_cont_auto);

    Serial.print("altC:\t");
    Serial.println(mot_alt_cont_auto);

    Serial.print("dirGy:\t");
    Serial.println(mot_rl_cont_gyro);

    Serial.print("RAM:\t");
    Serial.println(freeRam());


    Serial.println();
    Serial.flush();
#endif


#if DBGOUTMODE == 9

    t[5] = micros() -t[5];
#endif

    serialready = 0;
    //parchng = 1;
  }//serialready

  /*----------------------------------------------------------------------------------------------------------------------------*/

  if(main_enable){
    if(motcont){

#if DBGOUTMODE == 9

      t[6] = micros();

#endif
      //reg_set_rl_ang = 0.0f;

      if(mot_rl_cont_auto){
        if(mot_rl_cont_gyro){
#if MOT_CONT_METH == MC_GYRO_OMEGA

          dbg1f = pid_rl.step(reg_set_rl_ang,gf.z)
            setMotDirection(deg2rad(dbg1f),reg_set_speed);

#elif MOT_CONT_METH == MC_GYRO_PHI        




          dbg1f = pid_rl.step(reg_set_rl_ang,gyAng.z);
          setMotDirection(deg2rad(dbg1f),reg_set_speed);
#endif
        }
        else{
#if MAGN_ENABLE
          if(!get_magn_offset){

            float magn_pid_in = magn_head_deg-magn_offset_deg;

            // calculate the derivative of the angle 
            float magn_delta =  magn_head_deg -rad2deg(atan2(lowpass_magn_y.getY(1), lowpass_magn_x.getY(1))) ;

            //look if the heading angle wrapped, if yes undo it
            if( magn_delta  > 270.0f ){
              magn_pid_in -= 360.0f;

            }
            else if (magn_delta  < -270.0f){

              magn_pid_in += 360.0f;

            }

            // wrap the whole thing if it exceeds the limits            
            if(magn_pid_in < -180.0f){
              magn_pid_in += 360.0f;

            }
            else if (magn_pid_in > 180.0f){

              magn_pid_in -= 360.0f;

            }

            // negate for the conroller
            magn_pid_in *= (-1.0f);                  



            //  magn_pid_in = constrain(magn_pid_in,-90.0f,90.0f);

            dbg1f = pid_rl.step(reg_set_rl_ang,magn_pid_in);
            setMotDirection(deg2rad(dbg1f),reg_set_speed);

          }
#endif          
        }



      }
      else{
        //control disabled
        setMotDirection(reg_set_rl_ang,reg_set_speed);


      } 

      if(mot_alt_cont_auto){

        float e = abs(reg_set_h - ults_h);


        if(e > PID_MOT_AGGRO_LIM){

          pid_alt.setCoeffs(pid_mot_alt_aggro);
          //  pid_alt.setIerr(0.0f);
        }
        else if(e> PID_MOT_NORMAL_LIM){

          pid_alt.setCoeffs(pid_mot_alt);
          //          pid_alt.setIerr(0.0f);
        }
        else {

          pid_alt.setCoeffs(pid_mot_alt_conservative);
          //            pid_alt.setIerr(0.0f);
        }

        dbg2f = pid_alt.step(reg_set_h,ults_h);
        setMotAlt((int)dbg2f);
      }
      else{

        setMotAlt(constrain((int)reg_set_h,0,255));


      }
      // setImuStruct(&gyAngSimp,gyAngSimp.x,gyAngSimp.y,0.0f); // reset gyro

      //    setMotAlt(100);


#if DBGOUTMODE == 9

      t[6] = micros() -t[6];
#endif

      motcont = 0;
    }//motcont
  }//main enable
  else{

    setMotAlt(0);
    setMotDirection(0.0f,0);

  }// main enable

  /*----------------------------------------------------------------------------------------------------------------------------*/



  if(prready){
#if DBGOUTMODE == 9
    t[1] = micros() -t[1];
#endif
    // digitalWrite(DBGPIN,1);
#if DBGOUTMODE == 1
    Serial.print("a/g/m: ");
    /*
    Serial.print( af.x ,5 ); 
     Serial.print(" ");
     
     Serial.print( af.y ,5 ); 
     Serial.print(" ");
     
     Serial.print( af.z ,5 ); 
     Serial.print(" G: ");
     
     Serial.print( gf.x ,5 ); 
     Serial.print(" ");
     
     Serial.print( gf.y ,5 ); 
     Serial.print(" ");
     */
    Serial.print( gf.z ,5 ); 
    Serial.print(" GI: ");
    /*
    Serial.print(gyAng.x,5);
     Serial.print(" ");
     
     Serial.print(gyAng.y,5);
     Serial.print(" ");
     */
    Serial.print(gyAng.z,5);
    Serial.print(" M: ");
    /*
    Serial.print( mf.XAxis ,5 ); 
     Serial.print(" ");
     
     Serial.print( mf.YAxis ,5 ); 
     Serial.print(" ");
     
     Serial.print( mf.ZAxis ,5 ); 
     Serial.print(" ");
     */
    Serial.print(magn_head_rad,5);
    Serial.print(" ");

    Serial.print(magn_head_deg,5);
    Serial.print(" ");
    /*
    Serial.print("Read:\t");
     Serial.print(t1);
     Serial.print("\t");
     
     Serial.print("Proc:\t");
     Serial.print(t2);
     Serial.print("\t");
     
     t3 = micros() - t3;
     Serial.print("Loop:\t");
     Serial.print(t3);
     
     
     */
    Serial.println();

    Serial.flush();
#elif DBGOUTMODE == 2
    Serial.print("a/g/m:\t ");

    Serial.print( af.x ,5 ); 
    Serial.print("\t");

    Serial.print( af.y ,5 ); 
    Serial.print("\t");

    Serial.print( af.z ,5 ); 
    Serial.print("\tR:\t");

    Serial.print( ((float)ar.x)/((float)(1<<14)) ,5 ); 
    Serial.print("\t");

    Serial.print( ((float)ar.y)/((float)(1<<14)) ,5 ); 
    Serial.print("\t");

    Serial.print( ((float)ar.z)/((float)(1<<14)) ,5 ); 
    Serial.print("\tG:\t");


    Serial.print( gf.x ,5 ); 
    Serial.print("\t");

    Serial.print( gf.y ,5 ); 
    Serial.print("\t");

    Serial.print( gf.z ,5 ); 
    Serial.print("\tGR:\t");

    Serial.print( ((float)(gr.x)/65.5f)-1,5);
    Serial.print("\t");

    Serial.print( ((float)(gr.y)/65.5f)-1,5);
    Serial.print("\t");

    Serial.print( ((float)(gr.z)/65.5f)-1,5);
    Serial.print("\tM\t");


    Serial.print( mf.XAxis ,5 ); 
    Serial.print("\t");

    Serial.print( mf.YAxis ,5 ); 
    Serial.print("\t");

    Serial.print( mf.ZAxis ,5 ); 
    Serial.print("\tR:\t");

    Serial.print( mr.XAxis ,5 ); 
    Serial.print("\t");

    Serial.print( mr.YAxis ,5 ); 
    Serial.print("\t");

    Serial.print( mr.ZAxis ,5 ); 
    Serial.print("\t");


    Serial.println();

    Serial.flush();
#elif DBGOUTMODE == 3

    Serial.print("\tAvg:\t");

    Serial.print(avg_x,5);
    Serial.print("\t");

    Serial.print(avg_y,5);
    Serial.print("\t");

    Serial.print(avg_z,5);
    Serial.print("\t");

    Serial.print(avgcnt);
    Serial.print("\t");

    Serial.println();

    Serial.flush();
#elif DBGOUTMODE == 4 //quaternion test    

    Serial.print("Quaternion:\t");

    Serial.print(ahrs.q0,5);
    Serial.print("\t");

    Serial.print(ahrs.q1,5);
    Serial.print("\t");

    Serial.print(ahrs.q2,5);
    Serial.print("\t");

    Serial.print(ahrs.q3,5);
    Serial.print("\t");

    Serial.println();

    Serial.flush(); 
#elif DBGOUTMODE == 5

    Serial.print("Setup:\t");
    Serial.print(t4);
    Serial.print("\t");


    Serial.print("Read:\t");
    Serial.print(t1);
    Serial.print("\t");

    Serial.print("Proc:\t");
    Serial.print(t2);
    Serial.print("\t");

    Serial.print("Magn:\t");
    Serial.print(t5);
    Serial.print("\t");

    Serial.print("Loop:\t");
    t3 = micros() - t3;
    Serial.print(t3);

    Serial.print(freeRam());
    Serial.print("\t");
    Serial.println();

    Serial.flush();

#elif DBGOUTMODE == 6

    Serial.print("G(z):\t");
    Serial.print(gyAngSimp.z,5);
    Serial.print("\t");

    Serial.print("PID:\t");
    Serial.print(dbg1f,5);
    Serial.print("\t");


    Serial.print("spl:\t");
    Serial.print(dbg2);
    Serial.print("\t");

    Serial.print("spr:\t");
    Serial.print(dbg1);
    Serial.print("\t");



    Serial.print("sum:\t");
    Serial.print(dbg2 + dbg1);
    Serial.print("\t");

    for(byte i = 0; i< 4;i++){
      Serial.print("e[");
      Serial.print(i);
      Serial.print("]:\t");
      Serial.print(pid_rl.getE(i));
      Serial.print("\t");
    }  

    Serial.print(freeRam());
    Serial.print("\t");

    Serial.println();

    Serial.flush();
#elif DBGOUTMODE == 7

    Serial.print("h raw:\t");
    Serial.print(ults_h_raw);
    Serial.print("\t");

    Serial.print("h:\t");
    Serial.print(ults_h);
    Serial.print("\t");

    Serial.print("pid:\t");
    Serial.print(dbg2f,5);
    Serial.print("\t");

    Serial.print(freeRam());
    Serial.print("\t");
    Serial.println();
    Serial.flush();

#elif DBGOUTMODE == 9

    Serial.print("Setup:\t");
    Serial.print(t[0]);
    Serial.print("\t");


    Serial.print("Loop:\t");
    Serial.print(t[1]);
    Serial.print("\t");


    Serial.print("IMU:\t");
    Serial.print(t[2]);
    Serial.print("\t");

#if MAGN_ENABLE

    Serial.print("Magn:\t");
    Serial.print(t[3]);
    Serial.print("\t");
#endif

    Serial.print("Ults:\t");
    Serial.print(t[4]);
    Serial.print("\t");

    Serial.print("Serial:\t");
    Serial.print(t[5]);
    Serial.print("\t");

    Serial.print("PID:\t");
    Serial.print(t[6]);
    Serial.print("\t");

    Serial.print("Ram:\t");
    Serial.print(freeRam());
    Serial.print("\t");

    Serial.println();
    Serial.flush();
#endif

    //digitalWrite(DBGPIN,0);
    prready = 0;
  }
}//loop

void setImuStruct( void *ptr, float xval,float yval,float zval){
  IMUFilt * imu = (IMUFilt*) ptr;
  imu->x = xval;
  imu->y = yval;
  imu->z = zval;
}//setIMuStruct

void setImuStruct( void *ptr, uint16_t xval,uint16_t yval,uint16_t zval){
  IMURaw * imu = (IMURaw*) ptr;
  imu->x = xval;
  imu->y = yval;
  imu->z = zval;
}//setImuStruct


void setMotSpeed(int sp,uint8_t in1,uint8_t in2 ,uint8_t pwmpin){ // sp in [-255, 255]
  if( sp == 0){

    //TODO: fasten up code by setting the pins directly in the respective port registers

    //freerun
#if MOT_HALTMODE == MOT_FREERUN
    digitalWrite(in1,0);
    digitalWrite(in2,0);


#elif MOT_HALTMODE == MOT_SHORT    
    //short the motor to stop it near instantly 

    digitalWrite(in1,1);
    digitalWrite(in2,1);
#endif
  }
  else if(sp >0){
    digitalWrite(in1,1);
    digitalWrite(in2,0);
  }
  else{
    digitalWrite(in1,0);
    digitalWrite(in2,1);
  }
  sp = abs(sp);

  analogWrite(pwmpin,sp);
}// setMotorSpeed



void setMotDirection(float angl,int sp0){
#define MOT_R_THRES 7
#define MOT_L_THRES 7
  sp0 = constrain(sp0,-128,128);
  angl = constrain(angl,-90,90);
  int spr =  ((int)(sp0*sin(angl) +sp0)  );
  int spl = ((int)(sp0*-sin(angl)  +sp0)  );

  if((sp0 == 0) && (angl != 0.0f)){
    spr = (int)(128*sin(angl)  );
    spl =  (int)(128*(-sin(angl)) );

  }

  spr = constrain(spr,-255,255);
  spl = constrain(spl,-255,255);

  if(abs(spr) <= MOT_R_THRES){
    spr = 0;
  }

  if(abs(spl) <= MOT_L_THRES){
    spl = 0;
  }

  dbg1 = spr;
  dbg2 = spl;

  setMotSpeed(spr,MOT_R_IN1_PIN,MOT_R_IN2_PIN,MOT_R_PWM_PIN);
  setMotSpeed(spl,MOT_L_IN1_PIN,MOT_L_IN2_PIN,MOT_L_PWM_PIN);

}


void setMotAlt(int sp0){
#define MOT_ALT_THRES 5
#define MOT_ALT_OFFSET 40 

    sp0 = constrain(sp0,-255,255);
  //    sp0 = constrain(sp0- MOT_ALT_OFFSET,-255,255);

  if(abs(sp0) <= MOT_ALT_THRES){
    sp0=0;
  }

  setMotSpeed(sp0,MOT_ALT_IN1_PIN,MOT_ALT_IN2_PIN,MOT_ALT_PWM_PIN);


}

float rad2deg( float rad){

  return (rad * 57.295779f);

}


float deg2rad( float deg){

  return (deg * 0.0174532f);

}


int sgn(float f){
  if(f > 0 ){

    return 1;
  }
  else if( f<0){

    return -1;
  }
  else {

    return 0;
  }


} //sgn

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}//freeram


void serialEvent(){

  while(Serial.available() > 0){

    char inChar = (char) Serial.read();

    inString += inChar;

    if(inChar ==  '\n'){
      serialready = 1;
    }


  }

}// SerialEvent



/*volatile int si = 0;
 
 int readSerial(volatile byte* volatile buf){
 
 for(byte i; i < SERIAL_BUFSIZE;i++){
 buf[i] = 0;
 
 }
 
 
 while(Serial.available() > 0){
 
 //char inChar = (char) Serial.read();
 buf[si] = (char)Serial.read();
 Serial.print((char)buf[si]);
 
 //inString += inChar;
 
 if(buf[si] ==  '\n'){
 serialready = 1;
 break;
 }
 si = (si +1) % SERIAL_BUFSIZE;
 }
 
 int k;
 
 for(k = 0; k<=SERIAL_BUFSIZE;k++){
 
 if(buf[k] == 'b'){
 
 break;
 
 }
 }
 
 
 //result[1] = strready
 return k;
 }
 */


ISR(TIMER2_OVF_vect){
  // Timer runs at 250khz

  if( ovfcnt == 9 ){ // 250khz/2*samplerate*256

#if DBG_ISR
     blinkstate = ~blinkstate;
       digitalWrite(DBGPIN,blinkstate);
#endif
    // digitalWrite(LED_PIN, blinkState); // 250khz/samplerate
    //accelgyro.getMotion6(&ar.ax, &ar.ay, &ar.az, &gr.gx, &gr.gy, &gr.gz);
#if MAGN_ENABLE == 1
    magnready=1;
#endif
    imuready = 1;
    motcont = 1;
    ovfcnt = 0;

    if(ovfcnt2 == 7){
#if IPS_TX_ENABLE
      ips_ready =1;
#endif
      ovfcnt2=0;
    }
    else{
      ovfcnt2++;
    }

    if(ovfcnt3 == 1){
#if ULTS_ENABLE 
      ultsready = 1;
#endif


      ovfcnt3 = 0;
    }
    else{

      ovfcnt3++;

    }


  }
  else{

    ovfcnt++;
  }



} //ISR

















































































