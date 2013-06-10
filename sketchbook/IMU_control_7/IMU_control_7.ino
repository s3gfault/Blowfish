
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


float dbg1f=0.0f,dbg2f=0.0f,dbg3f=0.0f;
int dbg1 = 0,dbg2 = 0,dbg3 = 0;

#define TRAPEZ 1
#define SIMPSON 2
#define INTEGRATION SIMPSON




#define INITMSG 0

#define DBGPIN 13

MPU6050 accelgyro(0x69);

HMC5883L compass;

SRF02 ultrasonic(0x70,SRF02_CENTIMETERS);
//prototypes
//void setStruct( IMUFilt *imu, float xval,float yval,float zval);

String inString;
String cmd;


volatile byte imuready = 0;
volatile byte magnready = 0;
volatile byte motcont = 0;
volatile byte ultsready = 0;
volatile byte serialready = 0;

volatile uint16_t ovfcnt = 0;
volatile uint8_t ovfcnt2 = 0;
volatile uint8_t ovfcnt3 = 0;

#define SERIAL_BUFSIZE 48

byte serialbuf[SERIAL_BUFSIZE];

volatile byte serialstrbegin = 0;

byte imudataready = 0;
byte magndataready = 0;

byte ultsread = 0;

byte main_enable = 0;

#define LPAORDER 1
#define LPBORDER 1
#define LPGAIN 0.02008f

// define the filter constants for the sensors
//general:
#define ACCEL_GYRO_ENABLE 1
#define ACCEL_GYRO_SAMPLERATE_HZ 50
//ACCELEROMETER


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


//GYROSCOPE

#define HP_GYRO_ORDER 2
#define HP_GYRO_CUTOFF_HZ 12.5f
#define HP_GYRO_RC (1.0f/HP_GYRO_CUTOFF_HZ)

#define GYRO_SCALE_LSB 65.5f

#define GYROOFFSET_X 1.3624f // found out experimentally, trough a moving average while gyro is static
#define GYROOFFSET_Y -1.5140f
#define GYROOFFSET_Z -1.3335f

float hp_gyro_alpha = ((HP_GYRO_RC)/(HP_GYRO_RC+(1.0f/ACCEL_GYRO_SAMPLERATE_HZ)));

float hp_gyro_b[HP_GYRO_ORDER+1] = {
  hp_gyro_alpha ,0.0f,0.0f};
float hp_gyro_a[HP_GYRO_ORDER+1] = {
  1.0f,(1.0f-hp_gyro_alpha),0.0f};

IIR highpass_gyro_x(hp_gyro_a,hp_gyro_b,HP_GYRO_ORDER,HP_GYRO_ORDER);
IIR highpass_gyro_y(hp_gyro_a,hp_gyro_b,HP_GYRO_ORDER,HP_GYRO_ORDER);
IIR highpass_gyro_z(hp_gyro_a,hp_gyro_b,HP_GYRO_ORDER,HP_GYRO_ORDER);

#if INTEGRATION == SIMPSON
byte simpcnt = 0;

#else


#endif


//magnetometer
#define MAGN_ENABLE 1


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

float heading = 0.0f;
float headingdegrees = 0.0f;

//ultrasonic


#define ULTS_ENABLE 1

#define ULTS_SAMPLERATE_HZ 12.5f
#define ULTS_INTERVAL 80
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

//controller
#define PID_KP 1.0f
#define PID_KI 1.0f
#define PID_KD 1.0f
#define PID_GAIN 1.0f
#define PID_H (1.0f/ACCEL_GYRO_SAMPLERATE_HZ)


#define PID_MOT_RL_KP 1.25f
#define PID_MOT_RL_KI 0.13f//0.1f
#define PID_MOT_RL_KD 0.12f

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


#define PID_MOT_ALT_KP 0.7f
#define PID_MOT_ALT_KI 0.0f//0.015f
#define PID_MOT_ALT_KD 0.0f//0.2f

#define PID_MOT_ALT_AGGRO_KP 1.2f
#define PID_MOT_ALT_AGGRO_KI 0.0f//0.015f
#define PID_MOT_ALT_AGGRO_KD 0.0f//0.2f

#define PID_MOT_ALT_CONSERVATIVE_KP 0.5f
#define PID_MOT_ALT_CONSERVATIVE_KI 0.0f //0.015f
#define PID_MOT_ALT_CONSERVATIVE_KD 0.0f//0.2f

#define PID_MOT_AGGRO_LIM 15.0f
#define PID_MOT_NORMAL_LIM 3.0f
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

#define MOT_R_IN1_PIN 7//7
#define MOT_R_IN2_PIN 8//8

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
IMUFilt gf,af,gyAng,gyAngSimp;



MagnetometerScaled mr,mf; 


volatile byte dpin = 0;

long acceltime = 0;

long t1 = 0,t2 = 0,t3 = 0,t4 = 0,t5=0;

byte prready = 0;

float avg_x = 0.0f;
float avg_y = 0.0f;
float avg_z = 0.0f;
unsigned long avgcnt = 1;


int error = 0;



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  //lowpass.setGain(LPGAIN);
  // lowpass2.setGain(LPGAIN);
  t4 = micros();
  TIMSK2 |= (1<<TOIE2);

  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  //Serial.begin(115200);
  Serial.begin(9600);

  // initialize device
#if INITMSG
  Serial.println("Initializing I2C devices...");
#endif
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
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("Constructing new HMC5883L");
#endif
  compass = HMC5883L(); // Construct a new HMC5883 compass.
#if INITMSG
  Serial.println("Setting scale to +/- 1.3 Ga");
#endif

  error = compass.SetScale(2.5f); // Set the scale of the compass.
  compass.Write(0,0x78); // 8Avg 75Hz
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));

#if INITMSG
  Serial.println("Setting measurement mode to continous.");
#endif
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  // configure Arduino LED for
  pinMode(DBGPIN, OUTPUT);

  ultrasonic.setInterval(ULTS_INTERVAL);

  setImuStruct(&ar,(uint16_t)0,(uint16_t)0,(uint16_t)0);
  setImuStruct(&gr,(uint16_t)0,(uint16_t)0,(uint16_t)0);

  setImuStruct(&gyAng,0.0f,0.0f,0.0f);
  setImuStruct(&gf,0.0f,0.0f,0.0f);
  setImuStruct(&af,0.0f,0.0f,0.0f);

  pid_alt.setOutputLimits(PID_MOT_ALT_LIM_MIN,PID_MOT_ALT_LIM_MAX);
  pid_rl.setOutputLimits(PID_MOT_RL_LIM_MIN,PID_MOT_RL_LIM_MAX);

  t4 =  micros() - t4;
}



void loop() {
  t3 = micros();

  if(imuready){
    t1 = micros();   
    //digitalWrite(DBGPIN,dpin^1);
    //  digitalWrite(DBGPIN,1);
    accelgyro.getMotion6(&ar.x, &ar.y, &ar.z, &gr.x, &gr.y, &gr.z);

    //  digitalWrite(DBGPIN,0);
    t1= (micros() - t1);
    imuready = 0;
    imudataready = 1;

  }//imuready

  if(imudataready){



    t2 = micros();
    //  digitalWrite(DBGPIN,1);
    //filtering
    af.x = lowpass_accel_x.step(((float)ar.x/(float)(1<<14)));
    af.y = lowpass_accel_y.step(((float)ar.y/(float)(1<<14)));
    af.z = lowpass_accel_z.step(((float)ar.z/(float)(1<<14)));

    gf.x = highpass_gyro_x.step(((float)(gr.x)/GYRO_SCALE_LSB)-GYROOFFSET_X);
    gf.y = highpass_gyro_y.step(((float)(gr.y)/GYRO_SCALE_LSB)-GYROOFFSET_Y);
    gf.z = highpass_gyro_z.step(((float)(gr.z)/GYRO_SCALE_LSB)-GYROOFFSET_Z);

    //num itegration of gyro vals
#define INT_GYRO_H_2  (1.0f/(ACCEL_GYRO_SAMPLERATE_HZ*2.0f))
#define INT_GYRO_H    (1.0f/(ACCEL_GYRO_SAMPLERATE_HZ))

    //calc moving average:

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


    //integrate
#define GYRO_INT_THRES_X 0.5f
#define GYRO_INT_THRES_Y 0.5f
#define GYRO_INT_THRES_Z 0.5f


#if INTEGRATION == SIMPSON

    if(simpcnt == 1){

      if(abs(highpass_gyro_x.getY(0))  >  GYRO_INT_THRES_X){

        gyAngSimp.x += ((INT_GYRO_H)/3)*(highpass_gyro_x.getY(2) + (4* highpass_gyro_x.getY(1)) + highpass_gyro_x.getY(0) );

      }
      if(abs(highpass_gyro_y.getY(0))  > GYRO_INT_THRES_Y){
        gyAngSimp.y += ((INT_GYRO_H)/3)*(highpass_gyro_y.getY(2) + (4*highpass_gyro_y.getY(1)) + highpass_gyro_y.getY(0) );
      }
      if(abs(highpass_gyro_z.getY(0))  > GYRO_INT_THRES_Z){
        gyAngSimp.z += ((INT_GYRO_H)/3)*(highpass_gyro_z.getY(2) + (4*highpass_gyro_z.getY(1)) + highpass_gyro_z.getY(0) );
      }

      simpcnt = 0;

    } 
    else{

      simpcnt++;
    }


#endif
    if(abs(highpass_gyro_x.getY(0))  > GYRO_INT_THRES_X){
      gyAng.x += ((INT_GYRO_H_2)*(highpass_gyro_x.getY(1) + highpass_gyro_x.getY(0)));
    }
    if(abs(highpass_gyro_y.getY(0))  > GYRO_INT_THRES_Y){
      gyAng.y += ((INT_GYRO_H_2)*(highpass_gyro_y.getY(1) + highpass_gyro_y.getY(0)));
    }
    if(abs(highpass_gyro_z.getY(0))  > GYRO_INT_THRES_Z){
      gyAng.z += ((INT_GYRO_H_2)*(highpass_gyro_z.getY(1) + highpass_gyro_z.getY(0)));
    }


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


    if(abs(gyAngSimp.x) > 180.0f){
      gyAngSimp.x += ( ((float)((  ((long)gyAngSimp.x) & 0x80000000L ) >> 31))? 360.0f:-360.0f);
    }

    if(abs(gyAngSimp.y) > 180.0f){
      gyAngSimp.y += ( ((float)((  ((long)gyAngSimp.y) & 0x80000000L ) >> 31))? 360.0f:-360.0f);
    }

    if(abs(gyAngSimp.z) > 180.0f){
      gyAngSimp.z += ( ((float)((  ((long)gyAngSimp.z) & 0x80000000L ) >> 31))? 360.0f:-360.0f);
    }


    t2 = micros() -t2 ;
    //digitalWrite(DBGPIN,0);
    imudataready = 0;

    prready = 1;
  }//imudata 
  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);


  /*----------------------------------------------------------------------------------------------------------------------------*/


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
    t5 = micros();
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

    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    heading = atan2(mf.YAxis, mf.XAxis);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your locatio

    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.


    // München: 2°27' -> 0.04276056
    // declinationangle  
    heading += 0.04276;

    // Correct for when signs are reversed.
    if(heading < -PI)
      heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > PI)
      heading -= 2*PI;

    // Convert radians to degrees for readability.
    headingdegrees = heading * 180/M_PI; 
    magndataready = 0;


#if DGBOUTMODE == 4
    ahrs.MadgwickAHRSupdateIMU(gf.x,gf.y,gf.z,af.x,af.y,af.z);
#endif

    t5=  micros() - t5 ;
  }//magndata
  /*----------------------------------------------------------------------------------------------------------------------------*/

  if(ultsready){

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



  }//ultsready


  /*----------------------------------------------------------------------------------------------------------------------------*/


  if(serialready){
Serial.print(inString);
    cmd = inString;
inString = "";

cmd.trim();

     byte * buf = serialbuf;
    int k = 0;

//Serial.println((char*)buf);
  Serial.println();
   
   
    for(byte i = 0; i < cmd.length();i++){
  serialbuf[i] = (byte) cmd.charAt(i);
  
  }
   Serial.println("BUFFER:");
   Serial.print((char*)buf);
 
 
    float test_f;
    float *pidf = (float*) calloc(3,sizeof(float));
    int16_t test_i16;

    // Serial.write(buf[k]);
    //  Serial.write(buf[k+1]);
    //  Serial.write(buf[k+2]);
    //  Serial.write(buf[k+3]);
 if((buf[k+1] == 'e' && buf[k+2] == 'n') || (buf[k+1] == 'E' && buf[k+2] == 'N')){

      main_enable = 1;
      setImuStruct(&gyAngSimp,0.0f,0.0f,0.0f);

    } 
    else if((buf[k+1] == 'd' && buf[k+2] == 'i' && buf[k+3] == 's' ) || (buf[k+1] == 'D' && buf[k+2] == 'I' && buf[k+3] == 'S')){

      main_enable = 0;
      setImuStruct(&gyAngSimp,0.0f,0.0f,0.0f);
    } else
    if(buf[k+1] == 'p' || buf[k+1] == 'P' ){


      pidf =  ((float*) (buf+5));


      /*
 buf_2[0] = buf[k+5];
       buf_2[1] = buf[k+7];
       buf_2[2] = buf[k+9];
       buf_2[3] = buf[k+11];
       
       test_f =  *((float*) buf_2);
       pidhoehe[0]= test_f;
       // Serial.print(pidhoehe[KP]);
       //  Serial.print(" ");
       
       //KI
       buf_2[0] = buf[k+12];
       buf_2[1] = buf[k+14];
       buf_2[2] = buf[k+16];
       buf_2[3] = buf[k+18];
       
       test_f = *((float*) buf_2);
       pidhoehe[1] = test_f;
       //  Serial.print(pidhoehe[KI]);
       //  Serial.print(" ");
       
       //KD
       buf_2[0] = buf[k+19];
       buf_2[1] = buf[k+21];
       buf_2[2] = buf[k+23];
       buf_2[3] = buf[k+25];
       
       test_f = *((float*) buf_2);
       pidhoehe[2] = test_f;
       //    Serial.print(pidhoehe[KD]);
       //    Serial.print(" ");
       */

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
              break;
            }

          case '2':
          case 'c':
            {

              for(byte i = 0 ; i<3;i++){
                pid_mot_alt_conservative[i] = pidf[i];
              }
              break;
            }

          case '3':
          case 'a':
            {

              for(byte i = 0 ; i<3;i++){
                pid_mot_alt_aggro[i] = pidf[i];
              }
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
          default:break;  
        }
          break;
         
        }

default:break;
      }//sw
    }
    else if(buf[k+1] == 'w' && buf[k+2] == 'i'){

      /*
       buf_2[0] = buf[k+5];
       buf_2[1] = buf[k+7];
       buf_2[2] = buf[k+9];
       buf_2[3] = buf[k+11];
       */

      test_f = *((float*) (buf+5));

      //if(test_f > -90.0 && test_f < 90.0){

      reg_set_rl_ang = constrain(test_f,-90.0f,90.0f);
      // Serial.print(winkel);
      //  Serial.print(" ");
      //}

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
              setImuStruct(&gyAngSimp,0.0f,gyAngSimp.y,gyAngSimp.z);

              break;
            }

          case 'Y':
          case 'y':
            {
              setImuStruct(&gyAngSimp,gyAngSimp.x,0.0f,gyAngSimp.z);

              break;
            }

          case 'Z':
          case 'z':
            {
              setImuStruct(&gyAngSimp,gyAngSimp.x,gyAngSimp.y,0.0f);

              break;
            }
          default:
            {


              setImuStruct(&gyAngSimp,0.0f,0.0f,0.0f);

              break;
            }
          }

          break;
        }



      default:
        break;
      }
    } 
    

#if DBGOUTMODE == 8
    Serial.print("ph1:\t");
    Serial.print(pid_mot_alt[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt[1],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt[2],6);

    Serial.print("\tph2:\t");
    Serial.print(pid_mot_alt_conservative[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt_conservative[1],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt_conservative[2],6);

    Serial.print("\tph3:\t");
    Serial.print(pid_mot_alt_aggro[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt_aggro[1],6);
    Serial.print(" ");
    Serial.print(pid_mot_alt_aggro[2],6);

    Serial.print("pr1:\t");
    Serial.print(pid_mot_rl[0],6);
    Serial.print(" ");
    Serial.print(pid_mot_rl[1],6);
    Serial.print(" ");
    Serial.print(pid_mot_rl[2],6);

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

    Serial.print("\twi:\t");
    Serial.print(reg_set_rl_ang,6);

    Serial.print("\twh:\t");
    Serial.print(reg_set_h,6);


    Serial.print("\tsc:\t");
    Serial.print(reg_set_speed);

    Serial.print("\tRAM:\t");
    Serial.print(freeRam());

    Serial.println();
    Serial.flush();
#endif


    free(pidf);
serialready = 0;
    //parchng = 1;
  }//serialready

  /*----------------------------------------------------------------------------------------------------------------------------*/
  if(main_enable){
    if(motcont){
      //reg_set_rl_ang = 0.0f;

      motcont = 0;
    
      dbg1f = pid_rl.step(reg_set_rl_ang,gyAngSimp.z);

      setMotDirection(deg2rad(dbg1f),reg_set_speed);
      // setMotDirection(0.0f,120);

      //setMotSpeed(0,MOT_R_IN1_PIN,MOT_R_IN2_PIN,MOT_R_PWM_PIN);
      //setMotSpeed(0,MOT_L_IN1_PIN,MOT_L_IN2_PIN,MOT_L_PWM_PIN);
      // setMotSpeed(-100,MOT_ALT_IN1_PIN,MOT_ALT_IN2_PIN,MOT_ALT_PWM_PIN);
      reg_set_h = 150.0f;
      // setMotDirection(0.0f,-50);

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

      // setImuStruct(&gyAngSimp,gyAngSimp.x,gyAngSimp.y,0.0f); // reset gyro

      //    setMotAlt(100);
      
    }//motcont
  }else{
  
  setMotAlt(0);
  setMotDirection(0.0f,0);
  
  }// main enable

  /*----------------------------------------------------------------------------------------------------------------------------*/



  if(prready){
    // digitalWrite(DBGPIN,1);
#if DBGOUTMODE == 1
    Serial.print("a/g/m:\t ");

    Serial.print( af.x ,5 ); 
    Serial.print("\t");

    Serial.print( af.y ,5 ); 
    Serial.print("\t");

    Serial.print( af.z ,5 ); 
    Serial.print("\tG:\t");

    Serial.print( gf.x ,5 ); 
    Serial.print("\t");

    Serial.print( gf.y ,5 ); 
    Serial.print("\t");

    Serial.print( gf.z ,5 ); 
    Serial.print("\tGI:\t");

    Serial.print(gyAng.x,5);
    Serial.print("\t");

    Serial.print(gyAng.y,5);
    Serial.print("\t");

    Serial.print(gyAng.z,5);
    Serial.print("\tM:\t");

    Serial.print( mf.XAxis ,5 ); 
    Serial.print("\t");

    Serial.print( mf.YAxis ,5 ); 
    Serial.print("\t");

    Serial.print( mf.ZAxis ,5 ); 
    Serial.print("\t");

    Serial.print(heading,5);
    Serial.print("\t");

    Serial.print(headingdegrees,5);
    Serial.print("\t");

    Serial.print("Read:\t");
    Serial.print(t1);
    Serial.print("\t");

    Serial.print("Proc:\t");
    Serial.print(t2);
    Serial.print("\t");

    t3 = micros() - t3;
    Serial.print("Loop:\t");
    Serial.print(t3);



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
    Serial.print("Simp:\t");

    Serial.print(gyAngSimp.x,5);
    Serial.print("\t");

    Serial.print(gyAngSimp.y,5);
    Serial.print("\t");

    Serial.print(gyAngSimp.z,5);
    Serial.print("\tTrap:\t");


    Serial.print(gyAng.x,5);
    Serial.print("\t");

    Serial.print(gyAng.y,5);
    Serial.print("\t");

    Serial.print(gyAng.z,5);
    Serial.print("\tError:\t");

    Serial.print((gyAngSimp.x - gyAng.x),5);
    Serial.print("\t");

    Serial.print((gyAngSimp.y - gyAng.y),5);
    Serial.print("\t");

    Serial.print((gyAngSimp.z - gyAng.z),5);
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


#define MOT_SHORT 0
#define MOT_FREERUN 1
#define MOT_HALTMODE MOT_FREERUN

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

  sp0 = constrain(sp0,-255,255);

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
volatile int si = 0;
/*
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

  if( ovfcnt == 9 ){ // 250hz/2*samplerate*256


    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState); // 250khz/samplerate
    //accelgyro.getMotion6(&ar.ax, &ar.ay, &ar.az, &gr.gx, &gr.gy, &gr.gz);
    magnready=1;
    imuready = 1;
    motcont = 1;
    ovfcnt = 0;

    if(ovfcnt2 == 1){

      ovfcnt2=0;
    }
    else{
      ovfcnt2++;
    }

    if(ovfcnt3 == 1){

      ultsready = 1;
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

















































