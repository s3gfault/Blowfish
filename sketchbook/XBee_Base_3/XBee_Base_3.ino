
#include <SoftwareSerial.h>

#define KP 0
#define KI 1
#define KD 2

#define DEBUG 1

//float pidhoehe[3] = {0.0f,0.0f,0.0f}; //6 Nachkommastellen
//float pidrichtung[3] = {0};//6 Nachkommastellen
//float winkel = 0.0; //2 Nachkommastellen [-90,90] 
//int16_t schub = 0; // [-255, 255]
//int16_t wunschhoehe = 0;// [30-300]
//char cmd[2] = {  'w','u'}; // Befehle ph ,pr ,wi ,sc ,wu

#define STRLEN 32
#define CHRBUFSIZE 16
String inString;
String cmd;

String xbeeString;
SoftwareSerial xbee(12,13);// rx tx

long t[5] ={0,0,0,0,0};

void serialFloatSend(float f){
  byte * b = (byte *) &f;

if (b[0]<0x10) {Serial.print("0");} 
  Serial.print(b[0],HEX);
  // Serial.print(" "); 
  if (b[1]<0x10) {Serial.print("0");} 
  Serial.print(b[1],HEX);
  // Serial.print(" "); 
  if (b[2]<0x10) {Serial.print("0");} 
  Serial.print(b[2],HEX);
  // Serial.print(" "); 
  if (b[3]<0x10) {Serial.print("0");} 
  Serial.print(b[3],HEX);


 if (b[0]<0x10) {xbee.print("0");} 
  xbee.print(b[0],HEX);

 if (b[1]<0x10) {xbee.print("0");} 
  // Serial.print(" "); 
  xbee.print(b[1],HEX);
   if (b[2]<0x10) {xbee.print("0");} 
  // Serial.print(" "); 
  xbee.print(b[2],HEX);
   if (b[3]<0x10) {xbee.print("0");} 
  // Serial.print(" "); 
  xbee.print(b[3],HEX);
  /*
  float  g = *((float*) b);
   
   Serial.print(" ");
   Serial.print(g ,6);
   Serial.print(" ");
   
   Serial.print("g/BIN:");
   */
  //Serial.print(b[0],BIN);
  //Serial.print(" "); 
  //Serial.print(b[1],BIN);
  //Serial.print(" "); 
  //Serial.print(b[2],BIN);
  //Serial.print(" "); 
  //Serial.print(b[3],BIN);
  //Serial.print(" ");


}

void serialInt16_tSend(int16_t i){
  byte * b = (byte *) &i;


if (b[0]<0x10) {Serial.print("0");} 
   
  Serial.print(b[0],HEX);
if (b[1]<0x10) {Serial.print("0");} 
  // Serial.write(" "); 
  Serial.print(b[1],HEX);

if (b[0]<0x10) {xbee.print("0");} 
  xbee.print(b[0],HEX);
if (b[1]<0x10) {xbee.print("0");} 
  // Serial.write(" "); 
  xbee.print(b[1],HEX);

}
/*
void sendDataHardware( char * cmd){
 
 if(cmd[0] == 'p' && cmd[1] == 'h'){
 
 Serial.print("bph: ");
 serialFloatSend(pidhoehe[KP]);
 //Serial.print(" ");
 serialFloatSend(pidhoehe[KI]);
 //Serial.print(" ");
 serialFloatSend(pidhoehe[KD]);
 Serial.println();
 }
 
 if(cmd[0] == 'p' && cmd[1] == 'r'){
 
 Serial.print("bpr: ");
 serialFloatSend(pidrichtung[KP]);
 //Serial.write(" ");
 serialFloatSend(pidrichtung[KI]);
 //Serial.write(" ");
 serialFloatSend(pidrichtung[KD]);
 Serial.println();
 }
 
 if(cmd[0] == 'w' && cmd[1] == 'i'){
 
 Serial.print("bwi: ");
 serialFloatSend(winkel);
 Serial.println();
 
 }
 
 if(cmd[0] == 's' && cmd[1] == 'c'){
 
 Serial.print("bsc");
 serialInt16_tSend(schub);
 Serial.println();
 ;
 }
 
 if(cmd[0] == 'w' && cmd[1] == 'u'){
 
 Serial.print("bwu: ");
 serialInt16_tSend(wunschhoehe);
 Serial.println();
 }
 }
 
 
 */
void setup() {

  Serial.begin(9600);
  xbee.begin(9600);
  //Serial1.begin(115200);
  xbee.listen();
}


float fbuf[3]= {
  0.0f,0.0f,0.0f};
int16_t ibuf[3] = {
  0,0,0};

volatile byte strready = 0;

void loop() { 


  //Zuweisung
  /*  pidhoehe[KP] = 2.0; 
   pidhoehe[KI] = -78.0; 
   pidhoehe[KD] = 4.0;
   
   pidrichtung[KP] = 0.0; 
   pidrichtung[KI] = 0.0; 
   pidrichtung[KD] = 0.0;
   
   winkel = 12.0;
   
   schub = 0; //eigentlich int16_t
   
   wunschhoehe = 100; //eigentlich int16_t
   
   */
  //
  // Senden via XBee:

  if(strready){
  
#if DEBUG == 1
  t[0] = micros();
#endif

    cmd = inString;
    strready = 0;
    inString = "";


    cmd.trim();


    if(cmd.startsWith("b")||cmd.startsWith("B")){

      char* xbuf = (char*) calloc( CHRBUFSIZE, sizeof(char));
      char* ybuf = (char*) calloc( CHRBUFSIZE, sizeof(char));
      char* zbuf = (char*) calloc( CHRBUFSIZE, sizeof(char));
      int xstart = 0;
      int ystart = 0;
      int zstart = 0;
      if(!(cmd.substring(1).compareTo("en")) || !(cmd.substring(1).compareTo("EN"))){
        Serial.print("ben");
        // Serial.print(cmd.charAt(1));
        //  Serial.print(cmd.charAt(2));
        //  Serial.print(cmd.charAt(3));
        Serial.print(" ");
        Serial.print(" ");

        xbee.print("ben");
        // xbee.print(cmd.charAt(1));
        //  xbee.print(cmd.charAt(2));
        //  xbee.print(cmd.charAt(3));
        xbee.print(" ");
        xbee.print(" ");
        xbee.println();
        Serial.println();


      }
      else if(!(cmd.substring(1).compareTo("dis")) || !(cmd.substring(1).compareTo("DIS"))){
        Serial.print("bdis");
        // Serial.print(cmd.charAt(1));
        //  Serial.print(cmd.charAt(2));
        //  Serial.print(cmd.charAt(3));
        Serial.print(" ");
        

        xbee.print("bdis");
        // xbee.print(cmd.charAt(1));
        //  xbee.print(cmd.charAt(2));
        //  xbee.print(cmd.charAt(3));
        xbee.print(" ");
        
        xbee.println();
        Serial.println();

      }
      else if(!(cmd.substring(1).compareTo("tog")) || !(cmd.substring(1).compareTo("TOG"))){


      }
      else if(cmd.substring(1).startsWith("meth") || cmd.substring(1).startsWith("METH")){





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

#if DEBUG == 1
            t[1] = micros();
#endif            
            fbuf[0] = atof(xbuf);
            fbuf[1] = atof(ybuf);
            fbuf[2] = atof(zbuf);

#if DEBUG == 1
            t[1] = micros() - t[1];
#endif


            switch((char)cmd.charAt(2)){
            case 'r':
            case 'R':
            case 'H':
            case 'h':
              {

                switch((char)cmd.charAt(3)){


                case '1':
                case 'n':
                case '2':
                case 'c':
                case '3':
                case 'a':
                  {

                    Serial.print("b");
                    Serial.print(cmd.charAt(1));
                    Serial.print(cmd.charAt(2));
                    Serial.print(cmd.charAt(3));
                    Serial.print(" ");
                    xbee.print("b");
                    xbee.print(cmd.charAt(1));
                    xbee.print(cmd.charAt(2));
                    xbee.print(cmd.charAt(3));
                    xbee.print(" ");
                    for(byte i = 0; i<3;i++){

                      serialFloatSend(fbuf[i]);
                    }
                    Serial.println();
                    xbee.println();

                    break;
                  }



                default:
                  break;


                }
                break;
              }


            default:
              break;

            }
            break;
          }
        case 'w':
        case 'W':
          {

            fbuf[0] = atof(xbuf);

            switch((char)cmd.charAt(2)){

            case 'i':
            case 'I':
            case 'h':
            case 'H':
              {

                Serial.print("b");
                Serial.print(cmd.charAt(1));
                Serial.print(cmd.charAt(2));
                Serial.print(" ");
                Serial.print(" ");

                xbee.print("b");
                xbee.print(cmd.charAt(1));
                xbee.print(cmd.charAt(2));
                xbee.print(" ");
                xbee.print(" ");

                serialFloatSend(fbuf[0]);
                xbee.println();
                Serial.println();
                break;
              }
            default:
              break;



            }
            break;
          }
        case 's':
        case 'S':
          {

            switch((char)cmd.charAt(2)){

            case 'C':
            case 'c':
              {
                ibuf[0] = atoi(xbuf);    
                Serial.print("b");
                Serial.print(cmd.charAt(1));
                Serial.print(cmd.charAt(2));
                Serial.print(" ");
                Serial.print(" ");

                xbee.print("b");
                xbee.print(cmd.charAt(1));
                xbee.print(cmd.charAt(2));
                xbee.print(" ");
                xbee.print(" ");
                serialInt16_tSend(ibuf[0]);

                xbee.println();
                Serial.println();
                break;
              }

            default:
              break;
            }
            break;
          }
          case 'r':
          case 'R':{
          
            switch((char)cmd.charAt(3)){
              case 'x':
              case 'X':
              case 'y':
              case 'Y':
              case 'z':
              case 'Z':{
                  
                Serial.print("b");
                Serial.print(cmd.charAt(1));
                Serial.print(cmd.charAt(2));
                Serial.print(cmd.charAt(3));
                Serial.print(" ");

                xbee.print("b");
                xbee.print(cmd.charAt(1));
                xbee.print(cmd.charAt(2));
                xbee.print(cmd.charAt(3));
                xbee.print(" ");
              
              break;}
              default:break;
              
            }
          break;
          }
          
        default:
          break;
        }


        free(xbuf);
        free(ybuf);
        free(zbuf);  



      }//switch char 1
    }// if startswith b

#if DEBUG == 1
t[0] = micros() - t[0];
#endif


#if DEBUG == 1
  Serial.println();
  
  Serial.print("srdy: ");
  Serial.print(t[0]);
  Serial.print(" ");
  
  Serial.print("atof: ");
  Serial.print(t[1]);
  Serial.print(" ");
  
    Serial.print("Ram: ");
    Serial.print(freeRam());
    Serial.print(" ");
   // Serial.println();
  
  Serial.println();
  Serial.flush();
  
  
#endif


  }//serready
  else  {
  xbee.listen();
  while(xbee.available() > 0){
 
   
   char inChar = (char) xbee.read();

    xbeeString += inChar;

    if(inChar ==  '\n'){
      Serial.println(xbeeString);
      xbeeString = "";
    }


  }
   
    


  
  }

  /*
  int u = 0;
   if(u < 100){
   
   //    sendDataHardware(cmd);
   
   }
   
   // delay sollte nicht viel größer werden!
   delay(100);
   
   
   */
  Serial.flush();
  xbee.flush();
  //Ende Senden via XBee
}



void serialEvent(){

  while(Serial.available() > 0){

    char inChar = (char) Serial.read();

    inString += inChar;

    if(inChar ==  '\n'){
      strready = 1;
    }


  }

}// SerialEvent


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}//freeram




