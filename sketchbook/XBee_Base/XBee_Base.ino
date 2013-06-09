
#include <SoftwareSerial.h>

#define KP 0
#define KI 1
#define KD 2

float pidhoehe[3] = {
  0}; //6 Nachkommastellen
float pidrichtung[3] = {
  0};//6 Nachkommastellen
float winkel = 0.0; //2 Nachkommastellen [-90,90] 
int16_t schub = 0; // [-255, 255]
int16_t wunschhoehe = 0;// [30-300]
//char cmd[2] = {  'w','u'}; // Befehle ph ,pr ,wi ,sc ,wu

#define STRLEN 32
#define CHRBUFSIZE 16
String inString;
String cmd;

SoftwareSerial xbee(12,13);


void serialFloatSend(float f){
  byte * b = (byte *) &f;

  Serial.write(b[0]);
  // Serial.print(" "); 
  Serial.write(b[1]);
  // Serial.print(" "); 
  Serial.write(b[2]);
  // Serial.print(" "); 
  Serial.write(b[3]);

  xbee.write(b[0]);
  // Serial.print(" "); 
  xbee.write(b[1]);
  // Serial.print(" "); 
  xbee.write(b[2]);
  // Serial.print(" "); 
  xbee.write(b[3]);
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


  Serial.write(b[0]);
  // Serial.write(" "); 
  Serial.write(b[1]);


  xbee.write(b[0]);
  // Serial.write(" "); 
  xbee.write(b[1]);

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



            fbuf[0] = atof(xbuf);
            fbuf[1] = atof(ybuf);
            fbuf[2] = atof(zbuf);





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
        default:
          break;
        }


        free(xbuf);
        free(ybuf);
        free(zbuf);  



      }//switch char 1
    }// if startswith b



  }//serready


  /*
  int u = 0;
   if(u < 100){
   
   //    sendDataHardware(cmd);
   
   }
   
   // delay sollte nicht viel größer werden!
   delay(100);
   
   
   */
  Serial.flush();
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







