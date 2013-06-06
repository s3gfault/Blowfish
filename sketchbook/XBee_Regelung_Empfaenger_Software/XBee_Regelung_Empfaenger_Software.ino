#include <SoftwareSerial.h>

SoftwareSerial XBee(12,13); //RX/TX

char buf[32] ;
byte buf_2[4];
byte buf_3[2];

void setup(){
  
  Serial.begin(9600);
  XBee.begin(9600);
  
}


float pidhoehe[3] = {0}; //6 Nachkommastellen
float pidrichtung[3] = {0};//6 Nachkommastellen
float winkel = 0.0; //2 Nachkommastellen [-90,90] 
int16_t schub = 0; // [-255, 255]
int16_t wunschhoehe = 0;// [30-300]
char cmd[2] = {'0','0'}; // Befehle ph ,pr ,wi ,sc ,wu

int strready = 0;

void loop(){
  
  int i = 0;
  while(XBee.available() > 0){
    
    //char inChar = (char) Serial.read();
    buf[i] = (char)(XBee.read());
    i++;
    
    //if (buf[i] == 'A'){
    //Serial.write('A');  
    //Serial.write((buf[i]));
    //}
    //inString += inChar;

    if(buf[i] ==  '\n'){
      strready = 1;
    }
    //Serial.print(strready);
    
 }
int match = 0;
int k = 0;
 for(k = 0; k<=i;k++){
  
  if(buf[k] == 'b'){
  
  match = k;  
  Serial.write((buf[k]));
  Serial.write((buf[k+1]));
  Serial.write((buf[k+2]));
  Serial.write((buf[k+3]));

  break;
  
}
 }
  
   wrapper(k);
   /*
   Serial.print("ph: ");
   Serial.print(pidhoehe[0]);
   Serial.println();
    */
  Serial.flush();
  Serial.println();
  //delay synchron mit Sender halten!
  delay(100);
}

void wrapper(int k){
  
  if(buf[k+1] == 'p' && buf[k+2] == 'h'){
    
    //KP
    buf_2[0] = buf[k+5];
    buf_2[1] = buf[k+7];
    buf_2[2] = buf[k+9];
    buf_2[3] = buf[k+11];
    
    pidhoehe[0] = *((float*) buf_2);
    Serial.print(pidhoehe[0]);
    Serial.print(" ");
    
    //KI
    buf_2[0] = buf[k+12];
    buf_2[1] = buf[k+14];
    buf_2[2] = buf[k+16];
    buf_2[3] = buf[k+18];
    
    pidhoehe[1] = *((float*) buf_2);
    Serial.print(pidhoehe[1]);
    Serial.print(" ");
    
    //KP
    buf_2[0] = buf[k+19];
    buf_2[1] = buf[k+21];
    buf_2[2] = buf[k+23];
    buf_2[3] = buf[k+25];
    
    pidhoehe[2] = *((float*) buf_2);
    Serial.print(pidhoehe[2]);
    Serial.print(" ");
  }
  
  else if(buf[0] == 'p' && buf[1] == 'r'){
    
        
    //KP
    buf_2[0] = buf[4];
    buf_2[1] = buf[5];
    buf_2[2] = buf[6];
    buf_2[3] = buf[7];
    
    pidrichtung[0] = *((float*) buf_2);
    
    //KI
    buf_2[0] = buf[9];
    buf_2[1] = buf[10];
    buf_2[2] = buf[11];
    buf_2[3] = buf[12];
    
    pidrichtung[1] = *((float*) buf_2);
    
    //KP
    buf_2[0] = buf[14];
    buf_2[1] = buf[15];
    buf_2[2] = buf[16];
    buf_2[3] = buf[17];
    
    pidrichtung[2] = *((float*) buf_2);
  }
  
  else if(buf[0] == 'w' && buf[1] == 'i'){
    
    buf_2[0] = buf[4];
    buf_2[1] = buf[5];
    buf_2[2] = buf[6];
    buf_2[3] = buf[7];
    
    winkel = *((float*) buf_2);
    
  }
  
  else if(buf[0] == 's' && buf[1] == 'c'){
    
    buf_3[0] = buf[4];
    buf_3[1] = buf[5];
    
    schub = *((int16_t*) buf_3);
    
  }
  
  else if(buf[0] == 'w' && buf[1] == 'u'){
    
    buf_3[0] = buf[4];
    buf_3[1] = buf[5];
    
    wunschhoehe = *((int16_t*) buf_3);
  }
 
  
}



/*void serialEvent(){
  
  int i = 0;
    while(XBee.available() > 0){
  
    //char inChar = (char) Serial.read();
    buf[i] = (char)(XBee.read());
    i++;
    Serial.print(buf[i]);
    //inString += inChar;

    if(buf[i] ==  '\n'){
      strready = 1;
      
    }
    

  }
}// SerialEvent*/
