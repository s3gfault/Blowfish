
#define KP 0
#define KI 1
#define KD 2

void serialFloatSend(float f){
  byte * b = (byte *) &f;
  
  
  Serial.write(b[0]);
  Serial.print(" "); 
  Serial.write(b[1]);
  Serial.print(" "); 
  Serial.write(b[2]);
  Serial.print(" "); 
  Serial.write(b[3]);
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
  Serial.write(" "); 
  Serial.write(b[1]);
   
}
  
void setup() {
  
  Serial.begin(9600);
  
  //Serial1.begin(115200);
}





void loop() {
 
 
  float pidhoehe[3] = {0}; //6 Nachkommastellen
  float pidrichtung[3] = {0};//6 Nachkommastellen
  float winkel = 0.0; //2 Nachkommastellen [-90,90] 
  int16_t schub = 0; // [-255, 255]
  int16_t wunschhoehe = 0;// [30-300]
  char cmd[2] = {'p','h'}; // Befehle ph ,pr ,wi ,sc ,wu 
  
 //Zuweisung
 pidhoehe[KP] = 2.0; 
 pidhoehe[KI] = -78.0; 
 pidhoehe[KD] = 4.0;

 pidrichtung[KP] = 0.0; 
 pidrichtung[KI] = 0.0; 
 pidrichtung[KD] = 0.0;

 winkel = 0.0;

 schub = 0; //eigentlich int16_t

 wunschhoehe = 0; //eigentlich int16_t
//

 // Senden via XBee:
  
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
    
    Serial.print("bsc: ");
    serialInt16_tSend(schub);
    Serial.println();;
  }
  
  if(cmd[0] == 'w' && cmd[1] == 'u'){
    
    Serial.print("bwu: ");
    serialInt16_tSend(wunschhoehe);
    Serial.println();
  }
  
  delay(400);
  Serial.flush();
  //Ende Senden via XBee
}
