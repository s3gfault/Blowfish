#define KP 0
#define KI 1
#define KD 2

char buf[32] ;
byte buf_2[4];
byte buf_3[2];

float pidhoehe[3] = {0}; //6 Nachkommastellen
float pidrichtung[3] = {0};//6 Nachkommastellen
float winkel = 0.0; //2 Nachkommastellen [-90,90] 
int16_t schub = 0; // [-255, 255]
int16_t wunschhoehe = 0;// [30-300]
char cmd[2] = {'0','0'}; // Befehle ph ,pr ,wi ,sc ,wu

int strready = 0;

int getDataHardware(){
  
  int result[2] ={0};
  int i = 0;
  while(Serial.available() > 0){
    
    
    //char inChar = (char) Serial.read();
    buf[i] = (char)(Serial.read());
    i++;
    
    //inString += inChar;

    if(buf[i] ==  '\n'){
      strready = 1;
    }
  }
  
   int k;
   
   for(k = 0; k<=i;k++){
    
    if(buf[k] == 'b'){
      
      break;
    
    }
   }
   
   result[0] = k;
   //result[1] = strready
   return k;
}


void wrapper(int k){
    
  float test_f;
  int16_t test_i16;
  
  Serial.write(buf[k]);
  Serial.write(buf[k+1]);
  Serial.write(buf[k+2]);
  Serial.write(buf[k+3]);
  
  if(buf[k+1] == 'p' && buf[k+2] == 'h'){
    
    //KP
    buf_2[0] = buf[k+5];
    buf_2[1] = buf[k+7];
    buf_2[2] = buf[k+9];
    buf_2[3] = buf[k+11];
    
    test_f =  *((float*) buf_2);
    pidhoehe[0]= test_f;
    Serial.print(pidhoehe[KP]);
    Serial.print(" ");
    
    //KI
    buf_2[0] = buf[k+12];
    buf_2[1] = buf[k+14];
    buf_2[2] = buf[k+16];
    buf_2[3] = buf[k+18];
    
    test_f = *((float*) buf_2);
    pidhoehe[1] = test_f;
    Serial.print(pidhoehe[KI]);
    Serial.print(" ");
    
    //KD
    buf_2[0] = buf[k+19];
    buf_2[1] = buf[k+21];
    buf_2[2] = buf[k+23];
    buf_2[3] = buf[k+25];
    
    test_f = *((float*) buf_2);
    pidhoehe[2] = test_f;
    Serial.print(pidhoehe[KD]);
    Serial.print(" ");
  }
  
  else if(buf[k+1] == 'p' && buf[k+2] == 'r'){
    
        
    //KP
    buf_2[0] = buf[k+5];
    buf_2[1] = buf[k+7];
    buf_2[2] = buf[k+9];
    buf_2[3] = buf[k+11];
    
    test_f = *((float*) buf_2);
    pidrichtung[0] = test_f;
    Serial.print(pidrichtung[KP]);
    Serial.print(" ");
    
    //KI
    buf_2[0] = buf[k+12];
    buf_2[1] = buf[k+14];
    buf_2[2] = buf[k+16];
    buf_2[3] = buf[k+18];
    
    test_f = *((float*) buf_2);
    pidrichtung[1] = test_f;
    Serial.print(pidrichtung[KI]);
    Serial.print(" ");
    
    //KD
    buf_2[0] = buf[k+19];
    buf_2[1] = buf[k+21];
    buf_2[2] = buf[k+23];
    buf_2[3] = buf[k+25];
    
    test_f = *((float*) buf_2);
    pidrichtung[2] = test_f;
    Serial.print(pidrichtung[KD]);
    Serial.print(" ");
  }
  
  else if(buf[k+1] == 'w' && buf[k+2] == 'i'){
    
    buf_2[0] = buf[k+5];
    buf_2[1] = buf[k+7];
    buf_2[2] = buf[k+9];
    buf_2[3] = buf[k+11];
    
    test_f = *((float*) buf_2);
 
    if(test_f > -90.0 && test_f < 90.0){
      
      winkel = test_f;
      Serial.print(winkel);
      Serial.print(" ");
    }
    
  }
  
  else if(buf[k+1] == 's' && buf[k+2] == 'c'){
    
    buf_3[0] = buf[k+5];
    buf_3[1] = buf[k+7];
    
    test_i16 = *((int16_t*) buf_3);
    
    if(test_i16 > -255 && test_i16 < 255){
      
      schub = test_i16;
      Serial.print(schub);
      Serial.print(" ");
    }
  }
  
  else if(buf[k+1] == 'w' && buf[k+2] == 'u'){
    
    buf_3[0] = buf[k+5];
    buf_3[1] = buf[k+7];
    
    test_i16 = *((int16_t*) buf_3);
    
    if(test_i16 > 30 && test_i16 < 300){
      
      wunschhoehe = test_i16;
      Serial.print(wunschhoehe);
      Serial.print(" ");
    }
  }
}
  


void setup(){
  
  Serial.begin(9600);
  
}


 int z = 2;
 
void loop(){
   
   
 int matchpoint = 0;

 
   matchpoint = getDataHardware();
   
   if(strready == 1 && z < 20){
     
     wrapper(matchpoint);
   /*
   Serial.print("ph: ");
   Serial.print(pidhoehe[0]);
   Serial.println();
   */
    Serial.println();
    z++;
   }
   
   Serial.flush(); 
  
   delay(100);
}





/*void serialEvent(){
  
  int i = 0;
    while(Serial.available() > 0){
  
    //char inChar = (char) Serial.read();
    buf[i] = Serial.read();
    i++;
    
    //inString += inChar;

    if(buf[i] ==  '\n'){
      strready = 1;
    }
    

  }
}// SerialEvent*/
