

char serialbuf[14] = {'b','w','i',' ',' ','B','7','F','3','9','D','3','F','\n'};




void setup(){

Serial.begin(9600);
}
char buf[4] = {0,0,0,0};
void loop(){

  for(struct {byte i ;byte  k;} fl = {5,0} ; fl.i < (((14 -5 -1)/2)+5) ;fl.i++, fl.k++){
  
  serialbuf[fl.i] = (char) ( ( ( ((serialbuf[fl.i+fl.k] - 48)>9)?(serialbuf[fl.i+fl.k]-48-7):(serialbuf[fl.i+fl.k]-48))<<4) + (((serialbuf[fl.i+fl.k+1] - 48)>9)?(serialbuf[fl.i+fl.k+1]-48-7):(serialbuf[fl.i+fl.k+1]-48)) );
  Serial.print(fl.i,DEC);
    Serial.print(";");
    Serial.print(fl.k,DEC);
Serial.print("|");
  }

Serial.println();

float f = *((float*)(serialbuf+5));

for(int i = 0; i< 14; i++){
Serial.print(serialbuf[i],DEC);
Serial.print("|");
}
Serial.println();

Serial.println(f,6);

delay(1500);
}
