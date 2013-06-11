
//Funktion um am Zeppelin geänderte Daten am Boden auszugeben
//11.6.13
void recieve_changed_Data(){
  
  //inString = "";
  xbee.listen();
  
  int i = 1;
  
  while(xbee.available() > 0){
      
    char inChar = (char) xbee.read();
    //inString += inChar
    Serial.print(inChar);
    
  }
    
    
