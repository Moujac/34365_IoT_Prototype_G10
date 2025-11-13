// Remember to include this, or else you cannot sleep or wake the MCU! 
#include <avr/sleep.h> 
#include <avr/wdt.h> 
#include <Wire.h>

#define PCF8574_ADDR 0x20 // Standardadresse (kan variere afhængigt af A0, A1, A2)

void writePCF8574(byte data) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(data); // Inverter output: LOW tænder LED
  Wire.endTransmission();
}

int getState(){
  uint8_t wdtTSetting = WDTCSR & 0x27;
  uint8_t sleepMode = _SLEEP_CONTROL_REG & 0x0E; // 000 = idle, 001 = ADC Noise rEDUCTION, 010 = Power-downm 011 = Power-save, 110 = Standby, 111 = Extended Standby
  int longSleep;
  int deepSleep;
  if(wdtTSetting == 33){
    longSleep = 1;
  }else{
    longSleep = 0;
  }
  
  if(sleepMode == 0x04){
    deepSleep = 1;
  } else if (sleepMode == 0x0C){
    deepSleep = 0;
  } else {
    deepSleep = 2;
  }
  int result = (longSleep & 0x01) | ((deepSleep & 0x01) << 1);
  writePCF8574(1 << (result-1)); // Kun én LED tændt ad gangen
  delay(1000);
  //delay(3000);
  return  result;
}

void setState(int state){
  int longSleep = state & 0x01; 
  int deepSleep = state & 0x02 >> 1;

  if(longSleep == 1){
    wdt_enable(WDTO_8S);
    WDTCSR |= 1 << WDIE; 
  } else {
    wdt_enable(WDTO_4S);
    WDTCSR |= 1 << WDIE; 
  }
  if(deepSleep == 1){
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  } else {
    set_sleep_mode(SLEEP_MODE_STANDBY);
  }
}

//THIS FUNCTION MUST BE DELETED
unsigned long getWDTIntervalMs() {
 
  uint8_t wdtTSetting = WDTCSR & 0x27;  // Mask out WDP[3:0] bits
  //uint8_t wdp = wdtcsr & 0x27;     // Timeout prescaler bits
  
  switch (wdtTSetting) {
    case 0: return 16;//16;       // 16 ms
    case 1: return 32;//32;
    case 2: return 64;
    case 3: return 125;
    case 4: return 250;
    case 5: return 500;
    case 6: return 1000;
    case 7: return 2000;
    case 32: return 4000;
    case 33: return 8000;
  }
  return 0;
}

void setup() { 
  Wire.begin();
  //wdt_disable();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000); 
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000); 



  // Update WDT before it triggers 
  wdt_enable(WDTO_8S); //# HERE TO INSERT DIFFERENT SLEEP TIME
  WDTCSR |= 1 << WDIE; 
  // Enable the Watchdog Timer interrupt 
  
  /*digitalWrite(LED_BUILTIN, HIGH); 
  delay(getWDTIntervalMs()); 
  digitalWrite(LED_BUILTIN, LOW); 
  delay(getWDTIntervalMs()); 
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000); 
  digitalWrite(LED_BUILTIN, LOW); 
  delay(1000); */

  // Enable interrupts 
  sei(); 
  // Set up sleep mode and enable it 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //# HERE TO INSERT DIFFERENT SLEEPMODE
  sleep_enable(); 
  // Enable built-in LED pin for output 
  

} 

void Active(){

  int status = 0;
  switch(status){
    case 0: //not long break, just standby - emergency
      emergency();
    break;
    case 1: //sleeps for a long time, but standby - Active
    case 2: //sleeps for short time, but deepsleep - Active
      Active();
    break;
    case 3: //sleep for long time and deepsleep - Sleep
      sleep();
    break;
  }
}

void emergency(){

}

void sleep(){

}

void mainChain(){
  switch(getState()){
    case 0: //not long break, just standby - emergency
      emergency();
    break;
    case 1: //sleeps for a long time, but standby - Active
    case 2: //sleeps for short time, but deepsleep - Active
      Active();
    break;
    case 3: //sleep for long time and deepsleep - Sleep
      sleep();
    break;
  }
  sleep_cpu(); 
}

void loop() { 
  // Blink LED to see if period between blinking is extended 
  /*digitalWrite(LED_BUILTIN, HIGH); 
  delay(3000); 
  digitalWrite(LED_BUILTIN, LOW); */

  if(getWDTIntervalMs() == 8000){
    /*digitalWrite(LED_BUILTIN, HIGH); 
    delay(3000); 
    digitalWrite(LED_BUILTIN, LOW); 
    delay(1000); */
    wdt_enable(WDTO_4S);
    WDTCSR |= 1 << WDIE;
  }
  else if(getWDTIntervalMs() == 4000){
    /*digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000); 
    digitalWrite(LED_BUILTIN, LOW); 
    delay(300); 
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000); 
    digitalWrite(LED_BUILTIN, LOW); 
    delay(1000); */
    wdt_enable(WDTO_8S);
    WDTCSR |= 1 << WDIE;
  }
  if(getState() == 3){
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000); 
    digitalWrite(LED_BUILTIN, LOW); 
    delay(1000);
  };
  sleep_cpu(); 
  } 
ISR (WDT_vect) { 
  // Reset WDIE to 1 
  WDTCSR |= 1 << WDIE; 
} 