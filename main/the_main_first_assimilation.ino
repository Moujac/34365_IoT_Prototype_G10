// Remember to include this, or else you cannot sleep or wake the MCU! 
#include <avr/sleep.h> 
#include <avr/wdt.h> 
#include <Wire.h>


#include <rn2xx3.h>
#include <SoftwareSerial.h>
// PIN Allocation:
// LoraWan (RN2483A) = D10, D11, D12 (As previusly used in class)
// GPS Tx & Rx = D3, D4
// LED(s) = D5
// Speaker = D6
// SWITCH (Button) = D7
// Ultra Sonic = D8, D9
// Photocell = A0
// Battery voltage sensor = A1
// GPIO Extender (maybe redundant) = A4, A5

#define PCF8574_ADDR 0x20 // Standardadresse (kan variere afhængigt af A0, A1, A2)

SoftwareSerial LoRaSerial(10, 11); // RX, TX
rn2xx3 LoraComponent(LoRaSerial);

const int ledPin = 5; // this can be handled by the GPIO extender
const int speakerPin = 6; 
const int buttonPin = 7;
// this one can be turned on and off by IO Extender
const int USEcho = 8; 
const int USTrig = 9;
const int LoRaResetPin = 12;

const int photoRes = A0;
const int voltPin = A1;

//i am uncertain how many variables can carry over through deepsleep.
//float timing = 0.0;
//float distance = 0.0;

const float Vcc = 3.3;
const float R1 = 10000.0;   
const float R2 = 10000.0;   

//int buttonState = 0;

//int photoVal = 0; // 0 to 1023


/*******************************
LoRa setup
********************************/
void initialize_radio(){
  //reset rn2483
  pinMode(LoRaResetPin, OUTPUT);
  digitalWrite(LoRaResetPin, LOW);
  delay(500);
  digitalWrite(LoRaResetPin, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  LoRaSerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  LoraComponent.autobaud();

  //check communication with radio
  String hweui = LoraComponent.hweui();
  while(hweui.length() != 16){
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = LoraComponent.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(LoraComponent.hweui());
  Serial.println("RN2xx3 firmware version:");
  Serial.println(LoraComponent.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  // CHANGE THIS TO OUR APP EUI AND APP KEY!!!
  const char *appEui = "0000000000000000";
  const char *appKey = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

  join_result = LoraComponent.initOTAA(appEui, appKey);

  while(!join_result){
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = LoraComponent.init();
  }
  Serial.println("Successfully joined TTN");
}

/*******************************
The components' setup - need to double check what persists through deepsleep
********************************/
void setupComponents(){
  pinMode(ledPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(USEcho, INPUT);
  pinMode(USTrig, OUTPUT);
  pinMode(photoRes, INPUT);
  pinMode(voltPin, INPUT);

  LoRaSerial.begin(9600); //serial port to radio
  initialize_radio();

  LoraComponent.tx("TTN Mapper on TTN Enschede node");

}
/*******************************
Ultra sonic measurement and the speaker
J-FIX tone notone
********************************/
float USMeasurement(){
  digitalWrite(USTrig, LOW);
  delay(2);
  digitalWrite(USTrig, HIGH);
  delay(10);
  digitalWrite(USTrig, LOW);
  float timing = pulseIn(USEcho, HIGH);
  float distance = (timing*0.0343)/2; // distance in cm - float
  return distance;
}
/*#!-J ISSUE: Tone.h and Tone.cpp are not defined in this scope - i do not know if our board supports it https://docs.arduino.cc/libraries/tone/
void detectedObject(){
   if(USMeasurement() < 20){ // check if accuracy remains at close range 
    tone(speakerPin, 500); // Start 500 Hz tone
  }else{
    #include <Tone.h>
    notone(speakerPin); // Stop tone, might kill distress sound
  }
}
*/
/*******************************
Emergency button
J-IMPLEMENT as interrupt
J-FIX tone notone
J-IMPLEMENT blinking cane? (more visible)
********************************/
/*TURN INTO A INTERRUPT, HANDLED BY SPECIFIC BUTTON*/
void emergencyButtonHandler(){
  // button handling
  int buttonState = digitalRead(buttonPin);
  if(buttonState == HIGH){
    //#! if pressed send SOS, also needs GPS coords!!!
    Serial.println("TXing");
    /*INSERT GNSS RETRIEVAL*/
    LoraComponent.tx("SOS");
    //#! Also sound speaker?
    //tone(speakerPin, 1000); // Start 1 KHz tone for 10 secs
    delay(10000); //#!-J this delay is good, as a constant siren would get tedious - but maybe it ought to be one that you turn off by pressing the button again.
                      //#+J this would require more coordination with deepsleep though.
    //notone(speakerPin); // Stop tone
  }else{
    //#! if not pressed do nothing? maybe have normal behavior run here?
      //#-J no need - it will be an interrupt. However, we could consider instead having the interrupt start GNSS'es cold/warm/hot start, initialize the LoRa, and then afterwards evaluate whenever GNSS is done. otherwise, it sets the appropiate state, and goes back to sleep
        //#-J might be easier to talk about than read. 
  }
}



/*******************************
Standard turn on button
J-IMPLEMENT turn on button
********************************/
/*INSERT TURN ON/OFF BUTTON - AS INTERRUPT*/


/*******************************
Battery Voltage Calculator
********************************/
float calculateBatteryVoltage(){
   // Calc battery voltage
  int rawVolt = analogRead(voltPin);
  float VoutVolt = (rawVolt / 1023.0) * Vcc;
  float Vcap = VoutVolt * (R1 + R2) / R2;
  return VoutVolt; //#!-J or should we return Vcap?
}
/*******************************
Photoresistor
********************************/
//Has been modified, to have a "do not change" zone between light and dark - so the cane does not constantly switch on and off
int LightOrDark(){
  // Handle PhotoResistor behavior
  int photoVal = analogRead(photoRes);
  if(photoVal < 512){ // Need to figure out how value relates to light level
    // It's dark, turn on LED(s)?
    digitalWrite(ledPin, HIGH);
    return 1;
  }else if(photoVal > 300){
    // It's light, turn off LED(s)?
    digitalWrite(ledPin, LOW);
    return 0;
  }
  return -1;
}
/*******************************
The GPIO extender
********************************/
void writePCF8574(byte data) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(data); // Inverter output: LOW tænder LED
  Wire.endTransmission();
}

/*******************************
logic for determining and setting state for next wake up.
********************************/
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

/*******************************
DEBUGGING FUNCTIONS
********************************/
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