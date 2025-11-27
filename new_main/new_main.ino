// Remember to include this, or else you cannot sleep or wake the MCU!
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <stdint.h>
#include <math.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
//this section sets up LCD - used for debugging
LiquidCrystal_I2C lcd(0x27, 16, 2);


//Things to do - connect components to pins
//connect alls power to mosfets

/**MOSFET usage*/
//1, 2, and 4 correspond to LEDs for debugging.
//8 is not used
#define GPS_MOS 16
#define LoRa_MOS 32
#define LED_MOS 64
#define SONIC_MOS 128


// PIN Allocation:
// LoraWan (RN2483A) = D10, D11, D12 (As previusly used in class)
// GPS Tx & Rx = D3, D4
// LED(s) = D5 % USE INSTEAD GPIO EXTENDER
// Speaker = D6
// SWITCH (Button) = D7
// Ultra Sonic = D8, D9
// Photocell = A0
// Battery voltage sensor = A1
// GPIO Extender (maybe redundant) = A4, A5


//// GPS STUFF ////
// Define GPS Module RX/TX Pins and Baud Rate
static const int RXPinGPS = 4, TXPinGPS = 5;  //On the actual module are: RXPinGPS = TX on the module, TXPinGPS = RX on the module
static const uint32_t GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Initialize GPS serial communication
SoftwareSerial gps_serial(RXPinGPS, TXPinGPS);

// Define LoRa Module RX/TX Pins and Baud Rate
static const int RXPinLORA = 10, TXPinLORA = 11;
static const uint32_t LORABaud = 9600;

// Initialize LORA serial communication
SoftwareSerial lora_serial(RXPinLORA, TXPinLORA);  // RN2483: Pro Mini D10=TX->RN RX, D11=RX->RN TX
rn2xx3 myLora(lora_serial);

// Initialize random global location coordinates
float LAT_DEG = 56.174734f;
float LON_DEG = 13.586034f;

// Helpers function
static inline void encodeInt32BE(int32_t value, uint8_t *out) {
  out[0] = (uint8_t)((value >> 24) & 0xFF);
  out[1] = (uint8_t)((value >> 16) & 0xFF);
  out[2] = (uint8_t)((value >> 8) & 0xFF);
  out[3] = (uint8_t)(value & 0xFF);
}

/*THIS NEEDS TO BE UPDATED!*/
static inline void led_on() {
  digitalWrite(13, HIGH);
}
static inline void led_off() {
  digitalWrite(13, LOW);
}


// Global singla received variable

bool signal_received = false;

//// GPS END ////

#define PCF8574_ADDR 0x20  // Standardadresse (kan variere afhængigt af A0, A1, A2)

const int ledPin = 5;  // this can be handled by the GPIO extender
const int speakerPin = 6;
const int buttonPin = 2;
// this one can be turned on and off by IO Extender
const int USEcho = 8;
const int USTrig = 9;
const int LoRaResetPin = 12;

const int photoRes = A0;
const int voltPin = A1;

const float Vcc = 3.3;
const float R1 = 10000.0;
const float R2 = 10000.0;



//i am uncertain how many variables can carry over through deepsleep.
//float timing = 0.0;
//float distance = 0.0;
//int buttonState = 0;
//int photoVal = 0; // 0 to 1023


/*******************************
GPS STUFF
********************************/
// LoRaWAN init
void initialize_radio() {
  lora_serial.listen();  // Listen to the LORA Serial
  pinMode(12, OUTPUT);   // RN2483 RESET
  digitalWrite(12, LOW);
  delay(500);
  digitalWrite(12, HIGH);

  delay(100);
  lora_serial.flush();

  myLora.autobaud();

  String hweui = myLora.hweui();
  while (hweui.length() != 16) {
    Serial.println("RN2xx3 not responding. Retrying in 10s...");
    delay(10000);
    hweui = myLora.hweui();
  }

  Serial.print("DevEUI: ");
  Serial.println(hweui);
  Serial.print("FW: ");
  Serial.println(myLora.sysver());
  Serial.println("Joining TTN (OTAA)...");

  // AppEUI/AppKey from TTN
  const char *appEui = "0004A30B01068CD7";
  const char *appKey = "F1F4FB8E0D22788C38FE41D7E1AAE9C6";

  bool joined = myLora.initOTAA(appEui, appKey);
  while (!joined) {
    Serial.println("Join failed. Retrying in 60s...");
    delay(60000);
    joined = myLora.initOTAA(appEui, appKey);
  }
  Serial.println("Joined TTN.");
}

// Uplink packet + downlink check
void sendAlertPacket(bool alert, float lat_deg, float lon_deg) {
  lora_serial.listen();  // Listen to the LORA Serial
  // Convert to signed 32-bit microdegrees
  int32_t lat_i = (int32_t)lroundf(lat_deg * 1000000.0f);
  int32_t lon_i = (int32_t)lroundf(lon_deg * 1000000.0f);

  // Payload: [alert(1)][lat(4)][lon(4)]
  uint8_t payload[9];
  payload[0] = alert ? 1 : 0;
  encodeInt32BE(lat_i, &payload[1]);
  encodeInt32BE(lon_i, &payload[5]);

  Serial.print("TX alert=");
  Serial.print(alert ? "true" : "false");
  Serial.print(" lat=");
  Serial.print(lat_deg, 6);
  Serial.print(" lon=");
  Serial.println(lon_deg, 6);

  led_on();
  myLora.txBytes(payload, sizeof(payload));  // default FPort=1
  led_off();

  // Check for downlink
  String downlink = myLora.getRx();
  if (downlink.length() > 0) {
    Serial.print("Downlink received: ");
    Serial.println(downlink);
    handleDownlink(downlink);
  } else {
    Serial.println("No downlink message.");
  }
}

// Interpret downlink payload (HEX ASCII string from rn2xx3 lib)
void handleDownlink(String msg) {
  lora_serial.listen();  // Listen to the LORA Serial
  msg.trim();
  // If we receive: 01 = help confirmation
  if (msg == "01" || msg == "1") {
    Serial.println("Signal received. Help is coming!");
    // buzzer will also be triggered here
  } else {
    Serial.print("Unknown downlink command: ");
    Serial.println(msg);
  }
}

// Get location
void get_location() {
  int sent_loc = false;
  while (sent_loc == false) {
    gps_serial.listen();  // Listen to the GPS Serial
    while (gps_serial.available() > 0) {
      gps.encode(gps_serial.read());
      if (gps.location.isUpdated()) {
        LAT_DEG = gps.location.lat();
        LON_DEG = gps.location.lng();
        sent_loc = true;
      }
    }
  }
}

/*******************************
The components' setup - need to double check what persists through deepsleep
********************************/
void setupComponents() {
  pinMode(ledPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(USEcho, INPUT);
  pinMode(USTrig, OUTPUT);
  pinMode(photoRes, INPUT);
  pinMode(voltPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPin, RISING);

  // GPS INIT //
  // Set Buildin LED to Output mode
  pinMode(13, OUTPUT);

  // Initialize Serial Communication
  Serial.begin(57600);

  // Initialize LORA and GPS Serial
  lora_serial.begin(LORABaud);
  gps_serial.begin(GPSBaud);

  // Print feedback to the user
  Serial.println("Startup");

  // Initialize radio/lora
  initialize_radio();

 // Wait 1s

  delay(1000);



  // Send an initial false alert

  sendAlertPacket(false, LAT_DEG, LON_DEG);



  // Wait 15s. Simulate that the user did not pressed the button yet

  delay(15000);

}
/*******************************
Ultra sonic measurement and the speaker
J-FIX tone notone
********************************/
float USMeasurement() {
  digitalWrite(USTrig, LOW);
  delay(2);
  digitalWrite(USTrig, HIGH);
  delay(10);
  digitalWrite(USTrig, LOW);
  float timing = pulseIn(USEcho, HIGH);
  float distance = (timing * 0.0343) / 2;  // distance in cm - float
  return distance;
}
//#!-J ISSUE: Tone.h and Tone.cpp are not defined in this scope - i do not know if our board supports it https://docs.arduino.cc/libraries/tone/
// Just spelled notone incorrectly!!!
void detectedObject() {
  if (USMeasurement() < 20) {  // check if accuracy remains at close range
    tone(speakerPin, 500);     // Start 500 Hz tone
  } else {
    noTone(speakerPin);  // Stop tone, might kill distress sound
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
float calculateBatteryVoltage() {
  // Calc battery voltage
  int rawVolt = analogRead(voltPin);
  float VoutVolt = (rawVolt / 1023.0) * Vcc;
  float Vcap = VoutVolt * (R1 + R2) / R2;
  return VoutVolt;  //#!-J or should we return Vcap?
}
/*******************************
Photoresistor
//# needs to be changed to use MOSFETs
********************************/
//Has been modified, to have a "do not change" zone between light and dark - so the cane does not constantly switch on and off
int LightOrDark() {
  // Handle PhotoResistor behavior
  int photoVal = analogRead(photoRes);  //Update to GPIO extender
  if (photoVal > 512) {                 // Need to figure out how value relates to light level
    // It's dark, turn on LED(s)?
    //digitalWrite(ledPin, HIGH);
    return 64;  //the new turn off SIGNAL (for writePCF8574)
  } else if (photoVal < 300) {
    // It's light, turn off LED(s)?
    //digitalWrite(ledPin, LOW);
    return 0;  //the new turn off SIGNAL (for writePCF8574)
  }
  return;
}
/*******************************
The GPIO extender
********************************/
/** use these:
1, 2, and 4 correspond to LEDs for debugging.
8 is not used
#define GPS_MOS 16
#define LoRa_MOS 32
#define LED_MOS 64
#define SONIC_MOS 128
*/
//read
byte readPCF8574(byte data) {

  Wire.requestFrom(PCF8574_ADDR, 1);
  while(Wire.available()) {
      return Wire.read();    // Receive a byte as character
  }
}
//write
void writePCF8574(byte data) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(~data);  // Inverter output: LOW tænder LED
  Wire.endTransmission();
}
//the following function sets all appropiate components on for the emergency mode.
//should only be called when initially entering emergency mode, and not subsequent times
void emergencyModeInit() {
  writePCF8574(128 + 32 + 16 + LightOrDark());
}
//the following function sets all appropiate components on for the active mode.
//should only be called when initially entering active mode, and not subsequent times
void activeModeInit() {
  writePCF8574(128 + LightOrDark());
}
//the following function sets all appropiate components on for the sleep mode.
//should only be called when initially entering sleep mode, and not subsequent times
void sleepModeInit() {
  writePCF8574(0);
}
/*******************************
logic for determining and setting state for next wake up.
********************************/
int getState() {
  uint8_t wdtTSetting = WDTCSR & 0x27;
  uint8_t sleepMode = _SLEEP_CONTROL_REG & 0x0E;  // 000 = idle, 001 = ADC Noise rEDUCTION, 010 = Power-downm 011 = Power-save, 110 = Standby, 111 = Extended Standby
  int longSleep;
  int deepSleep;
  if (wdtTSetting == 33) {
    longSleep = 1;
  } else {
    longSleep = 0;
  }


  if (sleepMode == 0x04) {
    deepSleep = 1;
  } else if (sleepMode == 0x0C) {
    deepSleep = 0;
  } else {
    deepSleep = 2;
  }
  int result = (longSleep & 0x01) | ((deepSleep & 0x01) << 1);
  writePCF8574(1 << (result - 1));  // Kun én LED tændt ad gangen
  delay(1000);
  //delay(3000);
  return result;
}

void setState(int state) {
  int longSleep = state & 0x01;
  int deepSleep = state & 0x02 >> 1;

  if (longSleep == 1) {
    wdt_enable(WDTO_8S);
    WDTCSR |= 1 << WDIE;
  } else {
    wdt_enable(WDTO_4S);
    WDTCSR |= 1 << WDIE;
  }
  if (deepSleep == 1) {
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
    case 0: return 16;  //16;       // 16 ms
    case 1: return 32;  //32;
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
void intermissionPower() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

//this function sets up the LCD. 
//after calling this, we can call 
  //lcd.setCursor([X],[Y]);
  //lcd.print(["text"] OR [variables]);
  //lcd.clear();
//print will push the cursor further ahead
void setupLCD() {
  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  /*lcd.print("hello, world!");
  lcd.setCursor(3,1);
  lcd.print(32);
  delay(10000);
  lcd.print("git");
  delay(1000);
  lcd.clear();
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("git");
  delay(1000);*/
}


void gateway(){
  //ALTERNATIVE TO THE /*  // */ trick, you could use ifdef DEBUG to use or hide these segments.
  /*lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("...");//*/
  while(digitalRead(buttonPin) == HIGH){

  }
  /*lcd.setCursor(0,1);
  lcd.print("STEP");//*/
  while(digitalRead(buttonPin) == LOW){
    
  }
  /*lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("STAP");//*/
  while(digitalRead(buttonPin) == HIGH){

  }
}

/** use these:
1, 2, and 4 correspond to LEDs for debugging.
8 is not used
#define GPS_MOS 16
#define LoRa_MOS 32
#define LED_MOS 64
#define SONIC_MOS 128
*/
void powerMeasure() {
  pinMode(buttonPin,INPUT);
  
  //setupComponents();
  Wire.begin();
  setupLCD();
  delay(100);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("READY?");
  gateway();

  //REMEMBER to set mosfets with GPIO extender!
  
  intermissionPower();
  //ACTIVATE buzzer
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Buzzer tone 1");
  tone(speakerPin, 500);
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Buzzer tone 2");
  tone(speakerPin, 1000);
  delay(5000);
  //DEACTIVATE buzzer
  noTone(speakerPin);  // Stop tone, might kill distress sound
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Buzzer done");
  lcd.setCursor(0,1);
  lcd.print("Distance Next");
  gateway();

  intermissionPower();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("distance sensor");
  //ACTIVATE distance sensor - not necessary - buuuut
  writePCF8574(SONIC_MOS);
  for (int i = 0; i < 100; i++) {
    USMeasurement();
  }
  //DEACTIVATE distance sensor
  writePCF8574(0);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Distance Done");
  lcd.setCursor(0,1);
  lcd.print("GNSS Next");
  gateway();
  
  intermissionPower();
  //ACTIVATE GNSS
  writePCF8574(GPS_MOS);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("GNSS module: TBD?");
  for (int i = 0; i > 100; i++) {
  }
  delay(4000);
  //DEACTIVATE GNSS
  writePCF8574(0);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("GNSS Done");
  lcd.setCursor(0,1);
  lcd.print("LoRa Next");
  gateway();
  
  intermissionPower();
  //ACTIVATE LoRa
  writePCF8574(LoRa_MOS);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LoRa Module: TBD?");
  for (int i = 0; i > 100; i++) {
  }
  delay(4000);
  //DEACTIVATE LoRa
  writePCF8574(0);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LoRa Done");
  lcd.setCursor(0,1);
  lcd.print("LED  Next");
  gateway();

  intermissionPower();
  //ACTIVATE LED
  writePCF8574(LED_MOS);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LED - want 100-250lm");
  //digitalWrite(ledPin, HIGH);
  delay(10000);
  //DEACTIVATE LED
  writePCF8574(0);
  //digitalWrite(ledPin, HIGH);
  lcd.clear();
    lcd.setCursor(0,0);
  lcd.print("LED  Done");
  lcd.setCursor(0,1);
  lcd.print("All Next");
  gateway();

  intermissionPower();
  //ACTIVATE ALL
  writePCF8574(LED_MOS + LoRa_MOS + GPS_MOS + SONIC_MOS);
  delay(10000);
  //DEACTIVATE ALL
  writePCF8574(0);
  //digitalWrite(ledPin, HIGH);
  lcd.clear();
    lcd.setCursor(0,0);
  lcd.print("ALL  Done");
  lcd.setCursor(0,1);
  lcd.print("FINISHED");
  gateway();


}

void setup() {
  while(true){
    powerMeasure();   
  }
  
  setupComponents();
  Wire.begin();
  //wdt_disable();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);



  // Update WDT before it triggers
  wdt_enable(WDTO_8S);  //# HERE TO INSERT DIFFERENT SLEEP TIME
  // Enable the Watchdog Timer interrupt
  WDTCSR |= 1 << WDIE;



  //enable pin interrupt
  //PCICR |= 1 << PCIE0;
  //PCMSK0 |= 1 << PCINT0;

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

void Active() {
  //NOTE - might need the LED pin to be driven by the GPIO EXTENDER
  LightOrDark();  //checks whenever light or dark, and write
  detectedObject();
}
void buttonPressed() {}

void emergency() {
  Active();         //it should still light up
  buttonPressed();  //TO BE IMPLEMENTED - we are still waiting for this function
}

void sleep() {
}

void mainChain() {
  switch (getState()) {
    case 0:  //not long break, just standby - emergency
      emergency();
      break;
    case 1:  //sleeps for a long time, but standby - Active
    case 2:  //sleeps for short time, but deepsleep - Active
      Active();
      break;
    case 3:  //sleep for long time and deepsleep - Sleep
      sleep();
      break;
  }
}

void loop() {
  mainChain();
  sleep_cpu();
}



/*******************************
The interrupt handlers button
J-IMPLEMENT as interrupt
J-FIX tone notone
J-IMPLEMENT blinking cane? (more visible)
********************************/
void onOffButtonHandler() {
  // button handling
  int buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {


    //SET APPROPIATE STATE -
    //first bit is the length of sleep. second is the sleep mode. Emergency is 00, idle is 11, and  active is 10
    if (getState() == 2) {
      setState(3);
    } else {
      setState(2);
    }



  } else {
    //#! if not pressed do nothing? maybe have normal behavior run here?
    //#-J no need - it will be an interrupt. However, we could consider instead having the interrupt start GNSS'es cold/warm/hot start, initialize the LoRa, and then afterwards evaluate whenever GNSS is done. otherwise, it sets the appropiate state, and goes back to sleep
    //#-J might be easier to talk about than read.
  }
  sleep_cpu();
}



/*TURN INTO A INTERRUPT, HANDLED BY SPECIFIC BUTTON*/
void emergencyButtonHandler() {
  // button handling
  int buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {


    //SET APPROPIATE STATE -
    //first bit is the length of sleep. second is the sleep mode. Emergency is 00, idle is 11, and  active is 10
    setState(0);
    buttonPressed();

    tone(speakerPin, 1000);  // Start 1 KHz tone for 10 secs
    //TEST IF DEEPSLEEP CAN DO STUFF
    delay(10000);        //#!-J this delay is good, as a constant siren would get tedious - but maybe it ought to be one that you turn off by pressing the button again.
                         //#+J this would require more coordination with deepsleep though.
    noTone(speakerPin);  // Stop tone

  } else {
    //#! if not pressed do nothing? maybe have normal behavior run here?
    //#-J no need - it will be an interrupt. However, we could consider instead having the interrupt start GNSS'es cold/warm/hot start, initialize the LoRa, and then afterwards evaluate whenever GNSS is done. otherwise, it sets the appropiate state, and goes back to sleep
    //#-J might be easier to talk about than read.
  }
  sleep_cpu();
}



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!it does not like this one
ISR(WDT_vect) {
  // Reset WDIE to 1
  WDTCSR |= 1 << WDIE;
}

// NEW ISR
// Would it not be better to just have a flag here and handle based on flag in main loop???
void wakeButton() {
  //if on/off button
  onOffButtonHandler();
  // else if Emergency Button
  emergencyButtonHandler();
}

/*ISR(PCINT0_vect) {
  //if on/off button
  onOffButtonHandler();
  // else if Emergency Button
  emergencyButtonHandler();
}*/
