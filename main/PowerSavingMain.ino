//#################################################
//# Includes                                 ¤FLS #
//#################################################

/***
 *
 * Name: PowerSaving main
 * Author: primarily Julian.T.D (we were not able to 
      split files up into separate ones, so apologies 
      for the large file size)
 * date made: second week of the project, however it
      became its own thing on the day we bricked the 
      first and second arduino pro mini - at which
      point we split up into the functionality main
      and power saving main
 * date changed: 09/12/2025 (adding comments and 
      table of content)
 * Description: This file has powerMeasure() and mainChain(). 
      The first individually activates and/or uses the 
      I/Os on at a time and uses the button to allow the user
      to assess each component individually, before moving on
      to the next. an LCD screen prints some status info, but
      another version called powerMeasureNoLCD excludes it.
      Keep in mind it will still draw power untill you unplug it!
      
      The second is the main function that the cane will cycle
      through everytime it wakes up. This could be put in the WDT
      handler  (ISR(WDT_vect)) - but it also works when put in loop()
      as the WDT handler returns to, and not setup. currently in 
      active mode, it will just check the light level and turn on or 
      off the LED depending on its readings, then check 
      Distance sensor, and beep if it detects an object within a range. 
      Light is adjusted for demo, and distance sensor is 
      not attuned at all (but still works for demo).
      It will also check whenever a button is currently being held down, 
      and if so it will switch from active to idle, or idle to active.
    

 * Limitations: the buttons seem to be unpredictable, so we bypass by 
      applying VCC to the resistor connected to the digital pin, 
      everytime we want to be "pressing" it.

      The button(s) were supposed to be two buttons, one for emergency,
      and one for turning on and off. however, there were more pressing 
      matters to attend.

      As the GPIO extender was excluded from the last design (assumed 
      culprit for bricking two Arduino pro mini - though in hindsight,
      it was more likely that it worsened the issue, rather than cause it)
      it meant we did not have enough pins to drive a MOSFET for the 
      distance sensor. Additionally, without GPIO extender not be able 
      to perserve the MOSFET gate signals for the components through
      deepsleep.
      this meant they would always be turned off, and we would have to
      reinitialize all components again (see ¤LOP and ¤CSF). 
      There were more pressing matters, so we never confirmed 
      functionality past ensuring that ACTIVE and IDLE/SLEEP* mode worked
        *not to be confused with "sleep" as in 
        "sleepmode/deepsleep/power down/standby"

      we faced some issues with softwareSerial, blocking us from using 
      ISR(PCINT0_vect(), or any of the other vectors. this is why the
      button pin is not part of PCINT0's domain in this version (d13)


 * Warnings: this file is to be used on the
      Arduino Uno - as the Arduino Pro Mini bricks
      for unknown reasons when connected to the circuit
      while running this code. 
      this could possibly be fixed by redesigning the circuit to 
      not have GND and VCC on opposite ends of the I/O components on 
      the breadboard, while also having VCC and GND right next to 
      one another. (the current circuit have both of these issues - 
      though to my understanding we want the former, to lower noise)
      Another option is to have the power supply be supplied by a battery, 
      rather than the Arduino Pro Mini, and then run the 3V3 voltage 
      into its Raw pin.

  * retrospect/ what went wrong:
      we believe that as power hungry components (GPS+LoRa) are cut off or reattached 
        to the circuit, then in the slight amount of time, in which the 
        circuit adapts to the new draw, it experience excessive or insufficient 
        voltage and current, causing the voltage to potentially raise beyond 
        what the components can take, and what the Arduino pro mini can survive
        - as we run the power supply into its vcc, rather than through RAW. 
        (at least thats what we think - looking at old pictures, i cannot 
        see that we first run it into the VCC and then into the circuit... instead
        we run the power from ISParp into the VCC trace of the breadboard, 
        which goes into all components, including the VCC of the arduino pro mini)
        This is however odd, as any microcontroller should have decoupling capacitors, 
        but who knows - maybe the very noise heavy system, causes the unstable period 
        to extend beyond the capacitors capacitance.

        This would also explain why the Arduino Uno survives, because it can take 5V, 
        and it is the one that supplies the 3V3 volt... and it likely has more 
        fail-safe features.

 *
 *
 */


/**
 * Table of content
 *      //FILE HEAD                     ¤FLS
 *      //INCLUDES                      ¤INC
 *      //VARIABLES, CONST, STRUCT      ¤VCS
 *      //GPS AND LoRa FUNCTIONS        ¤GLF
 *      //COMPONENT SETUP FUNCTIONS     ¤CSF
 *      //-||- INTERRACTION FUNCTIONS   ¤CIF
 *      //GPIO EXTENDER                 ¤GEX
 *      //Deepsleep mode extractor      ¤DSX
 *      //DEBUGGING FUNCTIONS           ¤DBG
 *      //SETUP FUNCTION                ¤STP
 *      //SLEEPMODE MAIN                ¤SMM
 *      //LOOP FUNCTION                 ¤LOP
 *      //INTERRUPT AND WDT FUNCTIONS   ¤ITR
 * */
//#################################################
//# Includes                                 ¤INC #
//#################################################
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <stdint.h>
#include <math.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
//#################################################
//# Variables, Constants and structs         ¤VCS #
//#################################################

//// PIN LAYOUT ////
/*
D2 - LED
D3 - GPS
D4 - LoRa
D5&6 -GPS
D7 - Speaker
D8&9 - Ultra sonic
D10&11&12 - LoRa
D13 - button
A0 - Photocell
A1 - battery voltage sensor - NOT IMPLEMENTED 
  (AND LIKELY NOT NEEDED UNLESS WE WANT TO 
  WARN OF LOW BATTERY)
A2 - Button extra - NOT IMPLEMENTED
*/ 
#define LED_MOS_P 2
#define GPS_MOS_P 3
#define LoRa_MOS_P 4
#define GPS_P_RX 5
#define GPS_P_TX 6
#define SPE_P 7
#define US_P_ECH 8
#define US_P_TRI 9
#define LoRa_P_RX 10
#define LoRa_P_TX 11
#define LoRa_P_RST 12
#define BTN_P 13
#define Photo_P 14 //a0
//#define BVS_P 15 //a1

//// DEBUG STUFF ////


//used for setting up LCD - used for debugging
LiquidCrystal_I2C lcd(0x27, 16, 2);

//// MOSFET STUFF ////
//values written to GPIO extender - corresponds to a bitmask (128 being MSb)
//1, 2, and 4 correspond to LEDs for debugging 
  //(not used anymore, but helped locate which state we were in).
//8 is not used
#define GPS_MOS 16
#define LoRa_MOS 32 //do not mind that the GPS and LoRa here are 
  //listed out of order compared to _P (pin number).
#define LED_MOS 64
#define SONIC_MOS 128








//// GPS+LoRa STUFF ////
// Define GPS Module RX/TX Pins and Baud Rate
static const int RXPinGPS = 5, TXPinGPS = 6;  //On the actual module are: 
  //RXPinGPS = TX on the module, TXPinGPS = RX on the module
static const uint32_t GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Initialize GPS serial communication
SoftwareSerial gps_serial(RXPinGPS, TXPinGPS);

// Define LoRa Module RX/TX Pins and Baud Rate
static const int RXPinLORA = 10, TXPinLORA = 11;
static const uint32_t LORABaud = 9600;

// Initialize LORA serial communication
SoftwareSerial lora_serial(RXPinLORA, TXPinLORA);  // RN2483: 
              //Pro Mini D10=TX->RN RX, D11=RX->RN TX
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

/*Would technically work - but should be standardized with writePCF8574() 
  - to turn on and off LED appropiately, and ensure seamless transition 
  between with and without GPIO extender*/
static inline void led_on() {
  digitalWrite(LED_MOS_P, HIGH);
}
static inline void led_off() {
  digitalWrite(LED_MOS_P, LOW);
}


// Global singla received variable

bool signal_received = false;

//// PIN LAYOUT 2 ////
    // the second one - Because we still have not consolidated into one

    //NOT USED -> #define PCF8574_ADDR 0x20  // Standardadresse (kan variere afhængigt af A0, A1, A2)

const int ledPin = 2;  // this can be handled by the GPIO extender
const int speakerPin = 7;
const int buttonPin = 13;
// this one can be turned on and off by IO Extender
const int USEcho = 8;
const int USTrig = 9;
const int LoRaResetPin = 12;

const int photoRes = A2;
const int voltPin = A1;

//placeholder constants for battery 
  //- not implemented outside of code due to lack of hardware
const float Vcc = 3.3;
const float R1 = 10000.0;
const float R2 = 10000.0;



//Because of deepsleep resetting memory, we cannot use these variables.
//float timing = 0.0;
//float distance = 0.0;
//int buttonState = 0;
//int photoVal = 0; // 0 to 1023


//#################################################
//# GPS, LoRa, and that sort of thing        ¤GLF #
//#################################################
//Did not have much to do with this aspect,
  //so have neglected commenting any of this section
//LoRaWAN init
void initialize_radio(int i) {
  lora_serial.listen();  // Listen to the LORA Serial
  pinMode(LoRa_P_RST, OUTPUT);   // RN2483 RESET
  digitalWrite(LoRa_P_RST, LOW);
  delay(500);
  digitalWrite(LoRa_P_RST, HIGH);

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
  Serial.println(hweui); // 0004A30B01105AD3
  Serial.print("FW: ");
  Serial.println(myLora.sysver());
  Serial.println("Joining TTN (OTAA)...");

  // AppEUI/AppKey from TTN
  const char *appEui = "0004A30B01068CD7";
  const char *appKey = "F1F4FB8E0D22788C38FE41D7E1AAE9C6";

  bool joined = myLora.initOTAA(appEui, appKey);
  while (!joined && i != 0) {
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
        Serial.print(LAT_DEG);
        Serial.print(LON_DEG);
        sent_loc = true;
      }
    }
  }
}

//#################################################
//# Component setup functions                ¤CSF #
//#################################################

// The components setup function. powerTest indicates 
  //- if 1 - that we are only checking power, and thus
  //should not do the radio_initialize: it will be 
  //called later in powerMeasurements, without ensurance 
  //of a connection being established. (otherwise we would
  //have a softlock, as we do not have the APP or backup
  //LoRa module mapped to the backend or connection service)
void setupComponents(int powerTest) {
  pinMode(LED_MOS_P, OUTPUT);
  pinMode(GPS_MOS_P, OUTPUT);
  pinMode(LoRa_MOS_P, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(USEcho, INPUT);
  pinMode(USTrig, OUTPUT);
  pinMode(photoRes, INPUT_PULLUP);
  pinMode(voltPin, INPUT);
  digitalWrite(LED_MOS_P,HIGH);
    digitalWrite(GPS_MOS_P,HIGH);
      digitalWrite(LoRa_MOS_P,HIGH);
  if(powerTest){ //here ends the setup for power measurements
    return;
  }

  // GPS INIT //
  // Set Buildin LED to Output mode
  //pinMode(13, OUTPUT);

  // Initialize Serial Communication
  //Serial.begin(57600);

  // Initialize LORA and GPS Serial
  lora_serial.begin(LORABaud);
  gps_serial.begin(GPSBaud);

  // Print feedback to the user
  Serial.println("Startup");

  // Initialize radio/lora
  initialize_radio(1);
 // Wait 1s
  delay(1000);
  // Send an initial false alert
  sendAlertPacket(false, LAT_DEG, LON_DEG);
  // Wait 15s. Simulate that the user did not pressed the button yet
  delay(15000);

}
//#################################################
//# Component Interraction functions         ¤CIF #
//#################################################
//US = Ultra Sonic - measures distance
float USMeasurement() {
  //trigger pin activates the sensor.
  digitalWrite(USTrig, LOW);
  delay(2);
  digitalWrite(USTrig, HIGH);
  delay(10);
  digitalWrite(USTrig, LOW);
  //as we are not using SDA SCL, the echo pin stays HIGH 
    //for the entire measuring period, and will go LOW 
    //when it gets a reading back.
  float timing = pulseIn(USEcho, HIGH);
  float distance = (timing * 0.0343) / 2;  // distance in cm - float
  return distance;
}
//parent of the above function - and will play the tone when 
  //there is an object that is too close.
void detectedObject() {
  if (USMeasurement() < 20) {  // check if accuracy remains at close range
    tone(speakerPin, 500);     // Start 500 Hz tone
  } else {
    noTone(speakerPin);  // Stop tone, might kill distress sound
  }
  delay(500);
}

/*******************************
Battery Voltage Calculator
********************************/
// As we did not implement the hardware for this, i have not commented on this.
float calculateBatteryVoltage() {
  // Calc battery voltage
  int rawVolt = analogRead(voltPin);
  float VoutVolt = (rawVolt / 1023.0) * Vcc;
  float Vcap = VoutVolt * (R1 + R2) / R2;
  return VoutVolt;  //#!-J or should we return Vcap?
}
/*******************************
Photoresistor
********************************/
// Has been modified, to have a "do not change" zone between light and dark 
  //- so the cane does not constantly switch on and off during twilight
int LightOrDark() {
  // Handle PhotoResistor behavior
  int photoVal = analogRead(photoRes);  //Update to GPIO extender
  Serial.println(photoVal);
  if (photoVal < 100) {                 // Need to figure out how value relates to light level
    // It's dark, turn off LED(s)?
    return 0;  //the new turn off SIGNAL (for writePCF8574)PO
  } else if (photoVal > 80) {

    // It's light, turn on LED(s)?
    return LED_MOS;  //the new turn off SIGNAL (for writePCF8574)
  }
  return;
}
//#################################################
//# GPIO Extender                            ¤GEX #
//#################################################
//It is worth noting, that we did not implement the read function, 
  //and that the write function has been modified to interract
  //with each MOSFET separately, as we no longer use the 
  //GPIO Extender
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
  /*
  Wire.requestFrom(PCF8574_ADDR, 1);
  while(Wire.available()) {
      return Wire.read();    // Receive a byte as character
  }*/
}
//write
void writePCF8574(byte data) {
  if(data & 16){
    digitalWrite(GPS_MOS_P, LOW);
  }
  else {
    digitalWrite(GPS_MOS_P, HIGH);
  }
  if(data & 32){
    digitalWrite(LoRa_MOS_P, LOW);
  }
  else {
    digitalWrite(LoRa_MOS_P, HIGH);
  }
  if(data & 64){
    digitalWrite(LED_MOS_P, LOW);
  }
  else {
    digitalWrite(LED_MOS_P, HIGH);
  }
  /*Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(~data);  // Inverter output: LOW tænder LED
  Wire.endTransmission();*/
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
//#################################################
//# Mode extractor from DEEPSLEEP and WDT   ¤DSX  #
    //settings                                    #
//#################################################
//As deepsleep clears memory, we need a way to store 
  //our canes current mode (Active, Emergency, Sleep/idle)
  //so, we store it as the deepsleep and WDT settings ;)
  //this does mean that new sleep settings require these 
  //functions to be updated, so their output/input still
  //are 0,1 and 3.
/*******************************
logic for determining and setting state for next wake up.
********************************/
int getState() {
  uint8_t wdtTSetting = WDTCSR & 0x27;
  uint8_t sleepMode = _SLEEP_CONTROL_REG & 0x0E;  //sleep modes: 
    //000 = idle, 
    //001 = ADC Noise rEDUCTION, 
    //010 = Power-down, 
    //011 = Power-save, 
    //110 = Standby, 
    //111 = Extended Standby
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
  //writePCF8574(1 << (result - 1));  // Kun én LED tændt ad gangen
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




//#################################################
//# DEBUGGING FUNCTIONS                     ¤DBG  #
//#################################################
//These functions are both for debugging, and for 
  //measuring power - apologies for the misleading
  //name.
/*******************************
DEBUGGING FUNCTIONS
********************************/
//THIS FUNCTION MUST BE DELETED - NO PURPOSE
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
//Used to indicate that we are now moving on to 
  //the next part of the measuring process
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

//The button checkpoint in relation to
  //measuring power of each component
void gateway(){
  //ALTERNATIVE TO THE /*  // */ trick, you could use ifdef DEBUG 
    //to use or hide these segments. (would make it easier to 
    //turn on and off debug mode)
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



//  NOW COMES TWO IDENTICAL FUNCTIONS - 
  //one is with LCD. the other without.
  //both are for measuring power.
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
  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Buzzer tone 2");
  tone(speakerPin, 1000);
  delay(4000);
  //DEACTIVATE buzzer
  noTone(speakerPin);  // Stop tone, might kill distress sound
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Buzzer done2");
  lcd.setCursor(0,1);
  lcd.print("Distance Next");
  gateway();

  intermissionPower();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("distance sensor");
  //ACTIVATE distance sensor - not necessary - buuuut
  writePCF8574(SONIC_MOS);
  delay(1000);
  for (int i = 0; i < 100; i++) {
    USMeasurement();
  }
  delay(1000);
  //DEACTIVATE distance sensor
  writePCF8574(0);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Distance Done");
  lcd.print("Distance Done");
  lcd.setCursor(0,1);
  lcd.print("GNSS Next");
  gateway();
  
  intermissionPower();
  //ACTIVATE GNSS
  writePCF8574(GPS_MOS);
  delay(10000);
  gps_serial.begin(GPSBaud);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("GNSS module:");
  for (int i = 0; i > 100; i++) {
    get_location();
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
  delay(100);
  lora_serial.begin(LORABaud);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LoRa Module:");
  initialize_radio(0);
  for (int i = 0; i > 100; i++) {
    sendAlertPacket(false, 0, 0);
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

void powerMeasureNoLCD() {
  pinMode(buttonPin,INPUT);
  
  //setupComponents();
  Wire.begin();
  //setupLCD();
  delay(100);

  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("READY?");
  gateway();

  //REMEMBER to set mosfets with GPIO extender!
  
  intermissionPower();
  //ACTIVATE buzzer
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Buzzer tone 1");
  tone(speakerPin, 500);
  delay(4000);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Buzzer tone 2");
  tone(speakerPin, 1000);
  delay(4000);
  //DEACTIVATE buzzer
  noTone(speakerPin);  // Stop tone, might kill distress sound
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Buzzer done2");
  //lcd.setCursor(0,1);
  //lcd.print("Distance Next");
  gateway();

  intermissionPower();
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("distance sensor");
  //ACTIVATE distance sensor - not necessary - buuuut
  writePCF8574(SONIC_MOS);
  delay(1000);
  for (int i = 0; i < 100; i++) {
    USMeasurement();
  }
  delay(1000);
  //DEACTIVATE distance sensor
  writePCF8574(0);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("Distance Done");
  //lcd.print("Distance Done");
  //lcd.setCursor(0,1);
  //lcd.print("GNSS Next");
  gateway();
  
  intermissionPower();
  //ACTIVATE GNSS
  writePCF8574(GPS_MOS);
  delay(10000);
  gps_serial.begin(GPSBaud);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("GNSS module:");
  for (int i = 0; i > 100; i++) {
    get_location();
  }
  delay(4000);
  //DEACTIVATE GNSS
  writePCF8574(0);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("GNSS Done");
  //lcd.setCursor(0,1);
  //lcd.print("LoRa Next");
  gateway();
  
  intermissionPower();
  //ACTIVATE LoRa
  writePCF8574(LoRa_MOS);
  delay(100);
  lora_serial.begin(LORABaud);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("LoRa Module:");
  initialize_radio(0);
  for (int i = 0; i > 100; i++) {
    sendAlertPacket(false, 0, 0);
  }
  delay(4000);
  //DEACTIVATE LoRa
  writePCF8574(0);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("LoRa Done");
  //lcd.setCursor(0,1);
  //lcd.print("LED  Next");
  gateway();

  intermissionPower();
  //ACTIVATE LED
  writePCF8574(LED_MOS);
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("LED - want 100-250lm");
  //digitalWrite(ledPin, HIGH);
  delay(10000);
  //DEACTIVATE LED
  writePCF8574(0);
  //digitalWrite(ledPin, HIGH);
  //lcd.clear();
  //  lcd.setCursor(0,0);
  //lcd.print("LED  Done");
  //lcd.setCursor(0,1);
  //lcd.print("All Next");
  gateway();

  intermissionPower();
  //ACTIVATE ALL
  writePCF8574(LED_MOS + LoRa_MOS + GPS_MOS + SONIC_MOS);
  delay(10000);
  //DEACTIVATE ALL
  writePCF8574(0);
  //digitalWrite(ledPin, HIGH);
  //lcd.clear();
  //  lcd.setCursor(0,0);
  //lcd.print("ALL  Done");
  //lcd.setCursor(0,1);
  //lcd.print("FINISHED");
  gateway();


}


//#################################################
//# SETUP FUNCTION                          ¤STP  #
//#################################################
void setup() {
  Serial.begin(9600);
  setupComponents(1);
  /*while(true){
    //powerMeasureNoLCD();   
    powerMeasure();   
  }
  
  setupComponents(0);*/
  Wire.begin();
  //wdt_disable();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);



  // Update WDT before it triggers
  wdt_enable(WDTO_4S);  //# HERE TO INSERT DIFFERENT SLEEP TIME
  // Enable the Watchdog Timer interrupt
  WDTCSR |= 1 << WDIE;



  //enable pin interrupt
  PCICR |= 1 << PCIE0;
  PCMSK0 |= 1 << PCINT0;

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


//#################################################
//# SLEEPMODE MAIN                          ¤SMM  #
//#################################################
//Contains all functions used in our functional loop
  //as can be seen, it is not alot, but it does not
  //need to be... all the complicated stuff is within
  //each function call. Note also, that we would have
  //to call setupComponents(0); rather than
  //setupComponents(1); in a final product - otherwise
  //the LoRa would not be connected... and we would
  //have to add some sort of timeout, so the cane is
  //not softlocked when outside the LoRa Range.
void Active() {
  //NOTE - might need the LED pin to be driven by the GPIO EXTENDER
  writePCF8574(LightOrDark());  //checks whenever light or dark, and write
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
  onOffButtonHandler();
}

// 
//#################################################
//# LOOP FUNCTION                           ¤LOP  #
//#################################################
void loop() {
  setupComponents(1); // should give 0 to indicate "not demo" - but as the functionality side uses the app and LoRa, we should just not setup a connection
  mainChain();
  sleep_cpu();
}


//#################################################
//# INTERRUPT AND WDT FUNCTIONS             ¤ITR  #
//#################################################
//As interrupt is blocked by SoftwareSerial, it is 
  //only WDT that can wake us up. once it completes, 
  //it returns to loop()
  //interrupt handler, should reset WDT and go back 
  //to sleep directly instead. - sleep time should be
  //lowered, so the user has an immediate feedback
  //to their press, and to ensure that the cane can
  //detect objects immediately as they come.
  //though the USMeasurement() function might need 
  //modifications, so that it returns automatically
  //after a certain timeframe has passed, ensuring
  //that it is not stuck awake for far too long, when
  //we can already confirm that there is no object
  //that the user would collide with.
/*******************************
The interrupt handlers button
J-IMPLEMENT as interrupt
J-FIX tone notone
J-IMPLEMENT blinking cane? (more visible)
********************************/
void onOffButtonHandler() {
  // button handling
  pinMode(buttonPin,INPUT);
  int buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    Serial.println("IT DID THE THING");

    //SET APPROPIATE STATE -
    //first bit is the length of sleep. second is the sleep mode. Emergency is 00, idle is 11, and  active is 10
    if (getState() == 2) {
      setState(3);
      sleepModeInit();

    } else {
      setState(2);
      activeModeInit();
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
  noTone(speakerPin);  // Stop tone, might kill distress sound
}

/*ISR(PCINT0_vect) {
  //if on/off button
  onOffButtonHandler();
  // else if Emergency Button
  emergencyButtonHandler();
}*/
