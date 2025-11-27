#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <stdint.h>
#include <math.h>
#include <TinyGPS++.h>

// PIN Allocation:
// LoraWan (RN2483A) = D10, D11, D12 (As previusly used in class)
// GPS Tx & Rx = D4, D5
// LED(s) = D3
// Speaker = D6
// SWITCH (Button) = D7
// Ultra Sonic = D8, D9
// Photocell = A0
// Battery voltage sensor = A1 (CURRENTLY NOT USED)
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

const int led = 3;
const int speaker = 6;
const int button = 7;
const int US_Echo = 8;
const int US_Trig = 9;
const int photoRes = A0;
const int voltPin = A1;

float timing = 0.0;
float distance = 0.0;

const float Vcc = 3.3;
const float R1 = 10000.0;   
const float R2 = 10000.0;   

int buttonState = 0;

int photoVal = 0; // 0 to 1023

void setup() {
  // Setup pins
  pinMode(led, OUTPUT);
  pinMode(speaker, OUTPUT);
  pinMode(button, INPUT);
  pinMode(US_Echo, INPUT);
  pinMode(US_Trig, OUTPUT);
  pinMode(photoRes, INPUT);
  pinMode(voltPin, INPUT);

  // GPS //
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

void loop() {
  // Ultrasonic behavior
  digitalWrite(US_Trig, LOW);
  delay(2);
  digitalWrite(US_Trig, HIGH);
  delay(10);
  digitalWrite(US_Trig, LOW);

  timing = pulseIn(US_Echo, HIGH);
  distance = (timing*0.0343)/2; // distance in cm

  // debug distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // handle sound if too close
  if(distance < 20){ // check if accuracy remains at close range 
    tone(speaker, 500); // Start 500 Hz tone
  }else{
    noTone(speaker); // Stop tone, might kill distress sound
  }

  // button handling
  buttonState = digitalRead(button);
  if(buttonState == HIGH){
    // Serial.print("BUTTON_DEBUG");
    // if pressed send SOS
    buttonPressed();
    // Also sound speaker?
    tone(speaker, 1000); // Start 1 KHz tone for 10 secs
    delay(2000);
    noTone(speaker); // Stop tone
  }else{
    // if not pressed do nothing? maybe have normal behavior run here?
  }

  // Calc battery voltage
  /*
  int rawVolt = analogRead(voltPin);
  float VoutVolt = (rawVolt / 1023.0) * Vcc;
  float Vcap = VoutVolt * (R1 + R2) / R2;
  */

  // Handle PhotoResistor behavior
  photoVal = analogRead(photoRes);
  if(photoVal < 512){ // Need to figure out how value relates to light level
    // It's dark, turn on LED(s)?
    digitalWrite(led, HIGH);
  }else{
    // It's light, turn off LED(s)?
    digitalWrite(led, LOW);
  }

  // Go sleep? Need to handle some kind of sleep behavior
  delay(4000);
}

/*******************************
GPS STUFF
********************************/
// LoRaWAN init
void initialize_radio() {
  lora_serial.listen(); // Listen to the LORA Serial
  pinMode(12, OUTPUT);          // RN2483 RESET
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

  Serial.print("DevEUI: "); Serial.println(hweui);
  Serial.print("FW: ");     Serial.println(myLora.sysver());
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
  lora_serial.listen(); // Listen to the LORA Serial
  // Convert to signed 32-bit microdegrees
  int32_t lat_i = (int32_t)lroundf(lat_deg * 1000000.0f);
  int32_t lon_i = (int32_t)lroundf(lon_deg * 1000000.0f);

  // Payload: [alert(1)][lat(4)][lon(4)]
  uint8_t payload[9];
  payload[0] = alert ? 1 : 0;
  encodeInt32BE(lat_i, &payload[1]);
  encodeInt32BE(lon_i, &payload[5]);

  Serial.print("TX alert="); Serial.print(alert ? "true" : "false");
  Serial.print(" lat=");     Serial.print(lat_deg, 6);
  Serial.print(" lon=");     Serial.println(lon_deg, 6);

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
  lora_serial.listen(); // Listen to the LORA Serial
  msg.trim();
  // If we receive: 01 = help confirmation
  if (msg == "01" || msg == "1") {
    Serial.println("Signal received. Help is coming!");
    signal_received = true;
    delay(1000);
    sendAlertPacket(false, LAT_DEG, LON_DEG);
    // buzzer will also be triggered here
  } else {
    Serial.print("Unknown downlink command: ");
    Serial.println(msg);
  }
}

// Get location
void get_location(){
  int sent_loc = false;
  while (sent_loc == false){
    gps_serial.listen(); // Listen to the GPS Serial
      while (gps_serial.available() > 0){
        gps.encode(gps_serial.read());
        if (gps.location.isUpdated()){
          LAT_DEG = gps.location.lat();
          LON_DEG = gps.location.lng();
          sent_loc = true;
        }
      }
  }
}

void buttonPressed(){
  get_location(); // Get current location
  signal_received = false; // Initialize signal receival as False
  sendAlertPacket(true, LAT_DEG, LON_DEG); // Send location and alert
  delay(10000);
  // Send an uplink message until the caregiver confirms acknowledgment of the alert
  // This is needed because LORA can receive downlink only after an uplink
  while (signal_received == false){ 
    sendAlertPacket(false, LAT_DEG, LON_DEG);
    delay(1000);
    break; // remove later
  }
}