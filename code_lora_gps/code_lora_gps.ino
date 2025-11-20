#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <stdint.h>
#include <math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Define GPS Module RX/TX Pins and Baud Rate
static const int RXPinGPS = 4, TXPinGPS = 5; //On the actual module are: RXPinGPS = TX on the module, TXPinGPS = RX on the module
static const uint32_t GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Initialize GPS serial communication
SoftwareSerial gps_serial(RXPinGPS, TXPinGPS);

// Define LoRa Module RX/TX Pins and Baud Rate
static const int RXPinLORA = 10, TXPinLORA = 11;
static const uint32_t LORABaud = 9600;

// Initialize LORA serial communication
SoftwareSerial lora_serial(RXPinLORA, TXPinLORA); // RN2483: Pro Mini D10=TX->RN RX, D11=RX->RN TX
rn2xx3 myLora(lora_serial);

// Initialize random global location coordinates
float LAT_DEG = 56.174734f;
float LON_DEG = 13.586034f;

// Helpers function
static inline void encodeInt32BE(int32_t value, uint8_t* out) {
  out[0] = (uint8_t)((value >> 24) & 0xFF);
  out[1] = (uint8_t)((value >> 16) & 0xFF);
  out[2] = (uint8_t)((value >> 8)  & 0xFF);
  out[3] = (uint8_t)( value        & 0xFF);
}

static inline void led_on()  { digitalWrite(13, HIGH); }
static inline void led_off() { digitalWrite(13, LOW);  }

// Global singla received variable
bool signal_received = false;

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

void setup() {
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

void buttonPressed(){
  get_location(); // Get current location
  signal_received = false; // Initialize signal receival as False
  sendAlertPacket(true, LAT_DEG, LON_DEG); // Send location and alert
  delay(1000);
  // Send an uplink message until the caregiver confirms acknowledgment of the alert
  // This is needed because LORA can receive downlink only after an uplink
  while (signal_received == false){ 
    sendAlertPacket(false, LAT_DEG, LON_DEG);
    delay(1000);
  }
}

void loop() {
  // Simulate button press (every 60s):
  // Get location from GPS module and send it to TTN.
  // If a downlink confirmation is recived, we will display the message (and tigger the buzzer) as well.
  
  buttonPressed();
  delay(60000);
}
