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

SoftwareSerial mySerial(10, 11); // RX, TX
rn2xx3 myLora(mySerial);

const int led = 5;
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

  // Setup LoraWan uplink
  // Open serial communications and wait for port to open:
  Serial.begin(57600); //serial port to computer
  mySerial.begin(9600); //serial port to radio
  Serial.println("Startup");
  initialize_radio();
  myLora.tx("TTN Mapper on TTN Enschede node");
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
    notone(speaker); // Stop tone, might kill distress sound
  }

  // button handling
  buttonState = digitalRead(button);
  if(buttonState == HIGH){
    // if pressed send SOS, also needs GPS coords!!!
    Serial.println("TXing");
    myLora.tx("SOS");
    // Also sound speaker?
    tone(speaker, 1000); // Start 1 KHz tone for 10 secs
    delay(10000);
    notone(speaker); // Stop tone
  }else{
    // if not pressed do nothing? maybe have normal behavior run here?
  }

  // Calc battery voltage
  int rawVolt = analogRead(voltPin);
  float VoutVolt = (rawVolt / 1023.0) * Vcc;
  float Vcap = VoutVolt * (R1 + R2) / R2;

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
  delay(1000);
}

void initialize_radio(){
  //reset rn2483
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(500);
  digitalWrite(12, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  mySerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16){
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(myLora.hweui());
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  // CHANGE THIS TO OUR APP EUI AND APP KEY!!!
  const char *appEui = "0000000000000000";
  const char *appKey = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

  join_result = myLora.initOTAA(appEui, appKey);

  while(!join_result){
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
}