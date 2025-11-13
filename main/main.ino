// PIN Allocation:
// LoraWan (RN2483A) = D10, D11, D12 (As previusly used in class)
// GPS Tx & Rx = D3, D4
// LED(s) = D5
// Speaker = D6
// SWITCH (Button) = D7
// Ultra sonic = D8, D9
// Photocell = A0
// Battery voltage sensor = A1
// GPIO Extender (maybe redundant) = A4, A5

const int led = 5;
const int speaker = 6;
const int SWITCH = 7;
const int photoRes = A0;
const int voltPin = A1;

const float Vcc = 3.3;
const float R1 = 10000.0;   
const float R2 = 10000.0;   

int switchState = 0;

int photoVal = 0; // 0 to 1023

void setup() {
  // Setup pins
  pinMode(led, OUTPUT);
  pinMode(speaker, OUTPUT);
  pinMode(SWITCH, INPUT);
  pinMode(photoRes, INPUT);
  pinMode(voltPin, INPUT);
}

void loop() {

  // Switch
  switchState = digitalRead(SWITCH);
  if(switchState == HIGH){
    // if pressed send SOS? Also sound speaker?
    tone(speaker, 1000); // Start 1 KHz tone
    delay(1000);
    notone(speaker); // Stop tone
  }else{
    // if not pressed do nothing?
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

  // Go sleep?
  delay(1000);
}
