// RC car motor test - ESP32 + DRV8833
// Wiring: AIN1=D18, AIN2=D19, BIN1=D21, BIN2=D22, STBY=D23, GND=GND

const int AIN1 = 18;  // drive motor
const int AIN2 = 19;
const int BIN1 = 21;  // steering motor
const int BIN2 = 22;
const int STBY = 23;  // driver enable

void stopAll() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void pulse(int pin, int ms, const char* label) {
  Serial.println(label);
  digitalWrite(pin, HIGH);
  delay(ms);
  stopAll();
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  stopAll();
  digitalWrite(STBY, HIGH);   // wake the driver
  Serial.println("Motor test starting");
}

void loop() {
  pulse(AIN1, 1500, "Drive: AIN1");
  pulse(AIN2, 1500, "Drive: AIN2");
  pulse(BIN1, 500,  "Steer: BIN1");
  pulse(BIN2, 500,  "Steer: BIN2");
  delay(2000);
}