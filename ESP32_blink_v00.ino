// Board: "DOIT ESP32 DEVKIT V1"
// Board: "ESP32 Dev Module"

const int onboard_ledPin = 2;
int icount;

void setup() {
  pinMode(onboard_ledPin, OUTPUT); // initialize the LED pin as an output
  Serial.begin(115200);
  icount = 0;
  Serial.println();
  Serial.println("Start blink test");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(900);
  digitalWrite(onboard_ledPin, HIGH);  // Turn the LED on by setting the voltage HIGH
  delay(100);
  digitalWrite(onboard_ledPin, LOW);  // Turn the LED off by setting the voltage LOW
  Serial.println(icount++);
}
