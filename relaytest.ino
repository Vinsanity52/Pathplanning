const int RELAY_ENABLE_Pump = 2;
const int RELAY_ENABLE_Sol13 = 3;
const int RELAY_ENABLE_Sol24 = 4;
void setup() {
  // put your setup code here, to run once:
  
  pinMode(RELAY_ENABLE_Sol13, OUTPUT);
  pinMode(RELAY_ENABLE_Sol24, OUTPUT);
  pinMode(RELAY_ENABLE_Pump, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Relay ON");
  digitalWrite(RELAY_ENABLE_Sol13, LOW);
  digitalWrite(RELAY_ENABLE_Pump, LOW);
  delay(8000);

  Serial.println("Relay OFF");
  digitalWrite(RELAY_ENABLE_Sol13, HIGH);
  digitalWrite(RELAY_ENABLE_Pump, HIGH);
  delay(1000);

  Serial.println("Relay ON");
  digitalWrite(RELAY_ENABLE_Sol24, LOW);
  digitalWrite(RELAY_ENABLE_Pump, LOW);
  delay(8000);

  Serial.println("Relay OFF");
  digitalWrite(RELAY_ENABLE_Sol24, HIGH);
  digitalWrite(RELAY_ENABLE_Pump, HIGH);
  delay(1000);
}
