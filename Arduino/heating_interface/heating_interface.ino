  #define TEMP_IN 0
#define RELAY_OUT 2

#define CH_LED_OUT 3
#define CH_LED_BRIGHTNESS 50

#define PWR_LED_OUT 4
#define PWR_LED_BRIGHTNESS 128

#define STAT1_LED_OUT 5
#define STAT1_LED_BRIGHTNESS 255

#define STAT2_LED_OUT 6
#define STAT2_LED_BRIGHTNESS 255

#define ARDUINO_VOLTAGE 5

void setup() {
  pinMode(RELAY_OUT, OUTPUT);
  pinMode(CH_LED_OUT, OUTPUT);
  pinMode(PWR_LED_OUT, OUTPUT);
  pinMode(STAT1_LED_OUT, OUTPUT);
  pinMode(STAT2_LED_OUT, OUTPUT);
  Serial.begin(115200);
  analogWrite(PWR_LED_OUT, PWR_LED_BRIGHTNESS); 
}

void loop() {
  int rawValue = analogRead(TEMP_IN);
  
  float voltage = (rawValue / 1024.0) * float(ARDUINO_VOLTAGE);
  float temperature = (voltage - 0.5) / 0.01;
  
  Serial.println(voltage);
  Serial.println(temperature);
  delay(500);
  digitalWrite(RELAY_OUT, HIGH);
  analogWrite(CH_LED_OUT, CH_LED_BRIGHTNESS);
  analogWrite(STAT1_LED_OUT, STAT1_LED_BRIGHTNESS);
  digitalWrite(STAT2_LED_OUT, LOW);
  delay(500);
  digitalWrite(RELAY_OUT, LOW);
  digitalWrite(CH_LED_OUT, LOW);
  analogWrite(STAT2_LED_OUT, STAT2_LED_BRIGHTNESS);
  digitalWrite(STAT1_LED_OUT, LOW);
}
