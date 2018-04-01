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

#define AVG_TEMP_NUM_READINGS 20
#define AVG_TEMP_READING_DELAY 50

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
  Serial.println(averagedTemp());
  delay(5000);
}

float averagedTemp() {
  float tempSum = 0;
  for(int i=0; i < AVG_TEMP_NUM_READINGS; i++) {
    tempSum += readTemp();

    stat1_led_on();
    delay(AVG_TEMP_READING_DELAY/2);
    stat1_led_off();
    delay(AVG_TEMP_READING_DELAY/2);
  }

  float avgTemp = tempSum/AVG_TEMP_NUM_READINGS;
  avgTemp = floor(avgTemp * 2 + 0.5)/2; // Round temperature to nearest 0.5
  
  return avgTemp;
}

float readTemp() {
  int rawValue = analogRead(TEMP_IN);
  
  float voltage = (rawValue / 1024.0) * float(ARDUINO_VOLTAGE);
  float temperature = (voltage - 0.5) / 0.01;
  
  return temperature;
}

void heatingOn() {
  digitalWrite(RELAY_OUT, HIGH);
  analogWrite(CH_LED_OUT, CH_LED_BRIGHTNESS);
}

void heatingOff() {
  digitalWrite(RELAY_OUT, LOW);
  digitalWrite(CH_LED_OUT, LOW);
}

void stat1_led_on() {
  analogWrite(STAT1_LED_OUT, STAT1_LED_BRIGHTNESS);
}

void stat2_led_on() {
  analogWrite(STAT2_LED_OUT, STAT2_LED_BRIGHTNESS);
}

void stat1_led_off() {
  digitalWrite(STAT1_LED_OUT, LOW);
}

void stat2_led_off() {
  digitalWrite(STAT2_LED_OUT, LOW);
}
