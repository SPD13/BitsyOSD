#define DEBOUNCE_TIME_MS 1000
int IRSensor = 2; // connect ir sensor to arduino pin 2
unsigned long last_trigger_time = 0;
int last_sensor_status = 0;

void setup() {
  pinMode (IRSensor, INPUT); // sensor pin INPUT
  Serial.begin(9600);
  Serial.println("Started");

}

void loop() {
  int statusSensor = digitalRead (IRSensor);
  
  if (statusSensor == 0 && last_sensor_status != statusSensor && millis()-last_trigger_time > DEBOUNCE_TIME_MS) {
    last_trigger_time = millis();
    last_sensor_status = 1;
    Serial.println("Triggered");
  }
  last_sensor_status = statusSensor;
}
