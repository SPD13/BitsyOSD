#include <RH_ASK.h>
#include <SPI.h>

//Configuration
//#define SIMULATE //Only enable when in simulation mode: sends a new lap every 5s
#define PILOT_ID 1
#define TX_PIN 11
#define BURST_NUM 10 //Number of burst to send
#define BUTTON1_PIN 2
#define BUTTON1_RF_COMMAND 1
#define BUTTON2_PIN 3
#define BUTTON2_RF_COMMAND 2
#define LED1_PIN 5
#define LED2_PIN 6
#define BUZZER_PIN 8
#define BATTERY_PIN A1
#define BATTERY_WARN_VOLT 3.4 //Warning voltage for battery
#define BATTERY_CHECK_TIME_S 10 //Check battery voltage every X s
#define BUTTON_DEBOUNCE_DELAY 100 //delay in MS to debounce button press

RH_ASK driver(2000, 2, TX_PIN, 5); //Not receiving, just transmitting

int seq_id = 1;
int buttonState[2]; // the current reading from the input pin
int lastButtonState[2] = {LOW, LOW};
unsigned long lastDebounceTime[2] = {0,0};  // the last time the output pin was toggled
unsigned long last_battery_check_time = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(9600); // Debugging only
  if (!driver.init())
      Serial.println("init failed");
  digitalWrite(LED1_PIN, HIGH);
}

void loop() {
  digitalWrite(LED1_PIN, HIGH);
  checkButton1State(0, BUTTON1_PIN, BUTTON1_RF_COMMAND, 0);
  checkButton1State(1, BUTTON2_PIN, BUTTON2_RF_COMMAND, 0);
  //Battery check
  checkBattery();

  #ifdef SIMULATE
    send_rf_message(2, 0);
    delay(5000);
  #endif
}

void checkBattery() {
  if ((millis()-last_battery_check_time)/1000 > BATTERY_CHECK_TIME_S) {
    last_battery_check_time = millis();
    int sensorValue = analogRead(BATTERY_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    Serial.print("Read Battery voltage ");
    Serial.println(voltage);
    if (voltage <= BATTERY_WARN_VOLT && voltage > 0) {
      Serial.println("Battery voltage warning");
      for (int i=0; i<4; i++) {
        digitalWrite(LED1_PIN, LOW);
        tone(BUZZER_PIN, 1000);
        delay(200);
        digitalWrite(LED1_PIN, HIGH);
        noTone(BUZZER_PIN);
        delay(200);
      }
    }
  }
}

void checkButton1State(int button_id, int buttton_pin, uint8_t rf_command, uint8_t rf_parameter) {
   // read the state of the switch into a local variable:
  int reading = digitalRead(buttton_pin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState[button_id]) {
    // reset the debouncing timer
    lastDebounceTime[button_id] = millis();
  }

  if ((millis() - lastDebounceTime[button_id]) > BUTTON_DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState[button_id]) {
      buttonState[button_id] = reading;

      // only toggle the buttton if the new button state is HIGH
      if (buttonState[button_id] == HIGH) {
          Serial.print("BUTTON ");
          Serial.print(button_id+1);
          Serial.println(" pressed");
        send_rf_message(button_id, rf_command, rf_parameter);
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState[button_id] = reading;
}

void send_rf_message(int button_id, uint8_t rf_command, uint8_t rf_parameter) {
    //Light leds
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  seq_id = seq_id + 1;
  for (int i=0; i<BURST_NUM; i++) {
    uint8_t buf[4];
    buf[0] = seq_id;
    buf[1] = PILOT_ID;
    buf[2] = rf_command;
    buf[3] = rf_parameter;
    driver.send(buf, strlen(buf));
    driver.waitPacketSent();
    delay(10);
  }
  //Stop leds
  Serial.print("Command ");
  Serial.print(rf_command);
  Serial.println(" sent");
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED1_PIN, HIGH);
  //Sound confirmation
  if (button_id == 0) {
    tone(BUZZER_PIN, 1000);
  }else if (button_id == 1) {
    tone(BUZZER_PIN, 500);
  }
  delay(1000);
  noTone(BUZZER_PIN);
}
