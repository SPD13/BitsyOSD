#include <RH_ASK.h>
#include <SPI.h>

//Configuration
#define PILOT_ID 1
#define TX_PIN 11
#define BURST_NUM 10 //Number of burst to send
#define BUTTON1_PIN 3
#define BUTTON1_RF_COMMAND 1
#define BUTTON2_PIN 4
#define BUTTON2_RF_COMMAND 2
#define LED1_PIN 5
#define LED2_PIN 6
#define BUTTON_DEBOUNCE_DELAY 50 //delay in MS to debounce button press

RH_ASK driver(2000, 2, TX_PIN, 5); //Not receiving, just transmitting

int seq_id = 1;
int buttonState[2]; // the current reading from the input pin
int lastButtonState[2] = {LOW, LOW};
unsigned long lastDebounceTime[2] = {0,0};  // the last time the output pin was toggled


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  Serial.begin(9600); // Debugging only
  if (!driver.init())
      Serial.println("init failed");
  //Led 1 always ON
  digitalWrite(LED1_PIN, HIGH);
}

void loop() {
  checkButton1State(0, BUTTON1_PIN, BUTTON1_RF_COMMAND, 0);
  checkButton1State(1, BUTTON2_PIN, BUTTON2_RF_COMMAND, 0);
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
        send_rf_message(rf_command, rf_parameter);
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState[button_id] = reading;
}

void send_rf_message(uint8_t rf_command, uint8_t rf_parameter) {
    //Light leds
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED1_PIN, HIGH);
  seq_id = seq_id + 1;
  for (int i=0; i<BURST_NUM; i++) {
    uint8_t buf[4];
    buf[0] = seq_id;
    buf[1] = PILOT_ID;
    buf[2] = rf_command;
    buf[3] = rf_parameter;
    driver.send(buf, strlen(buf));
    driver.waitPacketSent();
  }
  //Stop leds
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED1_PIN, LOW);
}
