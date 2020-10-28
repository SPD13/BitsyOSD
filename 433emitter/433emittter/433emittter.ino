#include <RH_ASK.h>
#include <SPI.h>

RH_ASK driver(2000, 2, 11, 5);
#define BURST_NUM 10 //Number of burst to send

int seq_id = 1;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
Serial.begin(9600); // Debugging only
if (!driver.init())
    Serial.println("init failed");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  seq_id = seq_id + 1;
  for (int i=0; i<BURST_NUM; i++) {
    uint8_t buf[4];
    buf[0] = seq_id;
    buf[1] = 1;
    buf[2] = 1;
    buf[3] = 1;
    driver.send(buf, strlen(buf));
    driver.waitPacketSent();
  }
  digitalWrite(LED_BUILTIN, LOW); 
  delay(5000);
}
