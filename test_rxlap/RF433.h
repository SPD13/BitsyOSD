// -------- Send RF Message ----------
void send_rf_message(uint8_t rf_command, uint8_t rf_parameter) {
    //Light leds
  digitalWrite(led1Pin, HIGH);
  seq_id = seq_id + 1;
  for (int i=0; i<BURST_NUM; i++) {
    uint8_t buf[4];
    buf[0] = seq_id;
    buf[1] = PILOT_ID;
    buf[2] = rf_command;
    buf[3] = rf_parameter;
    driver.send(buf, strlen(buf));
    driver.waitPacketSent();
    digitalWrite(led1Pin, LOW);
    delay(10);
  }
  //Stop leds
  Serial.print("Command ");
  Serial.print(rf_command);
  Serial.println(" sent");
  digitalWrite(LED_BUILTIN, LOW);

  digitalWrite(led1Pin, HIGH);
}
