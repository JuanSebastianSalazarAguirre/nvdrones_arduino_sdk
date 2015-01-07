void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available()) {
      float dat = readFloat();
      if (dat == 3.141) {
         digitalWrite(13, HIGH);
      }
  }
  delay(100);
}

float readFloat() {
    float data;
    Serial.readBytes((char*)&data, sizeof(data));
    return data;
}





