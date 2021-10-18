#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <SPI.h>
#include <String.h>
//ce, csn pins
RF24 radio(9,10);
const uint64_t pipe = 0xF0F0F0F0E1LL;
int val1, val2, val3, but1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(0x76);
  radio.openWritingPipe(pipe);
  radio.enableDynamicPayloads();
  radio.powerUp();
  pinMode(7, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  val1 = analogRead(A0);
  val2 = analogRead(A1);
  val3 = analogRead(A2);
  but1 = digitalRead(7);
  int text[4] = {val1, val2, val3, but1};
  radio.write(&text, sizeof(text));
  Serial.print(val1);
  Serial.print(val2);
  Serial.print(val3);
  Serial.println(but1);
  delay(10);
}
