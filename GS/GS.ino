#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);

uint8_t address[6] = { "EFEFE"};

void setup() {
  delay(5000);
  Serial.begin(9600);
  SPI.begin();
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while(1){
      digitalWrite(2, 1);
      delay(1000);
      digitalWrite(2, 0);
      delay(1000);
    }
  }
  radio.setChannel(62);
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(32);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();
  
  digitalWrite(2, 1);
}


char txMSG[32];
char rxMSG[32];

void loop() {
  uint8_t pipe;
  if (radio.available(&pipe)) {
    digitalWrite(3, 1);
    uint8_t b = radio.getPayloadSize();
    radio.read(&rxMSG, b);
    Serial.println(rxMSG);
    digitalWrite(3, 0);
  }

  if (Serial.available()) {
    //clear array before use
    for(int a=0; a<32; a++){
      txMSG[a] = 0x20; //unicode space
    }
    digitalWrite(4, 1);
    delay(100); //wait for buffer
    int rxB = Serial.available();
    for(int i=0; i<rxB; i++){
      txMSG[i] = Serial.read();
    }
    Serial.println(txMSG);
    radio.stopListening();
    transmitPacket();
    radio.startListening();
    digitalWrite(4, 0);
  }
}


void transmitPacket(){
  unsigned long start_timer = micros();
  bool r = radio.write(&txMSG, 32);
  unsigned long end_timer = micros();
  
  if (r) {
    Serial.print(F("Transmission successful! "));
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);
    Serial.print(F(" us. Sent: "));
    Serial.println(txMSG);
  } else {
    Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
  }
}
