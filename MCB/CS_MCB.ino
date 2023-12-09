#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

#define PACKET_SIZE 40
RF24 radio(7, 8);
const byte address[6] = "EFEFE";
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  radio.setChannel(62);
  printf_begin();             // needed only once for printing details
  radio.printDetails();       // (smaller) function that prints raw register values
  radio.printPrettyDetails(); // (larger) function that prints human readable data
  
}

void loop() {
  if (radio.available()) {
    char t[PACKET_SIZE] = "";
    radio.read(&t, PACKET_SIZE);
    Serial.println(t);
  }
}
