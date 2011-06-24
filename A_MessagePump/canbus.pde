#include "canbus.h"

Canbus::Canbus() {
  //pass
}

void Canbus::begin(int s) {
  Serial.begin(s);
}
void Canbus::end() {
  Serial.end();
}
    
int Canbus::available() {
  return Serial.available();
}
    
void Canbus::write(char c) {
  Serial.write(c);
}

void Canbus::print(char c) {
  Serial.print(c);
}

char Canbus::read() {
  return Serial.read();
}

char Canbus::peek() {
  return Serial.peek();
}
