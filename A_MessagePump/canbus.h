#pragma once

class Canbus {
  public:
    Canbus();
    
    void begin(int);
    void end();
    
    int available();
    
    void write(char);
    void print(char);
    char read();
    char peek();  
};
