#pragma once

#include <Arduino.h>

// SSD1306
#include <spi.h>
#include <wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4

class SDD1306
{
  public:
    SDD1306();
    void init();
    void setWelcome(String text);
    void setTitel(String text);
    void setMessage(String text);
    void display();
  
  private:
    Adafruit_SSD1306* _display;
    String _welcome;
    String _title;
    String _message;
    int _counter = 0;
    bool showAlive = false;

    void incrementCounter();
};
