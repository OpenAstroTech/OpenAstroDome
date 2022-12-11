#include "sdd1306.h"

SDD1306::SDD1306()
{
    _display = new Adafruit_SSD1306(OLED_RESET);
}

void SDD1306::init()
{
    _display->begin(SSD1306_SWITCHCAPVCC, 0x3C);
	_display->clearDisplay();

    _display->setTextSize(2);
	_display->setTextColor(WHITE);
	_display->setCursor(0,0);
	_display->println(_welcome);
    _display->display();
}

void SDD1306::setWelcome(String text)
{
	_welcome = text;
}

void SDD1306::setTitel(String text)
{
    _title = text;
}

void SDD1306::setMessage(String message)
{
    _message = message;
}

void SDD1306::incrementCounter()
{
	if(_counter >= 7)
	{
		_counter = 0;
	}
	else {
		_counter++;
	}
}

void SDD1306::display()
{
    _display->clearDisplay();
	_display->setTextSize(1);
	_display->setTextColor(WHITE);
	_display->setCursor(0,0);
	_display->println(_title);
	_display->setTextSize(2);
	_display->setCursor(0,16);
	_display->println(_message);
	
	if(_counter == 0)
	{
		_display->drawLine(0, 12, 16, 12, 1);
	}
	else 
	{
		int offset = 16*_counter;
		_display->drawLine(offset, 12, offset+16, 12, 1);
	}
	_display->display();
	incrementCounter();
}