#include "Encoder.h"

Encoder::Encoder(volatile int32_t &currentPosition, uint8_t ENCA, uint8_t ENCB)
{
    _ENCA = ENCA;
    _ENCB = ENCB;
    _currentPosition = &currentPosition;
    pinMode(_ENCA, INPUT_PULLUP);
    //pinMode(_ENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_ENCA), updateEncoderA, RISING);
    //attachInterrupt(digitalPinToInterrupt(_ENCB), updateEncoderB, RISING);
}

void Encoder::updateEncoderA(){
  //*_currentPosition += digitalRead(_ENCA) != digitalRead(_ENCB) ? -1 : 1;
  
  if(digitalRead(_ENCB) > 0){
    (*_currentPosition)-=4;
  } else{
    (*_currentPosition)+=4;
  }
  
}

void Encoder::updateEncoderB(){
  // *_currentPosition += digitalRead(_ENCA) == digitalRead(_ENCB) ? -1 : 1;
  
  /*
  if(digitalRead(_ENCB) > 0){
    (*_currentPosition)-=4;
  } else{
    (*_currentPosition)+=4;
  }
  */
}