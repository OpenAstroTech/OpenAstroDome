#include "Encoder.h"

Encoder::Encoder(volatile int32_t *currentPosition, uint8_t ENCA, uint8_t ENCB)
{
    _ENCA = ENCA;
    _ENCB = ENCB;
    _currentPosition = currentPosition;
    pinMode(_ENCA, INPUT_PULLUP);
    pinMode(_ENCB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_ENCA),updateEncoder,RISING);
}

void Encoder::updateEncoder(){
  if(digitalRead(_ENCB) > 0){
    *_currentPosition++;
  } else{
    *_currentPosition++;
  }
}