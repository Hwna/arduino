// LED breathing using analog input (PWM)
#include <math.h>

void setup()
{
  pinMode(6, OUTPUT); // analog output
}

void loop()
{
  float val = (exp(sin(millis()/1000.0*PI)) - 0.36787944)*108.0;
  analogWrite(6, 255 - val);
}
