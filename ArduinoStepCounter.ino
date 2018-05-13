#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Circuit_Playground.h>


volatile boolean updateFlag;
byte numSteps;
byte ledDelay;
byte ledFrequency;
byte ledDelayCounter;
byte colorUpdateDelay;
byte colorUpdateFrequency;
byte colorUpdateDelayCounter;
byte colorWheelValue;
byte updateFrequency;

void setup()
{
  numSteps = 0;
  ledDelayCounter = 0;
  colorUpdateDelayCounter = 0;
  colorWheelValue = 0;
  updateFrequency = 50; // setting sensor update frequency to 50Hz
  ledFrequency = 1;
  colorUpdateFrequency = 10;
  
  ledDelay = updateFrequency / (2 * ledFrequency);
  colorUpdateDelay = updateFrequency / (colorUpdateFrequency);
  
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(CPLAY_LEFTBUTTON), resetPinIsr, CHANGE);
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = (8000000 / 256) / updateFrequency; // compare match register 8MHz/256/updateFrequency
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

  CircuitPlayground.begin();

  updateFlag = true;
}

// Timer fires at 50Hz
ISR(TIMER1_COMPA_vect)
{
  updateFlag = true;
}

void loop()
{
  if (updateFlag)
  {
    // Blink the main red LED at 1Hz
    ledDelayCounter++;
    if (ledDelayCounter == ledDelay)
    {
      CircuitPlayground.redLED(digitalRead(CPLAY_REDLED) ^ 1);
      ledDelayCounter = 0;
    }
    
    numSteps++;
    
    setPixels(numSteps);
    Serial.print("Number of steps: "); Serial.println(numSteps);
    updateFlag = false;
  }
}

void resetPinIsr ()
{
  numSteps = 0;
  Serial.println("Number of steps reset to 0");
}

void setPixels(byte number)
{
  size_t numberOfBits = sizeof(number) * 8;
  //Update the color every 10 * (50Hz)
  colorUpdateDelayCounter++;
  if (colorUpdateDelayCounter == colorUpdateDelay)
  {
    colorWheelValue++;
    colorUpdateDelayCounter = 0;
  }

  CircuitPlayground.clearPixels();

  for (int i = 0; i < numberOfBits; i++)
  {
    if (bitRead(number, i))
    {
      CircuitPlayground.setPixelColor(i, CircuitPlayground.colorWheel(colorWheelValue));
    }
  }
}
