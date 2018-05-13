#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Circuit_Playground.h>

#define WINDOW_LENGTH 30
#define MOVING_AVERAGE_LENGTH 7
#define ACCEL_THRESHOLD 11.5

volatile boolean updateFlag;

//Frequency at which the various LEDs/sensor-reads update in Hz
byte updateFrequency = 50;
byte ledFrequency = 1;
byte colorUpdateFrequency = 20;

byte numSteps;
byte ledDelay;
byte ledDelayCounter;
byte colorUpdateDelay;
byte colorUpdateDelayCounter;
byte colorWheelValue;

//Impelmenting our window as a circular buffer to reduce the number of writes when the buffer is full (which is always)
float window[WINDOW_LENGTH];
byte windowHead; // If tail == head, then the buffer is empty. If head == tail - 11, then the buffer is full
byte windowTail;

//float movingAverageCalculator[MOVING_AVERAGE_LENGTH];
//byte mAHead;
//byte mATail;

//add a new entry into our window
void addToWindow(float * win, float data)
{
  int newHead = windowHead + 1;
  if (newHead >= WINDOW_LENGTH)
  {
    newHead = 0;
  }

  if (newHead == windowTail)
  {
    if (windowTail + 1 >= WINDOW_LENGTH)
    {
      windowTail = 0;
    }
    else
    {
      windowTail++;
    }
  }
  windowHead = newHead;
  win[windowHead] = data;
}

//returns the count of the data in our window
byte windowDataCount()
{
  if (windowTail >= windowHead)
  {
    return windowHead + WINDOW_LENGTH - windowTail;
  }
  else
  {
    return windowHead - windowTail;
  }
}

//returns the middle datapoint in our window
float windowMidpoint(float * win)
{
  //ignore if our window isn't full
  if (windowHead = windowTail - 1)
  {
    return win[(windowHead + WINDOW_LENGTH / 2) % WINDOW_LENGTH];
  }
  else
  {
    return 0;
  }
}

void setup()
{
  numSteps = 0;
  ledDelayCounter = 0;
  colorUpdateDelayCounter = 0;
  colorWheelValue = 0;

  ledDelay = updateFrequency / (2 * ledFrequency);
  colorUpdateDelay = updateFrequency / (colorUpdateFrequency);

  windowHead = 0;
  windowTail = 0;

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

    float x, y, z, mag;
    x = CircuitPlayground.motionX();
    y = CircuitPlayground.motionY();
    z = CircuitPlayground.motionZ();
    mag = sqrt(sq(x) + sq(y) + sq(z));
//    Serial.print("x: "); Serial.println(x);
//    Serial.print("y: "); Serial.println(y);
//    Serial.print("z: "); Serial.println(z);
//    Serial.print("mag: "); Serial.println(mag);
    addToWindow(window, mag);
//    Serial.print("Current head: "); Serial.println(window[windowHead]);

    float midpoint = windowMidpoint(window);
//    Serial.print("Current midpoint: "); Serial.println(midpoint);

//    for (int i = 0; i < WINDOW_LENGTH; i++)
//    {
//      Serial.print("window["); Serial.print(i); Serial.print("]: "); Serial.println(window[i]);
//    }

    if (midpoint > ACCEL_THRESHOLD)
    {
//      Serial.println("Midpoint is greater than threshold");
      bool isMax = true;
      for (int i = 0; i < WINDOW_LENGTH; i++)
      {
        if (max(midpoint, window[i]) != midpoint)
        {
          //Serial.print("midpoint: "); Serial.print(midpoint); Serial.print(" < "); Serial.println(window[i]);
          isMax = false;
        }
        else
        {
          //Serial.print("midpoint: "); Serial.print(midpoint); Serial.print(" > "); Serial.println(window[i]);
        }
      }

      if (isMax)
      {
        numSteps++;
        Serial.print("Number of steps: "); Serial.println(numSteps);
        if (!CircuitPlayground.slideSwitch())
        {
          CircuitPlayground.playTone(440, 100);
        }
      }
    }
    setPixels(numSteps);

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


