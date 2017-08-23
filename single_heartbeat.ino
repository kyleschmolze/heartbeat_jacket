//////////////////////
// HSV2RGB COLORS
//////////////////////

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

static hsv   rgb2hsv(rgb in);
static rgb   hsv2rgb(hsv in);

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

//////////////////////
// SHIELD + NEOPIXELS
//////////////////////

#include <SPI.h>
#include <TCL.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

const int NUM_LEDS = 50;
const int LED_STRIP_PIN = 11;
const int BUTTON_PRESSED = LOW;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);


//////////////////////
// CUSTOM SETTINGS
//////////////////////

// Set the distances of each LED from the heart
long distanceFromHeart[NUM_LEDS] = {
  /* 0: */ 10, /* 1: */ 14, /* 2: */ 18, /* 3: */ 16, /* 4: */ 13, /* 5: */ 9, /* 6: */ 6, /* 7: */ 4, /* 8: */ 0, /* 9: */ 0,
  /* 10: */ 0, /* 11: */ 5, /* 12: */ 7, /* 13: */ 4, /* 14: */ 3, /* 15: */ 5, /* 16: */ 8, /* 17: */ 10, /* 18: */ 8, /* 19: */ 8,
  /* 20: */ 10, /* 21: */ 13, /* 22: */ 16, /* 23: */ 18, /* 24: */ 15, /* 25: */ 13, /* 26: */ 13, /* 27: */ 12, /* 28: */ 14, /* 29: */ 15,
  /* 30: */ 17, /* 31: */ 19, /* 32: */ 20, /* 33: */ 22, /* 34: */ 21, /* 35: */ 19, /* 36: */ 17, /* 37: */ 16, /* 38: */ 11, /* 39: */ 8,
  /* 40: */ 7, /* 41: */ 8, /* 42: */ 12, /* 43: */ 16, /* 44: */ 19, /* 45: */ 20, /* 46: */ 18, /* 47: */ 12, /* 48: */ 10, /* 49: */ 10
};
long maxDistanceFromHeart = 0;

long __heartRateMs = 1000; // rate in ms, can be changed at any time to influence next beat
long __propagationSpeed = 40 ; // measured in cm's per second (100 is 1 m/s)
unsigned long _heartbeatAnchor; // only need a single timestamp for 1 heartbeat, can just % the rest!

double __minBrightness = 0; // fun to control later!
double __maxBrightness = 1;
double __maxHeartBrightness = 1;
long __mainColor = 0;

long __minHeartRateMs = 300;
long __maxHeartRateMs = 5000;

unsigned long _manualHeartTicksAt[5] = {0,0,0,0,0}; // stores up to 5 manual ticks
int _lastManualHeartTickIndex = 0; // a counter to let us cycle through it;

// PS vars
int PulseSensorPurplePin = 5;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore. 


void setup() {
  Serial.begin(9600);
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

//  TCL.begin(); // this seems to prevent the adafruit library from controlling the LEDs
  TCL.setupDeveloperShield(); // thie enables the sheild inputs
  
  pinMode(TCL_POT1, INPUT);
  pinMode(TCL_POT2, INPUT);
  pinMode(TCL_POT3, INPUT);
  pinMode(TCL_POT4, INPUT);
  pinMode(TCL_MOMENTARY1, INPUT_PULLUP);
  pinMode(TCL_MOMENTARY2, INPUT_PULLUP);
  pinMode(TCL_SWITCH1, INPUT_PULLUP);
  pinMode(TCL_SWITCH2, INPUT_PULLUP);

  _heartbeatAnchor = millis();

  for(long i = 0; i < NUM_LEDS; i++) {
    if(distanceFromHeart[i] > maxDistanceFromHeart) {
      maxDistanceFromHeart = distanceFromHeart[i];
    }
  }
}

void loop() {
  updateSettings();
  updateLEDs();
  readPulse();
  delay(10);
  //checkForManualHeartTick();
}

void readPulse() {
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value. 
  Serial.print(Signal);Serial.print(",");Serial.println(1024);
}

void updateSettings() {
  __maxBrightness = ((double) analogRead(TCL_POT3)) / 1024;
  if (__maxBrightness < 0) __maxBrightness = 0;
  if (__maxBrightness > 1) __maxBrightness = 1;
  
  __maxHeartBrightness = ((double) analogRead(TCL_POT4)) / 1024;
  if (__maxHeartBrightness < 0) __maxHeartBrightness = 0;
  if (__maxHeartBrightness > 1) __maxHeartBrightness = 1;
  
  __mainColor = (long)analogRead(TCL_POT2) * 360 / 1024;
  if (__mainColor < 0) __mainColor = 0;
  if (__mainColor > 360) __mainColor = 360;
  
  
}

void updateLEDs() {
//  Serial.print("_heartbeatAnchor: "); Serial.println(_heartbeatAnchor);
  
  long timeSinceLastBeatAtHeart = (millis() - _heartbeatAnchor) % __heartRateMs;
  //Serial.print("timeSinceLastBeatAtHeart: "); Serial.println(timeSinceLastBeatAtHeart);
  for(long i = 0; i < NUM_LEDS; i++) {
    // speed is in cm per sec, distance is in cm, and we want to get timeDelay in ms
    //distanceFromHeart[i] = i*3;
    if (distanceFromHeart[i] == -1) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    } else {
      long timeDelay = distanceFromHeart[i] * 1000 / __propagationSpeed;
      // speed of 100, distance of 100 => 1000ms = 1 sec (correct!)
  
      long timeSinceLastBeatAtLED = timeSinceLastBeatAtHeart - timeDelay;
      timeSinceLastBeatAtLED = (timeSinceLastBeatAtLED + __heartRateMs) % __heartRateMs; // negative #s
      
      strip.setPixelColor(i, pickColor(timeSinceLastBeatAtLED, i));
    }
  }
  strip.show();
}

long pickColor(long timeSinceBeat, long i) {
  hsv color;

  // HUE
  
  //color.h = i * 10; //linear rainbow

  // MODE 1 - all colors controlled directly by pot
  //color.h = __mainColor; 

  // MODE 2 - use pot color to scale the size of the rainbow (out of 359)
  color.h = (double) distanceFromHeart[i] * (double)__mainColor / (double) maxDistanceFromHeart; // rainbow by distance!
  color.s = 1;

  double brightness;

  // BRIGHTNESS - heart brightness controlled by sep. variable than all others
  if (distanceFromHeart[i] == 0) {
    //heart brightness  
    long percent = (timeSinceBeat * 100 / __heartRateMs);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    // our desmos eq:
    //brightness = 3.0 / (((double) percent * 4.0 / 100.0) + 2) - 0.5; // original
    brightness = 3.1 / (((double) percent * 20.0 / 100.0) + 2.5) - 0.1; // more aggresive
    brightness *= __maxHeartBrightness;
    if (brightness < __minBrightness) brightness = __minBrightness;
    if (brightness > __maxHeartBrightness) brightness = __maxHeartBrightness;
  } else {
    long percent = (timeSinceBeat * 100 / __heartRateMs);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    // linear: double brightness = __maxBrightness - ((double) percent / 100.0 * __maxBrightness);
  
    // our desmos eq:
    brightness = 3.0 / (((double) percent * 4.0 / 100.0) + 2) - 0.5;
    brightness *= __maxBrightness;
    if (brightness < __minBrightness) brightness = __minBrightness;
    if (brightness > __maxBrightness) brightness = __maxBrightness;
  }

  color.v = brightness;
  
  rgb rgbColor = hsv2rgb(color);
  return strip.Color(rgbColor.r*255.0, rgbColor.g*255.0, rgbColor.b*255.0);
}



void printAllInput() {
  Serial.print("[ ");  
  Serial.print(analogRead(TCL_POT1)); Serial.print(" ");
  Serial.print(analogRead(TCL_POT2)); Serial.print(" ");
  Serial.print(analogRead(TCL_POT3)); Serial.print(" ");
  Serial.print(analogRead(TCL_POT4)); Serial.print(" ");
  Serial.print(digitalRead(TCL_MOMENTARY1)); Serial.print(" ");
  Serial.print(digitalRead(TCL_MOMENTARY2)); Serial.print(" ");
  Serial.print(digitalRead(TCL_SWITCH1)); Serial.print(" ");
  Serial.print(digitalRead(TCL_SWITCH2)); Serial.print(" ");
  Serial.println("]");
}

///////////////////////////////////



/*
void setBrightnesses() {
//  strip.setPixelColor(i, strip.Color(brightn essForPixel(i), 0, 0));
  for(int i = 0; i < NUM_LEDS; i++)
    strip.setPixelColor(i, strip.Color(getBrightnessForPixel(i), 0, 0));
  strip.show();
}

int getBrightnessForPixel(int i) {
  int timeDelay = delayForPin(i);

  // delay it by subtracting it from timeSinceLast, but don't go negative, wrap around:
  unsigned long offsetTime = (timeSinceLastBeat + heartRate - timeDelay) % heartRate;

  // linear down from max to min:
  // currentBrightness = maxBrightness - ((maxBrightness - minBrightness) * timeSinceLastBeat / heartRate);
  
  // asymptote from maxB to minB:
  // https://www.desmos.com/calculator/vdcuur3cyp
  // scale X from 0-maxB
  double x = maxBrightness * offsetTime / heartRate;
  
  // more sustain = slower decay
  double sustain = heartRate / 67; // sustains from 5-20 seem to work well
  int y = (maxBrightness-minBrightness) / ((x / sustain) + 1) + minBrightness;
  // Serial.print(x);
  // Serial.print(" => ");
  // Serial.print(y);
  // Serial.print(" - time: ");
  // Serial.println(timeSinceLastBeat);
  return y;
}

void checkButton() {
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(INPUT_PIN);
  //print out the value of the pushbutton
  //Serial.println(sensorVal);

  if (sensorVal == BUTTON_PRESSED) {
    
    // put the timestamp into presses[]
    if (recordButtonPress()) {
      // if it was recorded (more than maxDiff from last one), look for averages!
      int average = calcAverage(); 
      if (average != -1) { //yes!
        Serial.print("Average: ");
        Serial.println(average);
        targetHeartRate = average;
        heartRate = 1; // instantly beat! then set to target
      }
    }
  }
}


int calcAverage() {
  // calc with as many averages as we can, that we within maxDiff
  unsigned long avgDiff;
  int i = (lastPressIndex+1) % 5; // start at the farthest "back" press
  int numPresses = 4; // track how many presses are between i and lastPressIndex
  
  // just to prevent the "trailing click" bug (3 quick clicks followed by one 10 seconds later),
  // let's make sure lastPress and 1 before were within maxDiff
  int prev = (lastPressIndex + 4) % 5;
  if (presses[lastPressIndex] - presses[prev] > maxDiff)
    return -1;
  
  while(i != lastPressIndex) {
    // loop forwards, searching for the first timestamp which indicates that the average
    // interval ahead of it is under maxDiff. Get average by taking total diff, and dividing
    avgDiff = (presses[lastPressIndex] - presses[i]) / numPresses;
    // presses are never recorded in less than minDiff, so only worry about max
    if (avgDiff <= maxDiff && presses[i] != 0) { // ignore initialized 0s
      return avgDiff;
    } else {
      i = (i + 1) % 5;
      numPresses -= 1;
    }
  }
  return -1;
}

boolean recordButtonPress() {
  // record all presses, unless they're too close to the last one
  int diff = currentTime - presses[lastPressIndex];
  if (diff >= minDiff) {
    lastPressIndex = (lastPressIndex + 1) % 5;
    presses[lastPressIndex] = currentTime;
    printPresses();
    return true;
  }
  return false;
}


void printPresses() {
  Serial.print("Presses: [");
  Serial.print(presses[0]);
  Serial.print(", ");
  Serial.print(presses[1]);
  Serial.print(", ");
  Serial.print(presses[2]);
  Serial.print(", ");
  Serial.print(presses[3]);
  Serial.print(", ");
  Serial.print(presses[4]);
  Serial.println("]");
}
*/



