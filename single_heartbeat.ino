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

#include <FastLED.h>

const int NUM_LEDS = 50;
const int LED_STRIP_PIN = 11;
const int NUM_TICKS = 5;

CRGB leds[NUM_LEDS];

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
long __startColor = 0;
long __endColor = 0;
boolean colorAutoWipe = false;

unsigned long _heartTicks[NUM_TICKS]; // stores tick timestamps
int _lastHeartTickIndex = 0; // a counter to let us cycle through it;
unsigned long __minHeartTickDiff = 300;
unsigned long __maxHeartTickDiff = 3000;
unsigned long currentTime = 0;


// PS vars
int PulseSensorPurplePin = 5;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore. 


void setup() {
  Serial.begin(9600);

  FastLED.addLeds<WS2812, LED_STRIP_PIN, GRB>(leds, NUM_LEDS);
  
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
  for(int i = 0; i < NUM_TICKS; i++) { _heartTicks[i] = 0; }
}

void loop() {
  currentTime = millis();
  if(listeningForHeartbeat()){
    turnOffLEDs();
    if (heartbeatDetected()) { // buttons are in right configuration
      if (recordHeartbeat()) // returns true if tick was recorded (not too fast)
        if (pulseDetected()) // calcs average, sets heart rate
          stopListening(); // back to other mode
    }
  } else {
    updateSettings();
    updateLEDs();
    //printAllInput();
  }
  //delay(30);
}

unsigned long lastColorModePress = 0;

void updateSettings() {
  __maxBrightness = ((double) analogRead(TCL_POT3)) / 1024;
  if (__maxBrightness < 0) __maxBrightness = 0;
  if (__maxBrightness > 1) __maxBrightness = 1;
  
  __maxHeartBrightness = ((double) analogRead(TCL_POT4)) / 1024;
  if (__maxHeartBrightness < 0) __maxHeartBrightness = 0;
  if (__maxHeartBrightness > 1) __maxHeartBrightness = 1;

  //double frac = (double)analogRead(TCL_POT1) / 512;
  double frac = 0.88;
  __propagationSpeed = 40000.0 * frac / __heartRateMs;

  if (digitalRead(TCL_SWITCH1) == 1) { // read directly from POT1
    int newRate = analogRead(TCL_POT1) * 2;
    if (newRate < __heartRateMs - 25 || newRate > __heartRateMs + 25) {
      __heartRateMs = newRate;
      _heartbeatAnchor = currentTime;
    }
  }

  // switch color modes?
  // button pressed, don't allow more than 2 per second
  if (digitalRead(TCL_MOMENTARY2) == 1 && currentTime - lastColorModePress > 500) {
    lastColorModePress = currentTime;
    colorAutoWipe = !colorAutoWipe;
  }

  int colorVal;
  
  if(colorAutoWipe) {
    int period = analogRead(TCL_POT2) * 30; // ms for whole cycle (30sec highest)
    //int period = 30000;
    int t = currentTime % period;
    if (t < period / 2) colorVal = (long)t * (long)1024 / (long)(period / 2); // from 0 to 1024 in first half of period
    else colorVal = 1024 - ((long)(t-(period/2)) * (long)1024 / (long)(period / 2)); // from 1024 to 0 in second half of period
    //Serial.println(colorVal);
  } else {
    colorVal = analogRead(TCL_POT2);
  }

  // CALC COLORVAL
  // cutoffs: 341, 682, 1023
  if (colorVal < 341) {
    // stage 1, just mess with the endColor
    __startColor = 0;
    __endColor = (long)colorVal * 360 / 341;
  } else if (colorVal < 682) {
    // stage 2, bring start color up to match.
    __startColor = (long)(colorVal-341)* 360 / 341;
    __endColor = 360;
  } else {
    // stage 3, bring both colors DOWN
    __startColor = (long)(colorVal-682) * 360 / 341;
    __endColor = (long)(colorVal-682) * 360 / 341;
  }

  
}

long y = 0;
void updateLEDs() {
//  Serial.print("_heartbeatAnchor: "); Serial.println(_heartbeatAnchor);
  
  long timeSinceLastBeatAtHeart = (currentTime - _heartbeatAnchor) % __heartRateMs;
  if(timeSinceLastBeatAtHeart <= 30) {
//    Serial.print("Beating: ");
//    Serial.print(timeSinceLastBeatAtHeart);
//    Serial.print(" Diff: ");
//    Serial.println(currentTime - y);
    y = currentTime;
  }
  //Serial.print("timeSinceLastBeatAtHeart: "); Serial.println(timeSinceLastBeatAtHeart);
  for(long i = 0; i < NUM_LEDS; i++) {
    // speed is in cm per sec, distance is in cm, and we want to get timeDelay in ms
    //distanceFromHeart[i] = i*3;
    if (distanceFromHeart[i] == -1) {
      leds[i] = CRGB(0,0,0);   
    } else {
      long timeDelay = distanceFromHeart[i] * 1000 / __propagationSpeed;
      // speed of 100, distance of 100 => 1000ms = 1 sec (correct!)
  
      long timeSinceLastBeatAtLED = timeSinceLastBeatAtHeart - timeDelay;
      timeSinceLastBeatAtLED = (timeSinceLastBeatAtLED + __heartRateMs) % __heartRateMs; // negative #s
      setColorFor(i, timeSinceLastBeatAtLED);
    }
  }
  FastLED.show();
}

void setColorFor(int i, long timeSinceBeat) {
  hsv color;

  // HUE
  
  //color.h = i * 10; //linear rainbow

  // MODE 1 - all colors controlled directly by pot
  //color.h = __endColor; 

  // MODE 2 - use pot color to scale the size of the rainbow (out of 359)
  color.h = (double) distanceFromHeart[i] * ((double)__endColor - (double)__startColor) / (double) maxDistanceFromHeart + (double)__startColor; // rainbow by distance!
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
  leds[i] = CRGB((int)(rgbColor.r*255.0), (int)(rgbColor.g*255.0), (int)(rgbColor.b*255.0));
}

void lightUpHeart() {
  for (int i = 0; i < NUM_LEDS; i++) {
    if (distanceFromHeart[i] == 0) {
      leds[i] = CRGB(255,0,0);   
    }
    FastLED.show();
  }
}

void turnOffLEDs() {
  for(long i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0,0,0);   
  }
  FastLED.show();
}

///////////////////////
// BUTTON PRESSIN' CODE
///////////////////////

int prevListeningVal = 0;
boolean listeningKillswitch = false;

boolean listeningForHeartbeat() {
  int val = digitalRead(TCL_SWITCH2);
  if (prevListeningVal == 0 && val == 1) // just turned back on
    listeningKillswitch = false;
  
  prevListeningVal = val;
  
  if (listeningKillswitch)
    return false;
  
  return val == 1;
}

boolean heartbeatDetected() {
  unsigned long x = currentTime % 1000;
  //if (x > 0 && x < 200)
    //return true;
  if (digitalRead(TCL_MOMENTARY2) == 1)
    return true;
  if (digitalRead(TCL_MOMENTARY1) == 1 && analogRead(PulseSensorPurplePin) > Threshold)
    return true;
  return false;
}

void stopListening() {
  listeningKillswitch = true;
  for(int i = 0; i < NUM_TICKS; i++)
    _heartTicks[_lastHeartTickIndex] = 0;
}


boolean recordHeartbeat() {
  lightUpHeart();
  
  int diff = currentTime - _heartTicks[_lastHeartTickIndex];
  if (diff >= __minHeartTickDiff) {
    _lastHeartTickIndex = (_lastHeartTickIndex + 1) % NUM_TICKS;
    _heartTicks[_lastHeartTickIndex] = currentTime;
    //printHeartTicks();
    return true;
  }
  return false;
}

boolean pulseDetected() {
  // return digitalRead(TCL_MOMENTARY1) == 1;
  // trickiest code to get right!!
  // let's require that we have NUM_TICKS ticks, all within 5% of their average diff,
  // under a maximum diff of 2 seconds

  unsigned long averageDiff = (_heartTicks[_lastHeartTickIndex] - _heartTicks[(_lastHeartTickIndex+1)%NUM_TICKS]) / (NUM_TICKS-1);
  unsigned long lastDiff = currentTime - _heartTicks[(_lastHeartTickIndex-1+NUM_TICKS)%NUM_TICKS];
  //Serial.print("last Diff: ");
  //Serial.println(lastDiff);
  
  unsigned long sum = 0;
  //Serial.print("averageDiff: ");
  //Serial.print(averageDiff);
  //Serial.print(" Diffs: [ ");
  
  for(int i = 1; i < NUM_TICKS; i++) {
    // start loop at farthest "back" tick (index + 1). diffs look forward 1 tick.
    int index = (_lastHeartTickIndex + i ) % NUM_TICKS;
    if (_heartTicks[index] == 0) return false; // any tick is 0 - no good!
    unsigned long diff = _heartTicks[(index+1)%NUM_TICKS] - _heartTicks[index];
    //Serial.print(diff);
    //Serial.print(", ");
    if (diff > __maxHeartTickDiff) return false;  // any diff is too long - no good!
    if (diff > averageDiff*1.15) return false;
    if (diff < averageDiff*0.85) return false;
  }
  
  setHeartRate(averageDiff);

  return true;
}

void setHeartRate(unsigned long rate) {
  __heartRateMs = rate;
  _heartbeatAnchor = currentTime;
  //Serial.println("");
  //Serial.print("Found new average: ");
  //Serial.println(rate);
}


////////////////
// PRINTING CODE
////////////////


void printAllInput() {
  Serial.print("[ ");  
  Serial.print("POT1: "); Serial.print(analogRead(TCL_POT1)); Serial.print(" ");
  Serial.print("POT2: "); Serial.print(analogRead(TCL_POT2)); Serial.print(" ");
  Serial.print("POT3: "); Serial.print(analogRead(TCL_POT3)); Serial.print(" ");
  Serial.print("POT4: "); Serial.print(analogRead(TCL_POT4)); Serial.print(" ");
  Serial.print("MOM1: "); Serial.print(digitalRead(TCL_MOMENTARY1)); Serial.print(" ");
  Serial.print("MOM2: "); Serial.print(digitalRead(TCL_MOMENTARY2)); Serial.print(" ");
  Serial.print("SWT1: "); Serial.print(digitalRead(TCL_SWITCH1)); Serial.print(" ");
  Serial.print("SWT2: "); Serial.print(digitalRead(TCL_SWITCH2)); Serial.print(" ");
  Serial.println("]");
}

void printHeartTicks() {
  Serial.println("");
  Serial.print("Heart ticks: [");
  for(int i = 1; i < NUM_TICKS; i++) {
    Serial.print(_heartTicks[i]);
    Serial.print(", ");
  }
  Serial.println("]");
}
