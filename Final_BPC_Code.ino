// pins and settings
const int PIN_PRESSURE = A0;
const int PIN_OSC = A1;
const int PIN_BTN = 2;
const int PIN_PUMP = 3;
const int PIN_VALVE = 4;

const float ADC_MAX_V_INFLATE = 4.40f;           // maximum voltage reached during INFLATION (averaged, since it falls over the course of inflation)
const float ADC_MAX_V_DEFLATE = 4.74f;           // maximum voltage reached during DEFLATION
bool AUTO_START = 0;

// taking readings at regular intervals. limiting inflate time for safety.
const unsigned long READING_INTERVAL_MS = 10;     // reading sensor every 10 ms
const unsigned long MAX_DEFLATE_TIME_MS = 120000; // keeps deflation going for 2 mins or till manual stop
const unsigned long DEFLATE_WAIT_TIME = 100;      // holding off on calculating deflation voltage till adc max reference has time to return to ~5 v
const float DEFLATE_PRES = 50.0f;                 // adjusted system: now stopping deflation according to pressure, as opposed to time
const unsigned long MAX_INFLATE_TIME_MS = 70000;  // stops inflating automatically after 70 s
float maxInflateVoltage = 0.0f;                   // for calibration purposes-- observed max inflation voltage to exclude transient/jump upon deflation 
float maxDeflateVoltage = 0.0f;                   // for calibration purposes               

const float TARGET_MMHG = 200.0f;                 // deflate when target reached in mmHg
const float TARGET_VOLTAGE = 3.0f;               // same goes for voltage. <3.55 to account for jump in pressure curve voltage between inflate and deflate

// started in volts because I originally couldn't access the pressure gauge. now switched to mmHg!
const float m = 60.0f;                            // slope equation after calibrating
const float b = -50.0f;                           // y-intercept
bool USE_MMHG = true;                             // now switched to mmHg

// sampling
const unsigned long FS_HZ = 100;                    // frequency = 100 readings per second
const unsigned long DT_MS = 1000 / FS_HZ;           // delta t, AKA sampling window, is 10 ms
const unsigned long FIRST_MS = 250;                 // because of initial step, we won't take the first 1/4 second's worth of deflation readings
const int MOV_AVG_LEN = 50;                         // moving avg window is 0.5 s worth of readings
const int SMOOTH_LEN = 5;                           // for smoothing out pres_ossc

// detecting beats
const float MIN_GAP_S = 0.2;                        // 40–200 bpm are valid: 60 sec/200 beats = 0.2
const float MAX_GAP_S = 1.5;                        // 40–200 bpm are valid: 60 sec/40 beats = 1.5
const int   MAX_BEATS = 64;                         // beats stored for one run
const float MIN_DBP = 40.0f;                        // minimum diastolic in mmHg, so we don't record false values
const int SKIP_BEATS_FOR_AMAX = 3;

// fixed-ratios for computing sys/dia as some fraction of map
const float R_SYS = 0.55f;                          // SBP amplitude ratio
const float R_DIA = 0.65f;                          // DBP amplitude ratio

// moving average for simple filter-- 50 samples at 100 hz = 0.5 s
float bufMA[MOV_AVG_LEN];                           // storing last MA samples
int maIndex = 0;                                    // index where next sample will be written 
float maSum = 0;                                    // running sum of all values in bufMA
float bufSmooth[SMOOTH_LEN];                        // smoother buffer (5 samples) operating similarly
int smIndex = 0;                                    // index for next sample
float smSum = 0;                                    // running sum

// struct for storing one oscillogram point: cuff pressure, heartbeat amplitude in V, ms since run start
struct BeatPoint { float P_mmhg; float Amp_v;};    
BeatPoint beats[MAX_BEATS]; 
int numBeats = 0;

float envAmp[MAX_BEATS];                            // smoothed beat-amplitude envelope
float envWork[MAX_BEATS];                           // temporary/working array for smoothing

const float BEAT_DROP = 2.0f;                       // each 2 mmHg pressure drop, we store a new beat
const int ENV_SMOOTH_PASSES = 2;                    // number of passes over smoothed envelope

// tracking heartbeats within the current window
bool lastPos = false;                               // previous sign of osc_signal
bool inBeat = false;                                // checking if inside beat window
float curPeak = -1e9, curTrough = 1e9;              // tracking peaks/troughs
unsigned long tLastZero = 0;                        // tracking time of last upward zero crossing
float lastStoredPres = -1.0f;                       // last cuff pressure where a beat was stored
bool newbp = true;                                  // is there new bp test data? false if not, true if so

// state machine
enum State {IDLE, INFLATE, DEFLATE, DONE};
State state = IDLE;

// ignoring bounces for 30 ms after the button is pressed
bool lastRaw = HIGH;                                // last raw button state initialized at HIGH (unpressed)
bool btnState = HIGH;                               // stable button state, i.e. after debouncing
bool lastBtnState = HIGH;                           // last stable button state, to determine if button has been pressed or released
unsigned long timeLastChange = 0;                   // time since debouncing began
const unsigned long DEBOUNCE_MS = 30;               // ignoring bounces for 30 ms after the button is pressed

// timestamps
unsigned long timeStart = 0;
unsigned long timeLastReading = 0;
unsigned long timeDeflateStart = 0;

// translating analog input to corresponding voltage (during inflation)
float adcToVoltsInflate(int adc_pressure) {
  return (adc_pressure * ADC_MAX_V_INFLATE) / 1023.0f;
}

// translating analog input to corresponding voltage (during deflation)
float adcToVoltsDeflate(int adc_pressure) {
  return (adc_pressure * ADC_MAX_V_DEFLATE) / 1023.0f;
}

// voltage to pressure
float voltsToMmhg(float v) {
  return m * v + b;
}

// HIGH turns on my pump, LOW shuts it off
void pumpOn(bool on) {
  if (on) {
    digitalWrite(PIN_PUMP, HIGH);
  } else {
    digitalWrite(PIN_PUMP, LOW);
  }
}

// LOW opens my valve, HIGH closes it
void valveOpen(bool on) {
  if (on) {
    digitalWrite(PIN_VALVE, LOW);
  } else {
    digitalWrite(PIN_VALVE, HIGH);
  }
}

// taking the moving average over the last MOV_AVG_LEN number of samples
float movingAvg(float x) {
  maSum -= bufMA[maIndex];
  bufMA[maIndex] = x;
  maSum += x;
  maIndex = (maIndex + 1) % MOV_AVG_LEN;
  return maSum / MOV_AVG_LEN;
}

// smoothing out noise in a given sample
float smoothShort(float x) {
  smSum -= bufSmooth[smIndex];
  bufSmooth[smIndex] = x;
  smSum += x;
  smIndex = (smIndex + 1) % SMOOTH_LEN;
  return smSum / SMOOTH_LEN;
}

// when state goes from IDLE to INFLATE, resets all filter buffers and beat tracking numbers
void resetOscInfo() {
  for (int i = 0; i < MOV_AVG_LEN; i++) bufMA[i] = 0;
  for (int i = 0; i < SMOOTH_LEN; i++) bufSmooth[i] = 0;
  maIndex = smIndex = 0; maSum = smSum = 0;
  lastPos = false; inBeat = false;
  curPeak = -1e9; curTrough = 1e9;
  numBeats = 0; tLastZero = 0;
  lastStoredPres = -1.0f;
}

// when the next positive zero-crossing comes, stores one pressure/amplitude point
void finalizeBeat(float P_mmhg, unsigned long t_ms, float amp) {
  if (numBeats >= MAX_BEATS) return;
  bool gapOK = true;                                     // checking if gap between beats is reasonable
  if (tLastZero != 0) {
    float gap = (t_ms - tLastZero) / 1000.0f;
    gapOK = (gap >= MIN_GAP_S && gap <= MAX_GAP_S);
  }
  if (gapOK && amp > 0) {
    if (lastStoredPres < 0) {                            // we only start store beats once per each 2 mmHg pressure drop
      beats[numBeats++] = { P_mmhg, amp };
      lastStoredPres = P_mmhg;
    }
    else if ((lastStoredPres - P_mmhg) >= BEAT_DROP) {
      beats[numBeats++] = { P_mmhg, amp };
      lastStoredPres = P_mmhg;
    }
  }
  tLastZero = t_ms;
}

// envelope of smoothed amplitudes for computing SBP/DBP
void smoothEnvelope() {
  for (int i = 0; i < numBeats; i++) {
    envAmp[i] = beats[i].Amp_v;
  }

  for (int pass = 0; pass < ENV_SMOOTH_PASSES; pass++) {
    envWork[0] = envAmp[0];
    envWork[numBeats - 1] = envAmp[numBeats - 1];

    for (int i = 1; i < numBeats - 1; i++) {
      envWork[i] = 0.25f * envAmp[i - 1]
                 + 0.50f * envAmp[i]
                 + 0.25f * envAmp[i + 1];
    }

    for (int i = 0; i < numBeats; i++) {
      envAmp[i] = envWork[i];
    }
  }
}

// finding max index in smoothed array
int findEnvelopeMaxIndex() {
  int iMax = SKIP_BEATS_FOR_AMAX;

  for (int i = SKIP_BEATS_FOR_AMAX + 1; i < numBeats; i++) {
    if (envAmp[i] > envAmp[iMax]) {
      iMax = i;
    }
  }

  return iMax;
}

// corresponding pressure for max index
bool crossingPressure(int startIndex, int endIndex, float targetAmp, float &Pcross) {
  if (startIndex < 0 || endIndex >= numBeats || startIndex >= endIndex) {
    return false;
  }

  for (int i = startIndex; i < endIndex; i++) {
    float A0 = envAmp[i];
    float A1 = envAmp[i + 1];

    float d0 = A0 - targetAmp;
    float d1 = A1 - targetAmp;

    if ((d0 * d1) <= 0.0f) {
      float denom = A1 - A0;

      if (fabs(denom) < 1e-6f) {
        return false;
      }

      float frac = (targetAmp - A0) / denom;

      Pcross = beats[i].P_mmhg
             + frac * (beats[i + 1].P_mmhg - beats[i].P_mmhg);

      return true;
    }
  }

  return false;
}

// computing simple MAP/SBP/DBP from stored window of values
bool computeBP(float &MAP, float &SBP, float &DBP) {
  if (numBeats < 6) return false;                                         // AKA not enough points
                      
  int iMax = SKIP_BEATS_FOR_AMAX;                                         // finding max amplitude and its pressure index

  if (numBeats <= SKIP_BEATS_FOR_AMAX) return false;                      // not allowing max amplitude count to start for first few beats (sometimes there's a transient)

  smoothEnvelope();
  
  for (int i = SKIP_BEATS_FOR_AMAX + 1; i < numBeats; i++) {
    
    if (envAmp[i] > envAmp[iMax]) {
      iMax = i; 
    }
  }

  Serial.print(F(" Max amplitude index: "));
  Serial.println(iMax);
  Serial.print(F(" Pressure at iMax: "));
  Serial.println(beats[iMax].P_mmhg);
  Serial.print(F(" Max amplitude (Amax): "));
  Serial.println(envAmp[iMax]);
  Serial.print(F(" Beats counted: "));
  Serial.println(numBeats);


  MAP = beats[iMax].P_mmhg;
  float Amax = envAmp[iMax];

Serial.println(F("ENV_BEGIN"));
Serial.println(F("Index,P_mmHg,Amp_v,Ratio"));

for (int i = SKIP_BEATS_FOR_AMAX; i < numBeats; i++) {

  Serial.print(i);
  Serial.print(",");

  Serial.print(beats[i].P_mmhg, 2);
  Serial.print(",");

  Serial.print(envAmp[i], 5);
  Serial.print(",");

  Serial.println(envAmp[i] / Amax, 5);
}

Serial.println(F("ENV_END"));

  // target amplitudes for fixed ratios
  float targetSys = R_SYS * Amax;
  float targetDia = R_DIA * Amax;

  Serial.print(F("First stored pressure: "));
  Serial.println(beats[0].P_mmhg);

  Serial.print(F("First envelope amplitude: "));
  Serial.println(envAmp[0], 4);

  Serial.print(F("Systolic target amplitude: "));
  Serial.println(targetSys, 4);

float sysPressure;
float diaPressure;

bool sysOK = crossingPressure(SKIP_BEATS_FOR_AMAX, iMax, targetSys, sysPressure);
bool diaOK = crossingPressure(iMax, numBeats - 1, targetDia, diaPressure);

if (!sysOK || !diaOK) {

  Serial.print(F("sysOK = "));
  Serial.println(sysOK);
  Serial.print(F("diaOK = "));
  Serial.println(diaOK);

  return false;
}
  
if (diaPressure <= MIN_DBP) return false;

SBP = sysPressure;
DBP = diaPressure;

  return true;

}


// setting pins as output/input; starting with pump and valve off
void setup() {
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);

  pumpOn(false);
  valveOpen(false);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("Ready. Press button to start."));

  resetOscInfo(); // resets filters/tracking values
}

// main loop
void loop() {
  // debouncing initially
   bool rawBtn = digitalRead(PIN_BTN);                // raw (not yet debounced) button read. LOW for button pressed, HIGH if not

 if (rawBtn != lastRaw) {                             // if input changes from raw, start debounce timer
   timeLastChange = millis();                         // timer count begins
   lastRaw = rawBtn;                                  // setting last raw state to current raw state
 }

 if ((millis() - timeLastChange) >= DEBOUNCE_MS) {    // minimum debounce time exceeded (30 ms), so I'm assuming stable button state
  btnState = rawBtn;                                  // stable (i.e. debounced) button state now set to raw button state
 }

 bool btnPressed = (lastBtnState == HIGH && btnState == LOW);    // stable state change, so the button is confirmed pressed
 bool btnReleased = (lastBtnState == LOW && btnState == HIGH);   // same idea but for release

 lastBtnState = btnState;                             // resetting former stable state to current

 bool startInflation = false;
 bool stopInflation = false;

   if (AUTO_START && state == IDLE) {                                     // optional coded automatic start
    startInflation = true;
   }
   
   if (!AUTO_START && btnPressed && (state == IDLE || state == DONE)) {    // regular button press changes state to inflate
    startInflation = true;
   }
   
   if (btnPressed && state == INFLATE) {              // manually stopping inflation with button
    stopInflation = true;
   }

   if (startInflation) {
    state = INFLATE;
       timeStart = millis();
       timeLastReading = 0;
       resetOscInfo();
       maxInflateVoltage = 0.0f;                      // testing max voltage found during inflation (as opposed to deflation)
       maxDeflateVoltage = 0.0f;                      // similarly testing max voltage found during deflation
       newbp = true;
       pumpOn(true);                                  // when inflating, pump is on
       valveOpen(false);                              // when inflating, valve is closed
       Serial.println(F("Inflation beginning."));
   }
   
   if (stopInflation) {
        pumpOn(false);                                // stops inflation process if button is pressed
        valveOpen(true);
        state = DONE;
        Serial.println(F("Inflation process halted by user."));
   }
  

unsigned long now = millis();
if (now - timeLastReading >= DT_MS) {
  timeLastReading = now;

  int adc_pressure = analogRead(PIN_PRESSURE);
  float v_pressure_inflate = adcToVoltsInflate(adc_pressure);
  float v_pressure_deflate = adcToVoltsDeflate(adc_pressure);
  float P_mmhg = voltsToMmhg(v_pressure_deflate);

  int adc_oscillometric = analogRead(PIN_OSC);
  float v_osc = adcToVoltsDeflate(adc_oscillometric);
  float mov_avg = movingAvg(v_osc);                     // taking moving average of oscillometric signal to help establish offset
  float offset_osc = v_osc - mov_avg;                   // giving signal dc offset to correctly recenter it 
  float osc_signal = smoothShort(offset_osc);           // smoothing to obtain clean oscillometric signal values
  bool signPos = (osc_signal >= 0);

  // tracking extrema inside current beat
  if (inBeat) {
    if (osc_signal > curPeak)   curPeak = osc_signal;
    if (osc_signal < curTrough) curTrough = osc_signal;
  }

  // detecting positive zero-crossing
  if (!lastPos && signPos) {

    // finish previous beat
    if (inBeat && state == DEFLATE) {
      float amp = curPeak - curTrough;

      if ((now - timeDeflateStart) >= FIRST_MS) {      // the first second of each deflation run, we don't collect values (higher likelihood of inaccuracy)
        finalizeBeat(P_mmhg, now - timeStart, amp);
      }       
    }

    // start new beat window
    inBeat = true;
    curPeak = -1e9; curTrough = 1e9;
  }
  lastPos = signPos;

  switch(state) {
    case IDLE:                                          // system idle, so nothing's happening
    break;

    case INFLATE: {
      
      if (v_pressure_inflate > maxInflateVoltage) {            // for testing purposes, keeping track of max voltage recorded during inflate (as opposed to deflate)
        maxInflateVoltage = v_pressure_inflate;
      }

      bool reached_P_mmhg = (P_mmhg >= TARGET_MMHG);        
      bool reached_volts = (maxInflateVoltage >= TARGET_VOLTAGE);
      bool reached = reached_P_mmhg || reached_volts;

      bool timeout = (now - timeStart) > MAX_INFLATE_TIME_MS;

      // stopping inflation when I reach either of my targets
      // but now I'm holding off opening the valve for 100 ms first, because otherwise the arduino voltage calculations between inf/def are off
      if (reached) {
        pumpOn(false);
        valveOpen(false);
        state = DEFLATE;
        timeDeflateStart = now;
        Serial.println(F("Now beginning deflation."));
        Serial.print(F("Max inflation voltage = "));
        Serial.println(maxInflateVoltage, 4);

      // stopping inflation automatically if the system is inflating for longer than the max time set
      // but now I'm holding off opening the valve for 100 ms first, because otherwise the arduino voltage calculations between inf/def are off
      } else if (timeout) {
          pumpOn(false);
          valveOpen(false);
          state = DEFLATE;
          timeDeflateStart = now;
          Serial.print(F("Max inflation voltage = "));
          Serial.println(maxInflateVoltage, 4);
          Serial.println(F("Safety stop reached. Inflation halted automatically."));
      }        
    break;
    }
  
// closes valve after deflation is finished
    case DEFLATE: {

      unsigned long deflateElapsed = now - timeDeflateStart;

      // gotta briefly hold pressure while Arduino/ADC supply recovers from pump load
      if (deflateElapsed < DEFLATE_WAIT_TIME) {
        valveOpen(false);
        }

      // beginning inflation
      else {
        valveOpen(true);
      }
      
      if (v_pressure_deflate > maxDeflateVoltage) {                                 // keeping track of max voltage recorded during deflate
        maxDeflateVoltage = v_pressure_deflate;
      }


    bool hitLowPressure = (P_mmhg <= DEFLATE_PRES);                   // deflation halts once we hit 50 mmHg
    bool timeout = (now - timeDeflateStart >= MAX_DEFLATE_TIME_MS);   // deflation halts automatically if it exceeds around 1.5 minutes (value raised for new system goals)

    if (hitLowPressure || timeout) {              
        valveOpen(true);
        state = DONE;
        Serial.print(F("Max deflation voltage = "));
        Serial.println(maxDeflateVoltage, 4);
        Serial.println(F("Deflation concluded! :D"));
    }
    break;
    }

    case DONE:
    if (newbp){
      float MAP, SBP, DBP;
      bool findValues = computeBP(MAP, SBP, DBP);

      if (findValues) {
        
        Serial.print(" Final recorded pressure: ");
        Serial.println(P_mmhg);

      Serial.print("Blood pressure values: ");
      Serial.print("MAP = ");
      Serial.print(MAP);
      Serial.print("mmhg. SBP = ");
      Serial.print(SBP);
      Serial.print("mmhg. DBP = ");
      Serial.print(DBP);
      Serial.print("mmhg.");

      newbp = false;
      }

      else {
        Serial.println("Value determination error.");
      }
      
      valveOpen (true);
      state = IDLE;


    break;
  }
}
}
}