#include <RH_ASK.h>
#include <SPI.h>

//#define BAUDRATE 115200
#define PILOT_ID 1 //Pilot ID for this module
#define ENABLE_RF //Enable RF transmission
#define BURST_NUM 10 //Number of burst to send in RF
#define RF_NEWLAP_COMMANDID 2
#define RF_NEWLAP_PARAMETER 1

// number of analog rssi reads to average for the current check.
// single analog read with FASTADC defined (see below) takes ~20us on 16MHz arduino
// so e.g. 10 reads will take 200 ms, which gives resolution of 5 RSSI reads per ms,
// this means that we can theoretically have 1ms timing accuracy :)
#define RSSI_READS 4 // numer of rssi reads to average the value

//----- RSSI --------------------------------------
#define FILTER_ITERATIONS 5 // software filtering iterations; set 0 - if filtered in hardware; set 5 - if not
uint16_t rssiArr[FILTER_ITERATIONS + 1];
uint16_t rssiThreshold = 190;
uint16_t rssi; // rssi measured using first (slight) filter
uint16_t rssi2; // rssi measured using second (deeper/slower) filter
uint16_t rssi3; // rssi measured using third (even deeper/slower) filter that is expected to produce a smooth curve
uint16_t rssiForThresholdSetup; // special rssi for threshold setup (slooow filter)

#define PROXIMITY_STEPS 20 // starts tracking proximity rssi from threshold decreased by this amount
bool isApproaching = false;
const uint16_t proximityTimesArray[PROXIMITY_STEPS] = {
    20, 50, 100, 200, 500, 700, 1000, 1200, 1500, 2000,
    2500, 2500, 2500, 2500, 2500, 2500, 3000, 3000, 3000, 3000
};
uint8_t currentProximityIndex;
uint32_t currentProximityIndexTime;

//lap detection variables
bool isFirstThresholdCrossed;
bool didLeaveDeviceAreaThisLap;
bool isLapDetectionTimeoutExpired;
uint16_t upperSecondLevelRssiThreshold;
uint16_t lowerSecondLevelThreshold;
uint16_t minDeepRssi;
uint16_t maxDeepRssi;
uint16_t maxRssi;
uint16_t maxDeepRssiAfterFirstThreshold;
uint32_t timeWhenMaxAfterFirstThresholdWasDetected;
uint32_t timeWhenFirstThresholdCrossed;
uint32_t maxRssiDetectionTime;
uint32_t maxDeepRssiDetectionTime;
uint32_t minDeepRssiDetectionTime;

uint32_t deepFilteredRssiMultiplied;
uint32_t smoothlyFilteredRssiMultiplied;

#define MAIN_RSSI_FILTER_CONSTANT 60 // this is a filtering constant that regulates both smoothness of filtering and delay at the same time
#define DEEP_FILTER_CONSTANT 100 // filtering constant for the second filter which generates threshold values for next lap
#define SMOOTH_FILTER_CONSTANT 1000 // filtering constant for the third filter which is used to track minimum rssi values to detect if drone left the finish gate area

#define DEFAULT_MAX_RSSI_SEARCH_DELAY 3000 // time to watch for max value after crossing a threshold
#define ALLOWED_LAP_DETECTION_TIMEOUT 1000 // time allowed for lap detection after first threshold is crossed

// #define RELIABLE_RSSI_DETECTION_SUBTRACT 4 // decrease found upperSecondLevelRssiThreshold by this amount to track initial threshold cross event on all laps after 1st
#define SECOND_LEVEL_RSSI_DETECTION_ADJUSTMENT 2 // decrease found maxDeepRssi by this amount to make next lap detection more reliable
#define EDGE_RSSI_ADJUSTMENT 10 // decrease found maximum (and increase minimum) by this value after each lap to better find a new one

#define RSSI_MAX 1024
#define RSSI_MIN 0
#define THRESHOLD_ARRAY_SIZE  100
uint16_t rssiThresholdArray[THRESHOLD_ARRAY_SIZE];

#define MIN_RSSI_MONITOR_INTERVAL 1 // in milliseconds
uint16_t rssiMonitorInterval = 0; // zero means the RSSI monitor is OFF
uint32_t lastRssiMonitorReading = 0; // millis when rssi monitor value was last read

#define RSSI_SETUP_INITIALIZE 0
#define RSSI_SETUP_NEXT_STEP 1

//----- Lap timings--------------------------------
uint32_t lastMilliseconds = 0;
uint32_t now = 0;
uint32_t raceStartTime = 0;
#define MIN_MIN_LAP_TIME 1 //seconds
#define MAX_MIN_LAP_TIME 120 //seconds
uint8_t minLapTime = 10; //seconds
#define MAX_LAPS 100
uint32_t lapTimes[MAX_LAPS];

//----- Time Adjustment (for accuracy) ------------
#define INFINITE_TIME_ADJUSTMENT 0x7FFFFFFFF // max positive 32 bit signed number
// Usage of signed int time adjustment constant inside this firmware:
// * calibratedMs = readMs + readMs/timeAdjustment
// Usage of signed int time adjustment constant from outside:
// * set to zero means time adjustment procedure was not performed for this node
// * set to INFINITE_TIME_ADJUSTMENT, means time adjustment was performed, but no need to adjust
int32_t timeAdjustment = 0;

//----- other globals------------------------------
uint8_t allowLapGeneration = 0;
uint8_t channelIndex = 2;
uint8_t bandIndex = 1;
uint8_t raceMode = 0; // 0: race mode is off; 1: lap times are counted relative to last lap end; 2: lap times are relative to the race start (sum of all previous lap times);
uint8_t isSoundEnabled = 1;
uint8_t isConfigured = 0; //changes to 1 if any input changes the state of the device. it will mean that externally stored preferences should not be applied
uint8_t newLapIndex = 0;
uint8_t shouldWaitForFirstLap = 0; // 0 means start table is before the laptimer, so first lap is not a full-fledged lap (i.e. don't respect min-lap-time for the very first lap)
uint8_t isSendingData = 0;
uint8_t sendStage = 0;
uint8_t sendLapTimesIndex = 0;
uint8_t sendLastLapIndex = 0;
uint8_t shouldSendSingleItem = 0;
uint8_t lastLapsNotSent = 0;
uint8_t thresholdSetupMode = 0;
uint8_t experimentalMode = 0;
uint16_t frequency = 0;
uint32_t millisUponRequest = 0;
uint8_t isModuleActive = 1; // 0 means this module is inactive and the VRX is disabled

int seq_id = 1; //RF Sequence ID

#include "fastReadWrite.h"
#include "fastADC.h"
#include "pinAssignments.h"
//------- RF Module ----------
RH_ASK driver(2000, 11, rfTxPin, 5); //Not receiving, just transmitting
#include "RF433.h"
#include "channels.h"
#include "rx5808spi.h"
#include "sounds.h"
#include "lapDetectionRoutines.h"
#include "mainDetectionAlgorithm.h"

//------- DEBUG ----------
//#define ENABLE_DEBUG
#define DEBUG_CYCLES 4000
int debug_cycle = DEBUG_CYCLES;


// ----------------------------------------------------------------------------
void setModuleActive(uint8_t active) {
    isModuleActive = active;
    if(active) {
        resetModule();
        // We need to set the freqency again on power up
        setModuleFrequency(frequency);
    } else {
        powerDownModule();
    }
}
// ----------------------------------------------------------------------------
// this is just a digital filter function
uint16_t getFilteredRSSI() {
    static uint32_t filteredRssiMultiplied;

    uint16_t localRssi = readRSSI();

    filteredRssiMultiplied = localRssi  + (filteredRssiMultiplied * (MAIN_RSSI_FILTER_CONSTANT - 1) / MAIN_RSSI_FILTER_CONSTANT );
    return filteredRssiMultiplied / MAIN_RSSI_FILTER_CONSTANT;

}
// ----------------------------------------------------------------------------
// this is just a digital filter function
// it runs over already filtered rssi
uint16_t getDeepFilteredRSSI() {
    deepFilteredRssiMultiplied = rssi  + (deepFilteredRssiMultiplied * (DEEP_FILTER_CONSTANT - 1) / DEEP_FILTER_CONSTANT );
    return deepFilteredRssiMultiplied / DEEP_FILTER_CONSTANT;
}
// ----------------------------------------------------------------------------
// this is just a digital filter function
// it runs over already filtered rssi
uint16_t getSmoothlyFilteredRSSI() {
    smoothlyFilteredRssiMultiplied = rssi  + (smoothlyFilteredRssiMultiplied * (SMOOTH_FILTER_CONSTANT - 1) / SMOOTH_FILTER_CONSTANT );
    return smoothlyFilteredRssiMultiplied / SMOOTH_FILTER_CONSTANT;
}
// ----------------------------------------------------------------------------
// this is just a digital filter function
uint16_t getRssiForAutomaticThresholdSetup() {
    #define SLOW_TIME_DELAY_CONSTANT 1000 // this is a filtering constant that regulates both depth of filtering and delay at the same time
    static uint32_t slowChangingRssiMultiplied;

    slowChangingRssiMultiplied = rssi  + (slowChangingRssiMultiplied * (SLOW_TIME_DELAY_CONSTANT - 1) / SLOW_TIME_DELAY_CONSTANT );
    return slowChangingRssiMultiplied / SLOW_TIME_DELAY_CONSTANT;
}
// ----------------------------------------------------------------------------
void sortArray(uint16_t a[], uint16_t size) {
    for(uint16_t i=0; i<(size-1); i++) {
        for(uint16_t j=0; j<(size-(i+1)); j++) {
                if(a[j] > a[j+1]) {
                    uint16_t t = a[j];
                    a[j] = a[j+1];
                    a[j+1] = t;
                }
        }
    }
}
// ----------------------------------------------------------------------------
uint16_t getMedian(uint16_t a[], uint16_t size) {
    return a[size/2];
}
// ----------------------------------------------------------------------------
uint16_t readRSSI() {
    int rssiA = 0;

    analogRead(rssiPinA); // first fake read to improve further readings accuracy (as suggested by Nicola Gorghetto)

    for (uint8_t i = 0; i < RSSI_READS; i++) {
        rssiA += analogRead(rssiPinA);
    }

    rssiA = rssiA/RSSI_READS; // average of RSSI_READS readings
    return rssiA;
}

void setupThreshold(uint8_t phase) {
    // this process assumes the following:
    // 1. before the process all VTXs are turned ON, but are distant from the Chorus device, so that Chorus sees the "background" rssi values only
    // 2. once the setup process is initiated by Chorus operator, all pilots walk towards the Chorus device
    // 3. setup process starts tracking top rssi values
    // 4. as pilots come closer, rssi should rise above the value defined by RISE_RSSI_THRESHOLD_PERCENT
    // 5. after that setup expects rssi to fall from the reached top, down by FALL_RSSI_THRESHOLD_PERCENT
    // 6. after the rssi falls, the top recorded value (decreased by TOP_RSSI_DECREASE_PERCENT) is set as a threshold

    // time constant for accumulation filter: higher value => more delay
    // value of 20 should give about 100 readings before value reaches the settled rssi
    // don't make it bigger than 2000 to avoid overflow of accumulatedShiftedRssi
    #define ACCUMULATION_TIME_CONSTANT 150
    #define MILLIS_BETWEEN_ACCU_READS 10 // artificial delay between rssi reads to slow down the accumulation
    #define TOP_RSSI_DECREASE_PERCENT 10 // decrease top value by this percent using diff between low and high as a base
    #define RISE_RSSI_THRESHOLD_PERCENT 25 // rssi value should pass this percentage above low value to continue finding the peak and further fall down of rssi
    #define FALL_RSSI_THRESHOLD_PERCENT 50 // rssi should fall below this percentage of diff between high and low to finalize setup of the threshold

    static uint16_t rssiLow;
    static uint16_t rssiHigh;
    static uint16_t rssiHighEnoughForMonitoring;
    static uint32_t accumulatedShiftedRssi; // accumulates rssi slowly; contains multiplied rssi value for better accuracy
    static uint32_t lastRssiAccumulationTime;

    if (!thresholdSetupMode) return; // just for safety, normally it's controlled outside

    if (phase == RSSI_SETUP_INITIALIZE) {
        // initialization step
        playThresholdSetupStartTones();
        thresholdSetupMode = 1;
        rssiLow = rssiForThresholdSetup; // using rssiForThresholdSetup to avoid catching random current rssi
        rssiHigh = rssiLow;
        accumulatedShiftedRssi = rssiLow * ACCUMULATION_TIME_CONSTANT; // multiply to prevent loss in accuracy
        rssiHighEnoughForMonitoring = rssiLow + rssiLow * RISE_RSSI_THRESHOLD_PERCENT / 100;
        lastRssiAccumulationTime = millis();
        Serial.println("Setup threshold Start");
    } else {
        // active phase step (searching for high value and fall down)
        if (thresholdSetupMode == 1) {
            // in this phase of the setup we are tracking rssi growth until it reaches the predefined percentage from low

            // searching for peak; using rssiForThresholdSetup to avoid catching sudden random peaks
            if (rssiForThresholdSetup > rssiHigh) {
                rssiHigh = rssiForThresholdSetup;
            }

            // since filter runs too fast, we have to introduce a delay between subsequent readings of filter values
            uint32_t curTime = millis();
            if ((curTime - lastRssiAccumulationTime) > MILLIS_BETWEEN_ACCU_READS) {
                lastRssiAccumulationTime = curTime;
                // this is actually a filter with a delay determined by ACCUMULATION_TIME_CONSTANT
                accumulatedShiftedRssi = rssi  + (accumulatedShiftedRssi * (ACCUMULATION_TIME_CONSTANT - 1) / ACCUMULATION_TIME_CONSTANT);
            }

            uint16_t accumulatedRssi = accumulatedShiftedRssi / ACCUMULATION_TIME_CONSTANT; // find actual rssi from multiplied value

            if (accumulatedRssi > rssiHighEnoughForMonitoring) {
                thresholdSetupMode = 2;
                accumulatedShiftedRssi = rssiHigh * ACCUMULATION_TIME_CONSTANT;
                playThresholdSetupMiddleTones();
                Serial.println("Setup threshold Middle");
            } else {
              #ifdef ENABLE_DEBUG
                debug_cycle--;
                if (debug_cycle<=0) {
                  debug_cycle = DEBUG_CYCLES;
                  Serial.print("MIDDLE Threshold Debug - testing: ");
                  Serial.print(accumulatedRssi);
                  Serial.print(">");
                  Serial.println(rssiHighEnoughForMonitoring);
                }
              #endif
            }
        } else {
            // in this phase of the setup we are tracking highest rssi and expect it to fall back down so that we know that the process is complete

            // continue searching for peak; using rssiForThresholdSetup to avoid catching sudden random peaks
            if (rssiForThresholdSetup > rssiHigh) {
                rssiHigh = rssiForThresholdSetup;
                accumulatedShiftedRssi = rssiHigh * ACCUMULATION_TIME_CONSTANT; // set to highest found rssi
            }

            // since filter runs too fast, we have to introduce a delay between subsequent readings of filter values
            uint32_t curTime = millis();
            if ((curTime - lastRssiAccumulationTime) > MILLIS_BETWEEN_ACCU_READS) {
                lastRssiAccumulationTime = curTime;
                // this is actually a filter with a delay determined by ACCUMULATION_TIME_CONSTANT
                accumulatedShiftedRssi = rssi  + (accumulatedShiftedRssi * (ACCUMULATION_TIME_CONSTANT - 1) / ACCUMULATION_TIME_CONSTANT );
            }
            uint16_t accumulatedRssi = accumulatedShiftedRssi / ACCUMULATION_TIME_CONSTANT;

            uint16_t rssiLowEnoughForSetup = rssiHigh - (rssiHigh - rssiLow) * FALL_RSSI_THRESHOLD_PERCENT / 100;
            if (accumulatedRssi < rssiLowEnoughForSetup) {
                rssiThreshold = rssiHigh - ((rssiHigh - rssiLow) * TOP_RSSI_DECREASE_PERCENT) / 100;
                thresholdSetupMode = 0;
                isConfigured = 1;
                playThresholdSetupDoneTones();
                Serial.println("Setup threshold DONE");
                raceMode = 1;
            } else {
              #ifdef ENABLE_DEBUG
                debug_cycle--;
                if (debug_cycle<=0) {
                  debug_cycle = DEBUG_CYCLES;
                  Serial.print("FINAL Threshold Debug - testing: ");
                  Serial.print(accumulatedRssi);
                  Serial.print("<");
                  Serial.println(rssiLowEnoughForSetup);
                }
              #endif
            }
        }
    }
}

// ----------------------------------------------------------------------------
void setThresholdValue(uint16_t threshold) {
    // stop the "setting threshold algorithm" to avoid overwriting the explicitly set value
    if (thresholdSetupMode) {
        thresholdSetupMode = 0;
    }
    rssiThreshold = threshold;
    if (threshold != 0) {
        playClickTones();
    } else {
        playClearThresholdTones();
    }
}



void setup() {
      // initialize led pin as output.
    pinMode(ledPin, OUTPUT);
    digitalHigh(ledPin);
    pinMode(led1Pin, OUTPUT);
    digitalHigh(led1Pin);

    // init buzzer pin
    pinMode(buzzerPin, OUTPUT);

    //init button
    pinMode(buttonPin, INPUT);

    //Init RF
    if (!driver.init())
      Serial.println("RF init failed");

    //init raspberrypi interrupt generator pin
    pinMode(pinRaspiInt, OUTPUT);
    digitalLow(pinRaspiInt);

    // SPI pins for RX control
    setupSPIpins();

    // set the channel as soon as we can
    // faster boot up times :)
    frequency = setModuleChannel(channelIndex, bandIndex);
    setModuleActive(true);

    initFastADC();

    // Setup Done - Turn Status ledPin off.
    digitalLow(ledPin);

    Serial.begin(9600);
    Serial.println("Started");
    Serial.print("Frequency: ");
    Serial.println(frequency);
}

void loop() {
    // read the state of the pushbutton value:
  int buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH && thresholdSetupMode == 0) {
    thresholdSetupMode = 1;
    setupThreshold(RSSI_SETUP_INITIALIZE);
  }
  
  rssi = getFilteredRSSI();
      if (!raceMode) { // no need to get rssiForThresholdSetup during race time because it's used only in threshold setting, which is already set by the race time
        rssiForThresholdSetup = getRssiForAutomaticThresholdSetup(); // filter RSSI
    }
    runLapDetectionAlgorithm();

    if (thresholdSetupMode) {
        setupThreshold(RSSI_SETUP_NEXT_STEP);
    }

    if (isSoundEnabled && playSound) {
        if (playStartTime == 0) {
            tone(buzzerPin,curToneSeq[curToneIndex]);
            playStartTime = millis();
        }
        uint32_t dur = millis() - playStartTime;
        if (dur >= curToneSeq[curDurIndex]) {
            if (curDurIndex >= lastToneSeqIndex) {
                noTone(buzzerPin);
                playSound = 0;
            } else {
                curToneIndex += 2;
                curDurIndex += 2;
                tone(buzzerPin, curToneSeq[curToneIndex]);
                playStartTime = millis();
            }
        }
    }
    
  //Serial.println(rssi);
  //delay(500);
}
