/*

  main.cpp

*/
// Include Config-file (moved for code clarity)
#include "main_config.h"
const String fileVersion = __TIMESTAMP__;

// Create Sensor Objects with Specified Slave-Select Pins
ADNS adnsA(CS_PIN_A);
ADNS adnsB(CS_PIN_B);
sensor_pair_t sensor = {adnsA, adnsB};

// Create Timing Objects
const int32_t samplePeriodMicros = 1000000L / (int32_t)NAVSENSOR_FPS;

// Capture Task (on interrupt)
IntervalTimer captureTimer;

// Create debounced manual input button
Bounce onoffSwitch;

// Initialize sample buffer
CircularBuffer<sensor_sample_t, 10> sensorSampleBuffer;

// Counter and Timestamp Generator
elapsedMicros microsSinceAcquisitionStart;
elapsedMicros microsSinceFrameStart;
// volatile time_t currentSampleTimestamp;
volatile int32_t sampleCountRemaining = 0;
volatile time_t currentFrameTimestamp;
volatile time_t currentFrameDuration;
volatile uint32_t currentFrameCount;

volatile bool isRunning = false;

// =============================================================================
//   SETUP & LOOP
// =============================================================================
#include "DeviceLib/devicemanager.h"
void setup() {
  delay(400);
  bool success;
  success = initializeCommunication();
  // if (success) {
  //   Serial.println("Communication initialized");
  //   Serial.println(fileVersion);
  // }
  delay(400);
  // printBoardID();

  success = initializeSensors();
  // if (success) {
  // Serial.println("Sensors initialized");
  // }
  success = initializeTriggering();
  // if (success) {
  // Serial.println("Triggers initialized");
  // }
  // sampleCountRemaining = 480;
  // cmdTicker.start();
  // transmitTicker.start();
  // delay(20);
}

void loop() {
  if (onoffSwitch.update()) {
    if (onoffSwitch.fell()) {
      beginAcquisition();
    } else {
      if (onoffSwitch.rose()) {
        endAcquisition();
      }
    }
  }

  // if (sampleCountRemaining > 0 {
  //   if (!isRunning) {
  //     beginAcquisition();
  //   }
  // } else {
  //   if (isRunning) {
  //     endAcquisition();
  //     delay(10);
  //   }
  //   receiveCommand();
  // }
  // // transmitTicker.update();
  // // cmdTicker.update();
  // // hbTicker.update();
}

// =============================================================================
//   TASKS: INITIALIZE
// =============================================================================
inline static bool initializeCommunication() {
  // Begin Serial
  Serial.begin(115200);
  while (!Serial) {
    ;  // may only be needed for native USB port
  }
  delay(10);
  return true;
};
inline static bool initializeSensors() {
  // Begin Sensors
  sensor.left.begin();
  delay(30);
  sensor.right.begin();
  delay(30);
  return true;
};
inline static bool initializeClocks() { return true; }
inline static bool initializeTriggering() {
  // Set Sync In Pin Mode
  fastPinMode(TRIGGER_IN_PIN, INPUT_PULLUP);
  // fastPinMode(MANUAL_TRIGGER_PIN, INPUT_PULLUP);
  onoffSwitch.attach(MANUAL_TRIGGER_PIN, INPUT_PULLUP);
  onoffSwitch.interval(50);
  // Set Sync Out Pin Modes
  fastPinMode(TRIGGER_OUT_1_PIN, OUTPUT);
  fastPinMode(TRIGGER_OUT_2_PIN, OUTPUT);
  fastPinMode(TRIGGER_OUT_3_PIN, OUTPUT);
  fastDigitalWrite(TRIGGER_OUT_1_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_2_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_3_PIN, !TRIGGER_ACTIVE_STATE);
  delay(1);
  // Setup Sync/Trigger-Output Timing
  // FrequencyTimer2::setPeriod(1e6 / DISPLACEMENT_SAMPLE_RATE)
  return true;
};

// =============================================================================
// TASKS: IDLE
// =============================================================================
static inline void receiveCommand() {
  // Read Serial to see if request for more frames has been sent
  if (Serial.available()) {
    // delayMicroseconds(1000);
    int32_t moreSamplesCnt = Serial.parseInt();
    sampleCountRemaining += moreSamplesCnt;
  }
}

static inline void beginAcquisition() {
  if (!isRunning) {
    // Print units and Fieldnames (header)
    sendHeader();

    // Trigger start using class methods in ADNS library
    sensor.left.triggerAcquisitionStart();
    sensor.right.triggerAcquisitionStart();

    // Flush sensors (should happen automatically -> needs bug fix)
    sensor.left.triggerSampleCapture();
    sensor.right.triggerSampleCapture();

    // Change State
    isRunning = true;
    currentFrameCount = 0;

    // Reset Elapsed Time Counter
    microsSinceAcquisitionStart = 0;
    // currentSampleTimestamp = microsSinceAcquisitionStart;
    microsSinceFrameStart = microsSinceAcquisitionStart;
    currentFrameDuration = microsSinceFrameStart;

    // Call begin-data-frame function (sets trigger outputs)
    beginDataFrame();

    // Begin IntervalTimer
    captureTimer.begin(captureDisplacement, samplePeriodMicros);
  }
}
static inline void beginDataFrame() {
  // Set Trigger Outputs to Active state
  fastDigitalWrite(TRIGGER_OUT_1_PIN, TRIGGER_ACTIVE_STATE);
  static int triggerOut2Cnt = 1;
  if (--triggerOut2Cnt <= 0) {
    fastDigitalWrite(TRIGGER_OUT_2_PIN, TRIGGER_ACTIVE_STATE);
    triggerOut2Cnt = TRIGGER_OUT_2_DIVISOR;
  }
  static int triggerOut3Cnt = 1;
  if (--triggerOut3Cnt <= 0) {
    fastDigitalWrite(TRIGGER_OUT_3_PIN, TRIGGER_ACTIVE_STATE);
    triggerOut3Cnt = TRIGGER_OUT_3_DIVISOR;
  }

  // Latch timestamp and designate/allocate current sample
  microsSinceFrameStart -= currentFrameDuration;
  currentFrameTimestamp = microsSinceAcquisitionStart;
  currentFrameCount += 1;
}
static inline void endDataFrame() {
  // Latch Frame Duration and Send Data
  currentFrameDuration = microsSinceFrameStart;

  fastDigitalWrite(TRIGGER_OUT_1_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_2_PIN, !TRIGGER_ACTIVE_STATE);
  fastDigitalWrite(TRIGGER_OUT_3_PIN, !TRIGGER_ACTIVE_STATE);
}
static inline void endAcquisition() {
  if (isRunning) {
    // End IntervalTimer
    captureTimer.end();
    endDataFrame();

    // Trigger start using class methods in ADNS library
    sensor.left.triggerAcquisitionStop();
    sensor.right.triggerAcquisitionStop();
    sensorSampleBuffer.clear();

    // Change state
    isRunning = false;
  }
}

// =============================================================================
// TASKS: TRIGGERED_ACQUISITION
// =============================================================================
void captureDisplacement() {
  // // Unset Trigger Outputs
  endDataFrame();

  // Initialize container for combined & stamped sample
  sensor_sample_t currentSample;
  currentSample.timestamp = currentFrameTimestamp;

  // Trigger capture from each sensor
  sensor.left.triggerSampleCapture();
  sensor.right.triggerSampleCapture();

  // Store timestamp for next frame
  currentFrameTimestamp = microsSinceAcquisitionStart;

  currentSample.left = {'L', sensor.left.readDisplacement(units)};
  currentSample.right = {'R', sensor.right.readDisplacement(units)};
  // todo: no units

  // Push current sample into buffer for transmission
  sensorSampleBuffer.push(currentSample);

  // Decrement sample counter and time counter
  if (sampleCountRemaining >= 0) {
    sampleCountRemaining--;
  }

  // Send Data
  sendData();

  beginDataFrame();
}

// =============================================================================
// TASKS: DATA_TRANSFER
// =============================================================================

void sendHeader() {
  const String dunit = getAbbreviation(units.distance);
  const String tunit = getAbbreviation(units.time);
  // Serial.flush();
  Serial.print(String(
      String("timestamp [us]") + delimiter + flatFieldNames[0] + " [" + dunit +
      "]" + delimiter + flatFieldNames[1] + " [" + dunit + "]" + delimiter +
      flatFieldNames[2] + " [" + tunit + "]" + delimiter + flatFieldNames[3] +
      " [" + dunit + "]" + delimiter + flatFieldNames[4] + " [" + dunit + "]" +
      delimiter + flatFieldNames[5] + " [" + tunit + "]" + "\n"));
}

void sendData() {
  // Print Velocity
  while (!(sensorSampleBuffer.isEmpty())) {
    sensor_sample_t sample = sensorSampleBuffer.shift();

    // Convert to String class
    const String timestamp = String(sample.timestamp);
    const String dxL = String(sample.left.p.dx, decimalPlaces);
    const String dyL = String(sample.left.p.dy, decimalPlaces);
    const String dtL = String(sample.left.p.dt, decimalPlaces);
    const String dxR = String(sample.right.p.dx, decimalPlaces);
    const String dyR = String(sample.right.p.dy, decimalPlaces);
    const String dtR = String(sample.right.p.dt, decimalPlaces);
    const String endline = String("\n");

    // Serial.availableForWrite
    // Print ASCII Strings
    Serial.print(timestamp + delimiter + dxL + delimiter + dyL + delimiter +
                 dtL + delimiter + dxR + delimiter + dyR + delimiter + dtR +
                 endline);
  }
}
