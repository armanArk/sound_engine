char codeVersion[] = "9.13.0"; // Software revision.

#include <Arduino.h>
#include "0_GeneralSettings.h"
#include "1_Vehicle.h"         
#include "2_Remote.h"          
#include "3_ESC.h"             
#include "4_Transmission.h"    
#include "5_Shaker.h"          
#include "6_Lights.h"          
#include "7_Servos.h"
#include "8_Sound.h"
#include "9_Dashboard.h"
#include "10_Trailer.h"        

#include <statusLED.h>        // https://github.com/TheDIYGuy999/statusLED <<------- required for LED control
#if not defined EMBEDDED_SBUS // SBUS library only required, if not embedded SBUS code is used -----------
#include <SBUS.h>             // https://github.com/TheDIYGuy999/SBUS      <<------- you need to install my fork of this library!
#endif                        // ------------------------------------------------------------------------------------------------
#include <rcTrigger.h>        // https://github.com/TheDIYGuy999/rcTrigger <<------- required for RC signal processing
#include <IBusBM.h>           // https://github.com/bmellink/IBusBM        <<------- required for IBUS interface
#include <TFT_eSPI.h>         // https://github.com/Bodmer/TFT_eSPI        <<------- required for LCD dashboard. Use v2.3.70
#if defined NEOPIXEL_ENABLED
#include <FastLED.h> // https://github.com/FastLED/FastLED        <<------- required for Neopixel support. Use V3.3.3
#endif
#include <ESP32AnalogRead.h> // https://github.com/madhephaestus/ESP32AnalogRead <<------- required for battery voltage measurement
#include <Tone32.h>          // https://github.com/lbernstone/Tone32      <<------- required for battery cell detection beeps // Not for platform = espressif32@4.3.0

// Additional headers (included)
#include "src/curves.h"    // Nonlinear throttle curve arrays
#include "src/helper.h"    // Various stuff
#include "src/dashboard.h" // For LCD dashboard. See: https://github.com/Gamadril/Rc_Engine_Sound_ESP32
#include "src/SUMD.h"      // For Graupner SUMD interface. See: https://github.com/Gamadril/Rc_Engine_Sound_ESP32
#if defined EMBEDDED_SBUS
#include "src/sbus.h" // For SBUS interface
#endif

// No need to install these, they come with the ESP32 board definition
#include "driver/uart.h"  // for UART macro UART_PIN_NO_CHANGE
#include "driver/rmt.h"   // for PWM signal detection
#include "driver/mcpwm.h" // for servo PWM output
#include "rom/rtc.h"      // for displaying reset reason
#include "soc/rtc_wdt.h"  // for watchdog timer
#include <esp_now.h>      // for wireless trailer
#include <WiFi.h>
// #include <esp_wifi.h>
// #include <Esp.h>    // for displaying memory information
#include <EEPROM.h> // for non volatile variable storage
#include <handle_canbus.h>
// Forward declare functions
void Task1code(void *parameters);
void readPwmSignals();
void processRawChannels();
void channelZero();
void eepromDebugRead();
void eepromRead();
void eepromInit();

// Serial DEBUG pins -----
#ifdef WEMOS_D1_MINI_ESP32 // Wemos D1 Mini board: GPIO3 is available
#define DEBUG_RX 3
#else // original board: GPIO3 is used for headlights!
#define DEBUG_RX UART_PIN_NO_CHANGE
#endif
#define DEBUG_TX 1 // The "RX0" is on pin 1

#define COMMAND_RX 36                 // pin 36, labelled with "VP", connect it to "Micro RC Receiver" pin "TXO"
#define COMMAND_TX UART_PIN_NO_CHANGE // 98 is just a dummy -1 (17 reversing)

#define COUPLER_PIN 27  // CH4 output for coupler (5th. wheel) servo (bus communication only)

#ifdef WEMOS_D1_MINI_ESP32 // switching headlight pin depending on the board variant
#define HEADLIGHT_PIN 22   // Headlights connected to GPIO 22
#define CABLIGHT_PIN -1    // No Cabin lights
#else
#define HEADLIGHT_PIN 3 // Headlights connected to GPIO 3
#define CABLIGHT_PIN 22 // Cabin lights connected to GPIO 22
#endif

#define TAILLIGHT_PIN 15       // Red tail- & brake-lights (combined)

#ifdef NEOPIXEL_ON_CH4           // Switching NEOPIXEL pin for WS2812 LED depending on setting
#define RGB_LEDS_PIN COUPLER_PIN // Use coupler pin on CH4
#else
#define RGB_LEDS_PIN 0 // Use GPIO 0 (BOOT button)
#endif


#define BATTERY_DETECT_PIN 23 // Shaker motor (shaking truck while idling and engine start / stop)
#define SHAKER_MOTOR_PIN 23 // Shaker motor (shaking truck while idling and engine start / stop)

#define DAC1 25 // connect pin25 (do not change the pin) to a 10kOhm resistor
#define DAC2 26 // connect pin26 (do not change the pin) to a 10kOhm resistor
// both outputs of the resistors above are connected together and then to the outer leg of a
// 20kOhm potentiometer. The other outer leg connects to GND. The middle leg connects to both inputs
// of a PAM8403 amplifier and allows to adjust the volume. This way, two speakers can be used.

// Objects *************************************************************************************
// Status LED objects (also used for PWM shaker motor and ESC control) -----
statusLED headLight(false); // "false" = output not inversed
statusLED shakerMotor(false);

// Battery voltage
ESP32AnalogRead battery;

// Webserver on port 80
WiFiServer server(80);

// Global variables **********************************************************************

// WiFi variables
String ssid;
String password;

// HTTP request memory
String header;

// For HTTP GET value
String valueString = "";
int pos1 = 0;
int pos2 = 0;

// PPM signal processing variables
#define NUM_OF_PPM_CHL 8                       // The number of channels inside our PPM signal (8 is the maximum!)
#define NUM_OF_PPM_AVG 1                       // Number of averaging passes (usually one, more will be slow)
volatile int ppmInp[NUM_OF_PPM_CHL + 1] = {0}; // Input values
volatile int ppmBuf[16] = {0};
volatile byte counter = NUM_OF_PPM_CHL;
volatile byte average = NUM_OF_PPM_AVG;
volatile boolean ready = false;
volatile unsigned long timelast;
unsigned long timelastloop;
uint32_t maxPpmRpmPercentage = 390; // Limit required to prevent controller from crashing @ high engine RPM

// SBUS signal processing variables
#if not defined EMBEDDED_SBUS // ------------------------
SBUS sBus(Serial2);           // SBUS object on Serial 2 port
// channel, fail safe, and lost frames data
uint16_t SBUSchannels[16];
bool SBUSfailSafe;
bool SBUSlostFrame;
#else  // ------------------------------------------------
bfs::SbusRx sBus(&Serial2);
std::array<int16_t, bfs::SbusRx::NUM_CH()> SBUSchannels;
#endif // -----------------------------------------------
bool sbusInit;
uint32_t maxSbusRpmPercentage = 390; // Limit required to prevent controller from crashing @ high engine RPM

// SUMD signal processing variables
HardwareSerial serial(2);
SUMD sumd(serial); // SUMD object on Serial 2 port
// channel, fail safe, and lost frames data
uint16_t SUMDchannels[16]; // only 12 are usable!
bool SUMD_failsafe;
bool SUMD_frame_lost;
bool SUMD_init;
uint32_t maxSumdRpmPercentage = 390; // Limit required to prevent controller from crashing @ high engine RPM

// IBUS signal processing variables
IBusBM iBus; // IBUS object
bool ibusInit;
uint32_t maxIbusRpmPercentage = 320; // Limit required to prevent controller from crashing @ high engine RPM (was 350, but sometimes crashing)

// Interrupt latches
volatile boolean couplerSwitchInteruptLatch; // this is enabled, if the coupler switch pin change interrupt is detected

// Control input signals
#define PULSE_ARRAY_SIZE 14                // 13 channels (+ the unused CH0)
uint16_t pulseWidthRaw[PULSE_ARRAY_SIZE];  // Current RC signal RAW pulse width [X] = channel number
uint16_t pulseWidthRaw2[PULSE_ARRAY_SIZE]; // Current RC signal RAW pulse width with linearity compensation [X] = channel number
uint16_t pulseWidthRaw3[PULSE_ARRAY_SIZE]; // Current RC signal RAW pulse width before averaging [X] = channel number
uint16_t pulseWidth[PULSE_ARRAY_SIZE];     // Current RC signal pulse width [X] = channel number
int16_t pulseOffset[PULSE_ARRAY_SIZE];     // Offset for auto zero adjustment


#define NONE 16                       // The non existing "Dummy" channel number (usually 16) TODO

volatile boolean failSafe = false; // Triggered in emergency situations like: throttle signal lost etc.

boolean mode1; // Signal state variables
boolean mode2;
boolean momentary1;
boolean hazard;
boolean left;
boolean right;
boolean unlock5thWheel;
boolean winchPull;
boolean winchRelease;
boolean winchEnabled;
int8_t winchSpeed;

// Sound
volatile boolean engineOn = false;                // Signal for engine on / off
volatile boolean engineStart = false;             // Active, if engine is starting up
volatile boolean engineRunning = false;           // Active, if engine is running
volatile boolean engineStop = false;              // Active, if engine is shutting down
volatile boolean jakeBrakeRequest = false;        // Active, if engine jake braking is requested
volatile boolean engineJakeBraking = false;       // Active, if engine is jake braking
volatile boolean wastegateTrigger = false;        // Trigger wastegate (blowoff) after rapid throttle drop
volatile boolean blowoffTrigger = false;          // Trigger jake brake sound (blowoff) after rapid throttle drop
volatile boolean dieselKnockTrigger = false;      // Trigger Diesel ignition "knock"
volatile boolean dieselKnockTriggerFirst = false; // The first Diesel ignition "knock" per sequence
volatile boolean airBrakeTrigger = false;         // Trigger for air brake noise
volatile boolean parkingBrakeTrigger = false;     // Trigger for air parking brake noise
volatile boolean shiftingTrigger = false;         // Trigger for shifting noise
volatile boolean hornTrigger = false;             // Trigger for horn on / off
volatile boolean sirenTrigger = false;            // Trigger for siren  on / off
volatile boolean sound1trigger = false;           // Trigger for sound1  on / off
volatile boolean couplingTrigger = false;         // Trigger for trailer coupling  sound
volatile boolean uncouplingTrigger = false;       // Trigger for trailer uncoupling  sound
volatile boolean bucketRattleTrigger = false;     // Trigger for bucket rattling  sound
volatile boolean indicatorSoundOn = false;        // active, if indicator bulb is on
volatile boolean outOfFuelMessageTrigger = false; // Trigger for out of fuel message

// Sound latches
volatile boolean hornLatch = false;  // Horn latch bit
volatile boolean sirenLatch = false; // Siren latch bit

// Sound volumes
volatile uint16_t throttleDependentVolume = 0;        // engine volume according to throttle position
volatile uint16_t throttleDependentRevVolume = 0;     // engine rev volume according to throttle position
volatile uint16_t rpmDependentJakeBrakeVolume = 0;    // Engine rpm dependent jake brake volume
volatile uint16_t throttleDependentKnockVolume = 0;   // engine Diesel knock volume according to throttle position
volatile uint16_t rpmDependentKnockVolume = 0;        // engine Diesel knock volume according to engine RPM
volatile uint16_t throttleDependentTurboVolume = 0;   // turbo volume according to rpm
volatile uint16_t throttleDependentFanVolume = 0;     // cooling fan volume according to rpm
volatile uint16_t throttleDependentChargerVolume = 0; // cooling fan volume according to rpm
volatile uint16_t rpmDependentWastegateVolume = 0;    // wastegate volume according to rpm
volatile uint16_t tireSquealVolume = 0;               // Tire squeal volume according to speed and cornering radius
// for excavator mode:
volatile uint16_t hydraulicPumpVolume = 0;             // hydraulic pump volume
volatile uint16_t hydraulicFlowVolume = 0;             // hydraulic flow volume
volatile uint16_t trackRattleVolume = 0;               // track rattling volume
volatile uint16_t hydraulicDependentKnockVolume = 100; // engine Diesel knock volume according to hydraulic load
volatile uint16_t hydraulicLoad = 0;                   // Hydraulic load dependent RPM drop

volatile uint64_t dacDebug = 0; // DAC debug variable TODO

volatile int16_t masterVolume = 100; // Master volume percentage
volatile uint8_t dacOffset = 0;      // 128, but needs to be ramped up slowly to prevent popping noise, if switched on

// Throttle
int16_t currentThrottle = 0;      // 0 - 500 (Throttle trigger input)
int16_t currentThrottleFaded = 0; // faded throttle for volume calculations etc.

// Engine
const int16_t maxRpm = 500;       // always 500
const int16_t minRpm = 0;         // always 0
int32_t currentRpm = 0;           // 0 - 500 (signed required!)
int32_t targetHydraulicRpm[3]; // The hydraulic RPM target for loader mode
volatile uint8_t engineState = 0; // Engine state
enum EngineState                  // Engine state enum
{
  OFF,      // Engine is off
  STARTING, // Engine is starting
  RUNNING,  // Engine is running
  STOPPING, // Engine is stopping
  PARKING_BRAKE
};
int16_t engineLoad = 0;                 // 0 - 500
volatile uint16_t engineSampleRate = 0; // Engine sample rate
int32_t speedLimit = maxRpm;            // The speed limit, depending on selected virtual gear

// Clutch
boolean clutchDisengaged = true; // Active while clutch is disengaged

// Transmission
uint8_t selectedGear = 1;             // The currently used gear of our shifting gearbox
uint8_t selectedAutomaticGear = 1;    // The currently used gear of our automatic gearbox
boolean gearUpShiftingInProgress;     // Active while shifting upwards
boolean doubleClutchInProgress;       // Double-clutch (Zwischengas)
boolean gearDownShiftingInProgress;   // Active while shifting downwards
boolean gearUpShiftingPulse;          // Active, if shifting upwards begins
boolean gearDownShiftingPulse;        // Active, if shifting downwards begins
volatile boolean neutralGear = false; // Transmission in neutral
boolean lowRange = false;             // Transmission range (off road reducer)

// ESC
volatile boolean escIsBraking = false; // ESC is in a braking state
volatile boolean escIsDriving = false; // ESC is in a driving state
volatile boolean escInReverse = false; // ESC is driving or braking backwards
volatile boolean brakeDetect = false;  // Additional brake detect signal, enabled immediately, if brake applied
int8_t driveState = 0;                 // for ESC state machine
 
uint16_t currentSpeed = 0;         // 0 - 500 (current ESC power)
volatile bool crawlerMode = false; // Crawler mode intended for crawling competitons (withouth sound and virtual inertia)
boolean cannonFlash = false;                   // Flashing cannon fire

// Eeprom size and storage addresses ----------------------------------------------
#define EEPROM_SIZE 512 // 512 Bytes (512 is maximum) 
#define adr_eprom_init 0 // Eeprom initialized or not?
 
// DEBUG stuff
volatile uint8_t coreId = 99;

// Our main tasks
TaskHandle_t Task1;

// Loop time (for debug)
uint16_t loopTime;

// Sampling intervals for interrupt timer (adjusted according to your sound file sampling rate)
uint32_t maxSampleInterval = 4000000 / sampleRate;
uint32_t minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

// Interrupt timer for variable sample rate playback (engine sound)
hw_timer_t *variableTimer = NULL;
portMUX_TYPE variableTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t variableTimerTicks = maxSampleInterval;

// Interrupt timer for fixed sample rate playback (horn etc., playing in parallel with engine sound)
hw_timer_t *fixedTimer = NULL;
portMUX_TYPE fixedTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t fixedTimerTicks = maxSampleInterval;

// Declare a mutex Semaphore Handles.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xPwmSemaphore;
SemaphoreHandle_t xRpmSemaphore;

// These are used to print the reset reason on startup
const char *RESET_REASONS[] = {"POWERON_RESET", "NO_REASON", "SW_RESET", "OWDT_RESET", "DEEPSLEEP_RESET", "SDIO_RESET", "TG0WDT_SYS_RESET", "TG1WDT_SYS_RESET", "RTCWDT_SYS_RESET", "INTRUSION_RESET", "TGWDT_CPU_RESET", "SW_CPU_RESET", "RTCWDT_CPU_RESET", "EXT_CPU_RESET", "RTCWDT_BROWN_OUT_RESET", "RTCWDT_RTC_RESET"};

// Convert µs to degrees (°)
float us2degree(uint16_t value)
{
  return (value - 500) / 11.111 - 90.0;
}

//
// =======================================================================================================
// INTERRUPT FOR VARIABLE SPEED PLAYBACK (Engine sound, turbo sound)
// =======================================================================================================
//

void IRAM_ATTR variablePlaybackTimer()
{

  // coreId = xPortGetCoreID(); // Running on core 1

  static uint32_t attenuatorMillis = 0;
  static uint32_t curEngineSample = 0;          // Index of currently loaded engine sample
  static uint32_t curRevSample = 0;             // Index of currently loaded engine rev sample
  static uint32_t curTurboSample = 0;           // Index of currently loaded turbo sample
  static uint32_t curFanSample = 0;             // Index of currently loaded fan sample
  static uint32_t curChargerSample = 0;         // Index of currently loaded charger sample
  static uint32_t curStartSample = 0;           // Index of currently loaded start sample
  static uint32_t curJakeBrakeSample = 0;       // Index of currently loaded jake brake sample
  static uint32_t curHydraulicPumpSample = 0;   // Index of currently loaded hydraulic pump sample
  static uint32_t curTrackRattleSample = 0;     // Index of currently loaded train track rattle sample
  static uint32_t lastDieselKnockSample = 0;    // Index of last Diesel knock sample
  static uint16_t attenuator = 0;               // Used for volume adjustment during engine switch off
  static uint16_t speedPercentage = 0;          // slows the engine down during shutdown
  static int32_t a, a1, a2, a3, b, c, d, e = 0; // Input signals for mixer: a = engine, b = additional sound, c = turbo sound, d = fan sound, e = supercharger sound
  static int32_t f = 0;                         // Input signals for mixer: f = hydraulic pump
  static int32_t g = 0;                         // Input signals for mixer: g = train track rattle
  uint8_t a1Multi = 0;                          // Volume multipliers

  // portENTER_CRITICAL_ISR(&variableTimerMux); // disables C callable interrupts (on the current core) and locks the mutex by the current core.

  switch (engineState)
  {

  case OFF:                                                   // Engine off -----------------------------------------------------------------------
    variableTimerTicks = 4000000 / startSampleRate;           // our fixed sampling rate
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

    a = 0; // volume = zero
    if (engineOn)
    {
      engineState = STARTING;
      engineStart = true;
    }
    break;

  case STARTING:                                              // Engine start --------------------------------------------------------------------
    variableTimerTicks = 4000000 / startSampleRate;           // our fixed sampling rate
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

    if (curStartSample < startSampleCount - 1)
    {
#if defined STEAM_LOCOMOTIVE_MODE
      a = (startSamples[curStartSample] * startVolumePercentage / 100);
#else
      a = (startSamples[curStartSample] * throttleDependentVolume / 100 * startVolumePercentage / 100);
#endif
      curStartSample++;
    }
    else
    {
      curStartSample = 0;
      engineState = RUNNING;
      engineStart = false;
      engineRunning = true;
      airBrakeTrigger = true;
    }
    break;

  case RUNNING: // Engine running ------------------------------------------------------------------

    // Engine idle & revving sounds (mixed together according to engine rpm, new in v5.0)
    variableTimerTicks = engineSampleRate;                    // our variable idle sampling rate!
    timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

    if (!engineJakeBraking && !blowoffTrigger)
    {
      if (curEngineSample < sampleCount - 1)
      {
        a1 = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100); // Idle sound
        a3 = 0;
        curEngineSample++;

        // Optional rev sound, recorded at medium rpm. Note, that it needs to represent the same number of ignition cycles as the
        // idle sound. For example 4 or 8 for a V8 engine. It also needs to have about the same length. In order to adjust the length
        // or "revSampleCount", change the "Rate" setting in Audacity until it is about the same.
#ifdef REV_SOUND
        a2 = (revSamples[curRevSample] * throttleDependentRevVolume / 100 * revVolumePercentage / 100); // Rev sound
        if (curRevSample < revSampleCount)
          curRevSample++;
#endif

        // Trigger throttle dependent Diesel ignition "knock" sound (played in the fixed sample rate interrupt)
        if (curEngineSample - lastDieselKnockSample > (sampleCount / dieselKnockInterval))
        {
          dieselKnockTrigger = true;
          dieselKnockTriggerFirst = false;
          lastDieselKnockSample = curEngineSample;
        }
      }
      else
      {
        curEngineSample = 0;
        if (jakeBrakeRequest)
          engineJakeBraking = true;
#ifdef REV_SOUND
        curRevSample = 0;
#endif
        lastDieselKnockSample = 0;
        dieselKnockTrigger = true;
        dieselKnockTriggerFirst = true;
      }
      curJakeBrakeSample = 0;
    }
    else
    { // Jake brake sound ----
#ifdef JAKE_BRAKE_SOUND
      a3 = (jakeBrakeSamples[curJakeBrakeSample] * rpmDependentJakeBrakeVolume / 100 * jakeBrakeVolumePercentage / 100); // Jake brake sound
      a2 = 0;
      a1 = 0;
      if (curJakeBrakeSample < jakeBrakeSampleCount - 1)
        curJakeBrakeSample++;
      else
      {
        curJakeBrakeSample = 0;
        if (!jakeBrakeRequest)
          engineJakeBraking = false;
      }

      curEngineSample = 0;
      curRevSample = 0;
#endif
    }

    // Engine sound mixer ----
#ifdef REV_SOUND
    // Mixing the idle and rev sounds together, according to engine rpm
    // Below the "revSwitchPoint" target, the idle volume precentage is 90%, then falling to 0% @ max. rpm.
    // The total of idle and rev volume percentage is always 100%

    if (currentRpm > revSwitchPoint)
      a1Multi = map(currentRpm, idleEndPoint, revSwitchPoint, 0, idleVolumeProportionPercentage);
    else
      a1Multi = idleVolumeProportionPercentage; // 90 - 100% proportion
    if (currentRpm > idleEndPoint)
      a1Multi = 0;

    a1 = a1 * a1Multi / 100;         // Idle volume
    a2 = a2 * (100 - a1Multi) / 100; // Rev volume

    a = a1 + a2 + a3; // Idle and rev sounds mixed together
#else
    a = a1 + a3; // Idle sound only
#endif

    // Turbo sound ----------------------------------
    if (curTurboSample < turboSampleCount - 1)
    {
      c = (turboSamples[curTurboSample] * throttleDependentTurboVolume / 100 * turboVolumePercentage / 100);
      curTurboSample++;
    }
    else
    {
      curTurboSample = 0;
    }

    // Fan sound -----------------------------------
    if (curFanSample < fanSampleCount - 1)
    {
      d = (fanSamples[curFanSample] * throttleDependentFanVolume / 100 * fanVolumePercentage / 100);
      curFanSample++;
    }
    else
    {
      curFanSample = 0;
    }
#if defined GEARBOX_WHINING
    if (neutralGear)
      d = 0; // used for gearbox whining simulation, so not active in gearbox neutral
#endif

    // Supercharger sound --------------------------
    if (curChargerSample < chargerSampleCount - 1)
    {
      e = (chargerSamples[curChargerSample] * throttleDependentChargerVolume / 100 * chargerVolumePercentage / 100);
      curChargerSample++;
    }
    else
    {
      curChargerSample = 0;
    }

    // Hydraulic pump sound -----------------------
#if defined EXCAVATOR_MODE
    if (curHydraulicPumpSample < hydraulicPumpSampleCount - 1)
    {
      f = (hydraulicPumpSamples[curHydraulicPumpSample] * hydraulicPumpVolumePercentage / 100 * hydraulicPumpVolume / 100);
      curHydraulicPumpSample++;
    }
    else
    {
      curHydraulicPumpSample = 0;
    }
#endif

#if defined STEAM_LOCOMOTIVE_MODE
    // Track rattle sound -----------------------
    if (curTrackRattleSample < trackRattleSampleCount - 1)
    {
      g = (trackRattleSamples[curTrackRattleSample] * trackRattleVolumePercentage / 100 * trackRattleVolume / 100);
      curTrackRattleSample++;
    }
    else
    {
      curTrackRattleSample = 0;
    }
#endif

    if (!engineOn)
    {
      speedPercentage = 100;
      attenuator = 1;
      engineState = STOPPING;
      engineStop = true;
      engineRunning = false;
    }
    break;

  case STOPPING:                                                       // Engine stop --------------------------------------------------------------------
    variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100; // our fixed sampling rate
    timerAlarmWrite(variableTimer, variableTimerTicks, true);          // // change timer ticks, autoreload true

    if (curEngineSample < sampleCount - 1)
    {
      a = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100 / attenuator);
      curEngineSample++;
    }
    else
    {
      curEngineSample = 0;
    }

    // fade engine sound out
    if (millis() - attenuatorMillis > 100)
    { // Every 50ms
      attenuatorMillis = millis();
      attenuator++;          // attenuate volume
      speedPercentage += 20; // make it slower (10)
    }

    if (attenuator >= 50 || speedPercentage >= 500)
    { // 50 & 500
      a = 0;
      speedPercentage = 100;
      parkingBrakeTrigger = true;
      engineState = PARKING_BRAKE;
      engineStop = false;
    }
    break;

  case PARKING_BRAKE: // parking brake bleeding air sound after engine is off ----------------------------

    if (!parkingBrakeTrigger)
    {
      engineState = OFF;
    }
    break;

  }  

  // Direct DAC access is faster according to: https://forum.arduino.cc/t/esp32-dacwrite-ersetzen/653954/5
  uint8_t value = constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5) + f + g) * masterVolume / 100 + dacOffset, 0, 255);
  SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, value, RTC_IO_PDAC1_DAC_S);

  // portEXIT_CRITICAL_ISR(&variableTimerMux);
}

//
// =======================================================================================================
// INTERRUPT FOR FIXED SPEED PLAYBACK (Horn etc., played in parallel with engine sound)
// =======================================================================================================
//

void IRAM_ATTR fixedPlaybackTimer()
{

  // coreId = xPortGetCoreID(); // Running on core 1

  static uint32_t curHornSample = 0;                            // Index of currently loaded horn sample
  static uint32_t curSirenSample = 0;                           // Index of currently loaded siren sample
  static uint32_t curSound1Sample = 0;                          // Index of currently loaded sound1 sample
  static uint32_t curReversingSample = 0;                       // Index of currently loaded reversing beep sample
  static uint32_t curIndicatorSample = 0;                       // Index of currently loaded indicator tick sample
  static uint32_t curWastegateSample = 0;                       // Index of currently loaded wastegate sample
  static uint32_t curBrakeSample = 0;                           // Index of currently loaded brake sound sample
  static uint32_t curParkingBrakeSample = 0;                    // Index of currently loaded brake sound sample
  static uint32_t curShiftingSample = 0;                        // Index of currently loaded shifting sample
  static uint32_t curDieselKnockSample = 0;                     // Index of currently loaded Diesel knock sample
  static uint32_t curCouplingSample = 0;                        // Index of currently loaded trailer coupling sample
  static uint32_t curUncouplingSample = 0;                      // Index of currently loaded trailer uncoupling sample
  static uint32_t curHydraulicFlowSample = 0;                   // Index of currently loaded hydraulic flow sample
  static uint32_t curTrackRattleSample = 0;                     // Index of currently loaded track rattle sample
  static uint32_t curBucketRattleSample = 0;                    // Index of currently loaded bucket rattle sample
  static uint32_t curTireSquealSample = 0;                      // Index of currently loaded tire squeal sample
  static uint32_t curOutOfFuelSample = 0;                       // Index of currently loaded out of fuel sample
  static int32_t a, a1, a2 = 0;                                 // Input signals "a" for mixer
  static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9 = 0; // Input signals "b" for mixer
  static int32_t c, c1, c2, c3 = 0;                             // Input signals "c" for mixer
  static int32_t d, d1, d2 = 0;                                 // Input signals "d" for mixer
  static boolean knockSilent = 0;                               // This knock will be more silent
  static boolean knockMedium = 0;                               // This knock will be medium
  static uint8_t curKnockCylinder = 0;                          // Index of currently ignited zylinder

  // portENTER_CRITICAL_ISR(&fixedTimerMux);

  // Group "a" (horn & siren) ******************************************************************

  if (hornTrigger || hornLatch)
  {
    fixedTimerTicks = 4000000 / hornSampleRate;         // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curHornSample < hornSampleCount - 1)
    {
      a1 = (hornSamples[curHornSample] * hornVolumePercentage / 100);
      curHornSample++;
#ifdef HORN_LOOP // Optional "endless loop" (points to be defined manually in horn file)
      if (hornTrigger && curHornSample == hornLoopEnd)
        curHornSample = hornLoopBegin; // Loop, if trigger still present
#endif
    }
    else
    { // End of sample
      curHornSample = 0;
      a1 = 0;
      hornLatch = false;
    }
  }

  if (sirenTrigger || sirenLatch)
  {
    fixedTimerTicks = 4000000 / sirenSampleRate;        // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

#if defined SIREN_STOP
    if (!sirenTrigger)
    {
      sirenLatch = false;
      curSirenSample = 0;
      a2 = 0;
    }
#endif

    if (curSirenSample < sirenSampleCount - 1)
    {
      a2 = (sirenSamples[curSirenSample] * sirenVolumePercentage / 100);
      curSirenSample++;
#ifdef SIREN_LOOP // Optional "endless loop" (points to be defined manually in siren file)
      if (sirenTrigger && curSirenSample == sirenLoopEnd)
        curSirenSample = sirenLoopBegin; // Loop, if trigger still present
#endif
    }
    else
    { // End of sample
      curSirenSample = 0;
      a2 = 0;
      sirenLatch = false;
    }
  }
  if (curSirenSample > 10 && curSirenSample < 500)
    cannonFlash = true; // Tank cannon flash triggering in TRACKED_MODE
  else
    cannonFlash = false;

  // Group "b" (other sounds) **********************************************************************

  // Sound 1 "b0" ----
  if (sound1trigger)
  {
    fixedTimerTicks = 4000000 / sound1SampleRate;       // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curSound1Sample < sound1SampleCount - 1)
    {
      b0 = (sound1Samples[curSound1Sample] * sound1VolumePercentage / 100);
      curSound1Sample++;
    }
    else
    {
      sound1trigger = false;
    }
  }
  else
  {
    curSound1Sample = 0; // ensure, next sound will start @ first sample
    b0 = 0;
  }

  // Reversing beep sound "b1" ----
  if (engineRunning && escInReverse)
  {
    fixedTimerTicks = 4000000 / reversingSampleRate;    // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curReversingSample < reversingSampleCount - 1)
    {
      b1 = (reversingSamples[curReversingSample] * reversingVolumePercentage / 100);
      curReversingSample++;
    }
    else
    {
      curReversingSample = 0;
    }
  }
  else
  {
    curReversingSample = 0; // ensure, next sound will start @ first sample
    b1 = 0;
  }

  // Indicator tick sound "b2" ----------------------------------------------------------------------
#if not defined NO_INDICATOR_SOUND
  if (indicatorSoundOn)
  {
    fixedTimerTicks = 4000000 / indicatorSampleRate;    // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curIndicatorSample < indicatorSampleCount - 1)
    {
      b2 = (indicatorSamples[curIndicatorSample] * indicatorVolumePercentage / 100);
      curIndicatorSample++;
    }
    else
    {
      indicatorSoundOn = false;
    }
  }
  else
  {
    curIndicatorSample = 0; // ensure, next sound will start @ first sample
    b2 = 0;
  }
#endif

  // Wastegate (blowoff) sound, triggered after rapid throttle drop -----------------------------------
  if (wastegateTrigger)
  {
    if (curWastegateSample < wastegateSampleCount - 1)
    {
      b3 = (wastegateSamples[curWastegateSample] * rpmDependentWastegateVolume / 100 * wastegateVolumePercentage / 100);
      curWastegateSample++;
    }
    else
    {
      wastegateTrigger = false;
    }
  }
  else
  {
    b3 = 0;
    curWastegateSample = 0; // ensure, next sound will start @ first sample
  }

  // Air brake release sound, triggered after stop -----------------------------------------------
  if (airBrakeTrigger)
  {
    if (curBrakeSample < brakeSampleCount - 1)
    {
      b4 = (brakeSamples[curBrakeSample] * brakeVolumePercentage / 100);
      curBrakeSample++;
    }
    else
    {
      airBrakeTrigger = false;
    }
  }
  else
  {
    b4 = 0;
    curBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Air parking brake attaching sound, triggered after engine off --------------------------------
  if (parkingBrakeTrigger)
  {
    if (curParkingBrakeSample < parkingBrakeSampleCount - 1)
    {
      b5 = (parkingBrakeSamples[curParkingBrakeSample] * parkingBrakeVolumePercentage / 100);
      curParkingBrakeSample++;
    }
    else
    {
      parkingBrakeTrigger = false;
    }
  }
  else
  {
    b5 = 0;
    curParkingBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Pneumatic gear shifting sound, triggered while shifting the TAMIYA 3 speed transmission ------
  if (shiftingTrigger && engineRunning && !automatic && !doubleClutch)
  {
    if (curShiftingSample < shiftingSampleCount - 1)
    {
      b6 = (shiftingSamples[curShiftingSample] * shiftingVolumePercentage / 100);
      curShiftingSample++;
    }
    else
    {
      shiftingTrigger = false;
    }
  }
  else
  {
    b6 = 0;
    curShiftingSample = 0; // ensure, next sound will start @ first sample
  }

  // Diesel ignition "knock" is played in fixed sample rate section, because we don't want changing pitch! ------
  if (dieselKnockTriggerFirst)
  {
    dieselKnockTriggerFirst = false;
    curKnockCylinder = 0;
  }

  //forcing false dieselKnockTrigger, we dont like this sound
  dieselKnockTrigger = false;
  if (dieselKnockTrigger)
  {
    dieselKnockTrigger = false;
    curKnockCylinder++; // Count ignition sequence
    curDieselKnockSample = 0;
  }

#ifdef V8 // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
  // Ford or Scania V8 ignition sequence: 1 - 5 - 4 - 2* - 6 - 3 - 7 - 8* (* = louder knock pulses, because 2nd exhaust in same manifold after 90°)
  if (curKnockCylinder == 4 || curKnockCylinder == 8)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef V8_MEDIUM // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
  // This is EXPERIMENTAL!! TODO
  if (curKnockCylinder == 5 || curKnockCylinder == 1)
    knockMedium = false;
  else
    knockMedium = true;
#endif

#ifdef V8_468 // (Chevy 468, containing 16 ignition pulses)
  // 1th, 5th, 9th and 13th are the loudest
  // Ignition sequence: 1 - 8 - 4* - 3 - 6 - 5 - 7* - 2
  if (curKnockCylinder == 1 || curKnockCylinder == 5 || curKnockCylinder == 9 || curKnockCylinder == 13)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef V2
  // V2 engine: 1st and 2nd knock pulses (of 4) will be louder
  if (curKnockCylinder == 1 || curKnockCylinder == 2)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef R6
  // R6 inline 6 engine: 6th knock pulse (of 6) will be louder
  if (curKnockCylinder == 6)
    knockSilent = false;
  else
    knockSilent = true;
#endif

#ifdef R6_2
  // R6 inline 6 engine: 6th and 3rd knock pulse (of 6) will be louder
  if (curKnockCylinder == 6 || curKnockCylinder == 3)
    knockSilent = false;
  else
    knockSilent = true;
#endif

  if (curDieselKnockSample < knockSampleCount)
  {
#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * rpmDependentKnockVolume / 100);
#elif defined EXCAVATOR_MODE // knock volume also depending on hydraulic load
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100 * hydraulicDependentKnockVolume / 100);
#else                        // Just depending on throttle
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
#endif
    curDieselKnockSample++;
    if (knockSilent && !knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 100; // changing knock volume according to engine type and cylinder!
    if (knockMedium)
      b7 = b7 * dieselKnockAdaptiveVolumePercentage / 75;
  }

#if not defined EXCAVATOR_MODE
  // Trailer coupling sound, triggered by switch -----------------------------------------------
#ifdef COUPLING_SOUND
  if (couplingTrigger)
  {
    if (curCouplingSample < couplingSampleCount - 1)
    {
      b8 = (couplingSamples[curCouplingSample] * couplingVolumePercentage / 100);
      curCouplingSample++;
    }
    else
    {
      couplingTrigger = false;
    }
  }
  else
  {
    b8 = 0;
    curCouplingSample = 0; // ensure, next sound will start @ first sample
  }

  // Trailer uncoupling sound, triggered by switch -----------------------------------------------
  if (uncouplingTrigger)
  {
    if (curUncouplingSample < uncouplingSampleCount - 1)
    {
      b9 = (uncouplingSamples[curUncouplingSample] * couplingVolumePercentage / 100);
      curUncouplingSample++;
    }
    else
    {
      uncouplingTrigger = false;
    }
  }
  else
  {
    b9 = 0;
    curUncouplingSample = 0; // ensure, next sound will start @ first sample
  }
#endif
#endif

  // Group "c" (excavator sounds) **********************************************************************

#if defined EXCAVATOR_MODE || defined LOADER_MODE

  // Hydraulic fluid flow sound -----------------------
  if (curHydraulicFlowSample < hydraulicFlowSampleCount - 1)
  {
    c1 = (hydraulicFlowSamples[curHydraulicFlowSample] * hydraulicFlowVolumePercentage / 100 * hydraulicFlowVolume / 100);
    curHydraulicFlowSample++;
  }
  else
  {
    curHydraulicFlowSample = 0;
  }

  // Track rattle sound -----------------------
  if (curTrackRattleSample < trackRattleSampleCount - 1)
  {
    c2 = (trackRattleSamples[curTrackRattleSample] * trackRattleVolumePercentage / 100 * trackRattleVolume / 100);
    curTrackRattleSample++;
  }
  else
  {
    curTrackRattleSample = 0;
  }

  // Bucket rattle sound -----------------------
  if (bucketRattleTrigger)
  {
    if (curBucketRattleSample < bucketRattleSampleCount - 1)
    {
      c3 = (bucketRattleSamples[curBucketRattleSample] * bucketRattleVolumePercentage / 100);
      curBucketRattleSample++;
    }
    else
    {
      bucketRattleTrigger = false;
    }
  }
  else
  {
    c3 = 0;
    curBucketRattleSample = 0; // ensure, next sound will start @ first sample
  }
#endif

  // Group "d" (additional sounds) **********************************************************************

#if defined TIRE_SQUEAL
  // Tire squeal sound -----------------------
  if (curTireSquealSample < tireSquealSampleCount - 1)
  {
    d1 = (tireSquealSamples[curTireSquealSample] * tireSquealVolumePercentage / 100 * tireSquealVolume / 100);
    curTireSquealSample++;
  }
  else
  {
    d1 = 0;
    curTireSquealSample = 0;
  }
#endif

#if defined BATTERY_PROTECTION
  // Out of fuel sound, triggered by battery voltage -----------------------------------------------
  if (outOfFuelMessageTrigger)
  {
    if (curOutOfFuelSample < outOfFuelSampleCount - 1)
    {
      d2 = (outOfFuelSamples[curOutOfFuelSample] * outOfFuelVolumePercentage / 100);
      curOutOfFuelSample++;
    }
    else
    {
      outOfFuelMessageTrigger = false;
    }
  }
  else
  {
    d2 = 0;
    curOutOfFuelSample = 0; // ensure, next sound will start @ first sample
  }
#endif

  // Mixing sounds together **********************************************************************
  a = a1 + a2; // Horn & siren
  // if (a < 2 && a > -2) a = 0; // Remove noise floor TODO, experimental
  b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7 + b8 + b9; // Other sounds
  c = c1 + c2 + c3;                                            // Excavator sounds
  d = d1 + d2;                                                 // Additional sounds

  // Direct DAC access is faster according to: https://forum.arduino.cc/t/esp32-dacwrite-ersetzen/653954/5
  uint8_t value = constrain(((a * 8 / 10) + (b * 2 / 10) + c + d) * masterVolume / 100 + dacOffset, 0, 255);
  SET_PERI_REG_BITS(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC, value, RTC_IO_PDAC2_DAC_S);
 
}

//
// =======================================================================================================
// PWM SIGNAL READ INTERRUPT
// =======================================================================================================
//

// Reference https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html?highlight=rmt
static void IRAM_ATTR rmt_isr_handler(void *arg)
{
 
}

//
// =======================================================================================================
// PPM SIGNAL READ INTERRUPT
// =======================================================================================================
//

void IRAM_ATTR readPpm()
{ 
}
 

//
// =======================================================================================================
// ESP NOW TRAILER DATA SENT CALLBACK
// =======================================================================================================
//

// callback when data is sent
void IRAM_ATTR onTrailerDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
 
}  
 
//
// =======================================================================================================
// EEPROM SETUP
// =======================================================================================================
//

void setupEeprom()
{
  EEPROM.begin(EEPROM_SIZE);
#if defined ERASE_EEPROM_ON_BOOT
  eepromErase(); // uncomment this option, if you want to erase all stored settings!
#endif
  eepromInit(); // Init new board with default values
  eepromRead(); // Read settings from Eeprom
  Serial.print("current eeprom_id: ");
  Serial.println(EEPROM.read(adr_eprom_init));
  Serial.println("change it for default value upload!\n");
  eepromDebugRead(); // Shows content of entire eeprom, except of empty areas
}
 
void setup()
{
  // Watchdog timers need to be disabled, if task 1 is running without delay(1)
  disableCore0WDT();
  // disableCore1WDT(); // Core 1 WDT can stay enabled TODO

  // Setup RTC (Real Time Clock) watchdog
  rtc_wdt_protect_off(); // Disable RTC WDT write protection
  rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  rtc_wdt_set_time(RTC_WDT_STAGE0, 10000); // set 10s timeout
  rtc_wdt_enable();                        // Start the RTC WDT timer
  // rtc_wdt_disable();            // Disable the RTC WDT timer
  rtc_wdt_protect_on(); // Enable RTC WDT write protection

  // Serial setup
  Serial.begin(115200, SERIAL_8N1, DEBUG_RX, DEBUG_TX); // USB serial (for DEBUG) Mode, Rx pin (99 = not used), Tx pin

  // ADC setup
  battery.attach(BATTERY_DETECT_PIN);

  // Print some system and software info to serial monitor
  delay(1000); // Give serial port/connection some time to get ready
  Serial.printf("\n**************************************************************************************************\n");
  Serial.printf("TheDIYGuy999 RC engine sound & light controller for ESP32 software version %s\n", codeVersion);
  Serial.printf("https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("Please read carefully: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/README.md\n");
  Serial.printf("XTAL Frequency: %i MHz, CPU Clock: %i MHz, APB Bus Clock: %i Hz\n", getXtalFrequencyMhz(), getCpuFrequencyMhz(), getApbFrequency());
  Serial.printf("Internal RAM size: %i Byte, Free: %i Byte\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("WiFi MAC address: %s\n", WiFi.macAddress().c_str());
  for (uint8_t coreNum = 0; coreNum < 2; coreNum++)
  {
    uint8_t resetReason = rtc_get_reset_reason(coreNum);
    if (resetReason <= (sizeof(RESET_REASONS) / sizeof(RESET_REASONS[0])))
    {
      Serial.printf("Core %i reset reason: %i: %s\n", coreNum, rtc_get_reset_reason(coreNum), RESET_REASONS[resetReason - 1]);
    }
  }
  Serial.printf("**************************************************************************************************\n\n");

  // Eeprom Setup
  setupEeprom(); 

  if (xPwmSemaphore == NULL) // Check to confirm that the PWM Semaphore has not already been created.
  {
    xPwmSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xPwmSemaphore) != NULL)
      xSemaphoreGive((xPwmSemaphore)); // Make the PWM variable available for use, by "Giving" the Semaphore.
  }

  if (xRpmSemaphore == NULL) // Check to confirm that the RPM Semaphore has not already been created.
  {
    xRpmSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage variable access
    if ((xRpmSemaphore) != NULL)
      xSemaphoreGive((xRpmSemaphore)); // Make the RPM variable available for use, by "Giving" the Semaphore.
  } 
  headLight.begin(HEADLIGHT_PIN, 15, 20000);           // Timer 15, 20kHz
  maxSampleInterval = 4000000 / sampleRate;
  minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

  // Time
  timelast = micros();
  timelastloop = timelast;

  // Task 1 setup (running on core 0)
  TaskHandle_t Task1;
  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      Task1code, // Task function
      "Task1",   // name of task
      8192,      // Stack size of task (8192)
      NULL,      // parameter of the task
      1,         // priority of the task (1 = low, 3 = medium, 5 = highest)
      &Task1,    // Task handle to keep track of created task
      0);        // pin task to core 0
 
  dacWrite(DAC1, 0);
  dacWrite(DAC2, 0);
  pinMode(33,INPUT_PULLDOWN);

  // Interrupt timer for variable sample rate playback
  variableTimer = timerBegin(0, 20, true);                           // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true); // edge (not level) triggered
  timerAlarmWrite(variableTimer, variableTimerTicks, true);          // autoreload true
  timerAlarmEnable(variableTimer);                                   // enable

  // Interrupt timer for fixed sample rate playback
  fixedTimer = timerBegin(1, 20, true);                        // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true); // edge (not level) triggered
  timerAlarmWrite(fixedTimer, fixedTimerTicks, true);          // autoreload true
  timerAlarmEnable(fixedTimer);                                // enable
  canbusSetup();
}

//
// =======================================================================================================
// DAC OFFSET FADER
// =======================================================================================================
//

static unsigned long dacOffsetMicros;
boolean dacInit;

void dacOffsetFade()
{
  if (!dacInit)
  {
    if (micros() - dacOffsetMicros > 100)
    { // Every 0.1ms
      dacOffsetMicros = micros();
      dacOffset++; // fade DAC offset slowly to prevent it from popping, if ESP32 powered up after amplifier
      if (dacOffset == 128)
        dacInit = true;
    }
  }
}

// it is required to disable interrupts prior to EEPROM access!
void disableAllInterrupts()
{

  timerDetachInterrupt(variableTimer);
  timerDetachInterrupt(fixedTimer);

  Serial.print("Interrupts disabled, reboot required!\n");
}

// Erase EEPROM ------
void eepromErase()
{
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("EEPROM erased!");
  delay(500);
  eepromDebugRead();
}

// Init new board with the default values you want ------
void eepromInit()
{
}

// Write new values to EEPROM ------
void eepromWrite()
{
  disableAllInterrupts(); // This is very important!
}

// Read values from EEPROM ------
void eepromRead()
{
}

void eepromDebugRead()
{
  
}

void mapThrottle()
{

  currentThrottle = map(analogRead(35),0,3900,0,500);

  // As a base for some calculations below, fade the current throttle to make it more natural
  static unsigned long throttleFaderMicros;
  // static boolean blowoffLock; // blowoffLock seems unused here, consider removing if truly unused globally
  if (micros() - throttleFaderMicros > 500)
  { // Every 0.5ms
    throttleFaderMicros = micros();

    // Fade currentThrottle for smoother volume transitions
    if (currentThrottleFaded < currentThrottle && currentThrottleFaded < 499) // Removed: !escIsBraking 
      currentThrottleFaded += 2;
    if (currentThrottleFaded > currentThrottle && currentThrottleFaded > 2) // Removed: || escIsBraking
      currentThrottleFaded -= 2;
    currentThrottleFaded = constrain(currentThrottleFaded, 0, 500);


    // Calculate throttle dependent engine idle volume
    // Simplified: if engine is running, volume depends on faded throttle
    if (engineRunning) 
      throttleDependentVolume = map(currentThrottleFaded, 0, 500, engineIdleVolumePercentage, fullThrottleVolumePercentage);
    else
      throttleDependentVolume = engineIdleVolumePercentage; // Or 0 if preferred when engine is off


    // Calculate throttle dependent engine rev volume
    if (engineRunning)
      throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, engineRevVolumePercentage, fullThrottleVolumePercentage);
    else
      throttleDependentRevVolume = engineRevVolumePercentage; // Or 0


    // Calculate throttle dependent Diesel knock volume
    // Simplified: if engine running and throttle above a point, map knock volume
    if (engineRunning && (currentThrottleFaded > dieselKnockStartPoint))
      throttleDependentKnockVolume = map(currentThrottleFaded, dieselKnockStartPoint, 500, dieselKnockIdleVolumePercentage, 100);
    else
      throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;


    // Calculate engine rpm dependent jake brake volume
    if (engineRunning)
      rpmDependentJakeBrakeVolume = map(currentRpm, 0, 500, jakeBrakeIdleVolumePercentage, 100);
    else
      rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage; // Or 0

#if defined RPM_DEPENDENT_KNOCK // knock volume also depending on engine rpm
    // Calculate RPM dependent Diesel knock volume
    if (currentRpm > knockStartRpm) // ensure knockStartRpm is defined and sensible
      rpmDependentKnockVolume = map(currentRpm, knockStartRpm, 500, minKnockVolumePercentage, 100);
    else
      rpmDependentKnockVolume = minKnockVolumePercentage;
#endif

    // Calculate engine rpm dependent turbo volume
    if (engineRunning)
      throttleDependentTurboVolume = map(currentRpm, 0, 500, turboIdleVolumePercentage, 100);
    else
      throttleDependentTurboVolume = turboIdleVolumePercentage; // Or 0

    // Calculate engine rpm dependent cooling fan volume
    if (engineRunning && (currentRpm > fanStartPoint))
      throttleDependentFanVolume = map(currentRpm, fanStartPoint, 500, fanIdleVolumePercentage, 100);
    else
      throttleDependentFanVolume = fanIdleVolumePercentage; // Or 0

    // Calculate throttle dependent supercharger volume
    // Original used currentRpm > chargerStartPoint AND throttle. For pot mode, currentRpm is more direct.
    if (engineRunning && (currentRpm > chargerStartPoint)) 
      throttleDependentChargerVolume = map(currentRpm, chargerStartPoint, 500, chargerIdleVolumePercentage, 100); // Changed from currentThrottleFaded
    else
      throttleDependentChargerVolume = chargerIdleVolumePercentage; // Or 0

    // Calculate engine rpm dependent wastegate volume
    if (engineRunning)
      rpmDependentWastegateVolume = map(currentRpm, 0, 500, wastegateIdleVolumePercentage, 100);
    else
      rpmDependentWastegateVolume = wastegateIdleVolumePercentage; // Or 0
  }

  // Calculate engine load (used for torque converter slip simulation, but also for sound character)
  engineLoad = currentThrottle - currentRpm;

  if (engineLoad < 0) // Removed: || escIsBraking || brakeDetect
    engineLoad = 0; 
  engineLoad = constrain(engineLoad, 0, 180); // Keep constrain, original range was 0-180

  tireSquealVolume = 0; // Simplification for now
  // tireSquealVolume = constrain(tireSquealVolume, 0, 100);
}


void engineMassSimulation()
{

  static int32_t targetRpm = 0;         // The engine RPM target
  static int32_t _currentRpm = 0;       // Private current RPM (to prevent conflict with core 1)
  static int32_t _currentThrottle = 0;
  static int32_t lastThrottle;
  // uint16_t converterSlip; // No longer needed for potentiometer mode
  static unsigned long throtMillis;
  static unsigned long wastegateMillis;
  static unsigned long blowoffMillis;
  uint8_t timeBase;

#ifdef SUPER_SLOW
  timeBase = 6; // super slow running, heavy engines, for example locomotive diesels
#else
  timeBase = 2;
#endif

  _currentThrottle = currentThrottle; // currentThrottle is now from potentiometer via mapThrottle()

  if (millis() - throtMillis > timeBase)
  { 
    // Every 2 or 6ms
    throtMillis = millis();

    if (_currentThrottle > 500)
      _currentThrottle = 500;
    if (_currentThrottle < 0) // Ensure throttle doesn't go negative
      _currentThrottle = 0;


    // For pure potentiometer mode, targetRpm is directly what the throttle requests.
    // You can use a direct mapping or a curve if desired.
    // Using reMap with curveLinear is a good general approach.
    targetRpm = reMap(curveLinear, _currentThrottle);
    targetRpm = constrain(targetRpm, minRpm, maxRpm);


    // Engine RPM Acceleration / Deceleration (Core Inertia Simulation) **********************************

    // Accelerate engine
    if (targetRpm > (_currentRpm + acc) && (_currentRpm + acc) < maxRpm && engineState == RUNNING && engineRunning)
    {
      // Removed: if (!airBrakeTrigger) - assuming less complexity for pot mode
      _currentRpm += acc;
      if (_currentRpm > maxRpm)
        _currentRpm = maxRpm;
    }

    // Decelerate engine
    if (targetRpm < _currentRpm)
    {
      _currentRpm -= dec;
      if (_currentRpm < minRpm)
        _currentRpm = minRpm;
    }

    // Speed (sample rate) output for sound generation - THIS IS CRITICAL
    engineSampleRate = map(_currentRpm, minRpm, maxRpm, maxSampleInterval, minSampleInterval); 

    // Update global currentRpm
    // if ( xSemaphoreTake( xRpmSemaphore, portMAX_DELAY ) ) // Already handled in Task1code
    //{
    currentRpm = _currentRpm;
    // xSemaphoreGive( xRpmSemaphore ); 
    // }
  }

  // Optional: Wastegate/Blowoff based on throttle changes.
  // This can add some character even in potentiometer mode.
  // If you don't want this, you can remove this section.

  // Prevent Wastegate from being triggered while downshifting (less relevant here but kept for structure)
  // if (gearDownShiftingInProgress) // gearDownShiftingInProgress no longer relevant for pot mode
  //   wastegateMillis = millis();

  // Trigger Wastegate, if throttle rapidly dropped
  if (lastThrottle - _currentThrottle > 70 && millis() - wastegateMillis > 1000) // Removed: !escIsBraking
  {
    wastegateMillis = millis();
    wastegateTrigger = true;
  }

#if defined JAKEBRAKE_ENGINE_SLOWDOWN && defined JAKE_BRAKE_SOUND
  // This logic might still be desirable if you rapidly drop the potentiometer
  // if (!wastegateTrigger) // This condition might be too restrictive now
  //   blowoffMillis = millis();
  // blowoffTrigger = ((/*gearUpShiftingInProgress || neutralGear - no longer relevant*/) && millis() - blowoffMillis > 20 && millis() - blowoffMillis < 250);
  // Simplified blowoff for pot mode: if throttle drops fast, trigger it.
  // This is a very simplified version and might need tuning or different logic.
  if (lastThrottle - _currentThrottle > 150 && !wastegateTrigger && millis() - blowoffMillis > 500) { // Adjusted threshold and timing
      blowoffMillis = millis();
      blowoffTrigger = true;
  } else if (millis() - blowoffMillis > 250) { // Auto-reset blowoff trigger
      blowoffTrigger = false;
  }

#endif

  lastThrottle = _currentThrottle;
  // Removed the debug print from here as you have one in loop() or mapThrottle()
  Serial.printf("currentThrottle:%i,rpm:%i,lassThr:%i,35:%i,eng:%i,timeBase:%i\n", currentThrottle,currentRpm,lastThrottle,analogRead(35),engineOn, timeBase);
}

//
// =======================================================================================================
// SWITCH ENGINE ON OR OFF (for automatic mode)
// =======================================================================================================
//
void SerialReceive() 
{
  if (Serial.available() > 0) 
  {
    char input = Serial.read();  // Read one character
    engineOn = true;
    Serial.println("input--------------------- ");
    delay(1000);
  }
}
void engineOnOff()
{
  
//   // static unsigned long pulseDelayMillis; // TODO
//   static unsigned long idleDelayMillis;

//   // Engine automatically switched on or off depending on throttle position and 15s delay timne
//   if (currentThrottle > 80 || driveState != 0)
//     idleDelayMillis = millis(); // reset delay timer, if throttle not in neutral

// #ifdef AUTO_ENGINE_ON_OFF
//   if (millis() - idleDelayMillis > 15000)
//   {
//     engineOn = false; // after delay, switch engine off
//   }
// #endif

// #ifdef AUTO_LIGHTS
//   if (millis() - idleDelayMillis > 10000)
//   {
//     lightsOn = false; // after delay, switch light off
//   }
// #endif

//   // Engine start detection
//   if (currentThrottle > 100 && !airBrakeTrigger)
//   {
//     engineOn = true;

// #ifdef AUTO_LIGHTS
//     lightsOn = true;
// #endif
//   }
  if(digitalRead(33)){
    engineOn = true;
  }else{
    engineOn = false;
  }
}

//
// =======================================================================================================
// SHAKER (simulates engine vibrations)
// =======================================================================================================
//

void shaker()
{
  int32_t shakerRpm = 0;

  // Set desired shaker rpm
  if (engineRunning)
    shakerRpm = map(currentRpm, minRpm, maxRpm, shakerIdle, shakerFullThrottle);
  if (engineStart)
    shakerRpm = shakerStart;
  if (engineStop)
    shakerRpm = shakerStop;

  // Shaker on / off
  if (engineRunning || engineStart || engineStop)
    shakerMotor.pwm(shakerRpm);
  else
    shakerMotor.off();
}

//
// =======================================================================================================
// MANUAL GEARBOX DETECTION (Real 3 speed, virtual 3 speed, virtual 16 speed, semi automatic)
// =======================================================================================================
//

void gearboxDetection()
{

  static uint8_t previousGear = 1;
  static bool previousReverse;
  static bool sequentialLock;
  static bool overdrive = false;
  static unsigned long upShiftingMillis;
  static unsigned long downShiftingMillis;
  static unsigned long lastShiftingMillis; // This timer is used to prevent transmission from oscillating!

#if defined TRACKED_MODE or defined STEAM_LOCOMOTIVE_MODE // CH2 is used for left throttle in TRACKED_MODE --------------------------------
  selectedGear = 2;

#else // only active, if not in TRACKED_MODE -------------------------------------------------------------

#if defined OVERDRIVE && defined VIRTUAL_3_SPEED // Additional 4th gear mode for virtual 3 speed ********************************
  if (!crawlerMode)
  {
    // The 4th gear (overdrive) is engaged automatically, if driving @ full throttle in 3rd gear
    if (currentRpm > 490 && selectedGear == 3 && engineLoad < 5 && currentThrottle > 490 && millis() - lastShiftingMillis > 2000)
    {
      overdrive = true;
    }
    if (!escIsBraking)
    { // Lower downshift point, if not braking
      if (currentRpm < 200 && millis() - lastShiftingMillis > 2000)
      {
        overdrive = false;
      }
    }
    else
    { // Higher downshift point, if braking
      if ((currentRpm < 400 || engineLoad > 150) && millis() - lastShiftingMillis > 2000)
      {
        overdrive = false;
      }
    }
    if (selectedGear < 3)
      overdrive = false;
  }
#endif                                           // End of overdrive ******************************************************************************************************

#if not defined VIRTUAL_16_SPEED_SEQUENTIAL && not defined SEMI_AUTOMATIC // 3 gears, selected by 3 position switch **************
  // Gear detection
  if (pulseWidth[2] > 1700)
    selectedGear = 3;
  else if (pulseWidth[2] < 1300)
    selectedGear = 1;
  else
    selectedGear = 2;
  if (overdrive && selectedGear == 3)
    selectedGear = 4;
#endif                                                                    // End of manual 3 speed *************************************************************************************************

#if defined VIRTUAL_16_SPEED_SEQUENTIAL // 16 gears, selected by up / down impulses *********************************************
  if (pulseWidth[2] > 1700 && selectedGear < 16 && !sequentialLock)
  {
    sequentialLock = true;
    selectedGear++;
  }
  else if (pulseWidth[2] < 1300 && selectedGear > 1 && !sequentialLock)
  {
    sequentialLock = true;
    selectedGear--;
  }
  if (pulseWidth[2] > 1400 && pulseWidth[2] < 1600)
    sequentialLock = false;
#endif                                  // End of VIRTUAL_16_SPEED_SEQUENTIAL *************************************************************************************

#if defined SEMI_AUTOMATIC // gears not controlled by the 3 position switch but by RPM limits ************************************
  if (currentRpm > 490 && selectedGear < 3 && engineLoad < 5 && currentThrottle > 490 && millis() - lastShiftingMillis > 2000)
  {
    selectedGear++;
  }
  if (!escIsBraking)
  { // Lower downshift point, if not braking
    if (currentRpm < 200 && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--; //
    }
  }
  else
  { // Higher downshift point, if braking
    if ((currentRpm < 400 || engineLoad > 150) && selectedGear > 1 && millis() - lastShiftingMillis > 2000)
    {
      selectedGear--; // Higher downshift point, if braking
    }
  }
  if (neutralGear || escInReverse)
    selectedGear = 1;
#endif                     // End of SEMI_AUTOMATIC **************************************************************************************************

  // Gear upshifting detection
  if (selectedGear > previousGear)
  {
    gearUpShiftingInProgress = true;
    gearUpShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
    lastShiftingMillis = millis();
  }

  // Gear upshifting duration
  static uint16_t upshiftingDuration = 700;
  if (!gearUpShiftingInProgress)
    upShiftingMillis = millis();
  if (millis() - upShiftingMillis > upshiftingDuration)
  {
    gearUpShiftingInProgress = false;
  }
  // Double-clutch (Zwischengas während dem Hochschalten)
#if defined DOUBLE_CLUTCH
  upshiftingDuration = 900;
  doubleClutchInProgress = (millis() - upShiftingMillis >= 500 && millis() - upShiftingMillis < 600); // Apply full throttle
#endif

  // Gear downshifting detection
  if (selectedGear < previousGear)
  {
    gearDownShiftingInProgress = true;
    gearDownShiftingPulse = true;
    shiftingTrigger = true;
    previousGear = selectedGear;
    lastShiftingMillis = millis();
  }

  // Gear downshifting duration
  if (!gearDownShiftingInProgress)
    downShiftingMillis = millis();
  if (millis() - downShiftingMillis > 300)
  {
    gearDownShiftingInProgress = false;
  }

  // Reverse gear engaging / disengaging detection
  if (escInReverse != previousReverse)
  {
    previousReverse = escInReverse;
    shiftingTrigger = true; // Play shifting sound
  }

#ifdef MANUAL_TRANS_DEBUG
  static unsigned long manualTransDebugMillis;
  if (millis() - manualTransDebugMillis > 100)
  {
    manualTransDebugMillis = millis();
    Serial.printf("MANUAL_TRANS_DEBUG:\n");
    Serial.printf("currentThrottle: %i\n", currentThrottle);
    Serial.printf("selectedGear: %i\n", selectedGear);
    Serial.printf("overdrive: %i\n", overdrive);
    Serial.printf("engineLoad: %i\n", engineLoad);
    Serial.printf("sequentialLock: %s\n", sequentialLock ? "true" : "false");
    Serial.printf("currentRpm: %i\n", currentRpm);
    Serial.printf("currentSpeed: %i\n", currentSpeed);
    Serial.printf("---------------------------------\n");
  }
#endif // MANUAL_TRANS_DEBUG

#endif // End of not TRACKED_MODE -----------------------------------------------------------------------
}

//
// =======================================================================================================
// SIMULATED AUTOMATIC TRANSMISSION GEAR SELECTOR (running on core 0)
// =======================================================================================================
//

void automaticGearSelector()
{

  static unsigned long gearSelectorMillis;
  static unsigned long lastUpShiftingMillis;
  static unsigned long lastDownShiftingMillis;
  uint16_t downShiftPoint = 200;
  uint16_t upShiftPoint = 490;
  static int32_t _currentRpm = 0; // Private current RPM (to prevent conflict with core 1)

  // if ( xSemaphoreTake( xRpmSemaphore, portMAX_DELAY ) )
  //{
  _currentRpm = currentRpm;
  // xSemaphoreGive( xRpmSemaphore ); // Now free or "Give" the semaphore for others.
  // }

  if (millis() - gearSelectorMillis > 100)
  { // Waiting for 100ms is very important. Otherwise gears are skipped!
    gearSelectorMillis = millis();

    // compute load dependent shift points (less throttle = less rpm before shifting up, kick down will shift back!)
    upShiftPoint = map(engineLoad, 0, 180, 390, 490);   // 390, 490
    downShiftPoint = map(engineLoad, 0, 180, 150, 250); // 150, 250

    if (escInReverse)
    { // Reverse (only one gear)
      selectedAutomaticGear = 0;
    }
    else
    { // Forward (multiple gears)

      // Adaptive shift points
      if (millis() - lastDownShiftingMillis > 500 && _currentRpm >= upShiftPoint && engineLoad < 5)
      {                          // 500ms locking timer!
        selectedAutomaticGear++; // Upshifting (load maximum is important to prevent gears from oscillating!)
        lastUpShiftingMillis = millis();
      }
      if (millis() - lastUpShiftingMillis > 600 && selectedAutomaticGear > 1 && (_currentRpm <= downShiftPoint || engineLoad > 100))
      {                          // 600ms locking timer! TODO was 1000
        selectedAutomaticGear--; // Downshifting incl. kickdown
        lastDownShiftingMillis = millis();
      }

      selectedAutomaticGear = constrain(selectedAutomaticGear, 1, NumberOfAutomaticGears);
    }

#ifdef AUTO_TRANS_DEBUG
    Serial.printf("AUTO_TRANS_DEBUG:\n");
    Serial.printf("currentThrottle: %i\n", currentThrottle);
    Serial.printf("selectedAutomaticGear: %i\n", selectedAutomaticGear);
    Serial.printf("engineLoad: %i\n", engineLoad);
    Serial.printf("upShiftPoint: %i\n", upShiftPoint);
    Serial.printf("_currentRpm: %i\n", _currentRpm);
    Serial.printf("downShiftPoint: %i\n", downShiftPoint);
    Serial.printf("-----------------------------------\n");
#endif
  }
}

//
// =======================================================================================================
// LOOP TIME MEASUREMENT
// =======================================================================================================
//

unsigned long loopDuration()
{
  static unsigned long timerOld;
  unsigned long loopTime;
  unsigned long timer = millis();
  loopTime = timer - timerOld;
  timerOld = timer;
  return loopTime;
}

void loop()
{
  mapThrottle();
  rtc_wdt_feed();
}

//
// =======================================================================================================
// 1st MAIN TASK, RUNNING ON CORE 0 (Interrupts are running on this core as well)
// =======================================================================================================
//

void Task1code(void *pvParameters)
{
  for (;;)
  {

    // DAC offset fader
    dacOffsetFade();

    if (xSemaphoreTake(xRpmSemaphore, portMAX_DELAY))
    {
      // Simulate engine mass, generate RPM signal
      engineMassSimulation();

      // Call gear selector
      if (automatic || doubleClutch)
        automaticGearSelector();
      xSemaphoreGive(xRpmSemaphore); // Now free or "Give" the semaphore for others.
    }

    // Switch engine on or off
    engineOnOff();

    // Gearbox detection
    gearboxDetection();
    
    // Feeding the RTC watchtog timer is essential!
    rtc_wdt_feed(); // TODO, test only
  }
}
