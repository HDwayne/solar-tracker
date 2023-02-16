#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

extern "C"
{
#include "SolTrack.h"
}

/* =================================================================== */
// Settings
/* =================================================================== */

// limit switch settings

#define LIMIT_SWITCH_LEFT GPIO_NUM_16  // pin where the left limit switch is connected
#define LIMIT_SWITCH_RIGHT GPIO_NUM_17 // pin where the right limit switch is connected
#define DEBOUNCING_TIME 300            // debounce time in ms

// relay settings

#define RELAY_AZIMUTH_LEFT GPIO_NUM_25  // pin, left rotation of azimuth motor
#define RELAY_AZIMUTH_RIGHT GPIO_NUM_33 // pin, right rotation of azimuth motor

// WiFi settings

#define WIFI_SSID "wifi"     // WiFi SSID
#define WIFI_PASSWORD "pass" // password for WiFi
#define WIFI_TIMEOUT 20000   // timeout for WiFi connection (in ms)
#define WIFI_DELAY 10000     // delay between each WiFi connection attempt (in ms)

// SolTrack config

#define ST_USE_DEGREES 1                   // Input (geographic position) and output are in degrees
#define ST_USE_NORTH_EQUALS_ZERO 1         // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
#define ST_COMPUTE_REFRACTION_EQUATORIAL 1 // Compute refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
#define ST_COMPUTE_DISTANCE 1              // Compute the distance to the Sun in AU: 0-no, 1-yes
#define ST_DELAY 300000                    // Delay between each computation (in ms)

// localisation config

#define ST_LATITUDE 43.8045  // Latitude of the panel
#define ST_LONGITUDE 1.3883  // Longitude of the panel
#define ST_PRESSURE 101.0    // Atmospheric pressure in kPa
#define ST_TEMPERATURE 283.0 // Atmospheric temperature in K

// panel settings

#define AZIMUTH_MIN 5              // minimum azimuth angle (in degrees) panel can reach
#define AZIMUTH_MAX 355            // maximum azimuth angle (in degrees) panel can reach
#define MINIMUM_ROTATION_TIME 2000 // minimum time to rotate the panel (in ms) (motor can't rotate with time < 2s)

// NTP config

const char *ntpServer = "fr.pool.ntp.org"; // NTP server (generic: pool.ntp.org)
// Do not change UTC offset, soltrack is based on UT time

/* =================================================================== */
// Variables
/* =================================================================== */

// limit switch variables

volatile int limitSwitchLeftReached = 0;  // nb of times left limit switch has been reached
volatile int limitSwitchRightReached = 0; // nb of times right limit switch has been reached

volatile unsigned long debounceLimitSwitchLeft;  // last time left limit switch has been reached (ms)
volatile unsigned long debounceLimitSwitchRight; // last time right limit switch has been reached (ms)

portMUX_TYPE muxLimitSwitchLeft = portMUX_INITIALIZER_UNLOCKED;  // mutex for left limit switch
portMUX_TYPE muxLimitSwitchRight = portMUX_INITIALIZER_UNLOCKED; // mutex for right limit switch

// task variables

bool dateTimeInitialized = false; // flag to know if date and time are initialized

// soltrack variables

struct Time CurrentTime; // current time (UT)
struct Location loc;     // location of the panel
struct Position pos;     // position of the sun
struct RiseSet riseSet;  // sunrise and sunset

// rotation variables

unsigned long durationRotationOneDeg; // duration of one degree rotation (in ms)
double CurrentAzimuth;                // current azimuth angle (in degrees)

/* =================================================================== */
// convert Functions
/* =================================================================== */

/**
 * Given the current desired azimuth, return the number of milliseconds it will take
 * to rotate the panel to the desired azimuth
 *
 * @param azimuth the azimuth to rotate to
 *
 * @return The time it takes to rotate the panel from the current azimuth to the desired azimuth.
 */
unsigned long timeToNextAzimuth(double azimuth)
{
  return durationRotationOneDeg * abs(azimuth - CurrentAzimuth);
}

/**
 * It takes a double and returns the hours (int).
 *
 * @param d The double value to convert to an hour.
 *
 * @return The number of hours.
 */
int doubleToHour(double d)
{
  return (int)d;
}

/**
 * It takes a double, returns the minutes (int).
 *
 * @param d The double value to convert to minutes.
 *
 * @return The number of minutes.
 */
int doubleToMinute(double d)
{
  return ((int)(d * 3600)) % 3600 / 60;
}

/* =================================================================== */
// Azimuth rotation Functions
/* =================================================================== */

/**
 * If the limit switch on the left is not reached, turn on the relay that rotates the azimuth left.
 */
void rotateAzimuthLeft()
{
  printf("Azimuth: rotate left\n");
  portENTER_CRITICAL_ISR(&muxLimitSwitchRight);
  limitSwitchRightReached = 0;
  portEXIT_CRITICAL_ISR(&muxLimitSwitchRight);

  if (limitSwitchLeftReached == 0)
  {
    digitalWrite(RELAY_AZIMUTH_LEFT, HIGH);
  }
}

/**
 * If the limit switch on the right is not reached, turn on the relay that rotates the azimuth right.
 */
void rotateAzimuthRight()
{
  printf("Azimuth: rotate Right\n");
  portENTER_CRITICAL_ISR(&muxLimitSwitchLeft);
  limitSwitchLeftReached = 0;
  portEXIT_CRITICAL_ISR(&muxLimitSwitchLeft);

  if (limitSwitchRightReached == 0)
  {
    digitalWrite(RELAY_AZIMUTH_RIGHT, HIGH);
  }
}

/**
 * This function stops the rotation of the azimuth motor
 */
void rotateAzimuthStop()
{
  printf("Azimuth: rotate stop\n");
  digitalWrite(RELAY_AZIMUTH_LEFT, LOW);
  digitalWrite(RELAY_AZIMUTH_RIGHT, LOW);
  // delay(1000);
}

/**
 * This function rotates the azimuth motor to the desired azimuth
 *
 * @param azimuth The azimuth to rotate to.
 */
void rotateAzimuthTo(double azimuth)
{
  printf("azimuth: rotate %f° to %f°\n", CurrentAzimuth, azimuth);

  unsigned long timeToRotate = timeToNextAzimuth(azimuth);
  printf("azimuth: time to rotate, %lu ms\n", timeToRotate);

  if (timeToRotate < MINIMUM_ROTATION_TIME)
  {
    printf("azimuth: rotation time too short, skip rotation\n");
    return;
  }

  if (azimuth < CurrentAzimuth)
  {
    rotateAzimuthLeft();

    unsigned long timeStart = millis();
    while (limitSwitchLeftReached == 0 && millis() < timeStart + timeToRotate)
      ;

    rotateAzimuthStop();
  }
  else
  {
    rotateAzimuthRight();

    unsigned long timeStart = millis();
    while (limitSwitchRightReached == 0 && millis() < timeStart + timeToRotate)
      ;

    rotateAzimuthStop();
  }

  if (limitSwitchLeftReached != 0)
  {
    CurrentAzimuth = AZIMUTH_MIN;
  }
  else if (limitSwitchRightReached != 0)
  {
    CurrentAzimuth = AZIMUTH_MAX;
  }
  else
  {
    CurrentAzimuth = azimuth;
  }

  printf("azimuth: new actual azimuth, %f°\n", CurrentAzimuth);
}

/**
 * It rotates the solar panel to the azimuth angle of the sun
 * check if the sun is not below the horizon by comparing the current time with the sunrise and sunset time
 * check if the sun position respect the boundaries of the solar panel
 */
void rotateAzimuthToSun()
{
  if (!(
          doubleToHour(riseSet.riseTime) * 60 + doubleToMinute(riseSet.riseTime) <= CurrentTime.hour * 60 + CurrentTime.minute &&
          doubleToHour(riseSet.setTime) * 60 + doubleToMinute(riseSet.setTime) >= CurrentTime.hour * 60 + CurrentTime.minute))
  {
    printf("current hour %d:%d is not between sunrise %d:%d and sunset %d:%d\n",
           CurrentTime.hour, CurrentTime.minute,
           doubleToHour(riseSet.riseTime), doubleToMinute(riseSet.riseTime),
           doubleToHour(riseSet.setTime), doubleToMinute(riseSet.setTime));
    printf("azimuth: sun is sleeping\n");
    // TODO night mode (panel horizontal position with elevation motor and deep sleep)
    return;
  }

  if (!(pos.azimuthRefract > (double)AZIMUTH_MIN && pos.azimuthRefract < (double)AZIMUTH_MAX))
  {
    printf("current azimuth %f° is not between %f° and %f°\n", pos.azimuthRefract, (double)AZIMUTH_MIN, (double)AZIMUTH_MAX);
    printf("azimuth: sun azimuth is not in the range of the panel\n");
    return;
  }

  rotateAzimuthTo(pos.azimuthRefract);
}

/**
 * Determine the time it takes to rotate the azimuth motor from the left limit switch to the right
 * limit switch
 */
void determineTimeToRotateAzimuth()
{
  printf("calibration: Determine time to rotate\n");
  rotateAzimuthLeft();

  while (limitSwitchLeftReached == 0)
    ;

  rotateAzimuthStop();
  printf("calibration: left limit switch reached\n");
  time_t begin = time(NULL);
  rotateAzimuthRight();

  while (limitSwitchRightReached == 0)
    ;

  rotateAzimuthStop();
  printf("calibration: right limit switch reached\n");
  time_t end = time(NULL);

  unsigned long durationRotation = (unsigned long)(difftime(end, begin) * 1000); // in ms
  durationRotationOneDeg = durationRotation / (AZIMUTH_MAX - AZIMUTH_MIN);
  CurrentAzimuth = AZIMUTH_MAX;

  printf("calibration:\n\tduration of rotation: %ld ms\n\tduration of rotation for 1°: %ld ms\n", durationRotation, durationRotationOneDeg);
}

/* =================================================================== */
// Limit switch Functions
/* =================================================================== */

/**
 * It increments a variable called limitSwitchLeftReached when the left limit switch is reached.
 * button debounce is implemented
 */
void IRAM_ATTR azimuthLimitLeft()
{
  portENTER_CRITICAL_ISR(&muxLimitSwitchLeft); // semaphore (0/1)
  if ((long)(millis() - debounceLimitSwitchLeft) >= DEBOUNCING_TIME)
  {
    limitSwitchLeftReached++;
    debounceLimitSwitchLeft = millis();
  }
  portEXIT_CRITICAL_ISR(&muxLimitSwitchLeft);
}

/**
 * It increments a variable called limitSwitchRightReached when the left limit switch is reached.
 * button debounce is implemented
 */
void IRAM_ATTR azimuthLimitRight()
{
  portENTER_CRITICAL_ISR(&muxLimitSwitchRight);
  if ((long)(millis() - debounceLimitSwitchRight) >= DEBOUNCING_TIME)
  {
    limitSwitchRightReached++;
    debounceLimitSwitchRight = millis();
  }
  portEXIT_CRITICAL_ISR(&muxLimitSwitchRight);
}

/* =================================================================== */
// FreeRTOS Tasks
/* =================================================================== */

/**
 * Ths fonction is a task use by FreeRTOS to keep the wifi connection alive
 *
 * If WiFi is connected, wait 10 seconds and check again. If WiFi is not connected, try to connect. If
 * connection fails, wait 10 seconds and try again
 */
void keepWiFiAlive(void *parameters)
{
  for (;;)
  {
    // check if WiFi is connected, yes: wait and check again, no: try to reconnect
    if (WiFi.status() == WL_CONNECTED)
    {
      vTaskDelay(pdMS_TO_TICKS(WIFI_DELAY)); // wait 10 seconds before checking again
      continue;
    }

    printf("WiFi disconnected, connecting...\n");

    WiFi.mode(WIFI_STA);                  // set wifi mode to station
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // connect to wifi network

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT)
      ;

    if (WiFi.status() != WL_CONNECTED)
    {
      printf("WiFi connection failed\n");
      vTaskDelay(pdMS_TO_TICKS(WIFI_DELAY)); // wait 10 seconds before trying again
      continue;
    }

    printf("WiFi connected\n");
  }
}

/**
 * This function is a task use by FreeRTOS to update the date time from the NTP server
 *
 * It's a loop that runs forever, and every minutes it updates the global variable CurrentTime with the
 * current time
 */
void UpdateDateTime(void *parameters)
{
  configTime(0, 0, ntpServer);
  struct tm timeinfo;

  for (;;)
  {
    if (!getLocalTime(&timeinfo))
    {
      printf("Failed to obtain time\n");
      dateTimeInitialized = false;
      vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1 second before trying again
      continue;
    }

    CurrentTime.year = timeinfo.tm_year + 1900;
    CurrentTime.month = timeinfo.tm_mon + 1;
    CurrentTime.day = timeinfo.tm_mday;
    CurrentTime.hour = timeinfo.tm_hour;
    CurrentTime.minute = timeinfo.tm_min;

    dateTimeInitialized = true;

    vTaskDelay(pdMS_TO_TICKS(60000)); // wait 1 minute before updating again
  }
}

/**
 * This function is a task use by FreeRTOS to update the solar data
 *
 * The function is called every 5 minutes and it calculates the position of the sun in the sky and then
 * rotates the solar panel to face the sun
 */
void UpdateSolarData(void *parameters)
{
  for (;;)
  {
    if (!dateTimeInitialized)
    {
      printf("Update solar data: Date time not initialized\n");
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    SolTrack_RiseSet(CurrentTime, loc, &pos, &riseSet, 0.0, ST_USE_DEGREES, ST_USE_NORTH_EQUALS_ZERO);
    SolTrack(CurrentTime, loc, &pos, ST_USE_DEGREES, ST_USE_NORTH_EQUALS_ZERO, ST_COMPUTE_REFRACTION_EQUATORIAL, ST_COMPUTE_DISTANCE);

    printf("------------------------------------------------------------------------\n");
    printf("Rise time:      %11.5lf,    azimuth sunrise:   %11.5lf°\n", riseSet.riseTime, riseSet.riseAzimuth);
    printf("Transit time:   %11.5lf,    altitude culmination:  %11.5lf°\n", riseSet.transitTime, riseSet.transitAltitude);
    printf("Set time:       %11.5lf,    azimuth sunset:   %11.5lf°\n\n", riseSet.setTime, riseSet.setAzimuth);
    printf("Corrected azimuth, altitude:         %10.6lf° %10.6lf°\n", pos.azimuthRefract, pos.altitudeRefract);
    printf("------------------------------------------------------------------------\n");

    rotateAzimuthToSun();

    vTaskDelay(pdMS_TO_TICKS(ST_DELAY));
  }
}

/* =================================================================== */
// Main Functions
/* =================================================================== */

void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;

  if (!digitalPinIsValid(LIMIT_SWITCH_LEFT) || !digitalPinIsValid(LIMIT_SWITCH_RIGHT))
  {
    printf("Limit switch pins are not valid\n");
    while (1)
      ;
  }

  pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);

  if (!digitalPinIsValid(RELAY_AZIMUTH_LEFT) || !digitalPinIsValid(RELAY_AZIMUTH_RIGHT))
  {
    printf("Relay pins are not valid\n");
    while (1)
      ;
  }

  if (!digitalPinCanOutput(RELAY_AZIMUTH_LEFT) || !digitalPinCanOutput(RELAY_AZIMUTH_RIGHT))
  {
    printf("Relay pins can't output\n");
    while (1)
      ;
  }

  pinMode(RELAY_AZIMUTH_LEFT, OUTPUT);
  pinMode(RELAY_AZIMUTH_RIGHT, OUTPUT);

  attachInterrupt(LIMIT_SWITCH_LEFT, azimuthLimitLeft, RISING);
  attachInterrupt(LIMIT_SWITCH_RIGHT, azimuthLimitRight, RISING);

  determineTimeToRotateAzimuth();

  xTaskCreatePinnedToCore(keepWiFiAlive, "keep WiFi Alive", 5000, NULL, 1, NULL, CONFIG_ARDUINO_RUNNING_CORE);

  printf("keepWiFiAlive task created\n");

  xTaskCreate(UpdateDateTime, "DateTime update", 5000, NULL, 1, NULL);

  printf("UpdateDateTime task created\n");

  loc.longitude = ST_LONGITUDE;
  loc.latitude = ST_LATITUDE;
  loc.pressure = ST_PRESSURE;
  loc.temperature = ST_TEMPERATURE;

  xTaskCreate(UpdateSolarData, "SolarData update", 5000, NULL, 2, NULL);

  printf("UpdateSolarData task created\n");
}

void loop() {}
