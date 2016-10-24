/*********************************************************************
 *
 * BikeLogger
 *
 * http://blog.cuyahoga.co.uk/bikelogger
 *
 *********************************************************************/
#include <SPI.h>
#include <SoftwareSerial.h>
#include <SdFat.h>               // https://github.com/greiman/SdFat
#include <Timer.h>               // https://github.com/JChristensen/Timer
#include <TinyGPS++.h>           // http://arduiniana.org/libraries/tinygpsplus/

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;


// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// Runtime configuration
#define LOGGER_INTERVAL     5000   // Millis between log writes
#define SD_CHIPSELECT       10     // The hardware chip select pin

Timer   t;
int     tickDisplay;
char    filename[20];
char    buffer[20]; 

SdFat   sd;       // File system object
SdFile  logfile;  // Log file

// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("Catching the GPS state every n seconds"));
  Serial.println();

  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  
  tickDisplay = t.every(LOGGER_INTERVAL, writeLog);
}

void loop()
{
  // Dispatch incoming characters
  while (ss.available() > 0)
    gps.encode(ss.read());

  // See if it's time to act on a timer
  t.update();
}

void writeLog() {

  if (gps.location.isValid())
  {

    if (!logfile.isOpen()) {
      sprintf(filename, "GPSLog-%04d%02d%02d.csv", gps.date.year(), gps.date.month(), gps.date.day());
      if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
        error("file.open");
      } else {
        Serial.print(F("Logging to "));
        Serial.println(filename);
      }
    }
    
    sprintf(buffer, "%d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

    if (logfile.isOpen()) {
      logfile.print(buffer);
      logfile.print(F(","));
      logfile.print(gps.location.lat(), 6);
      logfile.print(F(","));
      logfile.print(gps.location.lng(), 6);
      logfile.print(F(","));
      logfile.print(gps.speed.mph());
      logfile.print(F(","));
      logfile.print(gps.course.deg());
      logfile.print(F(","));
      logfile.print(gps.altitude.feet());
      logfile.print(F(","));
      logfile.print(gps.satellites.value());
      logfile.println();

      // Force data to SD and update the directory entry to avoid data loss.
      if (!logfile.sync() || logfile.getWriteError()) {
        error("write error");
      }

      // Write the same output to the console
      Serial.print(buffer);
      Serial.print(F(","));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      Serial.print(F(","));
      Serial.print(gps.speed.mph());
      Serial.print(F(","));
      Serial.print(gps.course.deg());
      Serial.print(F(","));
      Serial.print(gps.altitude.feet());
      Serial.print(F(","));
      Serial.print(gps.satellites.value());
      Serial.println();
    }
  } else {
    Serial.print(buffer);
    Serial.print(F(",,,,,,"));
    Serial.print(gps.satellites.value());
    Serial.println();
  }
}
