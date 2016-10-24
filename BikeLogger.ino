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
#define LOGGER_INTERVAL     10000   // Millis between log writes
#define SD_CHIPSELECT       10     // The hardware chip select pin

Timer   t;
int     tickDisplay;
char    filename[24];
char    timestamp[21]; 

SdFat   sd;
SdFile  csvFile, gpxFile;

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

void writeLog() 
{
  if (gps.location.isValid())
  {
    sprintf(timestamp, "%d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
    writeCSV();
    writeGPX();
    // Write the same output to the console
    Serial.print(timestamp);
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
    Serial.println(gps.satellites.value());
  } else {
    Serial.print(F("Awaiting fix - current satellites : "));
    Serial.println(gps.satellites.value());
  }
}

void openCSV() 
{
  boolean csvExists;
  
  if (!csvFile.isOpen()) {
    sprintf(filename, "BikeLogger-%04d%02d%02d.csv", gps.date.year(), gps.date.month(), gps.date.day());
    csvExists = sd.exists(filename); 
    if (!csvFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      error("csvfile.open");
    } else {
      Serial.print(F("Logging to "));
      Serial.println(filename);
      if (!csvExists) {
        csvFile.println(F("Timestamp,Lat,Lon,Speed_MPH,Course_Deg,Altitude_M,Satellites"));
      }
    }
  }
}

void writeCSV() 
{
  openCSV();
    
  if (csvFile.isOpen()) {
    csvFile.print(timestamp);
    csvFile.print(F(","));
    csvFile.print(gps.location.lat(), 6);
    csvFile.print(F(","));
    csvFile.print(gps.location.lng(), 6);
    csvFile.print(F(","));
    csvFile.print(gps.speed.mph());
    csvFile.print(F(","));
    csvFile.print(gps.course.deg());
    csvFile.print(F(","));
    csvFile.print(gps.altitude.meters());
    csvFile.print(F(","));
    csvFile.print(gps.satellites.value());
    csvFile.println();

    // Force data to SD and update the directory entry to avoid data loss.
    if (!csvFile.sync() || csvFile.getWriteError()) {
      error("csv write error");
    }
  }
}

void openGPX() 
{
  boolean gpxExists;

  if (!gpxFile.isOpen()) {
    sprintf(filename, "BikeLogger-%04d%02d%02d.gpx", gps.date.year(), gps.date.month(), gps.date.day());
    gpxExists = sd.exists(filename); 

    if (!gpxFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      error("gpxfile.open");
    } else {
      Serial.print(F("Logging to "));
      Serial.println(filename);
      if (!gpxExists) {
        gpxFile.println(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
        gpxFile.println(F("<gpx creator=\"BikeLogger\" version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">"));
        gpxFile.println(F("  <metadata>"));
        gpxFile.println(F("    <time>2016-01-31T14:57:03Z</time>"));
        gpxFile.println(F("  </metadata>"));
        gpxFile.println(F("  <trk>"));
        gpxFile.println(F("    <name>Sample Ride</name>"));
        gpxFile.println(F("    <trkseg>"));
        gpxFile.println(F("    </trkseg>"));
        gpxFile.println(F("  </trk>"));
        gpxFile.println(F("</gpx>"));
      }
    }
  }
}

void writeGPX() 
{
  openGPX();
  
  if (gpxFile.isOpen()) {
    gpxFile.seekEnd(-33);
    gpxFile.print(F("      <trkpt lat=\""));
    gpxFile.print(gps.location.lat(), 6);
    gpxFile.print(F("\" lon=\""));
    gpxFile.print(gps.location.lng(), 6);
    gpxFile.print(F("\"><ele>"));
    gpxFile.print(gps.altitude.meters());
    gpxFile.print(F("</ele><time>"));
    gpxFile.print(timestamp);
    gpxFile.println(F("</time></trkpt>"));
    gpxFile.println(F("    </trkseg>"));
    gpxFile.println(F("  </trk>"));
    gpxFile.println(F("</gpx>"));
    
    // Force data to SD and update the directory entry to avoid data loss.
    if (!gpxFile.sync() || gpxFile.getWriteError()) {
      error("gpx write error");
    }
  }
}

