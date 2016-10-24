/*********************************************************************
 *
 * BikeLogger
 *
 * http://blog.cuyahoga.co.uk/bikelogger
 *
 *********************************************************************/
#include <SPI.h>
#include <SoftwareSerial.h>
//#include <Adafruit_GFX.h>        // https://github.com/adafruit/Adafruit-GFX-Library
//#include <Adafruit_SSD1306.h>    // https://github.com/adafruit/Adafruit_SSD1306
#include <SdFat.h>               // https://github.com/greiman/SdFat
#include <Timer.h>               // https://github.com/JChristensen/Timer
#include <TinyGPS++.h>           // http://arduiniana.org/libraries/tinygpsplus/

// ===============================================================
// Adafruit MTK3339 Ultimate GPS Breakout    https://www.adafruit.com/products/746
//
//   VIN  ->  3.3V
//   GND  ->  GND
//   RX   ->  D2
//   TX   ->  D3
// ===============================================================
#define GPS_BAUD    9600
#define GPS_RX_PIN  3
#define GPS_TX_PIN  2

// To generate any other command sentences, check out the MTK command datasheet and use a checksum calculator such as http://www.hhhh.org/wiml/proj/nmeaxor.html
// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" // turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28" // turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_MOSTDATA "$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29" // turn on most data
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28" // turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28" // turn off output
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_NAV_SPEED_02_MSEC "$PMTK386,0.2*3F" // set nav speed threshold to 0.2m/sec
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"  // request for updates on antenna status 
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 

// ===============================================================
// Adafruit SSD1306 128x32 OLED
// ===============================================================
/*
#define OLED_MOSI   6
#define OLED_CLK    7
#define OLED_DC     8
#define OLED_CS     5
#define OLED_RESET  13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
*/

// ===============================================================
// SC Card     http://www.hobbytronics.co.uk/microsd-card-regulated-v2
//
//   3V3  ->  3.3V
//   0V   ->  GND
//   CLK  ->  D13
//   D0   ->  D12
//   DI   ->  D11
//   CS   ->  D10
// ===============================================================

const uint8_t chipSelect = SS; // SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
SdFat         sd;
SdFile        csvFile, gpxFile;

#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash

// ===============================================================
// TinyGPS++
// ===============================================================

TinyGPSPlus     gpsInfo;
double          prevLat = 0, prevLng = 0;
uint32_t        prevMovementTime = 0, prevStandstillTime = 0;
SoftwareSerial  gps(GPS_RX_PIN, GPS_TX_PIN);  // The serial connection to the GPS device

// ===============================================================
// Misc runtime configuration
// ===============================================================

#define LOGGER_INTERVAL     5000    // Millis between log writes
#define GPS_DIFF_THRESHOLD  0.0001  // The amount by which both Lat & Lng must differ from prior to be considered movement
#define GPS_SPEED_THRESHOLD 1.0     // The minimum speed (mph) to be considered movement
#define SD_CHIPSELECT       10      // The hardware chip select pin

Timer   t;
int     tickDisplay;
char    filename[24];
char    timestamp[21]; 


void setup()
{
  Serial.begin(115200);
  Serial.print(F("Catching the GPS state every "));
  Serial.print(LOGGER_INTERVAL / 1000);
  Serial.println(F(" seconds of movement"));

  // Set up the GPS (might as well do it here in case the battery backup on the unit expired and nobody noticed)
  Serial.print(F("Initialising the GPS..."));
  gps.begin(GPS_BAUD);
  gps.println(F(PMTK_API_SET_FIX_CTL_1HZ));        // update the position fix once a second
  gps.println(F(PMTK_SET_NMEA_UPDATE_1HZ));        // echo the position once a second
  gps.println(F(PMTK_SET_NMEA_OUTPUT_MOSTDATA));   // output position, altitude, speed, course & satellite info (DOP & elevation)
  gps.println(F(PMTK_ENABLE_WAAS));                // enable WAAS for DGPS
  gps.println(F(PGCMD_NOANTENNA));                 // we ain't got no antenna
  Serial.println(F(" complete"));

  
  // Set up the 128x32 OLED
//  display.begin(SSD1306_SWITCHCAPVCC);
//  display.clearDisplay();
//  display.setTextSize(1);
//  display.setTextColor(WHITE);
//  display.setCursor(0,0);
//  display.println("Hello, world!");
//  display.setTextColor(BLACK, WHITE); // 'inverted' text
//  display.println(3.141592);
//  display.setTextSize(2);
//  display.setTextColor(WHITE);
//  display.print("0x"); display.println(0xDEADBEEF, HEX);
//  display.display();


  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  Serial.print(F("Initialising the SD card..."));
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }
  Serial.println(F(" complete"));

  // Do anything else that needs done, I guess
  
  tickDisplay = t.every(LOGGER_INTERVAL, writeLog);
}

void loop()
{
  // Dispatch incoming characters
  while (gps.available() > 0)
    gpsInfo.encode(gps.read());
//  while (Serial1.available()) {
//    char c = Serial1.read();
//    gpsInfo.encode(c);
//  }
    
  // See if it's time to act on a timer
  t.update();
}

void writeLog() 
{
  if (gpsInfo.date.isValid() && gpsInfo.time.isValid())
  {
    sprintf(timestamp, "%d-%02d-%02dT%02d:%02d:%02dZ", gpsInfo.date.year(), gpsInfo.date.month(), gpsInfo.date.day(), gpsInfo.time.hour(), gpsInfo.time.minute(), gpsInfo.time.second());

    if (gpsInfo.location.isValid()) 
    {
      if ((prevLat == 0 && prevLng == 0) || ((fabs(prevLat - gpsInfo.location.lat()) > GPS_DIFF_THRESHOLD && fabs(prevLng - gpsInfo.location.lng()) > GPS_DIFF_THRESHOLD) && gpsInfo.speed.mph() > GPS_SPEED_THRESHOLD)) 
      {
        // We've got movement (or we've just got our first fix)
        prevLat     = gpsInfo.location.lat();
        prevLng     = gpsInfo.location.lng();
        prevMovementTime = millis();
        prevStandstillTime = 0;
  
        // Write the position to our log files
        writeCSV();
        writeGPX();
      
        // Write the position to the console
        Serial.print(timestamp);
        Serial.print(F("  "));
        Serial.print(gpsInfo.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gpsInfo.location.lng(), 6);
        Serial.print(F("  Speed: "));
        Serial.print(gpsInfo.speed.mph());
        Serial.print(F("  Course: "));
        Serial.print(gpsInfo.course.deg());
        Serial.print(F("  Alt: "));
        Serial.print(gpsInfo.altitude.feet());
        Serial.print(F("  Sats: "));
        Serial.print(gpsInfo.satellites.value());
        Serial.print(F("  HDOP: "));
        Serial.println(gpsInfo.hdop.value());
      } else {
        if (/*(millis() - prevMovementTime) > 60000 &&*/ (prevStandstillTime == 0 || (millis() - prevStandstillTime) > 600000)) 
        {
          // No need to say that we haven't moved more than once a minute
          prevStandstillTime = millis();
          Serial.print(timestamp);
          Serial.print(F("  No change in position - Sats: "));
          Serial.print(gpsInfo.satellites.value());
          Serial.print(F("  HDOP: "));
          Serial.println(gpsInfo.hdop.value());
        }
      }
    }
  } else {
    Serial.print(F("Awaiting fix - current satellites : "));
    Serial.println(gpsInfo.satellites.value());
  }
}

void openCSV() 
{
  boolean csvExists;
  
  if (!csvFile.isOpen()) {
    sprintf(filename, "BikeLogger-%04d%02d%02d.csv", gpsInfo.date.year(), gpsInfo.date.month(), gpsInfo.date.day());
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
    csvFile.print(gpsInfo.location.lat(), 6);
    csvFile.print(F(","));
    csvFile.print(gpsInfo.location.lng(), 6);
    csvFile.print(F(","));
    csvFile.print(gpsInfo.speed.mph());
    csvFile.print(F(","));
    csvFile.print(gpsInfo.course.deg());
    csvFile.print(F(","));
    csvFile.print(gpsInfo.altitude.meters());
    csvFile.print(F(","));
    csvFile.print(gpsInfo.satellites.value());
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
    sprintf(filename, "BikeLogger-%04d%02d%02d.gpx", gpsInfo.date.year(), gpsInfo.date.month(), gpsInfo.date.day());
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
        gpxFile.print(F("    <time>"));
        gpxFile.print(timestamp);
        gpxFile.println(F("</time>"));
        gpxFile.println(F("  </metadata>"));
        gpxFile.println(F("  <trk>"));
        gpxFile.println(F("    <name>Sample Ride</name>"));
        gpxFile.println(F("    <trkseg>"));
        gpxFile.println(F("    </trkseg>"));
        gpxFile.println(F("  </trk>"));
        gpxFile.println(F("</gpx>"));
      } else {
        gpxFile.seekEnd(-20);
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
    gpxFile.print(gpsInfo.location.lat(), 6);
    gpxFile.print(F("\" lon=\""));
    gpxFile.print(gpsInfo.location.lng(), 6);
    gpxFile.print(F("\"><ele>"));
    gpxFile.print(gpsInfo.altitude.meters());
    gpxFile.print(F("</ele><time>"));
    gpxFile.print(timestamp);
    gpxFile.print(F("</time><extensions><gpxtpx:TrackPointExtension><gpxtpx:speed>"));
    gpxFile.print(gpsInfo.speed.mps());
    gpxFile.print(F("</gpxtpx:speed><gpxtpx:course>"));
    gpxFile.print(gpsInfo.course.deg());
    gpxFile.print(F("</gpxtpx:course></gpxtpx:TrackPointExtension></extensions>"));
    gpxFile.println(F("</trkpt>"));
    gpxFile.println(F("    </trkseg>"));
    gpxFile.println(F("  </trk>"));
    gpxFile.println(F("</gpx>"));
    
    // Force data to SD and update the directory entry to avoid data loss.
    if (!gpxFile.sync() || gpxFile.getWriteError()) {
      error("gpx write error");
    }
  }
}

