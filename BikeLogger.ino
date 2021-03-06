/*********************************************************************
 *
 * BikeLogger for M0
 *
 * http://blog.cuyahoga.co.uk/bikelogger
 *
 *********************************************************************/
#include <SPI.h>
#include <Adafruit_GFX.h>        // https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>    // https://github.com/adafruit/Adafruit_SSD1306
#include <SdFat.h>               // https://github.com/greiman/SdFat
#include <TimeLib.h>             // https://github.com/PaulStoffregen/Time
#include <Timer.h>               // https://github.com/JChristensen/Timer
#include <Timezone.h>            // https://github.com/JChristensen/Timezone
#include <TinyGPS++.h>           // http://arduiniana.org/libraries/tinygpsplus/

// ===============================================================
// Adafruit Feather M0 Adalogger             https://www.adafruit.com/products/2796
// ===============================================================
#define PIN_SD_CHIPSELECT 4
#define PIN_LED_GREEN     8
#define PIN_LED_RED       13
#define PIN_VBAT          A7
SdFat         sd;
SdFile        csvFile, gpxFile;

#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash


// ===============================================================
// Adafruit MTK3339 Ultimate GPS Breakout    https://www.adafruit.com/products/746
//
//   VIN  ->  3.3V
//   GND  ->  GND
//   RX   ->  D0
//   TX   ->  D1
// ===============================================================
#define GPS_BAUD    9600

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
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 6
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// ===============================================================
// TinyGPS++
// ===============================================================

TinyGPSPlus     gpsInfo;
#define         gps Serial1
double          prevLat = 0, prevLng = 0;
uint32_t        prevMovementTime = 0, prevStandstillTime = 0;

// ===============================================================
// Dates & Times
// ===============================================================

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

TimeChangeRule tcrBST = {"BST", Last, Sun, Mar, 1, 60};    // British Summer Time = UTC + 1 hour
TimeChangeRule tcrGMT = {"GMT", Last, Sun, Oct, 1, 0};     // Greenwich Mean Time = UTC
Timezone tzGMT(tcrBST, tcrGMT);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utcTime, localTime;

// ===============================================================
// Misc runtime configuration
// ===============================================================

#define LOGGER_INTERVAL     1000    // Millis between log writes during movement
#define GPS_DIFF_THRESHOLD  0.00003 // The amount by which both Lat & Lng must differ from prior to be considered movement
#define GPS_SPEED_THRESHOLD 0.6     // The minimum speed (mph) to be considered movement
#define SD_CHIPSELECT       10      // The hardware chip select pin
#define DISPLAY_INTERVAL    250     // Millis between display refreshes
#define FLIPPER_INTERVAL    5000    // Millis between flipping elapsed time/distance on display

Timer         t;
int           tickDisplay, tickLogger, tickFlipper;
char          filename[24];
char          timestamp[21]; 
unsigned long startTime = 0, metresTravelled = 0;
boolean       timeDistFlipper, logging;

void setup()
{
  Serial.begin(115200);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
  Serial.print(F("Catching the GPS state every "));
  Serial.print(LOGGER_INTERVAL / 1000);
  Serial.println(F(" seconds of movement"));

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  //t.oscillate(PIN_LED_GREEN, 333, HIGH);

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
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.display();

  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  Serial.print(F("Initialising the SD card..."));
  if (!sd.begin(PIN_SD_CHIPSELECT, SPI_FULL_SPEED)) {
    sd.initErrorHalt();
  }
  Serial.println(F(" complete"));

  // Start the timer(s)
  tickDisplay = t.every(DISPLAY_INTERVAL, refreshDisplay);
  tickFlipper = t.every(FLIPPER_INTERVAL, updateFlipper);
  tickLogger  = t.every(LOGGER_INTERVAL, writeLog);
}

void loop()
{
  // Dispatch incoming characters
  while (gps.available() > 0)
    gpsInfo.encode(gps.read());


  // Do anything else that needs done, I guess

  
  // See if it's time to act on a timer
  t.update();
}


// ===============================================================
// Functions
// ===============================================================

boolean isGpsLocationValid()
{
  return (gpsInfo.location.rawLat().deg > 0 && gpsInfo.location.rawLng().deg > 0);
}

boolean isGpsDateValid() 
{
  return (gpsInfo.date.year() > 0 && gpsInfo.date.month() > 0 && gpsInfo.date.day() > 0);
}

void updateFlipper()
{
  timeDistFlipper = !timeDistFlipper;
}

void refreshDisplay() {

  char buffer[9];
  static boolean flipflop;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 1);

  // Satellites - one block each, solid if we have a location fix
  for (byte sat = 0; sat < min(gpsInfo.satellites.value(), 12); sat++) 
  {
    if (isGpsLocationValid()) 
    {
      display.fillRect(1 + (sat * 10), 1, 8, 5, WHITE);
    } else {
      display.drawRect(1 + (sat * 10), 1, 8, 5, WHITE);
    }
  }

  // Logging - solid circle if we're moving, otherwise just the outline
  if (logging) 
  {
    display.fillCircle(124, 3, 3, WHITE);
  } else {
    display.drawCircle(124, 3, 3, WHITE);
  }

  if (isGpsDateValid()) 
  {
    // Determine the time (that's in UTC from the GPS), accounting for BST if necessary
    localTime = tzGMT.toLocal(tmConvert_t(gpsInfo.date.year(), gpsInfo.date.month(), gpsInfo.date.day(), gpsInfo.time.hour(), gpsInfo.time.minute(), gpsInfo.time.second()), &tcr);
    renderTime(hour(localTime), minute(localTime), second(localTime), 4, 9);
    display.setTextSize(1);
    sprintf(buffer, "%02d-%02d-%02d", day(localTime), month(localTime), year(localTime) - 2000);
    display.setCursor(7, 25);
    display.print(buffer);
    
    if (timeDistFlipper)
    {
      // Elapsed Time
      unsigned long elapsedTime = ((millis() - startTime) / 1000);
      renderTime(numberOfHours(elapsedTime), numberOfMinutes(elapsedTime), gpsInfo.time.second(), 73, 9);
      display.setTextSize(1);
      display.setCursor(78, 25);
      display.print(F("Elapsed"));
    } else {
      // Distance
      display.setTextSize(2);
//      double distance = _GPS_MILES_PER_METER * metresTravelled;
//      display.setCursor(distance < 10 ? 79 : 67, 9); 
//      display.print(distance, 2);
//      display.setTextSize(1);
//      display.setCursor(78, 25);
//      display.print(F("Distance"));

      float measuredvbat = analogRead(PIN_VBAT);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
//      Serial.print("VBat: " ); Serial.println(measuredvbat);
      display.setCursor(75, 9);
      display.print(measuredvbat, 2);
      display.setTextSize(1);
      display.setCursor(78, 25);
      display.print(F("Voltage"));
    }
  }
  else 
  {
    display.setTextSize(2);
    display.setCursor(1, 14);
    display.print(F("Hang on..."));
  }
  display.display();
}

void renderTime(int hour, int minute, int second, int x, int y) 
{
  char buffer[3];
  
  display.setTextSize(2);
  sprintf(buffer, "%02d", hour);
  display.setCursor(x, y);
  display.print(buffer);
  if (second % 2 == 0) 
  {
    display.setCursor(x + 21, y);
    display.print(F(":"));
  }
  sprintf(buffer, "%02d", minute);
  display.setCursor(x + 30, y);
  display.print(buffer);
}

void writeLog() 
{
  if (isGpsDateValid())
  {
    sprintf(timestamp, "%d-%02d-%02dT%02d:%02d:%02dZ", gpsInfo.date.year(), gpsInfo.date.month(), gpsInfo.date.day(), gpsInfo.time.hour(), gpsInfo.time.minute(), gpsInfo.time.second());

    if (isGpsLocationValid()) 
    {
      if ((prevLat == 0 && prevLng == 0) || ((fabs(prevLat - gpsInfo.location.lat()) > GPS_DIFF_THRESHOLD && fabs(prevLng - gpsInfo.location.lng()) > GPS_DIFF_THRESHOLD) && gpsInfo.speed.mph() > GPS_SPEED_THRESHOLD)) 
      {
        // We've got movement (or we've just got our first fix)
        logging = true;
        if (startTime == 0) 
        {
          startTime = millis();
        }
        if (prevLat != 0 && prevLng != 0) 
        {
          metresTravelled += TinyGPSPlus::distanceBetween(gpsInfo.location.lat(), gpsInfo.location.lng(), prevLat, prevLng);
        }
        prevLat          = gpsInfo.location.lat();
        prevLng          = gpsInfo.location.lng();
        prevMovementTime = millis();
  
        // Write the position to our log files
        writeCSV();
        writeGPX();
      
        // Report the position to the console
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
        reportSatsAndHdop();
      } else {
        // We haven't moved 
        logging = false;
        if ((millis() - prevMovementTime) < 59000)
        {
          // Do nothing, as it's not been a minute
          prevStandstillTime = 0;
        } 
        else 
        {
          // It's been at least a minute since we moved
          if (prevStandstillTime == 0) 
          {
            // Report no movement
            reportNoMovement();
          } 
          else 
          {
            if ((millis() - prevStandstillTime) >= 60000) 
            {
              // Report no movement
              reportNoMovement();
            }
          }
        }
      }
    }
  } else {
    Serial.print(F("Awaiting fix - current satellites : "));
    Serial.println(gpsInfo.satellites.value());
  }
}

void reportNoMovement()
{
  prevStandstillTime = millis();
  Serial.print(timestamp);
  Serial.print(F("  No change in position -"));
  reportSatsAndHdop();
}

void reportSatsAndHdop()
{
  Serial.print(F("  Sats: "));
  Serial.print(gpsInfo.satellites.value());
  Serial.print(F("  HDOP: "));
  Serial.println(gpsInfo.hdop.value());
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
        gpxFile.println(F("<gpx creator=\"BikeLogger\" version=\"1.1\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v2\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">"));
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

time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss)
{
  tmElements_t tmSet;
  tmSet.Year = YYYY - 1970;
  tmSet.Month = MM;
  tmSet.Day = DD;
  tmSet.Hour = hh;
  tmSet.Minute = mm;
  tmSet.Second = ss;
  return makeTime(tmSet);         //convert to time_t
}


