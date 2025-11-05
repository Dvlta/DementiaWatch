// Geo Fencing project with Adafruit IO
// Libraries
#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <ctype.h>

// Alarm pins
const int ledPin = 6;

// Size of the geo fence (in meters)
const float maxDistance = 50;

// Latitude & longitude for distance measurement
float initialLatitude;
float initialLongitude;
float latitude, longitude, speed_kph, heading, altitude;

// FONA pins configuration
#define FONA_RX              2   // FONA serial RX pin (pin 2 for shield).
#define FONA_TX              3   // FONA serial TX pin (pin 3 for shield).
#define FONA_RST             4   // FONA reset pin (pin 4 for shield)

// FONA GPRS configuration
#define FONA_APN             "wholesale"  // APN used by cell data service (leave blank if unused)
#define FONA_USERNAME        ""  // Username used by cell data service (leave blank if unused).
#define FONA_PASSWORD        ""  // Password used by cell data service (leave blank if unused).

// Adafruit IO configuration
#define AIO_SERVER           "io.adafruit.com"  // Adafruit IO server name.
#define AIO_SERVERPORT       1883  // Adafruit IO port.
#define AIO_USERNAME         ""  // Adafruit IO username (see http://accounts.adafruit.com/).
#define AIO_KEY              ""  // Adafruit IO key (see settings page at: https://io.adafruit.com/settings).

// Feeds
#define LOCATION_FEED_NAME       "location"  // Name of the AIO feed to log regular location updates.
#define MAX_TX_FAILURES      3  // Maximum number of publish failures in a row before resetting the whole sketch.

// FONA instance & configuration
SoftwareSerial fonaSS = SoftwareSerial(FONA_RX, FONA_TX);     // FONA software serial connection.
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);                 // FONA library connection.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

uint8_t txFailures = 0;             // Count of how many publish failures have occured in a row.

// Feeds configuration
const char LOCATION_FEED[] PROGMEM = AIO_USERNAME "/feeds/" LOCATION_FEED_NAME "/csv";
Adafruit_MQTT_Publish location_feed = Adafruit_MQTT_Publish(&mqtt, LOCATION_FEED);

const char BATTERY_FEED[] PROGMEM = AIO_USERNAME "/feeds/battery";
Adafruit_MQTT_Publish battery_feed = Adafruit_MQTT_Publish(&mqtt, BATTERY_FEED);

const char ALERTS_FEED[] PROGMEM = AIO_USERNAME "/feeds/alerts";
Adafruit_MQTT_Publish alerts_feed = Adafruit_MQTT_Publish(&mqtt, ALERTS_FEED);

char smsText[250];
uint16_t smsLen;
char smsSender[32];
char callerIDbuffer[32];

void setup() {
  
  // Initialize serial output.
  Serial.begin(115200);
  Serial.println(F("Geofencing with Adafruit IO & FONA808"));

  // Set alarm components
  pinMode(ledPin, OUTPUT);

  // Initialize the FONA module
  Serial.println(F("Initializing FONA....(may take 10 seconds)"));
  fonaSS.begin(4800);
  if (!fona.begin(fonaSS)) {
    halt(F("Couldn't find FONA"));
  }
  fonaSS.println("AT+CMEE=2");
  Serial.println(F("FONA is OK"));

  // Use the watchdog to simplify retry logic and make things more robust.
  // Enable this after FONA is intialized because FONA init takes about 8-9 seconds.
  Watchdog.enable(8000);
  Watchdog.reset();

  fonaSS.println("AT+CMGF=1");
  fonaSS.println("AT+CNMI=2,1");

  // Wait for FONA to connect to cell network (up to 8 seconds, then watchdog reset).
  Serial.println(F("Checking for network..."));
  while (fona.getNetworkStatus() != 1) {
   delay(500);
  }

  // Enable GPS.
  fona.enableGPS(true);

  // Start the GPRS data connection.
  Watchdog.reset();
  fona.setGPRSNetworkSettings(F(FONA_APN));
  //fona.setGPRSNetworkSettings(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));
  delay(2000);
  Watchdog.reset();
  Serial.println(F("Disabling GPRS"));
  fona.enableGPRS(false);
  delay(2000);
  Watchdog.reset();
  Serial.println(F("Enabling GPRS"));
  if (!fona.enableGPRS(true)) {
    halt(F("Failed to turn GPRS on, resetting..."));
  }
  Serial.println(F("Connected to Cellular!"));

  // Wait a little bit to stabilize the connection.
  Watchdog.reset();
  delay(3000);

  // Now make the MQTT connection.
  int8_t ret = mqtt.connect();
  if (ret != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    halt(F("MQTT connection failed, resetting..."));
  }
  Serial.println(F("MQTT Connected!"));

  // Initial GPS read
  bool gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  initialLatitude = latitude;
  initialLongitude = longitude;

  // Set alert to 0
  logAlert(0, alerts_feed);

}

void loop() {
  
  // Watchdog reset at start of loop--make sure everything below takes less than 8 seconds in normal operation!
  Watchdog.reset();

  // Reset everything if disconnected or too many transmit failures occured in a row.
  if (!fona.TCPconnected() || (txFailures >= MAX_TX_FAILURES)) {
    halt(F("Connection lost, resetting..."));
  }

  // Grab a GPS reading.
  float latitude, longitude, speed_kph, heading, altitude;
  bool gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  Serial.print("Latitude: ");
  printFloat(latitude, 5);
  Serial.println("");

  Serial.print("Longitude: ");
  printFloat(longitude, 5);
  Serial.println("");

  // Calculate distance between new & old coordinates
  float distance = distanceCoordinates(latitude, longitude, initialLatitude, initialLongitude);
  
  Serial.print("Distance1: ");
  printFloat(distance, 5);
  Serial.println("");

  // Set alarm on?
  //if (distance > maxDistance) {
   // logAlert(1, alerts_feed);
  //}
  
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);

  // Log the current location to the path feed, then reset the counter.
  logLocation(latitude, longitude, altitude, location_feed);
  logBatteryPercent(vbat, battery_feed);

  handleIncomingSMSOnce_WhereAreYou();
  
  // Wait 5 seconds
  delay(5000);

}

// Log alerts
void logAlert(uint32_t alert, Adafruit_MQTT_Publish& publishFeed) {

  // Publish
  Serial.print(F("Publishing alert: "));
  Serial.println(alert);
  if (!publishFeed.publish(alert)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
  
}

// Log battery
void logBatteryPercent(uint32_t indicator, Adafruit_MQTT_Publish& publishFeed) {

  // Publish
  Serial.print(F("Publishing battery percentage: "));
  Serial.println(indicator);
  if (!publishFeed.publish(indicator)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
  
}

// Serialize the lat, long, altitude to a CSV string that can be published to the specified feed.
void logLocation(float latitude, float longitude, float altitude, Adafruit_MQTT_Publish& publishFeed) {
  // Initialize a string buffer to hold the data that will be published.
  char sendBuffer[120];
  memset(sendBuffer, 0, sizeof(sendBuffer));
  int index = 0;

  // Start with '0,' to set the feed value.  The value isn't really used so 0 is used as a placeholder.
  sendBuffer[index++] = '0';
  sendBuffer[index++] = ',';

  // Now set latitude, longitude, altitude separated by commas.
  dtostrf(latitude, 2, 6, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(longitude, 3, 6, &sendBuffer[index]);
  index += strlen(&sendBuffer[index]);
  sendBuffer[index++] = ',';
  dtostrf(altitude, 2, 6, &sendBuffer[index]);

  // Finally publish the string to the feed.
  Serial.print(F("Publishing location: "));
  Serial.println(sendBuffer);
  if (!publishFeed.publish(sendBuffer)) {
    Serial.println(F("Publish failed!"));
    txFailures++;
  }
  else {
    Serial.println(F("Publish succeeded!"));
    txFailures = 0;
  }
}


// Halt function called when an error occurs.  Will print an error and stop execution while
// doing a fast blink of the LED.  If the watchdog is enabled it will reset after 8 seconds.
void halt(const __FlashStringHelper *error) {
  Serial.println(error);
  while (1) {
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
    delay(100);
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void printFloat(float value, int places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

  // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d/= 10.0;    
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }

  // write out the negative if needed
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);

  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
    return;

  // otherwise, write the point and continue on
  Serial.print('.');  

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }
}

// Calculate distance between two points
float distanceCoordinates(float flat1, float flon1, float flat2, float flon2) {

  // Variables
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
}

bool equalsIgnoreCaseTrim(const char* msg, const char* key) {
  while (*msg == ' ' || *msg == '\t' || *msg == '\r' || *msg == '\n') {
    msg++;
  }

  size_t n = strlen(msg);

  while (n && (msg[n - 1] == ' ' || msg[n - 1] == '\t' || msg[n - 1] == '\r' || msg[n - 1] == '\n')) {
    n--;
  }

  if (n != strlen(key)) {
    return false;
  }

  for (size_t i = 0; i < n; i++) {
    char a = tolower((unsigned char)msg[i]);
    char b = tolower((unsigned char)key[i]);
    if (a != b) {
      return false;
    }
  }

  return true;
}

void handleIncomingSMSOnce_WhereAreYou() {
  int8_t numSMS = fona.getNumSMS();

  if (numSMS <= 0) {
    return;
  }

  for (uint8_t i = 1; i <= numSMS; i++) {
    smsText[0] = 0;
    smsLen = 0;
    smsSender[0] = 0;

    if (!fona.readSMS(i, smsText, sizeof(smsText) - 1, &smsLen)) {
      continue;
    }

    if (smsLen == 0) {
      fona.deleteSMS(i);
      continue;
    }

    if (!fona.getSMSSender(i, smsSender, sizeof(smsSender) - 1)) {
      fona.deleteSMS(i);
      continue;
    }

    if (equalsIgnoreCaseTrim(smsText, "where are you")) {
      strncpy(callerIDbuffer, smsSender, sizeof(callerIDbuffer) - 1);
      callerIDbuffer[sizeof(callerIDbuffer) - 1] = '\0';

      delay(6000);

      float latitude;
      float longitude;

      fona.getGPS(&latitude, &longitude);

      Serial.print(latitude, 5);
      Serial.print("\t");
      Serial.print(longitude, 5);
      Serial.print("\t");

      char message[96];
      char LAT[12];
      char LONG[12];

      dtostrf(latitude, 6, 5, LAT);
      dtostrf(longitude, 6, 5, LONG);

      sprintf(message, "https://maps.google.com/maps?q=%s, %s", LAT, LONG);

      Serial.println(message);

      fona.sendSMS(callerIDbuffer, message);

      Watchdog.reset();
      delay(7000);
    }

    fona.deleteSMS(i);

    break;
  }
}
