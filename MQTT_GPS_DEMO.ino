
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <Adafruit_SleepyDog.h>

//SoftwareSerial serial_connection(8,9); for GSM and GPS module
#define FONA_RX  9 //for gsm
#define FONA_TX  8 //for gsm
#define FONA_RST 4 //for gsm
#define GPS_RX  7 //gps
#define GPS_TX 6 //gps

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial serial_connection = SoftwareSerial(GPS_TX, GPS_RX);
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
TinyGPSPlus gps;// GPS object to process the NMEA data

/**************Access Point Settings *********/
#define FONA_APN       "internet"
#define FONA_USERNAME  ""
#define FONA_PASSWORD  ""

/************MQTT SETTINGS********************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "oscaronyango10"
#define AIO_KEY         "aio_osdu518npGsRqOIbBZA5yfMvWjvX"

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

Adafruit_MQTT_Publish gpsloc = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gps");  //topic for publishing gps data

uint8_t txfailures = 0; //maximum number of failures allowed
#define MAXTXFAILURES 3

char sendbuffer[30]; //buffer created for merging data
char *p = sendbuffer;

void setup()
{
  while (!Serial);

  // Watchdog is optional!
  //Watchdog.enable(8000);

  Serial.begin(115200);
  serial_connection.begin(9600);
  Serial.println(F("Adafruit FONA MQTT demo"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  // Initialise the FONA module
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
}


void loop()
{
  float latitude, longitude;
  while (serial_connection.available())             //While there are incoming characters  from the GPS
  {
    gps.encode(serial_connection.read());           //This feeds the serial NMEA data into the library one char at a time
  }
  if (gps.location.isUpdated())         //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    Serial.print("Satellite Count:");
    Serial.println(gps.satellites.value());
    Serial.print("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude:");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude Feet:");
    Serial.println(gps.altitude.feet());
    Serial.println("");
    latitude = (gps.location.lat());
    longitude = (gps.location.lng());

    Serial.println("");
    delay(2000);
  }
  Watchdog.reset();

  char *p = sendbuffer;
  // concat latitude
  dtostrf(latitude, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;
  // concat longitude
  dtostrf(longitude, 3, 6, p);
  p += strlen(p);
  p[0] = ','; p++;
  // null terminate
  p[0] = 0;

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  /**********here we send values to cloud ***********/
  Serial.print(F("\nSending values to Adafruit IO GSM feed "));
  Serial.print("...");
  if (! gpsloc.publish(sendbuffer)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
    delay(5000);
  }
  Watchdog.reset();
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
