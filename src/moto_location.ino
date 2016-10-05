/*
  Moto Location.
  This sketch use FONA 808 and sparkfun SAMD21 mini to locate through GPS my motorcicle
  It use io.adafruit.com platform(MQTT)
  
  Ricardo Mena C.
  ricardo@crcibernetica.com
  http://crcibernetica.com

  License
  **********************************************************************************
  This program is free software; you can redistribute it 
  and/or modify it under the terms of the GNU General    
  Public License as published by the Free Software       
  Foundation; either version 3 of the License, or        
  (at your option) any later version.                    
                                                        
  This program is distributed in the hope that it will   
  be useful, but WITHOUT ANY WARRANTY; without even the  
  implied warranty of MERCHANTABILITY or FITNESS FOR A   
  PARTICULAR PURPOSE. See the GNU General Public        
  License for more details.                              
                                                        
  You should have received a copy of the GNU General    
  Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>.
                                                        
  Licence can be viewed at                               
  http://www.gnu.org/licenses/gpl-3.0.txt

  Please maintain this license information along with authorship
  and copyright notices in any redistribution of this code
  **********************************************************************************
  */

/*
Conecctions
  SAMD21 |
 Pinout  |  FONA
------------------
  0          TX
  1          RX
  2          Rst
  3          Key
  4          PStat
  5          Vio
  Gnd        Gnd
  RAW        VBat
------------------
         | ADXL335
  A3          x
  Vcc         +
  GND         G
  
*/

#include <Adafruit_SleepyDog.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

#undef min
#undef max

#define serial SerialUSB
#define fonaSerial Serial1
//#define _dbg  debug

//---------FONA network-------------
#define FONA_RX 1
#define FONA_TX 0
#define FONA_RST 2
#define FONA_KEY 3
#define FONA_PS 4
#define FONA_VIO 5
//-----------------------------------

//-------Motion detection------------
#define ADXL_Y A2
#define ADXL_X A3
//-----------------------------------

#define SERIAL_BAUD   115200
#define DEBUG //uncoment for debuging

//-----------------FONA things--------------------
// this is a large buffer for replies
char replybuffer[255];

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

// Size of the geo fence (in meters)
const float maxDistance = 100;

//Last Fona Battery percentage
uint16_t last_fona_battery_state;

// Latitude & longitude for distance measurement
float initialLatitude, initialLongitude, latitude, longitude, speed_kph, heading, altitude, distance;

// Adafruit IO configuration
#define AIO_SERVER           "io.adafruit.com"  // Adafruit IO server name.
#define AIO_SERVERPORT       1883  // Adafruit IO port.
#define AIO_USERNAME         "USERNAME"  // Adafruit IO username (see http://accounts.adafruit.com/).
#define AIO_KEY              "AIO_KEY"  // Adafruit IO key (see settings page at: https://io.adafruit.com/settings).

// Feeds
#define LOCATION_FEED_NAME       "location"  // Name of the AIO feed to log regular location updates.
#define MAX_TX_FAILURES      4  // Maximum number of publish failures in a row before resetting the whole sketch.

const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

uint8_t txFailures = 0;                                       // Count of how many publish failures have occured in a row.

// Feeds configuration
const char LOCATION_FEED[] PROGMEM = AIO_USERNAME "/feeds/" LOCATION_FEED_NAME "/csv";
Adafruit_MQTT_Publish location_feed = Adafruit_MQTT_Publish(&mqtt, LOCATION_FEED);

const char BATTERY_FEED[] PROGMEM = AIO_USERNAME "/feeds/battery";
Adafruit_MQTT_Publish battery_feed = Adafruit_MQTT_Publish(&mqtt, BATTERY_FEED);

const char ALERTS_FEED[] PROGMEM = AIO_USERNAME "/feeds/alerts";
Adafruit_MQTT_Publish alerts_feed = Adafruit_MQTT_Publish(&mqtt, ALERTS_FEED);

const char ADXL_FEED[] PROGMEM = AIO_USERNAME "/feeds/adxl";
Adafruit_MQTT_Publish adxl_feed = Adafruit_MQTT_Publish(&mqtt, ADXL_FEED);


//------------------------------------------------

float deviation = 0;

uint32_t previousMillis_1 = 0;//Update connection time
uint32_t previousMillis_2 = 0;//Update adafruit feeds time

//======================Default Functions=============================

void setup() {
  pinMode(FONA_KEY, OUTPUT);
  pinMode(FONA_PS, INPUT);
  pinMode(FONA_VIO, OUTPUT);
  digitalWrite(FONA_VIO, HIGH);
  #if defined(DEBUG)
    serial.begin(SERIAL_BAUD);
  #endif
  delay(5000);
  
  init_fona();
  print_IMEI(); 
  
}//end setup

void loop(){
  float temp_deviant = standard_deviation();

  if(temp_deviant > deviation){//update deviation
    deviation = temp_deviant;
  }//end if
  
  temporizer_1(millis(), 3000);//Check connections

  temporizer_2(millis(), 15000);//Update Adafruit feeds
  
  #if defined(DEBUG)
  serial.flush();
  #endif
}//end loop

//====================================================================

//=======================Fona Related Functions=======================

void init_fona(){
  fona_off();
  delay(2000);
  fona_on();
  delay(2000);
  fonaSerial.begin(4800);
  if(!check_fona()){// See if the FONA is responding
    serial.println("FONA PROBLEMS");
    halt(F("Couldn't find FONA"));
  }else{
    serial.println(F("FONA is ok"));
  }
  
  //APN configuration
  //fona.setGPRSNetworkSettings(F("internet.movistar.cr"), F("movistarcr"), F("movistarcr"));
  #if defined(DEBUG)
  serial.println(F("Waiting 20s.."));
  #endif
  
  delay(20000);//Wait for FONA

  gprs_disable();
  gprs_enable(0);
  fona_gps_on();
  gpFIX();

  // Now make the MQTT connection.
  int8_t ret = mqtt.connect();
  if (ret != 0) {
    #if defined(DEBUG)
    serial.println(mqtt.connectErrorString(ret));
    #endif
    //init_fona();
    halt(F("FONA has some errors to initiate"));
  }else{
    #if defined(DEBUG)
    serial.println(F("MQTT Connected!"));
    #endif
  }//end if
}//end init_fona

uint8_t check_fona(){
  // See if the FONA is responding
  if (!fona.begin(fonaSerial)) {
    return 0;
  }//end if
  return 1;
}//end check_fona

void fona_on(){
  #if defined(DEBUG)
    serial.println("Turning on Fona: ");
  #endif
  while(digitalRead(FONA_PS)==LOW){
    digitalWrite(FONA_KEY, LOW);
    delay(3000);
    break;    
  }//end while
  digitalWrite(FONA_KEY, HIGH);
  //delay(2000); 
}//end fona_on

void fona_off(){
  #if defined(DEBUG)
    serial.println("Turning off Fona: ");
  #endif
  while(digitalRead(FONA_PS)==HIGH){
    digitalWrite(FONA_KEY, LOW);
    delay(3000);
    break;
  }//end while
  digitalWrite(FONA_KEY, HIGH);
}//end fona_off

int gprs_enable(int maxtry){
  // turn GPRS on
  if (!fona.enableGPRS(true)){
    #if defined(DEBUG)
      serial.print(F("Failed to turn on GPRS = "));
      serial.println(maxtry);
    #endif
    if(maxtry > 200){
      halt(F("GPRS Failed, shutdown"));
    }
    maxtry +=1;
    gprs_enable(maxtry);
  }else{
    #if defined(DEBUG)
      serial.println(F("GPRS ON"));
    #endif
  }//end if
}//end gprs_enable

int gprs_disable(){
  // turn GPRS off
  if (!fona.enableGPRS(false)){
    #if defined(DEBUG)
      serial.println(F("Failed to turn GPRS off"));
    #endif
    return 0;
  }else{
    #if defined(DEBUG)
      serial.println(F("GPRS OFF"));
    #endif
    return 1;
  }//end if
}//end gprs_disable

int fona_gps_on(void){                    //turn GPS on
  if (!fona.enableGPS(true)){
    return 0;                             //Failed to turn GPS on
  }//end if
  delay(2500);
  return 1;
}//end fona_gps_on

int fona_gps_off(void){                    // turn GPS off
  delay(2500);
  if (!fona.enableGPS(false)){
    return 0;                              //Failed to turn GPS off
  }//end if
  return 1;
}//end fona_gps_off

uint8_t fona_gps_location(){
  // check for GPS location, Latitude & longitude for distance measurement
  uint8_t gps_fix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  distance = distanceCoordinates(latitude, longitude, initialLatitude, initialLongitude);
  return gps_fix;
}//end fona_gps_location

uint16_t fona_get_battery(void){
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end fona_get_battery

//====================================================================

//======================MQTT Realated Functions=======================

void MQTT_connect() {
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.  
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()){
    return;
  }
  #if defined(DEBUG)
  serial.print("Connecting to MQTT... ");
  #endif

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    #if defined(DEBUG)
    serial.println(mqtt.connectErrorString(ret));
    serial.println("Retrying MQTT connection in 5 seconds...");
    #endif
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  #if defined(DEBUG)
  serial.println("MQTT Connected!");
  #endif
}//end MQTT_connect

void log_fix(uint32_t fix, Adafruit_MQTT_Publish& publishFeed){
  if (!publishFeed.publish(fix)) {
    #if defined(DEBUG)
    serial.println(F("Publish failed!"));
    #endif
    txFailures++;
  }else {
    #if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
    #endif
    txFailures = 0;
  }//end if  
}//log fix

void log_battery_percent(uint32_t indicator, Adafruit_MQTT_Publish& publishFeed) {// Log battery
  #if defined(DEBUG)
  serial.print(F("Publishing battery percentage: "));
  serial.println(indicator);
  #endif
  if (!publishFeed.publish(indicator)) {
    #if defined(DEBUG)
    serial.println(F("Publish failed!"));
    #endif
    txFailures++;
  }else {
    #if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
    #endif
    txFailures = 0;
  }//end if
}//end log_battery_percent
// Serialize the lat, long, altitude to a CSV string that can be published to the specified feed.
void log_location(float latitude, float longitude, float altitude, Adafruit_MQTT_Publish& publishFeed) {
  // Initialize a string buffer to hold the data that will be published.
  char sendBuffer[500];

  sprintf(sendBuffer, "0,%s,%s,%s", float_to_string(latitude, 6).c_str(), float_to_string(longitude, 6).c_str(), float_to_string(altitude, 6).c_str());

  // Publish the string to the feed.
  #if defined(DEBUG)
  serial.print(F("Publishing location: "));
  serial.println(sendBuffer);
  #endif
  if (!publishFeed.publish(sendBuffer)) {
    #if defined(DEBUG)
    serial.println(F("Publish failed!"));
    #endif
    txFailures++;
  }else {
    #if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
    #endif
    txFailures = 0;
  }//end if
}//end log_location

void log_motion_detection(uint32_t detection, Adafruit_MQTT_Publish& publishFeed){
  if (!publishFeed.publish(detection)) {
    #if defined(DEBUG)
    serial.println(F("Publish failed!"));
    #endif
    txFailures++;
  }else {
    #if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
    #endif
    txFailures = 0;
  }//end if  
}//end log_motion_detection

//====================================================================

//======================Miscelanius Funcions==========================

void print_IMEI(void){
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
  #if defined(DEBUG)
    #if defined(DEBUG)
    serial.print("SIM card IMEI: "); serial.println(imei);
    #endif
  #endif
  }//end if  
}//end print_IMEI

void temporizer_1(uint32_t timer, uint32_t interval){//Check connections state
  if(timer - previousMillis_1 > interval) {
    if((!fona.GPRSstate())||(fona.getNetworkStatus() != 1)){
      //halt(F("Network connection problems, resetting..."));
      init_fona();
    }else if (!fona.TCPconnected() || (txFailures >= MAX_TX_FAILURES)) {
      //halt(F("MQTT connection failed, resetting..."));
      init_fona();
    }//end if
    previousMillis_1 = timer;
  }//end if
}//end temporizer_1

void temporizer_2(uint32_t timer, uint32_t interval){
  if(timer - previousMillis_2 > interval) {
    if(fona_gps_location()){
      log_location(latitude, longitude, altitude, location_feed);
    }//end if         
    uint16_t fona_battery = fona_get_battery();
    if(fona_battery != last_fona_battery_state){// 
      last_fona_battery_state = fona_battery;
      log_battery_percent(fona_battery, battery_feed);
    }//end if

    if(deviation > 2){//Ignore small deviations
      log_motion_detection(deviation, adxl_feed);
    }//end if
    
    previousMillis_2 = timer;
  }//end if
}//end temporizer

void gpFIX(){
  // Initial GPS read
  bool gpsFix = false;
  if(fona.GPSstatus() >= 2){
    gpsFix = fona_gps_location();
    initialLatitude = latitude;
    initialLongitude = longitude;
    delay(100);
    if(txFailures > 200){
      halt(F("GPS not fix"));
    }//end if
    txFailures++;
    delay(10);
  }//end if
  txFailures = 0; 
}//end gpFIX

float standard_deviation(){//Calculate standard deviation
  uint8_t n = 690;
  float media[n], average, devian;  
  for(uint8_t i = 0; i < n; i++){
    media[i] = analogRead(ADXL_X);
  }//end for
  for(uint8_t i = 0; i < n; i++){
    average += media[i];
  }//end for
  average /=n;
  for(uint8_t i = 0; i < n; i++){
    devian = (pow(media[i]-average,2)/(n-1));
  }//end for
   return sqrt(devian);
}//end motion alert

String float_to_string(float value, uint8_t places) {//Adafruit funtion
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  //int i;
  float tempfloat = value;
  String float_obj = "";

    // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0){
    d *= -1.0;
  }
  // divide by ten for each decimal place
  for (uint8_t i = 0; i < places; i++){
    d/= 10.0;
  }
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0){
    tempfloat *= -1.0;
  }
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }
  // write out the negative if needed
  if (value < 0){
    float_obj += "-";
  }//en if
  
  if (tenscount == 0){
    float_obj += String(0, DEC);
  }//en if
  
  for (uint8_t i = 0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    float_obj += String(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }//en for

  // if no places after decimal, stop now and return
  if (places <= 0){
    return float_obj;
  }//end if

  // otherwise, write the point and continue on
  float_obj += ".";

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (uint8_t i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    float_obj += String(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }//end for
  return float_obj;
}//end float_to_string

void flushSerial() {
  #if defined(DEBUG)
  while (serial.available()) 
    serial.read();
  #endif
}//end flushSerial

void halt(const __FlashStringHelper *error) {
  serial.println(error);
  delay(1000);
  Watchdog.enable(1000);
  Watchdog.reset();
  while (1) {}
}//end halt
//====================================================================
