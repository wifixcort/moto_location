/*
  Moto Location.
  This sketch use FONA 808 and sparkfun SAMD21 mini to locate through GPS my motorcicle
  It use io.adafruit.com platform(MQTT)
  
  Ricardo Mena C
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
  SAMD21     FONA
  0          TX
  1          RX
  2          Rst
  3          Key
  4          PStat
  5          Vio
  Gnd        Gnd
  RAW        VBat
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

// Initial GPS read
//uint8_t gps_fix = 0;

// Latitude & longitude for distance measurement
float initialLatitude, initialLongitude, latitude, longitude, speed_kph, heading, altitude, distance;
float gsm_latitude, gsm_longitude;

//boolean lock_initial_gps = false;
//uint8_t gps_fix_failures = 0;

// FONA GPRS configuration
#define FONA_APN             ""  // APN used by cell data service (leave blank if unused)
#define FONA_USERNAME        ""  // Username used by cell data service (leave blank if unused).
#define FONA_PASSWORD        ""  // Password used by cell data service (leave blank if unused).

// Adafruit IO configuration
#define AIO_SERVER           "io.adafruit.com"  // Adafruit IO server name.
#define AIO_SERVERPORT       1883  // Adafruit IO port.
#define AIO_USERNAME         "username"  // Adafruit IO username (see http://accounts.adafruit.com/).
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

const char STATUS_FEED[] PROGMEM = AIO_USERNAME "/feeds/status";
Adafruit_MQTT_Publish status_feed = Adafruit_MQTT_Publish(&mqtt, STATUS_FEED);

//------------------------------------------------

#if defined(TEST)
  long interval = 1000;
  unsigned long start =0;
#endif

void init_fona(){
  fona_off();
  delay(2000);
  fona_on();
  delay(2000);
  fonaSerial.begin(4800);
  if(!check_fona()){// See if the FONA is responding
    halt(F("Couldn't find FONA"));
  }//end if
  
  //APN configuration
  //fona.setGPRSNetworkSettings(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));
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
  
  log_alert(0, alerts_feed);
}//end setup

void loop(){
/*  if(!lock_initial_gps){
    gpFIX();
    gps_ready(stat, status_feed);
  }*/
  #if defined(FREERAM)
    serial.print("Free 2 = ");
    serial.println(freeRam());
  #endif
  if((!fona.GPRSstate())||(fona.getNetworkStatus() != 1)){
    //halt(F("Network connection problems, resetting..."));
    init_fona();
  }else if (!fona.TCPconnected() || (txFailures >= MAX_TX_FAILURES)) {
    //halt(F("MQTT connection failed, resetting..."));
    init_fona();
  }//end if

  if(fona_gps_location()){
    log_location(latitude, longitude, altitude, location_feed);
    log_fix(1, status_feed);
    if (distance > maxDistance) { // Set alarm on?
      log_alert(1, alerts_feed);
    }//end if
  }//end if
  /*else if(fona_gsm_location()){
      log_location(gsm_latitude, gsm_longitude, 0, location_feed);
      log_fix(0, status_feed);
  }*/
  uint16_t fona_battery = fona_get_battery();
  if(fona_battery != last_fona_battery_state){// 
    last_fona_battery_state = fona_battery;
    log_battery_percent(fona_battery, battery_feed);
  }//end if
  
  //log_fix(gp_fix, status_feed);
  
  delay(15000);
  #if defined(DEBUG)
  serial.flush();
  #endif
}//end loop

void gpFIX(){
  // Initial GPS read
  bool gpsFix = false;
  if(fona.GPSstatus() >= 2){//}
  //do{
  //while(!gpsFix){
    //gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    gpsFix = fona_gps_location();
    initialLatitude = latitude;
    initialLongitude = longitude;
    delay(100);
    if(txFailures > 200){
      halt(F("GPS not fix"));
    }//end if
    txFailures++;
    delay(10);
//    return gpsFix;
  //}while(!gpsFix);
  }//end if
  txFailures = 0; 
//  return 0; 
}//end gpFIX

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

void flushSerial() {
  #if defined(DEBUG)
  while (serial.available()) 
    serial.read();
  #endif
}//end flushSerial

uint8_t check_fona(){
  // See if the FONA is responding
  if (!fona.begin(fonaSerial)) {
    #if defined(DEBUG)
      serial.println(F("Couldn't find FONA"));
    #endif
    return 0;
  }//end if
  #if defined(DEBUG)
    serial.println(F("FONA is OK"));
  #endif
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
  //delay(4000);
}//end fona_off

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

uint8_t fona_gsm_location(){
  if (fona.getNetworkStatus() == 1) {
    // network & GPRS? Great! Print out the GSM location to compare
    uint8_t gsm_fix = fona.getGSMLoc(&gsm_latitude, &gsm_longitude);
    return gsm_fix;
  }else{
    #if defined(DEBUG)
      serial.println("Network problems");
    #endif
  }//end if
  return 0;
}

uint16_t fona_get_battery(void){
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end fona_get_battery

void log_alert(uint32_t alert, Adafruit_MQTT_Publish& publishFeed) {// Log alerts
  #if defined(DEBUG)
  serial.print(F("Publishing alert: "));
  serial.println(alert);
  #endif
  if (!publishFeed.publish(alert)) {
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
}//end log_alert

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
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
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

// Calculate distance between two points
//Adafruit funtion
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

void gps_ready(char *statusF, Adafruit_MQTT_Publish& publishFeed){//uint32_t statusF
  // Finally publish the string to the feed.
  #if defined(DEBUG)
  serial.print(F("Publishing GPS ready: "));
  serial.println(statusF);
  #endif
  if (!publishFeed.publish(statusF)) {
    #if defined(DEBUG)
    serial.println(F("StatusF Publish failed!"));
    #endif
    txFailures++;
  }
  else {
    #if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
    #endif
    txFailures = 0;
  }//end if
}//end gps_ready

void halt(const __FlashStringHelper *error) {
  serial.println(error);
  delay(1000);
  Watchdog.enable(1000);
   Watchdog.reset();
  while (1) {}
}//end halt

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
