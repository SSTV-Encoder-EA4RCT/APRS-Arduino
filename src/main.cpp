/**
 * @author: Fran Acién and David Arias, with the help of Pablo Alvarez and Dani Kholer
 *  APRS emitter using arduino DUE
 *
**/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <DueTimer.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <LPS.h>

// Structures

typedef struct {
  float pressure;
  float altitude;
  float temperature;
} Bar;

// Imu template
template <typename T> struct vector{
  T x, y, z;
};

// Constants

#define buzzPin 2

#define GPSSerial Serial3

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();

// Magnetometer LIS3MDL
LIS3MDL mag;
vector<int16_t> m; // magnetometer readings

// Accelerometer LSM6DS33
LSM6 imu;
vector<int16_t> a; // accelerometer readings
vector<int16_t> g; // gyro readings

// Barometer LPS25H
LPS ps;
Bar bar;

// Function prototypes
void timer2_interrupt(void);
void read_mad(vector<int16_t>* m);
void read_imu(vector<int16_t>* a, vector<int16_t>* g);
void read_bar(Bar* res);
void print_altimu(void);  // Print values of mad imu and bar
void tone(uint32_t ulPin, uint32_t frequency, int32_t duration);  //Tone using buzzer
void noTone(uint32_t ulPin);    // Tone using Buzzer

void setup() {
  Serial.begin(9600);
  Wire.begin();   // i2C begin
  Serial.println("Adafruit GPS library basic test!");

  // Start magnetometer
  if (!mag.init()){
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();

  // Start accelerometer
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  // Start barometer
  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }

  ps.enableDefault();

  // Start GPS
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // Setup Timer with the emision interval
  //Timer2.attachInterrupt(timer2_interrupt).start(1000); // Read GPS every millisecond *****
}

void loop() {

}

void timer2_interrupt(){
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
  if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}

void read_mad(vector<int16_t>* m){
  mag.read();
  memcpy(m, &mag.m, sizeof(vector<int16_t>));
}

void read_imu(vector<int16_t>* a, vector<int16_t>* g){
  imu.read();
  memcpy(a, &imu.a, sizeof(vector<int16_t>));
  memcpy(g, &imu.g, sizeof(vector<int16_t>));
}

void read_bar(Bar* res){
  res->pressure = ps.readPressureMillibars();
  res->altitude = ps.pressureToAltitudeMeters(res->pressure);
  res->temperature = ps.readTemperatureC();
}

void print_altimu(void){
  char buff[80];
  delay(100);

  read_imu(&a, &g);
  read_mad(&m);
  read_bar(&bar);

  Serial.println("IMU Values");
  sprintf(buff, "A: %6d %6d %6d    G: %6d %6d %6d",
          a.x, a.y, a.z, g.x, g.y, g.z);
  Serial.println(buff);
  Serial.println("--------------");

  Serial.println("MAG Values");
  sprintf(buff, "M: %6d %6d %6d",
          m.x, m.y, m.z);
  Serial.println(buff);
  Serial.println("----------------");

  Serial.println("BAR values");
  sprintf(buff, "Pressure %f mbar     Altitude: %f m     Temperature: %f deg C",
          bar.pressure, bar.altitude, bar.temperature);
  Serial.println(buff);
  Serial.println("----------------");
}