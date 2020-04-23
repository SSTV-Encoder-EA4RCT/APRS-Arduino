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
#include <math.h>
#include <stdio.h>

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

typedef struct {
  bool fix;
  float lat;
  float lon;
  char latd;
  char lond;
} Gps;

// Constants
#define OUT_PIN 12

#define _1200   1
#define _2400   0

#define _FLAG       0x7e
#define _CTRL_ID    0x03
#define _PID        0xf0
#define _DT_EXP     ','
#define _DT_STATUS  '>'
#define _DT_POS     '!'

#define _GPRMC          1
#define _FIXPOS         2
#define _FIXPOS_STATUS  3
#define _STATUS         4
#define _BEACON         5

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

// AFSK Vars
bool nada = _2400;

/*
 * SQUARE WAVE SIGNAL GENERATION
 *
 * baud_adj lets you to adjust or fine tune overall baud rate
 * by simultaneously adjust the 1200 Hz and 2400 Hz tone,
 * so that both tone would scales synchronously.
 * adj_1200 determined the 1200 hz tone adjustment.
 * tc1200 is the half of the 1200 Hz signal periods.
 *
 *      -------------------------                           -------
 *     |                         |                         |
 *     |                         |                         |
 *     |                         |                         |
 * ----                           -------------------------
 *
 *     |<------ tc1200 --------->|<------ tc1200 --------->|
 *
 * adj_2400 determined the 2400 hz tone adjustment.
 * tc2400 is the half of the 2400 Hz signal periods.
 *
 *      ------------              ------------              -------
 *     |            |            |            |            |
 *     |            |            |            |            |
 *     |            |            |            |            |
 * ----              ------------              ------------
 *
 *     |<--tc2400-->|<--tc2400-->|<--tc2400-->|<--tc2400-->|
 *
 */
const float baud_adj = 0.975; //0.975;
const float adj_1200 = 1.0 * baud_adj;
const float adj_2400 = 1.0 * baud_adj;
unsigned int tc1200 = (unsigned int)(0.5 * adj_1200 * 1000000.0 / 1200.0);
unsigned int tc2400 = (unsigned int)(0.5 * adj_2400 * 1000000.0 / 2400.0);

/*
 * This strings will be used to generate AFSK signals, over and over again.
 */
char mycall[8] = "EA4RCT";
char myssid = 11;

char dest[8] = "APZ";     // Experimental see http://aprs.org/aprs11/tocalls.txt
char dest_beacon[8] = "BEACON";

char digi[8] = "WIDE2";
char digissid = 2;

char comment[128] = "RadioClub EIT";
char mystatus[128] = "..::| Experimental Arduino-APRS |::..";

int coord_valid;
const char sym_ovl = 'T';
const char sym_tab = 'a';

unsigned int tx_delay = 5000;
unsigned int str_len = 400;

char bit_stuff = 0;
unsigned short crc=0xffff;

char rmc[100];
char rmc_stat;

// Function prototypes
//void timer2_interrupt(void);
void read_mad(vector<int16_t>* m);
void read_imu(vector<int16_t>* a, vector<int16_t>* g);
void read_bar(Bar* res);
void read_gps(Gps* res);
void print_altimu(void);  // Print values of mad imu and bar
void tone(uint32_t ulPin, uint32_t frequency, int32_t duration);  //Tone using buzzer
void noTone(uint32_t ulPin);    // Tone using Buzzer

void set_nada_1200(void);
void set_nada_2400(void);
void set_nada(bool nada);

void send_char_NRZI(unsigned char in_byte, bool enBitStuff);
void send_string_len(const char *in_string, int len);

void calc_crc(bool in_bit);
void send_crc(void);
void randomize(unsigned int &var, unsigned int low, unsigned int high);

void send_packet(char packet_type);
void send_flag(unsigned char flag_len);
void send_header(char msg_type);
void send_payload(char type);

void print_code_version(void);
void print_debug(char type);

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
  Gps gps;
  read_gps(&gps);

  if(rmc_stat > 10){
    //send_packet(random(1,4), random(1,3));
    if(coord_valid > 0)
      send_packet(_FIXPOS);
    else
      send_packet(_BEACON);
  }

  delay(tx_delay);
  randomize(tx_delay, 5000, 9000);
}

void read_gps(Gps* res){
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
  if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    return; // we can fail to parse a sentence in which case we should just wait for another
  }

  res->fix = GPS.fix;

  if (GPS.fix) {
    res->lat = GPS.latitudeDegrees;
    res->lon = GPS.longitudeDegrees
    res->latd = GPS.lat;
    res->lond = GPS.lon;
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

/*
 *
 */
void set_nada_1200(void){
  digitalWrite(OUT_PIN, true);
  //PORTB |= (1<<PB4);
  delayMicroseconds(tc1200);
  digitalWrite(OUT_PIN, LOW);
  //PORTB &= ~(1<<PB4);
  delayMicroseconds(tc1200);
}

void set_nada_2400(void){
  digitalWrite(OUT_PIN, true);
  //PORTB |= (1<<PB4);
  delayMicroseconds(tc2400);
  digitalWrite(OUT_PIN, LOW);
  //PORTB &= ~(1<<PB4);
  delayMicroseconds(tc2400);

  digitalWrite(OUT_PIN, true);
  //PORTB |= (1<<OUT_PIN);
  delayMicroseconds(tc2400);
  digitalWrite(OUT_PIN, LOW);
  //PORTB &= ~(1<<OUT_PIN);
  delayMicroseconds(tc2400);
}

void set_nada(bool nada)
{
  if(nada)
    set_nada_1200();
  else
    set_nada_2400();
}

/*
 * This function will calculate CRC-16 CCITT for the FCS (Frame Check Sequence)
 * as required for the HDLC frame validity check.
 *
 * Using 0x1021 as polynomial generator. The CRC registers are initialized with
 * 0xFFFF
 */
void calc_crc(bool in_bit)
{
  unsigned short xor_in;

  xor_in = crc ^ in_bit;
  crc >>= 1;

  if(xor_in & 0x01)
    crc ^= 0x8408;
}

void send_crc(void)
{
  unsigned char crc_lo = crc ^ 0xff;
  unsigned char crc_hi = (crc >> 8) ^ 0xff;

  send_char_NRZI(crc_lo, true);
  send_char_NRZI(crc_hi, true);
}

void send_header(char msg_type){
  char temp;

  /*
   * APRS AX.25 Header
   * ........................................................
   * |   DEST   |  SOURCE  |   DIGI   | CTRL FLD |    PID   |
   * --------------------------------------------------------
   * |  7 bytes |  7 bytes |  7 bytes |   0x03   |   0xf0   |
   * --------------------------------------------------------
   *
   * DEST   : 6 byte "callsign" + 1 byte ssid
   * SOURCE : 6 byte your callsign + 1 byte ssid
   * DIGI   : 6 byte "digi callsign" + 1 byte ssid
   *
   * ALL DEST, SOURCE, & DIGI are left shifted 1 bit, ASCII format.
   * DIGI ssid is left shifted 1 bit + 1
   *
   * CTRL FLD is 0x03 and not shifted.
   * PID is 0xf0 and not shifted.
   */

  /********* DEST ***********/
  if(msg_type == _BEACON){
    temp = strlen(dest_beacon);
    for(int j=0; j<temp; j++)
      send_char_NRZI(dest_beacon[j] << 1, true);
  }
  else{
    temp = strlen(dest);
    for(int j=0; j<temp; j++)
      send_char_NRZI(dest[j] << 1, true);
  }
  if(temp < 6){
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, true);
  }
  send_char_NRZI('0' << 1, true);



  /********* SOURCE *********/
  temp = strlen(mycall);
  for(int j=0; j<temp; j++)
    send_char_NRZI(mycall[j] << 1, true);
  if(temp < 6){
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, true);
  }
  send_char_NRZI((myssid + '0') << 1, true);


  /********* DIGI ***********/
  temp = strlen(digi);
  for(int j=0; j<temp; j++)
    send_char_NRZI(digi[j] << 1, true);
  if(temp < 6){
    for(int j=0; j<(6 - temp); j++)
      send_char_NRZI(' ' << 1, true);
  }
  send_char_NRZI(((digissid + '0') << 1) + 1, true);

  /***** CTRL FLD & PID *****/
  send_char_NRZI(_CTRL_ID, true);
  send_char_NRZI(_PID, true);
}

void send_payload(char type){
  /*
   * APRS AX.25 Payloads
   *
   * TYPE : POSITION
   * ........................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|
   * --------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |
   * --------------------------------------------------------
   *
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   *
   *
   * TYPE : STATUS
   * ..................................
   * |DATA TYPE |    STATUS TEXT      |
   * ----------------------------------
   * |  1 byte  |       N bytes       |
   * ----------------------------------
   *
   * DATA TYPE  : >
   * STATUS TEXT: Free form text
   *
   *
   * TYPE : POSITION & STATUS
   * ..............................................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|    STATUS TEXT      |
   * ------------------------------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |       N bytes       |
   * ------------------------------------------------------------------------------
   *
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * STATUS TEXT: Free form text
   *
   *
   * All of the data are sent in the form of ASCII Text, not shifted.
   *
   */
  if(type == _GPRMC){
    send_char_NRZI('$', true);
    send_string_len(rmc, strlen(rmc)-1);
  }
  else if(type == _FIXPOS){
    send_char_NRZI(_DT_POS, true);
    send_string_len(lati, strlen(lati));
    send_char_NRZI(sym_ovl, true);
    send_string_len(lon, strlen(lon));
    send_char_NRZI(sym_tab, true);
  }
  else if(type == _STATUS){
    send_char_NRZI(_DT_STATUS, true);
    send_string_len(mystatus, strlen(mystatus));
  }
  else if(type == _FIXPOS_STATUS){
    send_char_NRZI(_DT_POS, true);
    send_string_len(lati, strlen(lati));
    send_char_NRZI(sym_ovl, true);
    send_string_len(lon, strlen(lon));
    send_char_NRZI(sym_tab, true);

    send_string_len(comment, strlen(comment));
  }
  else{
    send_string_len(mystatus, strlen(mystatus));
  }
}

/*
 * This function will send one byte input and convert it
 * into AFSK signal one bit at a time LSB first.
 *
 * The encode which used is NRZI (Non Return to Zero, Inverted)
 * bit 1 : transmitted as no change in tone
 * bit 0 : transmitted as change in tone
 */
void send_char_NRZI(unsigned char in_byte, bool enBitStuff){
  bool bits;

  for(int i = 0; i < 8; i++)
  {
    bits = in_byte & 0x01;

    calc_crc(bits);

    if(bits)
    {
      set_nada(nada);
      bit_stuff++;

      if((enBitStuff) && (bit_stuff == 5))
      {
        nada ^= 1;
        set_nada(nada);

        bit_stuff = 0;
      }
    }
    else
    {
      nada ^= 1;
      set_nada(nada);

      bit_stuff = 0;
    }

    in_byte >>= 1;
  }
}

void send_string_len(const char *in_string, int len){
  for(int j=0; j<len; j++)
    send_char_NRZI(in_string[j], true);
}

void send_flag(unsigned char flag_len){
  for(int j=0; j<flag_len; j++)
    send_char_NRZI(_FLAG, LOW);
}

/*
 * In this preliminary test, a packet is consists of FLAG(s) and PAYLOAD(s).
 * Standard APRS FLAG is 0x7e character sent over and over again as a packet
 * delimiter. In this example, 100 flags is used the preamble and 3 flags as
 * the postamble.
 */
void send_packet(char packet_type){
  print_debug(packet_type);

  digitalWrite(LED_BUILTIN, HIGH);

  //delay(100);

  /*
   * AX25 FRAME
   *
   * ........................................................
   * |  FLAG(s) |  HEADER  | PAYLOAD  | FCS(CRC) |  FLAG(s) |
   * --------------------------------------------------------
   * |  N bytes | 22 bytes |  N bytes | 2 bytes  |  N bytes |
   * --------------------------------------------------------
   *
   * FLAG(s)  : 0x7e
   * HEADER   : see header
   * PAYLOAD  : 1 byte data type + N byte info
   * FCS      : 2 bytes calculated from HEADER + PAYLOAD
   */

  send_flag(150);
  crc = 0xffff;
  send_header(packet_type);
  send_payload(packet_type);
  send_crc();
  send_flag(3);

  digitalWrite(_PTT, LOW);
  digitalWrite(LED_BUILTIN, 0);
}

/*
 * Function to randomized the value of a variable with defined low and hi limit value.
 * Used to create random AFSK pulse length.
 */
void randomize(unsigned int &var, unsigned int low, unsigned int high){
  randomSeed(analogRead(A0));
  var = random(low, high);
}

void print_code_version(void){
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println(" ");

  Serial.println("GPRMC APRS Transmitter - Started ! \n");
}

void print_debug(char type){
  /*
   * PROTOCOL DEBUG.
   *
   * Will outputs the transmitted data to the serial monitor
   * in the form of TNC2 string format.
   *
   * MYCALL-N>APRS,DIGIn-N:<PAYLOAD STRING> <CR><LF>
   */
  Serial.begin(115200);

  /****** MYCALL ********/
  Serial.print(mycall);
  Serial.print('-');
  Serial.print(myssid, DEC);
  Serial.print('>');

  /******** DEST ********/
  if(type == _BEACON)
    Serial.print(dest_beacon);
  else
    Serial.print(dest);
  Serial.print(',');

  /******** DIGI ********/
  Serial.print(digi);
  Serial.print('-');
  Serial.print(digissid, DEC);
  Serial.print(':');

  /******* PAYLOAD ******/
  if(type == _GPRMC)
  {
    Serial.print('$');
    Serial.print(rmc);
  }
  else if(type == _FIXPOS)
  {
    Serial.print(_DT_POS);
    Serial.print(lati);
    Serial.print(sym_ovl);
    Serial.print(lon);
    Serial.print(sym_tab);
  }
  else if(type == _STATUS)
  {
    Serial.print(_DT_STATUS);
    Serial.print(mystatus);
  }
  else if(type == _FIXPOS_STATUS)
  {
    Serial.print(_DT_POS);
    Serial.print(lati);
    Serial.print(sym_ovl);
    Serial.print(lon);
    Serial.print(sym_tab);

    Serial.print(comment);
  }
  else
  {
    Serial.print(mystatus);
  }

  Serial.println(' ');

  Serial.flush();
  Serial.end();
}
