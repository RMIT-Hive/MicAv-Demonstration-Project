// Author: Ash E
// Purpose: To take over the moon. Just kidding.
// This is a prototype program for a flight computer using a pressure sensor for altitude sensing.
// Most of this program is based on the bmp280test.ino that is provided by adafruit


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void Init_Serial(void);
bool Init_Pressure_Sensor(void);
void Configure_Pressure_Sensor(void);

// state variable - used to keep track of program state
enum states_e {INIT, WAIT_FOR_INPUT, RECORD_DATA}

void setup() {
  // initialise the microcontroller peripherals
  // do not initialise on-board sensors here

  // initialise serial port
  Init_Serial();

  enum states_e state = INIT;
}

void loop() {
  // put your main code here, to run repeatedly:

  switch(state) {
    default: break;
      case INIT:
        // try to initialise sensor
        bool pressureSensorStatus = Init_Pressure_Sensor();

        // if sensor is not found, print error message and break early
        if (!pressureSensorStatus) {
          Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
          Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
          break;
        }
        
        // apply settings to sensor
        Configure_Pressure_Sensor();
        
        // transition to next state
        state = WAIT_FOR_INPUT;

        break;
      case WAIT_FOR_INPUT:
        // do waiting stuff
        break;
      case RECORD_DATA:
        // do recording stuff
        break;
  }
}

// initialise the serial/uart port
// takes: nothing
// returns: nothing
void Init_Serial(void) {
   Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  (void)Serial.println(F("BMP280 test"));
}

// initialise the pressure sensor
// takes: nothing
// returns: error value (false - the sensor failed to initialise)
bool Init_Pressure_Sensor(void) {
  bool status = 0x00;

  status = bmp.begin();

  return status;
}

// apply settings to pressure sensor
// takes: nothing
// returns: nothing
void Configure_Pressure_Sensor(void) {
  // default datasheet settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}