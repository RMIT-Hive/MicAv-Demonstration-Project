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

#define SEA_LEVEL_hPa (float)1013.25 // adjust to local forecast

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void Init_Serial(void);
bool Init_Pressure_Sensor(void);
void Configure_Pressure_Sensor(void);
void Print_Pressure_Sensor_Data(float pressure, float temperature, float altitude);
void Print_Sensor_Error(void);
void Print_Sensor_Conformation(void);

// state variable - used to keep track of program state
enum states_e {INIT = 'A', WAIT_FOR_INPUT = 'B', RECORD_DATA = 'C'};
enum states_e state = INIT;

void setup() {
  // initialise the microcontroller peripherals
  // do not initialise on-board sensors here

  // initialise serial port
  Init_Serial();

  // initialise LED (debugging purposes)
  pinMode(LED_BUILTIN, OUTPUT);

  // debug message
  Serial.println("Setup complete. Entering main loop.");
}

void loop() {
  // wait (approximately) 2 seconds between each loop  
  delay(2000);
  Serial.print("Loop. State = ");
  Serial.println((char)state);

  // state machine
  switch(state) {
    case INIT:
        // sensor status
        bool sensorInitStatus = false;

        // try to initialise sensor
        sensorInitStatus = Init_Pressure_Sensor();

        // if sensor is not found, print error message and break early
        if (!sensorInitStatus) {
          Print_Sensor_Error();

          break;
        }
        else {
          Print_Sensor_Conformation();
        }
        
        // apply settings to sensor
        Configure_Pressure_Sensor();
        
        // transition to next state
        state = WAIT_FOR_INPUT;

        break;
    case WAIT_FOR_INPUT:
      // do waiting stuff

      // transition to next state
      state = RECORD_DATA;
      
      break;
    case RECORD_DATA:
      // data
      float pressure = 0;
      float temperature = 0;
      float altitude = 0;
      // sensor status
      bool sensorDataStatus = false;

      // take readings
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure();
      altitude = bmp.readAltitude(SEA_LEVEL_hPa);

      // do some range checking here based on sensor specs here
      // right now we are just assuming theres no issues
      sensorDataStatus = true;

      // if there were an issue with the sensor:
      // print error message and return to INIT state
      // exit early
      if (!sensorDataStatus) {
        Print_Sensor_Error();
        state = INIT;

        break;
      }

      // do something with data

      // print data to serial port
      Print_Pressure_Sensor_Data(pressure, temperature, altitude);

      break;
    default:
      state = INIT;

      break;
  }
}

// initialise the serial/uart port
// takes: nothing
// returns: nothing
void Init_Serial(void) {
   Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  (void)Serial.println();
  (void)Serial.println(F("Serial port active"));

  return;
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
  
  return;
}

// print error message about pressure sensor
// takes: nothing
// returns: nothign
void Print_Sensor_Error(void) {
  (void)Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
              "try a different address!"));
  (void)Serial.print("SensorID was: 0x");
  (void)Serial.println(bmp.sensorID(),16);

  return;
}

// print error (positive) message about pressure sensor
// takes: nothing
// returns: nothing
void Print_Sensor_Conformation(void) {
  (void)Serial.print("Found valid BMP280 sensor with SensorID: 0x");
  (void)Serial.println(bmp.sensorID(),16);

  return;
}

// print pressure sensor data
// PRESSURE (Pascals) | TEMPERATURE (degrees C) | ALTITUDE (m)
// takes: pressure, temp, altitude data (as float)
// returns: nothing
void Print_Pressure_Sensor_Data(float pressure, float temperature, float altitude) {
  (void)Serial.print(pressure);
  (void)Serial.print(" Pa |");
  (void)Serial.print(temperature);
  (void)Serial.print(" *C |");
  (void)Serial.print(altitude);
  (void)Serial.print(" m");

  return;
}