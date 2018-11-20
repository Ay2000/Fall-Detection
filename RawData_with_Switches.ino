/*

3 Button and Buzzer Arduino Test Driver

Daniel Carleton
Created: 18 November, 2018
Last Updated: 20 November, 2018
 
LED
 * LED attached internally from pin13 to ground.

Button 1
 * button1 connected to +5V.
 * Other side of button1 connected to pin2 via 10K resistor.
 * Power button for Arduino, not needed

Button 2
 * button2 connected to +5V.
 * Other side of button2 connected to pin3 via 10K resistor.

Buzzer
 * Requires a digital pin.
 * Buzzer is polarized.
    * Positive side of buzzer connected to buzzerpin
    * negative side of buzzer connected to ground via a 100 ohm resistor

The Reset Pin
 * Connect resetpin on arduino to one side of the button.
 * Connect the other side of the button to ground.

Build of Materials
 * Arduino
 * 2 10K Ohm Resistors
 * 1 100 Ohm Resistor
 * 2 Push Buttons
 * 1 Piezzo Buzzer (<5V)
 * Reset Pin (Built Into Arduino)
 * LED (Built into pin13 on Arduino)

*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Arduino.h>

// set constant pin numbers:
const byte switchOnePin = 2;  // pin number to run program
const byte interruptPin = 3;  // pin number to alert help
const byte buzzerPin = 4;     // pin number for buzzer, maybe change to pin8 if it doesn't work

// the follow variables are long's because the time, measured in milliseconds, will quickly become a bigger number than can be stored in an int.
long time = 0;                // the last time the output pin was toggled
long debounce = 200;          // the debounce time, increase if the output flicker

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void displaySensorDetails(void) //display sensor information, sensor API sensor_t type (see Adafruit_Sensor for more information)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void displaySensorStatus(void) //Display basic info about sensor status
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void displayCalStatus(void) //display calibration status
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) //displays raw calibration and radius data
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void displaydata()
{
/* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    /* Display the floating point data */
  Serial.print("accelerometer: ");
  Serial.print("X: ");
  Serial.print(accelerometer.x());
  Serial.print(" Y: ");
  Serial.print(accelerometer.y());
  Serial.print(" Z: ");
  Serial.print(accelerometer.z());

  Serial.print("  gyroscope:  ");
  Serial.print("X: ");
  Serial.print(gyroscope.x());
  Serial.print(" Y: ");
  Serial.print(gyroscope.y());
  Serial.print(" Z: ");
  Serial.print(gyroscope.z());
  
  Serial.print("\t\t");

  /* Optional: Display calibration status */
  //displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting new data */
}

void setup() // Arduino startup function
{
  Serial.begin(9600); //baud rate
  Serial.print("Loading... \n");
  delay(500);
  
  // initialize the outputs pins
  pinMode(buzzerPin, OUTPUT);
  
  // initialize the input pin
  pinMode(switchOnePin, INPUT_PULLUP);
  pinMode(interruptPin, INPUT_PULLUP);

  //initialize the interrupt pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), pin_ISR, CHANGE);

  if (!bno.begin())
    {
       Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
       while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning of EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

   //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);

    if (!foundCalib)
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
}

void pin_ISR() 
{
  tone(buzzerPin, 100, 2000); // tone(pin number, frequency: Hz, duration: ms)
}

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

void loop() 
{
  displaydata();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
