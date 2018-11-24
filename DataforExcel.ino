/*
Fall Detection Device
Compares linear acceleration and angular velocity to their respective thresholds to determine if the user has fallen

Daniel Carleton
Created: 18 November, 2018
Last Updated: 23 November, 2018

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
 * 2 Push buttons
 * 1 10K Ohm Resistors
 * 1 100 Ohm Resistor
 * 2 Push Buttons
 * 1 Piezzo Buzzer (<5V)
 * Reset Pin (Built Into Arduino)
 * Adafruit BNO055 IMU
 * Power bank with USB to power Device

Libraries for Adafruit BNO055 Accelerometers
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
  Scroll down to Software to:
    Download the Driver from Github
    Download Adafruit_Sensor 

*/

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Arduino.h>

// set constant pin numbers:
const byte interruptPin = 3;  // pin number to alert help
const byte buzzerPin = 7;     // pin number for buzzer, maybe change to pin8 if it doesn't work

// counter for arrays to keep track of entries
unsigned long counter = 0; // offset to keep keep entires in correct slots
 
#define TONE 1000 // 1000 is good, 100 is less annoying

// Thresholds
const float thresholdAccel = 14.0; //threshold for acceleration: m/s^2
const float thresholdGyro = 100.0; //threshold for angular velocity: rad/s

// arrays for 5 calculations of accelerometer magnitude and gyroscope magnitude
double accelArray [6]; // n+2 size or i get errors
double gyroArray [6];

#define BNO055_SAMPLERATE_DELAY_MS (100) // Set the delay between fresh samples

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//function prototypes
void pin_ISR(); // buzzer interrupt function
void displaySensorDetails(void); //display sensor information, sensor API sensor_t type (see Adafruit_Sensor for more information)
void displaySensorStatus(void); //Display basic info about sensor status
void displayCalStatus(void); //display calibration status
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData); //displays raw calibration and radius data
void displaydata(); //displays 3 axis vector readings for Gyroscope and Accelerometer
double magnitudeAccel(); // displays magnitude of Acceleration
double magnitudeGyro(); // displays magnitude of angular velocity
double averageGyroReadings(); // average all 5 entries of the gyroArray
double averageAccelReadings(); // average all 5 entries of the accelArray

void displaySensorDetails(void) //display sensor information, sensor API sensor_t type (see Adafruit_Sensor for more information)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    /*
    Serial.print("------------------------------------\n");
    Serial.print("Sensor:       "); Serial.print(sensor.name); Serial.print("\n");
    Serial.print("Driver Ver:   "); Serial.print(sensor.version); Serial.print("\n");
    Serial.print("Unique ID:    "); Serial.print(sensor.sensor_id); Serial.print("\n");
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.print(" xxx\n");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.print(" xxx\n");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.print(" xxx\n");
    Serial.print("------------------------------------\n\n");
    delay(500);
    */
}

void displaySensorStatus(void) //Display basic info about sensor status
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    /*
    Serial.print("\n");
    Serial.print("System Status: 0x");
    Serial.print(system_status, HEX); Serial.print("\n");
    Serial.print("Self Test:     0x");
    Serial.print(self_test_results, HEX); Serial.print("\n");
    Serial.print("System Error:  0x");
    Serial.print(system_error, HEX); Serial.print("\n");
    */
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
    //Serial.print("\t");
    if (!system)
    {
        //Serial.print("! ");
    }

    /* Display the individual values */
    /*
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
    */
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) //displays raw calibration and radius data
{
  /*
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); 
    Serial.print(calibData.accel_offset_y); 
    Serial.print(calibData.accel_offset_z); 

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); 
    Serial.print(calibData.gyro_offset_y); 
    Serial.print(calibData.gyro_offset_z); 

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); 
    Serial.print(calibData.mag_offset_y); 
    Serial.print(calibData.mag_offset_z); 

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
    */
}

void displaydata()
{
/* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    /* Display the floating point data
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
  */
  
  /* Optional: Display calibration status */
  //displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  //Serial.print("\n");
}

double magnitudeAccel()
{
  double xAccelSquared = 0;
  double yAccelSquared = 0;
  double zAccelSquared = 0;
  double accelSum = 0;
  double accelRoot = 0;

  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  xAccelSquared = (accelerometer.x())*(accelerometer.x());
  yAccelSquared = (accelerometer.y())*(accelerometer.y());
  zAccelSquared = (accelerometer.z())*(accelerometer.z());

  accelSum = xAccelSquared + yAccelSquared + zAccelSquared;
  accelRoot = sqrt(accelSum);
  //Serial.print("\tAcceleration Maximum: ");     
  return accelRoot;
}

double averageAccelReadings() // average all 5 entries of the accelArray
{
  int a = 0;
  double averageAccelReading = 0;

  for(a=0;a<5;a++)
  {
    averageAccelReading = averageAccelReading + accelArray[a];
  }
  averageAccelReading = averageAccelReading/5;
  return averageAccelReading;
}

double averageGyroReadings() // average all 5 entries of the gyroArray
{
  int g = 0;
  double averageGyroReading = 0;

  for(g=0;g<5;g++)
  {
    averageGyroReading = averageGyroReading + gyroArray[g];
  }
  averageGyroReading = averageGyroReading/5;
  return averageGyroReading;
}

double magnitudeGyro()
{
  double xGyroSquared = 0;
  double yGyroSquared = 0;
  double zGyroSquared = 0;
  double gyroSum = 0;
  double gyroRoot = 0;

  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  xGyroSquared = (gyroscope.x())*(gyroscope.x());
  yGyroSquared = (gyroscope.y())*(gyroscope.y());
  zGyroSquared = (gyroscope.z())*(gyroscope.z());

  gyroSum = xGyroSquared + yGyroSquared + zGyroSquared;
  gyroRoot = sqrt(gyroSum);
  //Serial.print("Gyroscope Maximum: ");
  //Serial.println(gyroRoot);
  return gyroRoot;
}

void setup() // Arduino startup function
{
  Serial.begin(9600); //baud rate
  //Serial.print("Loading... \n");
  delay(1000);
  
  // initialize the output pins
  pinMode(buzzerPin, OUTPUT);
  
  // initialize the input pin
  pinMode(interruptPin, INPUT_PULLUP);

  // initialize the interrupt pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), pin_ISR, CHANGE);

  int i;
  for (i=0;i<5;i++) // initialize arrays
  {
    accelArray[i] = 0;
    gyroArray[i] = 0;
  }

  delay(500);

  if (!bno.begin())
    {
       //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
       while (1);
    }
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    bno.getSensor(&sensor); // Look for the sensor's unique ID at the beginning of EEPROM.
    if (bnoID != sensor.sensor_id) // Must redo calibration
    {
        //Serial.print("\nNo Calibration Data for this sensor exists in EEPROM\n");
        delay(500);
    }
    else // Calibration data found
    {
        //Serial.print("\nFound Calibration for this sensor in EEPROM.\n");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        //Serial.print("\n\nRestoring Calibration data to the BNO055...\n");
        bno.setSensorOffsets(calibrationData);

        //Serial.print("\n\nCalibration data loaded into BNO055\n");
        foundCalib = true;
    }
    delay(1000);

    displaySensorDetails(); // Display some basic information on this sensor
    displaySensorStatus(); // Display current status
    bno.setExtCrystalUse(true); // Crystal must be configured AFTER loading calibration data into BNO055.

    sensors_event_t event;
    bno.getEvent(&event);

    if (!foundCalib)
    {
        //Serial.print("Please Calibrate Sensor: \n");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            //Serial.print("X: ");
            //Serial.print(event.orientation.x, 4);
            //Serial.print("\tY: ");
            //Serial.print(event.orientation.y, 4);
            //Serial.print("\tZ: ");
            //Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.print("\n");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    //Serial.print("\nFully calibrated!\n");
    //Serial.print("--------------------------------\n");
    //Serial.print("Calibration Results: \n");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    //Serial.print("\n\nStoring calibration data to EEPROM...\n");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    //Serial.print("Data stored to EEPROM.\n");

    //Serial.print("\n--------------------------------\n\n");
    delay(500);
}

void loop() 
{
  //Serial.print("\nNode: ");
  //Serial.print(counter);
  //Serial.print("\n");
  displaydata();

  double averageAccel = 0;
  double averageGyro = 0;
  
  int i;
  for(i=4;i>0;i--) // shifts every element in array to the right by 1
  {
    accelArray[i] = accelArray[i-1];
    gyroArray[i] = gyroArray[i-1];
  }
  // records new data into the 0th slot of the arrays
  accelArray[0]= magnitudeAccel();
  gyroArray[0] = magnitudeGyro();

  Serial.print(accelArray[0]);
  Serial.print("\t");
  Serial.print(gyroArray[0]/10);
  Serial.print("\n");

  int j;
  //Serial.print("       \t\t\t\taccel");    
  //Serial.print("  gyro\n");
  for(j=0;j<5;j++)
  {
    //Serial.print("       \t\t\t\t");
    //Serial.print(accelArray[j]);
    //Serial.print("\t");
    //Serial.print(gyroArray[j]);
    //Serial.print("\n");
  }
  
  if (accelArray[0] > thresholdAccel) // if newest acceleration reading is greater than the threshold
  {
    // Calculate the average angular velocity and acceleration for 5 readings
    averageAccel = averageAccelReadings();
    averageGyro = averageGyroReadings(); 

    if (averageAccel > thresholdAccel) // if 5 reading average > threshold then maybe falling
    {
      if (averageGyro > thresholdGyro) // if 5 readings of gyro > threshold then falling
      {
        //Serial.print("User is falling\n");
        tone(buzzerPin, TONE); // tone(pin number, frequency: Hz, duration: ms)
        delay(500);
      }
      else
      {
        //Serial.print("False positive\n");
      }
    }
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
  counter++;
}

void pin_ISR() 
{
  tone(buzzerPin, TONE, 2000); // tone(pin number, frequency: Hz, duration: ms
}
