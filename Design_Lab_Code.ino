#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
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

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
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

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
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

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
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


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void setup(void)
{
    Serial.begin(9600); //baud rate
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
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
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
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

/*
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
*/
    
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


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop() 
{
    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Display the floating point data */
  // This stores up to 5 readings of the accelerometer(xyz) to use for additive detection
  int switch_count1 = 0;
  const int amountOfReadings = 5;
  double firstReading[3] = {0.0, 0.0, 0.0};
  double secondReading[3] = {0.0, 0.0, 0.0};
  double thirdReading[3] = {0.0, 0.0, 0.0};
  double fourthReading[3] = {0.0, 0.0, 0.0};
  double fifthReading[3] = {0.0, 0.0, 0.0};
  if (switch_count1 < amountOfReadings){
   switch (switch_count1){
    case 0: firstReading[0] = accelerometer.x(), firstReading[1] = accelerometer.y(), firstReading[2] = accelerometer.z()
     break;
    case 1: secondReading[0] = accelerometer.x(), secondReading[1] = accelerometer.y(), secondReading[2] = accelerometer.z()
     break;
    case 2: thirdReading[0] = accelerometer.x(), thirdReading[1] = accelerometer.y(), thirdReading[2] = accelerometer.z()
     break;
    case 3: fourthReading[0] = accelerometer.x(), fourthReading[1] = accelerometer.y(), fourthReading[2] = accelerometer.z()
     break;
    case 4: fifthReading[0] = accelerometer.x(), fifthReading[1] = accelerometer.y(), fifthReading[2] = accelerometer.z()
     break;
    default:
     break;
  } 
  
  switch_count1++;
  if (switch_count1 == amountofReadings) {
    switch_count1 = 0;
  }
  }
  // move constant variables later
  // test XYZ components of each scenario
  const double thresholdFallX;
  const double thresholdFallY;
  const double thresholdFallZ;
  
  const double thresholdStandX;
  const double thresholdStandY;
  const double thresholdStandZ;
  
  const double thresholdSitX;
  const double thresholdSitY;
  const double thresholdSitZ;
  
  const double thresholdRunX;
  const double thresholdRunY;
  const double thresholdRunZ;

  // Current output of XYZ readings (accelerometer)
  Serial.print("accelerometer: ");
  Serial.print("X: ");
  Serial.print(accelerometer.x());
 
  Serial.print(" Y: ");
  Serial.print(accelerometer.y());
  
  Serial.print(" Z: ");
  Serial.print(accelerometer.z());

  //Calculating Anet (net acceleration)
  

  // Testing scenarios
  if (accelerometer.x > thresholdFallX){
    Serial.print ("The threshold x of fall has been exceeded\n");
    // Test for other components of the FALL
    if (accelerometer.y > thresholdFallY) {
      Serial.print ("The threshold y of fall has been exceeded\n");
      if (accelerometer.z > thresholdFallZ){
        Serial.print ("The threshold z of fall has been exceeded\n");
        //output a signal for a fall
      }
    } 
  }
  if (accelerometer.x > thresholdStandX) {
    Serial.print ("The threshold x of stand has been exceeded\n");
    // Test for other components of the STAND
    if (accelerometer.y > thresholdStandY) {
      Serial.print ("The threshold y of stand has been exceeded\n");
      if accelerometer.z > thresholdStandZ) {
        Serial.print ("The threshold z of stand has been exceeded\n");
        Serial.print ("The person has stood up\n");
      }
    }
  }
  if (accelerometer.x > thresholdSitX) {
    Serial.print ("The threshold x of sit has been exceeded\n");
    // Test for other components of the SIT
    if (accelerometer.y > thresholdSitY) {
      Serial.print ("The threshold y of sit has been exceeded\n");
      if (accelerometer.z > thresholdSitZ) {
        Serial.print ("The threshold z of sit has been exceeded\n");
        Serial.print ("The person is now sitting down\n");
      }
    }
  }
  if (accelerometer.x > thresholdRunX) {
    Serial.print ("The threshold x of run has been exceeded\n");
    // Test for other components of the RUN
    if (accelerometer.y > thresholdRunY) {
      Serial.print ("The threshold y of run has been exceeded\n");
      if (accelerometer.z > thresholdRunZ) {
        Serial.print ("The threshold z of run has been exceeded\n");
        Serial.print ("The person is now running\n");
      }
    }
  }
     int switch_count2 = 0;
  double firstReadingGyro[3] = {0.0, 0.0, 0.0};
  double secondReadingGyro[3] = {0.0, 0.0, 0.0};
  double thirdReadingGyro[3] = {0.0, 0.0, 0.0};
  double fourthReadingGyro[3] = {0.0, 0.0, 0.0};
  double fifthReadingGyro[3] = {0.0, 0.0, 0.0};
  if (switch_count2 < amountOfReadings){
   switch (switch_count1){
    case 0: firstReadingGyro[0] = gyroscope.x(), firstReadingGyro[1] = gyroscope.y(), firstReadingGyro[2] = gyroscope.z()
     break;
    case 1: secondReadingGyro[0] = gyroscope.x(), secondReadingGyro[1] = gyroscope.y(), secondReadingGyro[2] = gyroscope.z()
     break;
    case 2: thirdReadingGyro[0] = gyroscope.x(), thirdReadingGyro[1] = gyroscope.y(), thirdReadingGyro[2] = gyroscope.z()
     break;
    case 3: fourthReadingGyro[0] = gyroscope.x(), fourthReadingGyro[1] = gyroscope.y(), fourthReadingGyro[2] = gyroscope.z()
     break;
    case 4: fifthReadingGyro[0] = gyroscope.x(), fifthReadingGyro[1] = gyroscope.y(), fifthReadingGyro[2] = gyroscope.z()
     break;
    default:
     break;
  } 
  
  switch_count2++;
  if (switch_count2 == amountOfReadings) {
    switch_count2 = 0;
  }

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
    delay(BNO055_SAMPLERATE_DELAY_MS);
}
