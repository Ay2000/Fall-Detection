#include <iostream>
#include <cmath>
using namespace std;

int main()
{
  double xAccel = 0.0;
  double yAccel = 0.0;
  double zAccel = 0.0;
  double xGyro = 0.0;
  double yGyro = 0.0;
  double zGyro = 0.0;
  // Make thresholds into constants once we get the values
  double threshholdNetAccel = 0.0;
  double threshholdXGyro = 0.0;
  double threshholdYGyro = 0.0;
  double threshholdZGyro = 0.0;
  // bool statements to pass if true
  bool potentialfall = false;
  bool fallXGyro = false;
  bool fallYGyro = false;
  bool fallZGyro = false;
  
  intake;
  if (sqrt((xAccel^2)+(yAccel^2)+(zAccel^2)) >threshholdNetAccel)
  {
    potentialfall = true;
    Serial.print("The user MIGHT have fallen\n")
    if (potentialfall) { // if there is a potential fall, test for Gyro values
      //xGyro being tested
      if (xGyro > threshholdXGyro) {
        fallXGyro = true;
      }
      if (yGyro > threshholdYGyro) {
        fallYGyro = true;
      }
      if (zGyro > threshholdZGyro) {
        fallZGyro = true;
      }
      
      if (fallXGyro && fallYGyro && fallZGyro) { // If all XYZ Gyro readings have exceeded the threshhold value, activate Buzzer
        // Buzzer activates
        Serial.print("The User Has Fallen\n")
      }
      // Prints 1 if it has exceeded the Threshhold (For Testing Values)
      Serial.print(fallXGyro);
      Serial.print(fallYGyro);
      Serial.print(fallZGyro);
      // Could add if statements to test if the user is running/sitting down/etc. if wanted (probably wouldn't need it)
    }
    potentialfall = false;
    fallXGyro = false;
    fallYGyro = false;
    fallZGyro = false;
  }
  
}
