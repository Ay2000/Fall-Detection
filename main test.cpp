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
  bool potentialfall = false;
  bool fallXGyro = false;
  bool fallYGyro = false;
  bool fallZGyro = false;
  
  intake;
  if (sqrt((xAccel^2)+(yAccel^2)+(zAccel^2)) >threshholdNetAccel)
  {
    potentialfall = true;
    if (potentialfall) { // if statement only works if potential fall is true
      //Test Gyro
      if (xGyro > threshholdXGyro) {
        fallXGyro = true;
      }
      if (yGyro > threshholdYGyro) {
        fallYGyro = true;
      }
      if (zGyro > threshholdZGyro) {
        fallZGyro = true;
      }
      if (fallXGyro && fallYGyro && fallZGyro) {
        //Buzzer activates
      }
      //Prints 1 if it has exceeded the Threshhold (For Testing Values)
      Serial.print(fallXGyro);
      Serial.print(fallYGyro);
      Serial.print(fallZGyro);
    }
  }
  
}
