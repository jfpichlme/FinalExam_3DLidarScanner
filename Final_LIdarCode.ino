#include "TFMini.h"
#include <HardwareSerial.h>
#include <Stepper.h>

TFMini tfmini;
#define STEPS 2038

Stepper stepperVerti(STEPS, 2, 4, 3, 5);
Stepper stepperHor(STEPS, 8, 10, 9, 11);


double Distances[10];

int TotalStepVerti = 0;
int TotalStepHor = 0;
int cnt = 0;
int NrMe = 0;

double oneDagree = 6;

int HalfTrip = 1066;


// if false we go into the negative direction so step = -3, if true we go into the positive direction
bool roundtrip = false;



void setup() {

  delay(6000);
  
  // Step 1: Initialize hardware serial port (serial debug port)
  Serial.begin(115200);
  Serial1.begin(115200);
  

  //delay(4000);
  
  //Initialize the TFMini LIDAR
  tfmini.begin(&Serial1);

  //Initialize single measurement mode with external trigger
  //Serial.println("Setup Succeeded");
  
  tfmini.setSingleScanMode();
  delay(500);

  StforwardVerti(10, -HalfTrip);
  delay(1000);
  StOriginalVerti(HalfTrip);


  StforwardHori(10, -HalfTrip);
  delay(1000);
  StOriginalHori(HalfTrip);
  Serial.println("Setup succeeded");

 
  
}



void loop() {


  // Is the scan completed? 
  if (TotalStepHor >= HalfTrip){
    delay(40000000);}
    // Is Sensor 180 Dagree reached 
    else if(TotalStepVerti >= HalfTrip){
      //Serial.println(cnt);

      // Go Step Forward with Bottem stepper
      StforwardHori(5, -oneDagree);
      TotalStepHor += oneDagree;
      TotalStepVerti = 0;

      if(roundtrip == false)
      {roundtrip = true;} 
      else if (roundtrip == true){roundtrip = false;}
      }else{

        if(roundtrip == false){
           cnt += 1;
           double distance = TakeMeasurment();  
           Serial.println(distance);
           StforwardVerti(5, -oneDagree);
           TotalStepVerti += oneDagree;  
          }else if(roundtrip == true){
           double distance = TakeMeasurment();
           Serial.println(distance);
           StforwardVerti(5, oneDagree);
           TotalStepVerti += oneDagree; 
           }
        }
      }
    

 

  
 


double TakeMeasurment() {

  int test = tfmini.takeMeasurement();

  for (int i = 0; i < 10; i++){
    double distance = tfmini.getDistance();
    Distances[i] = distance;
    }

  double distance = Distances[9];

  uint16_t strength = tfmini.getRecentSignalStrength();

  // Display the measurement
  //Serial.println(distance);

  return distance;

  // Wait some time before taking the next measurement
  // without delay, measurement is super fast
 }

 


void StforwardVerti(int sped,int magnit){
 stepperVerti.setSpeed(sped);
 stepperVerti.step(magnit);
 delay(100);
  }


void StOriginalVerti(int magnit){
 stepperVerti.setSpeed(10);
 stepperVerti.step(magnit);
 delay(100);
  }


void StforwardHori(int sped,int magnit){
 stepperHor.setSpeed(sped);
 stepperHor.step(magnit);
 delay(100);
  }


void StOriginalHori(int magnit){
 stepperHor.setSpeed(10);
 stepperHor.step(magnit);
 delay(100);
  }




 
