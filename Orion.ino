/*
Code written by Jeff Taylor-Chang in 2017
System designed by Jeff Taylor-Chang and Mike Perreault
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

boolean calibrating = false;
float thetaX = 0.0f;
float thetaY = 0.0f;
float thetaZ = 0.0f;
float omegaX = 0.0f;
float omegaY = 0.0f;
float omegaZ = 0.0f;
float aX = 0.0f;
float aY = 0.0f;
float aZ = 0.0f;

float dir = 1;
float maxTheta = 0;
float localMaxTheta = 0;
boolean peaking = false;
int peakCount = 0;

int calibrationPin = 11;
boolean solState = false;

float startTime = 0;
float endTime = 0;
boolean ended = false;

void getSensorDetails() {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus() {
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

void displayCalStatus() {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  //cut unnecessary features to lower calibration time
  accel = 3;
  mag = 3;
  
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  //Serial.print("\t");
  if(system) {
    calibrating = true;
  }
  if (!system) {
    calibrating = false;
    //Serial.print("! ");
  }
  if(calibrating) {
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
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Orion Starting");

  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(calibrationPin, OUTPUT);

  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  getSensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

void getSensorData() {
  sensors_event_t event;
  bno.getEvent(&event);

  aX = event.acceleration.x;
  aY = event.acceleration.y;
  aZ = event.acceleration.z;
  thetaX = event.orientation.x;
  thetaY = event.orientation.y;
  thetaZ = event.orientation.z;

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  omegaX = gyro.x();
  omegaY = gyro.y();
  omegaZ = gyro.z();
}

void onPeak() {
  dir *= -1;
  if(dir == 1) {
    digitalWrite(12, HIGH);
    digitalWrite(3, LOW);
    sol(true);
  } else if(dir == -1) {
    digitalWrite(12, LOW);
    digitalWrite(3, HIGH);
    
  }
}

void sol(boolean on) {
  solState = on;
  if(on) {
    digitalWrite(9, HIGH);
    //Serial.println("Turning solenoid ON");
  } else {
    digitalWrite(9, LOW);
    //Serial.println("Turning solenoid OFF");
  }
}

void onFall() {
  sol(false);
}

/*

event.orientation.x/y/z         <-- angles
event.gyro.x/y/z                <-- angular velocity
event.acceleration.x/y/z        <-- linear acceleration

*/

void processSensorData() {
  float theta = abs(thetaY);
  if(theta > maxTheta) {
    peaking = true;
    peakCount += 1;
    if(theta > localMaxTheta) {
      localMaxTheta = theta;
    }
    maxTheta = theta;
  } else {
    if(peaking) {
      if(peakCount > 1) {
        localMaxTheta = 0;
        peakCount = 0;
        peaking = false;
        
        if(theta > 10) {
          if(startTime == 0) {
            startTime = millis();
            Serial.println("Start Time:");
            Serial.println(startTime);
          }
          onPeak();
          //int d = (int) (sin((maxTheta * 2 * PI) / 360) * 250);
          int d = (int) (maxTheta * 7);
          //Serial.println(d);
          delay(d);//maybe should be proportional to the peak angle
          onFall();
          maxTheta = 0;
        } else {
          if(startTime != 0) {
            endTime = millis();
            Serial.println("End Time:");
            Serial.println(endTime);
            Serial.println("Total Time:");
            Serial.println(endTime - startTime);
            ended = true;
          }
        }
      } else {
        peakCount = 0;
        peaking = false;
      }
    }
    maxTheta = 0;
    localMaxTheta = 0;
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  getSensorData();

  displayCalStatus();

  if(!calibrating) {
    if(!ended) {
      processSensorData();
      digitalWrite(calibrationPin, HIGH);
    }
  } else {
    Serial.println("Calibrating");
    digitalWrite(calibrationPin, LOW);
  }

  digitalWrite(9, LOW);
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
