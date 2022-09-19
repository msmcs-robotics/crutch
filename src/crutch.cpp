#include <Arduino.h>

// I2C Library
#include <Wire.h>

byte pi_address = 0x04;


//RPLidar Sendsor Library - https://github.com/robopeak/rplidar_arduino
#include <RPLidar.h>

RPLidar lidar;
int RPLIDAR_MOTOR = 9; //Anlalog Pin

float maxDistance = 100.0;
float minDistance = 0.0;
float angleAtMinDist = 0.0;

// UltraSonic Sensor Library - https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
#include <Ultrasonic.h>

int us1T = 4; //Digital Pins
int us1E = 5;
int us2T = 6;
int us2E = 7;
Ultrasonic us1(us1T, us1E);
Ultrasonic us2(us2T, us2E);


class DisatanceSensors {
  
  public:
  
    // up and down
    int check_ultrasonic_sensors() {
        int u1 = us1.Ranging(CM);
        int u2 = us2.Ranging(CM);
        if (u1 < 10) {
            return 1;
        } else if (u2 < 10) {
            return 2;
        } else {
            return 3;
        }
        
    }

    float check_RPLidar() {

        //If the lidar is not connected, return 0

        if (IS_OK(lidar.waitPoint())) {
            //perform data processing here... 
            float distance = lidar.getCurrentPoint().distance;
            float angle = lidar.getCurrentPoint().angle;
            
            // map angle from 0-360 triangle for x and y
            float x = distance * cos(angle);
            float y = distance * sin(angle);

            // Get distance to x and y coordinates
            float disToPoint = sqrt(pow(x, 2) + pow(y, 2));

            //map that distance to the max and min distance and get a percentage
            float disPercent = map(disToPoint, minDistance, maxDistance, 0, 100);

            
            if (distance >= maxDistance) {
                return 0.0, 0.0;
            } else if (distance <= minDistance) {
                return disPercent, angleAtMinDist;
            } else {
                return disPercent, angle;
            }

        } else {

            analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
            
            // try to detect RPLIDAR... 
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            //detected...
            lidar.startScan();
            analogWrite(RPLIDAR_MOTOR, 255);
            delay(1000);
            }
        }
    }

};


void setup() {

    Serial.begin(9600);
    lidar.begin(Serial);
    Wire.begin();

    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(us1T, OUTPUT);
    pinMode(us1E, INPUT);
    pinMode(us2T, OUTPUT);
    pinMode(us2E, INPUT);
}


void onReceive(int bytes) {
    while (Wire.available()) {
        char c = Wire.read();
        Serial.print(c);
    }
}

void loop() {

    DisatanceSensors ds;
    
    // get distances from objects
    // If danger close, also specify
    int mX, mY = ds.check_RPLidar();
    int mZ = ds.check_ultrasonic_sensors();

    // send data to pi
    Wire.beginTransmission(pi_address);
    Wire.write(mX);
    Wire.write(mY);
    Wire.write(mZ);
    Wire.endTransmission();
    delay(1000);
}