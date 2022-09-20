/*

 - Get data from sensors
    - x and y position object from RPLidar
    - z position object from Ultrasonic Sensors

 - Prioritize data
    - convert x,y,z to percentages of total distance of measurement

 - Send data to Main Controller
    - send via I2C

*/

#include <Arduino.h>

// I2C Library
#include <Wire.h>

// Address of Main Controller
byte mc_address = 0x04;


// RPLidar Sendsor Library - https://github.com/robopeak/rplidar_arduino
#include <RPLidar.h>

struct {
    int RPLIDAR_MOTOR = 9; // Anlalog Pin
    float maxDistance = 100.0; //not the max distance of the sensor, but the max distance we want to use
    float minDistance = 0.0; //not the min distance of the sensor, but the min distance we want to use
    float angleAtMinDist;
    float tol = maxDistance * 0.05; //  5% tolerance
} rpl;

RPLidar lidar;


// UltraSonic Sensor Library - https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
#include <Ultrasonic.h>

struct {
    int us1T = 4; // Digital Pins
    int us1E = 5; // T - Trigger, E - Echo
    int us2T = 6;
    int us2E = 7;
    float maxDistance = 100.0; //not the max distance of the sensor, but the max distance we want to use
    float minDistance = 0.0; //not the min distance of the sensor, but the min distance we want to use
    float tol = maxDistance * 0.05; // 5% tolerance
} usp;

Ultrasonic us1(usp.us1T, usp.us1E);
Ultrasonic us2(usp.us2T, usp.us2E);

class DoMath {
    
    public:

        float rpl_to_coords (float a, float d) {
            // convert rplidar angle and distance to x and y coordinates
            float x = d * cos(a);
            float y = d * sin(a);
            return x, y;
        }

        float check_rpl_dis(float d) {
            if (d >= rpl.maxDistance) {
                d = rpl.maxDistance;
                return d;
            } else if (d <= rpl.minDistance) {
                d = rpl.minDistance;
                return d;
            } else {
                return d;
            }
        }

        float handle_rpl_vals(float d, float a) {
            
            // make sure not out of range
            d = check_rpl_dis(d);

            // convert angle and distance to x and y coordinates
            float x, y = rpl_to_coords(a, d);


            //normalize to a domain and range of (-100, 100)
            //x -> (-100, 100) -> (left, right)
            //y -> (-100, 100) -> (back, front)
            float x_norm = map(x, rpl.minDistance, rpl.maxDistance, -100, 100);
            float y_norm = map(y, rpl.minDistance, rpl.maxDistance, -100, 100);

            // the result can be interpreted as a percentage of the distance from the robot to the object
            
            // the previous math has been used to calculate where the object is in relation to the robot
            // the robot should move in the opposite direction of the object
            // so the robot should move in the direction of the negative of the x and y values

            // return the new percentages
            return -x_norm, -y_norm;
        }



        float check_us_dis(float d) {
            if (d >= usp.maxDistance) {
                d = usp.maxDistance;
                return d;
            } else if (d <= usp.minDistance) {
                d = usp.minDistance;
                return d;
            } else {
                return d;
            }
        }

        float comp_us_vals(float d1, float d2) {

            // if the difference is within the tolerance, return the average

            if (d1 >= (d2 + usp.tol)) {
                return d1;
            } else if (d2 >= (d1 + usp.tol)) {
                return d2;
            } else {
                return (d1 + d2) / 2;
            }
        }

        float handle_us_vals(float u1, float u2) {

            float d = comp_us_vals(u1, u2);

            //normalize to a domain and range of (-100, 100)
            //x -> (-100, 100) -> (left, right)
            //y -> (-100, 100) -> (back, front)
            float d_norm = map(d, usp.minDistance, usp.maxDistance, -100, 100);

            // the previous math has been used to determine how far away the object is
            // the robot should move in the opposite direction of the object
            // so the robot should move in the direction of the negative percentage of the distance

            // return the new percentage
            return -d_norm;
        }

};

class DisatanceSensors {

    DoMath dm;

    public:

    // up and down
        float check_ultrasonic_sensors() {
            
            float u1 = us1.Ranging(CM);
            float u2 = us2.Ranging(CM);
            
            float z = dm.handle_us_vals(u1, u2);
            return z;
        
        }

        float check_RPLidar() {

            //More debuggin steps are needed to determine if the RPLidar is working properly

            if (IS_OK(lidar.waitPoint())) {
                analogWrite(rpl.RPLIDAR_MOTOR, 255);

                if (lidar.getCurrentPoint().startBit) {
                    // get average distance for four sectors out of 360
                    float d = lidar.getCurrentPoint().distance;
                    float a = lidar.getCurrentPoint().angle;

                    //perform data processing here... 
                
                    float x, y = dm.handle_rpl_vals(d, a);
                    return x, y;
                }
                
            } else {
            analogWrite(rpl.RPLIDAR_MOTOR, 0); //stop the rplidar motor
            // try to detect RPLIDAR... 
            rplidar_response_device_info_t info;
            if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            //detected...
            lidar.startScan();
            analogWrite(rpl.RPLIDAR_MOTOR, 255);
            delay(1000);
            }
        }
    }

};

void onReceive(int bytes) {
    while (Wire.available()) {
        // change this depending on the number of whatever you are receiving
        // if you are receiving anything at all
        char c = Wire.read();
        Serial.print(c);
    }
}

void setup() {

    Serial.begin(9600);
    lidar.begin(Serial);
    Wire.begin();

    pinMode(rpl.RPLIDAR_MOTOR, OUTPUT);
    pinMode(usp.us1T, OUTPUT);
    pinMode(usp.us1E, INPUT);
    pinMode(usp.us2T, OUTPUT);
    pinMode(usp.us2E, INPUT);

    lidar.startScan();
    analogWrite(rpl.RPLIDAR_MOTOR, 0);
}

void loop() {

    DisatanceSensors ds;
    
    // get distances from objects
    // If danger close, also specify
    int mX, mY = ds.check_RPLidar();
    int mZ = ds.check_ultrasonic_sensors();

    // send data to Main Controller
    Wire.beginTransmission(mc_address);
    Wire.write(mX); // percentage that the robot should move in the x direction
    Wire.write(mY); // percentage that the robot should move in the y direction
    Wire.write(mZ); // percentage that the robot should move in the z direction
    Wire.endTransmission();
    delay(1000);
}