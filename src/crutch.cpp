#include <Arduino.h>
#include <Wire.h>
#include <PWMServo.h>
#include <Ultrasonic.h>
// RPLidar Sendsor Library - https://github.com/robopeak/rplidar_arduino
#include <RPLidar.h>
// Vector Library - https://github.com/janelia-arduino/Vector
#include <Vector.h>

// Address of Main Controller
byte mc_address = 0x04;

struct {
    // Ultrasonic sensors
    const int us1T = 2;
    const int us1E = 3;
    const int us2T = 4;
    const int us2E = 5;
    const int us3T = 6;
    const int us3E = 7;
    const int us4T = 8;
    const int us4E = 9; 

    // Servos for ultrasonic sensors
    const int us_s1 = 10;
    const int us_s2 = 11;
    const int us_s3 = 12;
    const int us_s4 = 13;
} pinout;

struct {
    
    // Servos
    const int min_PWM = 900;
    const int max_PWM = 2100;
    const float half_speed = max_PWM - (max_PWM - min_PWM)/2;
    const int min_serv_range = 0;
    const int max_serv_range = 90;
    const int serv_speed = 18;

    // Ultrasonic sensors

    const int min_us_range = 10;
    const int max_us_range = 100;

    //Serial
    const int baud = 9600;

} mod_inf;

PWMServo us_s1;
PWMServo us_s2;
PWMServo us_s3;
PWMServo us_s4;

// Ultrasonic Sensor Objects

Ultrasonic us1(pinout.us1T, pinout.us1E);
Ultrasonic us2(pinout.us2T, pinout.us2E);
Ultrasonic us3(pinout.us3T, pinout.us3E);
Ultrasonic us4(pinout.us4T, pinout.us4E);

class Headless_Misc {
    public:

        void failsafe() {
            // Stop Motors
        }

        float scaleCommands (int direction, float multiplier) {
        // Scale Values from 0 to 1 to 0 to 255

        float speed;
        return speed;
}
};

class Headless_Sentry {
    
    // sec1 is the front sector
    // sec2 is the right sector
    // sec3 is the rear sector
    // sec4 is the left sector
    
    public:

        float get_sec_avg_and_multiplier(Vector<float> sector) {

            // verify the average of all the readings in a sector
            Vector<float> verified_sec;
            float sec_count;
            for (int i = 0; i < sector.size(); i++) {
                if (boundaries(sector[i])) {
                    sec_count += sector[i];
                    // insert failsafe here
                }
            }

            // the average of the sector is used to determine which direction the drone should move
            float sec_avg = sec_count / sector.size();
            
            // the multiplier is the percentage of the max range that the average is
            // this is used to determine how fast the drone should move
            float sec_mul = sec_avg / mod_inf.max_us_range; 
            
            return sec_avg, sec_mul;
        }

        float swivel(int direction) {
            // direction is either 1 or -1, or 0 for stop
            // create a Vector for each sector
            
            Vector<float> sec1;
            Vector<float> sec2;
            Vector<float> sec3;
            Vector<float> sec4;


            if (direction == 1) {
                for (int i = mod_inf.min_serv_range; i < mod_inf.max_serv_range; i+=mod_inf.serv_speed) {
                    us_s1.write(i);
                    sec1.push_back(us1.Ranging(CM));
                    us_s2.write(i);
                    sec2.push_back(us2.Ranging(CM));
                    us_s3.write(i);
                    sec3.push_back(us3.Ranging(CM));
                    us_s4.write(i);
                    sec4.push_back(us4.Ranging(CM));
                    delay(10);
                }
            } else if (direction == -1) {
                for (int i = mod_inf.max_serv_range; i > mod_inf.min_serv_range; i-=mod_inf.serv_speed) {
                    us_s1.write(i);
                    sec1.push_back(us1.Ranging(CM));
                    us_s2.write(i);
                    sec2.push_back(us2.Ranging(CM));
                    us_s3.write(i);
                    sec3.push_back(us3.Ranging(CM));
                    us_s4.write(i);
                    sec4.push_back(us4.Ranging(CM));
                    delay(10);
                }
            } else {
                // stop
                us_s1.write(90);
                us_s2.write(90);
                us_s3.write(90);
                us_s4.write(90);
            }
            
            float avg1, mul1 = get_sec_avg_and_multiplier(sec1);
            float avg2, mul2 = get_sec_avg_and_multiplier(sec2);
            float avg3, mul3 = get_sec_avg_and_multiplier(sec3);
            float avg4, mul4 = get_sec_avg_and_multiplier(sec4); 
            return avg1, mul1, avg2, mul2, avg3, mul3, avg4, mul4;
        }

        bool boundaries(int value) {
            // check if the value is within the mod_inf.min_us_range and mod_inf.max_us_range
            if (value > mod_inf.min_us_range && value < mod_inf.max_us_range) {
                return true;
            } else {
                return false;
            }
        }

        float decide_move(float sec1, float sec2, float sec3, float sec4) {
            if (boundaries(sec1) && boundaries(sec2) && boundaries(sec3) && boundaries(sec4)) {
                // move toward the sector with the highest average (most space)
                if (sec1 > sec2 && sec1 > sec3 && sec1 > sec4) {
                    // move forward
                    return 1;
                } else if (sec2 > sec1 && sec2 > sec3 && sec2 > sec4) {
                    // move right
                    return 2;
                } else if (sec3 > sec1 && sec3 > sec2 && sec3 > sec4) {
                    // move backward
                    return 3;
                } else if (sec4 > sec1 && sec4 > sec2 && sec4 > sec3) {
                    // move left
                    return 4;
                } else {
                    // stop
                    return 0;
                }
            } else if (boundaries(sec1) && boundaries(sec2) && boundaries(sec3) && !boundaries(sec4)) {
                // sec4 is out of range
                // move right
                return 2;
            } else if (boundaries(sec1) && boundaries(sec2) && !boundaries(sec3) && boundaries(sec4)) {
                // sec3 is out of range
                // move forward
                return 1;
            } else if (boundaries(sec1) && !boundaries(sec2) && boundaries(sec3) && boundaries(sec4)) {
                // sec2 is out of range
                // move left
                return 3;
            } else if (!boundaries(sec1) && boundaries(sec2) && boundaries(sec3) && boundaries(sec4)) {
                // sec1 is out of range
                // move backward
                return 4;


            }
        }

        float quick_init () {
          if (us_s1.read() >= 160) {
                float sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul = swivel(1);
                return sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul;
            } else {
                float sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul = swivel(-1);
                return sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul;
            }
        }

        float exec () {

            // get position of servo to swivel to empty space
            // swivel the servo & get the average of each sector
            
            float sec1, sec1_mul, sec2, sec2_mul, sec3, sec3_mul, sec4, sec4_mul = quick_init();

            // decide which direction to move based on the averages
            int move = decide_move(sec1, sec2, sec3, sec4);
            
            // return the direction, and the corresponding multiplier
            if (move == 1) {
                return 1, sec1_mul;
            } else if (move == 2) {
                return 2, sec2_mul;
            } else if (move == 3) {
                return 3, sec3_mul;
            } else if (move == 4) {
                return 4, sec4_mul;
            } else {
                return 0, 0;
            }
        }
};

class Headless_Pre_Defined_Moves {
    public:
        // This might change after trial and error, but...

        //us_s1 is front right
        //us_s2 is back right
        //us_s3 is back left
        //us_s4 is front left

        //spin specified motors 10% faster to move in a direction
        float movement_multiplier = 1.1;
        
        void forward(float speed) {
            // move forward
            // back motors spin faster
            us_s1.write(speed);
            us_s2.write(speed * movement_multiplier);
            us_s3.write(speed * movement_multiplier);
            us_s4.write(speed);
        }

        void backward(float speed) {
            // move backward
            // front motors spin faster
            us_s1.write(speed * movement_multiplier);
            us_s2.write(speed);
            us_s3.write(speed);
            us_s4.write(speed * movement_multiplier);
        }

        void left(float speed) {
            // move left
            // right motors spin faster
            us_s1.write(speed * movement_multiplier);
            us_s2.write(speed * movement_multiplier);
            us_s3.write(speed);
            us_s4.write(speed);
        }

        void right(float speed) {
            // move right
            // left motors spin faster
            us_s1.write(speed);
            us_s2.write(speed);
            us_s3.write(speed * movement_multiplier);
            us_s4.write(speed * movement_multiplier);
        }

        void hover() {
            // hover
            // write half PWM speed
            us_s1.write(mod_inf.half_speed);
            us_s2.write(mod_inf.half_speed);
            us_s3.write(mod_inf.half_speed);
            us_s4.write(mod_inf.half_speed);
        }
};

void setup() {
    Headless_Misc hm;
    Headless_Sentry hs;
    Headless_Pre_Defined_Moves hpd;

    // Ultrasonic Sensor Servos
    us_s1.attach(pinout.us_s1);
    us_s2.attach(pinout.us_s2);
    us_s3.attach(pinout.us_s3);
    us_s4.attach(pinout.us_s4);

    Serial.begin(mod_inf.baud);

}

void loop() {
    Serial.print("us1: ");
    Serial.print(us1.Ranging(CM));
    Serial.print("us2: ");
    Serial.print(us2.Ranging(CM));
    Serial.print("us4: ");
    Serial.print(us3.Ranging(CM));
    Serial.print("us4: ");
    Serial.println(us4.Ranging(CM));
    
    us_s1.write(mod_inf.max_serv_range);
    us_s2.write(mod_inf.max_serv_range);
    us_s3.write(mod_inf.max_serv_range);
    us_s4.write(mod_inf.max_serv_range);
    delay(1000);

    us_s1.write(mod_inf.min_serv_range);
    us_s2.write(mod_inf.min_serv_range);
    us_s3.write(mod_inf.min_serv_range);
    us_s4.write(mod_inf.min_serv_range);
    delay(1000);

}