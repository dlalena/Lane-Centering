/*
 * VMI ECE '23 CAPSTONE
 * QL PLUS PROJECT - SWIM LANE CENTERING
 * 10 NOV 2022
 */

#include <Arduino.h>
#include <Wire.h>
#include <Tone.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//setting tone to play in individual ear
Tone toneRight;
Tone toneLeft;

//variable to be used for when device is in calibration mode, and when device exits, entering use mode
//bool systemCalibrated;

void printEvent(sensors_event_t* event) {
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
        Serial.print("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        Serial.print("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        Serial.print("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
        Serial.print("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        Serial.print("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_GRAVITY) {
        Serial.print("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else {
        Serial.print("Unk:");
    }
  
    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
}

int main() {
    init();
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    toneRight.begin(11);
    toneLeft.begin(12);

    Serial.begin(115200);


    /* Set the delay between fresh samples */
    uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

    // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
    //                                   id, address
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

    /* Initialize the sensor */
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }

    sensors_event_t orientationData;
    uint8_t system, gyro, accel, mag = 0;
    for(;;) {
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        //printEvent(&orientationData);
        //Serial.print("x: ");
        //Serial.println(bno.getQuat().toEuler()[0]);
        //Serial.print("\y: ");
        //Serial.println(bno.getQuat().toEuler()[1]);
        //Serial.print("\z: ");
        //Serial.println(bno.getQuat().toEuler()[2]);

        float x, y, z, theta = 0.0f;
        auto e = orientationData.orientation.v;
        e[0] = e[0] * PI / 180;
        e[1] = e[1] * PI / 180;
        e[2] = e[2] * PI / 180;
        auto roll = e[2];
        auto pitch = e[1];
        auto yaw = e[0];
        x = cos(yaw)*cos(pitch);
        y = sin(yaw)*cos(pitch);
        z = sin(pitch);
        theta = atan2(y, x);
        theta = (theta * (180/PI));

        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.println();
        Serial.print("Calibration: Sys=");
        Serial.print(system);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.println(mag);

        //determine calibration based on noise
        /*
          if(gyro == 3){
            digitalWrite(8, HIGH);
            toneRight.play(NOTE_A2);
            toneLeft.play(NOTE_A2);
          }
          else if(accel == 3){
            digitalWrite(9, HIGH);
            toneRight.play(NOTE_A3);
            toneLeft.play(NOTE_A3);
          }
          else if(mag == 3){
            digitalWrite(10, HIGH);
            toneRight.play(NOTE_A4);
            toneLeft.play(NOTE_A4);
          }
        // exit calibration mode
          if (gyro == 3 && accel == 3 && mag == 3) {
            toneRight.play(NOTE_A5);
            delay(200);
            toneRight.stop();
            toneLeft.stop();
            systemCalibrated = true;
          }
          */
        

      //centering logic
      /*to do: experiment with adding delays to if-loops so that if correction needs to be made, 
      there is a delay that holds up the centering logic for a bit, allowing user to get back on center track without hearing other side's tone while making the correction */
      
        if(theta > 3.00){
         toneRight.play(NOTE_A2);
         digitalWrite(5, HIGH);
        }
        else if(theta < -3.00){
         toneLeft.play(NOTE_A5);
         digitalWrite(6, HIGH);
        }
        else if((theta > -3.00) && (theta < 3.00)){
         toneRight.stop();
         toneLeft.stop();
        }

        //Serial monitor outputs

        Serial.print("e: [");
        Serial.print(e[0]);
        Serial.print(", ");
        Serial.print(e[1]);
        Serial.print(", ");
        Serial.print(e[2]);
        Serial.print("]\n");
        Serial.print("x: ");
        Serial.print(x);
        Serial.print("    y: ");
        Serial.print(y);
        Serial.print("    z: ");
        Serial.print(z);
        Serial.print("\n");
        Serial.print("    theta: ");
        Serial.print(theta);
        Serial.print("    heading: ");
        Serial.print(orientationData.orientation.heading);
        Serial.print("\n");

        delay(50);
    }


    
    return 0;
}
