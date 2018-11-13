/*This software is for the Alpha robot, so far it can only follow black lines on any surface using a PD controller */
#include <QTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2
#define right_max 200
#define left_max 200
#define ENA 10
#define ENB 11


QTRSensorsRC qtrrc((unsigned char[]) {
  2, 3, 4, 5, 6, 7, 8, 9
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() {
  delay(500);
  /* This loop is used to calibrate all 8 sensors, thus allowing operation on any surface*/
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  /*// print the calibration minimum values measured when emitters were on
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
    }
    //Serial.println();
    //Serial.println();
    //delay(1000);*/


}



void loop() {

  /* Values initialiazation step*/
  int lastError = 0;
  int m1speed;
  int m2speed;
  int M1 = 30;
  int M2 = 30;
  int motor_correction = 0;
  int KP = 3;
  int KD = 20;
  int error = 0;
  while (true) {
    /*Serial.print("Motor1 Speed= ");
      Serial.println(m1speed);
      Serial.print(' ');
      Serial.print("Motor2 Speed= ");
      Serial.println(m2speed);
      Serial.println();

      Serial.print("Error= ");
      Serial.println(error);
      Serial.print(' ');*/
    unsigned int position = qtrrc.readLine(sensorValues); //get the position from the sensors

    /*Serial.print("Position= ");
      Serial.println(position); // comment this line out if you are using raw values
      Serial.print(' ');*/

    error = position - 3550; // get the error between the desired position (3550) and the actual position

    motor_correction = KP * error + KD * (error - lastError); // use that error into the PD controller to estimate the correction to be applied to the motors
    lastError = error;

    /*adjusting the speeed of the two motors to allow turing*/
    m1speed = M1 - motor_correction;
    m2speed = M2 + motor_correction;

    /*condition to prevent the motors' speed from dropping in the negative or rising over the maximum*/
    if (m1speed < 0)
      m1speed = 0;
    if (m2speed < 0)
      m2speed = 0;
    if (m1speed > left_max)
      m1speed = left_max;
    if (m2speed > right_max)
      m2speed = right_max;


    /*write the speed to the motors through PWM pins*/
    analogWrite(ENA, m1speed);
    analogWrite(ENB, m2speed);
  }
}
