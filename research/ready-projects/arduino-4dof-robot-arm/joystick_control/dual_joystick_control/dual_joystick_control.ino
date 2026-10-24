#include <Servo.h>  // add the servo libraries
Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;

int pos1=80, pos2=60, pos3=130, pos4=0;  // define the variable of 4 servo angle,and assign the initial value (that is the boot posture angle value)

const int right_X = A2; // define the right X pin to A2
const int right_Y = A5; // define the right Y pin to A5
const int right_key = 7; // define the right key pin to 7（that is the value of Z）

const int left_X = A3; // define the left X pin to A3
const int left_Y = A4;  // define the left X pin to A4
const int left_key = 8; //define the left key pin to 8（that is the value of Z）

int x1,y1,z1;  // define the variable, used to save the joystick value it read.
int x2,y2,z2;

void zhuazi();
void zhuandong();
void xiaobi();
void dabi();


void setup()
{
  myservo1.attach(A1);  // base rotation servo
  myservo2.attach(A0);  // right-side arm servo
  myservo3.attach(6);   // left-side arm servo
  myservo4.attach(9);   // gripper servo

  // boot posture
  myservo1.write(pos1);
  delay(1000);
  myservo2.write(pos2);
  myservo3.write(pos3);
  myservo4.write(pos4);
  delay(1500);

  pinMode(right_key, INPUT);   // set the right/left key to INPUT
  pinMode(left_key, INPUT);
  Serial.begin(9600); //  set the baud rate to 9600
}

void loop()
{
  x1 = analogRead(right_X); //read the right X value
  y1 = analogRead(right_Y);  // read the right Y value
  z1 = digitalRead(right_key);  //// read the right Z value

  x2 = analogRead(left_X);  //read the left X value
  y2 = analogRead(left_Y);  //read the left Y value
  z2 = digitalRead(left_key);  // read the left Z value
  //delay(5);  // lower the speed overall

  // claw
  zhuazi();
  // rotate
  zhuandong();
  // upper arm
  xiaobi();
  //lower arm
  dabi();
}

//claw
void zhuazi()
{
    //claw
  if(x2<50) // if push the left joystick to the right
  {
      pos4=max(pos4-2, 2);  // close without commanding beyond the calibrated limit
      //Serial.println(pos4);
      myservo4.write(pos4);  // servo 4 operates the action, claw is gradually closed.
      delay(5);
   }
  if(x2>1000) //// if push the left joystick to the left
  {
      pos4=min(pos4+8, 108); // open without commanding beyond the calibrated limit
      //Serial.println(pos4);
      myservo4.write(pos4); // servo 4 operates the motion, the claw gradually opens.
      delay(5);
  }
}
//******************************************************
 // turn
void zhuandong()
{
  if(x1<50)  // if push the right joystick to the right
  {
    pos1=max(pos1-1, 1);  // limit the right-turn angle before commanding it
    myservo1.write(pos1);  // servo 1 operates the motion, the arm turns right.
    delay(5);
  }
  if(x1>1000)  // if push the right joystick to the let
  {
    pos1=min(pos1+1, 180);  // limit the left-turn angle before commanding it
    myservo1.write(pos1);  // arm turns left
    delay(5);
  }
}

//**********************************************************/
//upper arm
void xiaobi()
{
    if(y1>1000) // if push the right joystick upward
  {
    pos2=max(pos2-1, 0);
    myservo2.write(pos2); // the upper arm will lift
    delay(5);
  }
  if(y1<50)  // if push the right joystick downward
  {
    pos2=min(pos2+1, 180);
    myservo2.write(pos2);  // upper arm will go down
    delay(5);
  }
}

//*************************************************************/
// lower arm
void dabi()
{
    if(y2<50)  // if push the left joystick upward
  {
    pos3=min(pos3+1, 180);
    myservo3.write(pos3);  // lower arm will stretch out
    delay(5);
  }
  if(y2>1000)  // if push the left joystick downward
  {
    pos3=max(pos3-1, 35);
    myservo3.write(pos3);  // lower arm will draw back
    delay(5);
  }
}
