#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

int pos1 = 80, pos2 = 60, pos3 = 130, pos4 = 0;

const int right_X = A2;
const int right_Y = A5;
const int right_key = 7;
const int left_X = A3;
const int left_Y = A4;
const int left_key = 8;

int x1, y1, x2, y2;
int saved1, saved2, saved3, saved4;
int previousRightKey = LOW;
int previousLeftKey = LOW;
bool hasSavedPose = false;

void zhuazi();
void zhuandong();
void xiaobi();
void dabi();

bool buttonPressed(int pin, int &previousState)
{
  int currentState = digitalRead(pin);
  bool pressed = currentState == HIGH && previousState == LOW;

  if (pressed)
  {
    delay(10);
    currentState = digitalRead(pin);
    pressed = currentState == HIGH;
  }

  previousState = currentState;
  return pressed;
}

void moveServoTo(Servo &servo, int &position, int target)
{
  while (position != target)
  {
    position += target > position ? 1 : -1;
    servo.write(position);
    delay(5);
  }
}

void setup()
{
  myservo1.attach(A1);
  myservo2.attach(A0);
  myservo3.attach(6);
  myservo4.attach(9);

  myservo1.write(pos1);
  delay(1000);
  myservo2.write(pos2);
  myservo3.write(pos3);
  myservo4.write(pos4);
  delay(1500);

  pinMode(right_key, INPUT);
  pinMode(left_key, INPUT);
  Serial.begin(9600);
}

void loop()
{
  x1 = analogRead(right_X);
  y1 = analogRead(right_Y);
  x2 = analogRead(left_X);
  y2 = analogRead(left_Y);

  if (buttonPressed(right_key, previousRightKey))
  {
    saved1 = myservo1.read();
    saved2 = myservo2.read();
    saved3 = myservo3.read();
    saved4 = myservo4.read();
    hasSavedPose = true;
    Serial.println("Pose saved");
  }

  if (buttonPressed(left_key, previousLeftKey))
  {
    if (hasSavedPose)
    {
      pos1 = myservo1.read();
      pos2 = myservo2.read();
      pos3 = myservo3.read();
      pos4 = myservo4.read();

      moveServoTo(myservo1, pos1, saved1);
      moveServoTo(myservo2, pos2, saved2);
      moveServoTo(myservo3, pos3, saved3);
      moveServoTo(myservo4, pos4, saved4);
    }
    else
    {
      Serial.println("No pose has been saved");
    }
  }

  zhuazi();
  zhuandong();
  xiaobi();
  dabi();
}

void zhuazi()
{
  if (x2 < 50)
  {
    pos4 = max(pos4 - 2, 2);
    myservo4.write(pos4);
    delay(5);
  }

  if (x2 > 1000)
  {
    pos4 = min(pos4 + 8, 90);
    myservo4.write(pos4);
    delay(5);
  }
}

void zhuandong()
{
  if (x1 < 50)
  {
    pos1 = max(pos1 - 1, 1);
    myservo1.write(pos1);
    delay(5);
  }

  if (x1 > 1000)
  {
    pos1 = min(pos1 + 1, 180);
    myservo1.write(pos1);
    delay(5);
  }
}

void xiaobi()
{
  if (y1 > 1000)
  {
    pos2 = max(pos2 - 1, 0);
    myservo2.write(pos2);
    delay(5);
  }

  if (y1 < 50)
  {
    pos2 = min(pos2 + 1, 180);
    myservo2.write(pos2);
    delay(5);
  }
}

void dabi()
{
  if (y2 > 1000)
  {
    pos3 = max(pos3 - 1, 35);
    myservo3.write(pos3);
    delay(5);
  }

  if (y2 < 50)
  {
    pos3 = min(pos3 + 1, 180);
    myservo3.write(pos3);
    delay(5);
  }
}
