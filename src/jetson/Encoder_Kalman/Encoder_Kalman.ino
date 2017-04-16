#include <Encoder.h>

/*
class Motor
 {
 public:
 int power;
 int port;
 String name;
 
 Motor(String, int);
 String toSerial();
 };
 
 Motor::Motor(String name, int port)
 {
 this->name = name;
 this->port = port;
 this->power = 0;
 }
 
 String Motor::toSerial()
 {
 String serial = "";
 serial += "(";
 serial += this->port;
 serial += ":";
 serial += this->power;
 serial += ")";
 
 return serial;
 }
 
 String SendMotorData(Motor* motors[])
 {
 String message = "{";
 for (int i = 0; i < 10; i++)
 {
 message += motors[i]->toSerial();
 }
 message += "}";
 
 return message;
 }
 */

class Encoder_Kalman
{
public:
  Encoder *enc;
  //process variance
  float Q = 0.01;
  //measurement variance
  float R = 0.001;

  float xhat;      //a posteri estimate of x
  float P;         //a posteri error estimate
  float xhatMinus; //a priori estimate of x
  float PMinus;    //a priori error estimate
  float K;         //gain of blending factor
  float z;         //measurement
  int value;

  Encoder_Kalman(int, int);


  void Update()
  {
    //Sensor Reading
    z = enc->read();

    //time update
    xhatMinus = xhat;
    PMinus = P + Q;

    //measurement
    K = PMinus / (PMinus + R);
    xhat = xhatMinus + K * (z - xhatMinus);
    P = (1 - K) * PMinus;
    value = xhat;
  }

};

Encoder_Kalman::Encoder_Kalman(int A, int B)
{
  enc = new Encoder(A, B);
  xhat = 0;
  P = 1;
}

class Potentiometer
{
public:
  int port;
  int value;
  int minValue;
  int maxValue;

  Potentiometer(int, int, int);
  void Update();
};

Potentiometer::Potentiometer(int port, int minValue, int maxValue)
{
  this->port = port;
  pinMode(port, INPUT);
  this->minValue = minValue;
  this->maxValue = maxValue;
}

void Potentiometer::Update()
{
  this->value = analogRead(this->port);
}


//Motor *motors[10];

Encoder_Kalman leftDriveEncoder(2, 3);
Encoder_Kalman rightDriveEncoder(18, 19);

Potentiometer liftPotentiometer(A0, 0, 128);
Potentiometer clawPotentiometer(A7, 0, 128);



void setup()
{
  Serial.begin(19200);
  /*
  Serial1.begin(19200);
   motors[0] = new Motor("leftClaw",  1);
   motors[1] = new Motor("left1",     2);
   motors[2] = new Motor("left2",     3);
   motors[3] = new Motor("left3",     4);
   motors[4] = new Motor("leftLift",  5);
   motors[5] = new Motor("rightLift", 6);
   motors[6] = new Motor("right3",    7);
   motors[7] = new Motor("right2",    8);
   motors[8] = new Motor("right1",    9);
   motors[9] = new Motor("rightClaw", 10);
   */
  Serial3.begin(19200);


}


int rightPower = 128;
int leftPower = 128;

String recvMsg;

void loop()
{
  //Sensor Update
  //Create sensor class which all other sensors inherit from
  leftDriveEncoder.Update();
  rightDriveEncoder.Update();
  clawPotentiometer.Update();
  liftPotentiometer.Update();

  //Cognition
  /*
   motors[1]->power = leftPower;
   motors[2]->power = leftPower;
   motors[3]->power = leftPower;
   
   motors[6]->power = rightPower;
   motors[7]->power = rightPower;
   motors[8]->power = rightPower;
   
   //Send Motor Powers
   String controlMessage = SendMotorData(motors);
   Serial1.println(controlMessage);
   Serial.println(controlMessage);
   */
  //Serial1.println();
  //Serial.println(leftDriveEncoder.value);
  String rightEnc =  String(rightDriveEncoder.value);  
  String leftEnc = String(leftDriveEncoder.value);
  String liftPot = String(liftPotentiometer.value);
  String clawPot = String(clawPotentiometer.value);

  Serial.write('{');
  for(int i = 0; i < rightEnc.length(); i++)
  {      
    Serial.write(rightEnc[i]); 
  }
  Serial.write(',');
  for(int i = 0; i < leftEnc.length(); i++)
  {      
    Serial.write(leftEnc[i]); 
  }
  Serial.write(',');
  for(int i = 0; i < liftPot.length(); i++)
  {      
    Serial.write(liftPot[i]); 
  }
  Serial.write(',');
  for(int i = 0; i < clawPot.length(); i++)
  {      
    Serial.write(clawPot[i]); 
  }

  Serial.write('}');
  Serial.write('\n');
  //Serial1.println(analogRead(A0));
  //Serial.println(clawPotentiometer.value);
  delay(1);

  if(Serial.available())
  {

    while(Serial.available())
    {
      recvMsg += (char)Serial.read();     
      delayMicroseconds(10); 
    } 
    for(int i = 0; i < recvMsg.length(); i++)
    {
      Serial.write((char)recvMsg[i]);
      Serial3.write((char)recvMsg[i]);
      delayMicroseconds(8);
    }
    recvMsg = "";
  }

  delay(1);
}











