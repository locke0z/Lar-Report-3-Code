#include <Wire.h>
#include <Servo.h>

 int leftMotor_speed, rightMotor_speed, servoAngle;
 float k=0;
 int centreAngle=82;
 int baseSpeed=125;

 Servo myservo;        // create servo object to control a servo
#define enA 5   // EnableA command line - should be a PWM pin
#define enB 6   // EnableB command line - should be a PWM pin
#define servoPin 4


// name the motor control pins - replace the ** with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  // Channel A direction 
#define INb A1  // Channel A direction 
#define INc A2  // Channel B direction 
#define INd A3  // Channel B direction 

void setup() {
  Serial.begin(9600);// open the serial port at 9600 bps:
  Wire.begin(0x07); //Set Arduino up as an I2C slave at address 0x07
  //Wire.onRequest(requestEvent); //Prepare to send data
  Wire.onReceive(receiveEvent); //Prepare to recieve data
    myservo.attach(servoPin);  // attach our servo object to pin D4
  // the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  // configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

}

void loop() {
   

}
/*
void requestEvent()
{
  unsigned char char_ar[16] = "Hi Raspberry Pi"; //Create String
  Wire.write(char_ar,16); //Write String to Pi.
}
*/

void receiveEvent(int numBytes){
  //Set Up Vars
  //Serial.println("test");
  int receive_int=0;

  int count=0;
  //We'll recieve one byte at a time. Stop when none left
  while(Wire.available())
  {
    char c = Wire.read();    // receive a byte as character
    //Create Int from the Byte Array
    receive_int = c << (8 * count) | receive_int;
    count++;
  }
  //Print the Int out.
  Serial.print("Received Number: "); 
  Serial.println(receive_int);

if (receive_int==99)
{
 leftMotor_speed=-100;
 rightMotor_speed=-100;
 servoAngle=82;
}
 else{
  servoAngle=centreAngle+receive_int;
  float k=0.3;
  leftMotor_speed=baseSpeed+(k*receive_int);
  rightMotor_speed=baseSpeed-(k*receive_int);
 }


  analogWrite(enA, abs(leftMotor_speed));
  analogWrite(enB, abs(rightMotor_speed));
  myservo.write(servoAngle);
  if (leftMotor_speed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);    
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotor_speed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);    
  }

}