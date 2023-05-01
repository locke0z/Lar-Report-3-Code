#include <Keypad.h>
#include <LiquidCrystal.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
MPU6050 mpu6050(Wire);
long timer = 0;
LiquidCrystal lcd(26, 27, 14, 32, 13, 15);
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
 {'1','2','3'},
 {'4','5','6'},
 {'7','8','9'},
 {'*','0','#'}
};
byte rowPins[ROWS] = {23, 18, 5, 17};                                                       //connect to the row pinouts of the keypad 19 18 5 17
byte colPins[COLS] = {16, 4, 2};                                                 //connect to the column pinouts of the keypad16 4 2
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
String inputString;
//int inputString[16];


  float orig=0;
  float gyro=0;
  float diff=0;

void setup()
{
 Serial.begin(9600);
 Wire.begin();
 mpu6050.begin();
 mpu6050.calcGyroOffsets(true);
 inputString.reserve(10); // maximum number of digit for a number is 10, change if needed
 // set up the LCD's number of columns and rows:
 lcd.begin(16, 2);
 lcd.setCursor(0, 0);
 lcd.print("4L|6R|2F|8B");
 lcd.setCursor(0, 1);
 lcd.print("*CLR|#END");
 //lcd.autoscroll();
}
 
void loop()
{
 char key = keypad.getKey();
 if (key)
 {
  /*Serial.println(key);
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print(key);
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);*/
    if (key >= '0' && key <= '9') 
    {     // only act on numeric keys
      inputString += key;               // append new character to input string
    } 
    else if (key == '#') 
    {
      if (inputString.length() > 0) 
      {
        movement(inputString);
        inputString = "";               // clear input
        // EEEBot Moves
      }
    } 
    else if (key == '*') 
    {
        inputString = "";                 // clear input
    }
    lcd.clear();
    lcd.print(inputString);
  }
}
//long long inputInt = inputString.toInt(); // YOU GOT AN INTEGER NUMBER
void movement(String inputString)
{
  char inputChar[16];
  inputString.toCharArray(inputChar,16);
  char pre; //present digit
  int len,digits,oldpos=0,newpos=0;

    //digits=pow(10,len-1);
  //Serial.println(digits);

  len=inputString.length();
  mpu6050.update();
  gyro=mpu6050.getAngleZ();
  orig=gyro;

  //Serial.println(inputString);
      /*Serial.println("pre=");
    Serial.println(pre);
    Serial.println("inputInt=");
    Serial.println(inputInt);
    Serial.println("digits=");
    Serial.println(digits);*/
        //pre = inputInt/digits;
    //inputInt=inputInt-pre*digits;
    //digits/=10;
          //delay(1000);

  for(int i=0;i<len;i++)
  {
    pre=inputChar[i];
    if(pre=='2')
    {
      transmit(150,150,86);
      oldpos=getcount();
      while(newpos-oldpos<23)
      {
        newpos=getcount();
        //Serial.println("newpos=");
        //Serial.println(newpos);
        //Serial.println("oldpos=");
        //Serial.println(oldpos);
      }
    }  
            //mpu6050.update();
        //gyro=mpu6050.getAngleZ();
        
        //Serial.println(diff);
        //Serial.println(gyro);
        //Serial.println(orig);
    else if(pre=='4')
    {
      transmit(140,150,50);
      diff=0;
      while(abs(diff)<90)
      {
        gyro=gyroscope();
        diff=gyro-orig;
      }
      orig=gyro;
    }
    else if(pre=='6')
    {
      transmit(150,140,120);
      diff=0;
      while(abs(diff)<90)
      {
        gyro=gyroscope();
        diff=gyro-orig;
        Serial.println(gyro);
      }
      orig=gyro;
    }
          //delay(1000);

    else if(pre=='8')
    {
      transmit(-140,-140,86);
      oldpos=getcount();
      while(abs(newpos-oldpos)<23)
        newpos=getcount();
        Serial.println(newpos);
    }
  }
  transmit(0,0,86);
}

  //int encr=Wire.read();
  //int enc=(encl+encr)/2;

int getcount()
{
  Wire.requestFrom(I2C_SLAVE_ADDR,4);
  int enc=Wire.read();

  return enc;
}

float gyroscope()
{
  mpu6050.update();
  float z=mpu6050.getAngleZ();
  return z;
}

void transmit(int left,int right,int angle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((left & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(left & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((right & 0x0000FF00) >> 8));   
  Wire.write((byte)(right & 0x000000FF));          
  Wire.write((byte)((angle & 0x0000FF00) >> 8));    
  Wire.write((byte)(angle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}