#include <Servo.h>

#define xMin 42
#define xZero 50
#define xMax 58
#define yMin 98
#define yZero 104
#define yMax 110

Servo xServo, yServo;
int tempAngle = 0, xAngle = xZero, yAngle = yZero;
boolean xAngleFlag, yAngleFlag;

void setup() {
  // put your setup code here, to run once:
  xServo.attach(2);
  yServo.attach(3);
  Serial.begin(9600);

}

void loop() {
  
  //Obtain serial data
  if(Serial.available()>0)
  {
    //Read new angle
    tempAngle = getSerial();
  
    //Determine the angle received with flag and offset.
    if(xAngleFlag)
    {
      //xAngle = xMax - tempAngle; not used
      xAngle = xMin + tempAngle;     
      //xAngle = tempAngle;
    }
    else if(yAngleFlag)
    {
      yAngle = yMax - tempAngle;
      //yAngle = tempAngle;
    }
  
    //Print test angle
    Serial.print("x: ");
    Serial.println(xAngle);
    Serial.print("y: ");
    Serial.println(yAngle);
  }
  
  //if(xAngle > xMax)
  //  xAngle = xMax;
  //else if(xAngle < xMin)
  //  xAngle = xMin;
    
  //if(yAngle > yMax)
  //  yAngle = yMax;
  //else if(yAngle < yMin)
  //  yAngle = yMin;
    
  xServo.write(xAngle);
  yServo.write(yAngle);
}

//int getSerial()
//Reads from serial and decodes the bytes sent by the PC
//and translates them to the output angles. Sets flags for
//the angle axis that it received.
//@return the angle number.
int getSerial()
{
  int serialData = 0;
  int inByte = 0;
  
  //Repeat until end of angle message.
  while (inByte != '/')
  {
    //Read new byte
    inByte = Serial.read();
    
    //Set x angle flag when x header.
    if(inByte > 0 && inByte == 'x')
    {
      xAngleFlag = true;
      yAngleFlag = false;
    }
    //Set y angle flag when y header.
    else if(inByte > 0 && inByte == 'y')
    {
      xAngleFlag = false;
      yAngleFlag = true;
    }
    //Get angle number.
    else if(inByte > 0 && inByte != '/')
    {
      serialData = (serialData * 10) + inByte - '0';
    }
  }
  
  return serialData;
}
