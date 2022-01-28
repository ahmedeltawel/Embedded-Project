// --------------------------------------------------
// ## (motor - light sensor) - (back blinkers) ## Bottom
// --------------------------------------------------

//Project-1: //ARDUINO LINE FOLLOWING CAR - QUAD Robotics - A unit of Quad Store//
//www.quadstore.in

// YOU HAVE TO INSTALL THE AFMOTOR LIBRARY BEFORE UPLOAD THE CODE//
// GO TO SKETCH >> INCLUDE LIBRARY >> ADD .ZIP LIBRARY >> SELECT AF MOTOR ZIP FILE //

//including the libraries
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <AFMotor.h>

//defining pins and variables
#define lefts A0
#define rights A1
#define lightSensor A3

// top send, bottom receive

//Motors
int motor1pin1 = 31;
int motor1pin2 = 33;

int motor2pin1 = 26;
int motor2pin2 = 24;

int rwheel = 12;
int lwheel = 11;
int rwarning = 51;
int lwarning = 13;

int OnLED = 5;
int LED1 = 40;
int led_pin1 = 5;
int led_pin2 = 6;

//int LED2 =;

//defining motors
//AF_DCMotor motor1(1, MOTOR12_1KHZ);
//AF_DCMotor motor2(2, MOTOR12_1KHZ);
//AF_DCMotor motor3(3, MOTOR34_1KHZ);
//AF_DCMotor motor4(4, MOTOR34_1KHZ);

void t1(void *params); // motor - linefollowers - blinker warning
void t2(void *params); // lightsenor - headlights

void setup()
{
  Wire.begin( 5 ) ; 
  Wire.onReceive( receiveEvent ) ;
  xTaskCreate(t1, "motor - linefollowers - blinker warning", 1000, NULL, 2, NULL);
  xTaskCreate(t2, "lightsenor - headlights", 1000, NULL, 1, NULL);

  //Setting the motor
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(led_pin1, OUTPUT);
  pinMode(led_pin2, OUTPUT);

  pinMode(rwarning, OUTPUT);
  pinMode(lwarning, OUTPUT);

  pinMode(lwheel, OUTPUT);
  pinMode(rwheel, OUTPUT);

  analogWrite(lwheel, 220); //ENA pin
  analogWrite(rwheel, 220); //ENB pin

  pinMode(LED1, OUTPUT);
  //  pinMode(LED2, OUTPUT);

  //  motor1.setSpeed(180);
  //  motor2.setSpeed(180);
  //  motor3.setSpeed(180);
  //  motor4.setSpeed(180);

  //Declaring PIN input types
  pinMode(lefts, INPUT);
  pinMode(rights, INPUT);
  pinMode(OnLED, INPUT);

  //Begin serial communication
  Serial.begin(9600);
}


void receiveEvent(int bytes ) {
  int rxdata = Wire.read();
  Serial.println(rxdata);
   if(rxdata==2){
         analogWrite(led_pin1, 255);
         analogWrite(led_pin2, 255);
//         txdata = 2;
     }
     else if(rxdata==1){
        analogWrite(led_pin1, 50);
        analogWrite(led_pin2, 50);
//        txdata = 1;
     } else {
        analogWrite(led_pin1, 0);
        analogWrite(led_pin2, 0);
//        txdata = 0;
     }
  
}

int t1delay = 20;
void t1(void *params) // motor - line followers - blinker warning
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(t1delay);
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    //Serial.println("T1 Running");
    //line detected by none
    if (analogRead(lefts) <= 350 && analogRead(rights) <= 350)
    {
      //Forward
      digitalWrite(lwarning, LOW);
      digitalWrite(rwarning, LOW);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor2pin2, HIGH);
      digitalWrite(motor2pin1, LOW);
      //    analogWrite(lwheel, 200); //ENA pin
      //  analogWrite(rwheel, 200); //ENB pin
    }
    //line detected by left sensor
    else if (analogRead(lefts) <= 350 && !analogRead(rights) <= 350)
    {
      //turn left
      digitalWrite(lwarning, HIGH);
      digitalWrite(rwarning, LOW);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor2pin2, LOW);
      digitalWrite(motor2pin1, HIGH);
      //  analogWrite(lwheel, 220); //ENA pin
    }
    //line detected by right sensor
    else if (!analogRead(lefts) <= 350 && analogRead(rights) <= 350)
    {
      //turn right
      digitalWrite(rwarning, HIGH);
      digitalWrite(lwarning, LOW);
      digitalWrite(motor1pin2, LOW);
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor2pin2, HIGH);
      digitalWrite(motor2pin1, LOW);
      //  analogWrite(rwheel, 220); //ENA pin
    }
    //line detected by both
    else if (!analogRead(lefts) <= 350 && !analogRead(rights) <= 350)
    {
      //stop
      digitalWrite(lwarning, HIGH);
      digitalWrite(rwarning, HIGH);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor2pin2, HIGH);
      digitalWrite(motor2pin1, LOW);
      //    analogWrite(lwheel, 200); //ENA pin
      //  analogWrite(rwheel, 200); //ENB pin
    }
  vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

int t2delay = 100;
void t2(void *params) // lightsenor - headlights
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(t2delay);
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    //Serial.println("T2 Running");
    //Serial.println(analogRead(lightSensor));
    
    //     Serial.println(analogRead(lightSensor));
    //Serial.println(digitalRead(OnLED));
    //Printing values of the sensors to the serial monitor
    if (analogRead(OnLED) == 1)
    {
      digitalWrite(LED1, HIGH);
      //    digitalWrite(LED2, HIGH);
    }
    else
    {
      digitalWrite(LED1, LOW);
      //    digitalWrite(LED2, LOW);
    }

   
    
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void loop()
{
  // empty
}
