// --------------------------------------------------
// ## (lcd - mp3) - (7 segment - joystick) ## TOP ### I2C
// --------------------------------------------------

#include <Wire.h>
#define lightSensor A13

//Eduarduino  - Mp3 com touch screen - DFPlayer Mini e LCD ILI9341
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#define LCD_CS A3    // Chip Select goes to Analog 3
#define LCD_CD A2    // Command/Data goes to Analog 2
#define LCD_WR A1    // LCD Write goes to Analog 1
#define LCD_RD A0    // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h" // Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
//#include <Adafruit_TFTLCD.h>
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Adafruit_GFX.h>
#include <Adafruit_TFTLCD.h>
#include <TouchScreen.h>
#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h" // Hardware-specific library
#include <MCUFRIEND_kbv.h>

#define XP 9
#define XM A3
#define YP A2
#define YM 8

#define TS_LEFT 879
#define TS_RT 74
#define TS_TOP 117
#define TS_BOT 894

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 100);
//Inicia a serial por software nos pinos 10 e 11
//SoftwareSerial mySoftwareSerial(50, 51); // RX, TX
// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 53; // Connects to module's RX
static const uint8_t PIN_MP3_RX = 52; // Connects to module's TX
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

DFRobotDFPlayerMini myDFPlayer;

//Definicao de cores
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

//PP_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
//Adafruit_TFTLCD tft(A3, A2, A1, A0, A4);

int txdata = 0;

// Armazena o estado dos botões
bool valor_botao1 = 0;
bool valor_botao2 = 0;
bool valor_botao3 = 0;
bool valor_botao4 = 0;
bool valor_botao5 = 0;
bool valor_botao6 = 0;
bool MP3Running =0;

int volume;
int v;
int equalizacao;

#define MINPRESSURE 10
#define MAXPRESSURE 1000

void t1(void *params); // lcd - mp3
void t2(void *params); // 7segs - joystick

void setup(void)
{
  Wire.begin(); //Master
  
  xTaskCreate(t1, "lcd-mp3", 1000, NULL, 1, NULL);
  xTaskCreate(t2, "7seg-joystick", 1000, NULL, 2, NULL);

  pinMode(25, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);

  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  Serial.begin(9600);

  softwareSerial.begin(9600);

  uint32_t when = millis();
  //    while (!Serial) ;   //hangs a Leonardo until you connect a Serial
  if (!Serial)
    delay(5000); //allow some time for Leonardo
  Serial.println("Serial took " + String((millis() - when)) + "ms to start");
  //    tft.reset();                 //hardware reset
  uint16_t ID = tft.readID(); //
  Serial.print("ID = 0x");
  Serial.println(ID, HEX);
  if (ID == 0xD3D3)
    ID = 0x9481; // write-only shield
                 //    ID = 0x9329;                             // force ID
  tft.begin(ID);

  delay(5000);
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);

  //Verifica se o modulo esta respondendo e se o
  //cartao SD foi encontrado
  tft.println("DFRobot DFPlayer Mini");
  tft.println("Initializing DFPlayer...");

  if (!myDFPlayer.begin(softwareSerial))
  {
    tft.println("Not Initialized:");
    tft.println("1.Check the DFPlayer Mini connections");
    tft.println("2.Insert an SD card");
    while (true)
      ;
  }
  tft.println(" ");
  tft.println("DFPlayer Mini Initialized");
  //Definicoes iniciais
  myDFPlayer.setTimeOut(500); //Timeout serial 500ms
  myDFPlayer.volume(4);       //Volume 5
  myDFPlayer.EQ(0);           //Equalizacao normal

  tft.print("Files on SD CardFiles on SD Card: ");
  tft.println(myDFPlayer.readFileCounts(DFPLAYER_DEVICE_SD));

  delay(5000);
  tft.fillScreen(BLACK);

  //Botão Play
  tft.fillCircle(160, 90, 38, RED);
  tft.fillCircle(160, 90, 35, WHITE);
  tft.fillTriangle(150, 104, 150, 74, 180, 89, BLACK);

  //Botão Prev
  tft.fillCircle(80, 90, 28, RED);
  tft.fillCircle(80, 90, 25, WHITE);
  //tft.fillTriangle(x2, y2, x1, y1, x0, y0, BLACK);
  tft.fillTriangle(82, 99, 82, 79, 67, 89, BLACK);
  tft.fillTriangle(97, 99, 97, 79, 82, 89, BLACK);
  //tft.fillRect(x0,y0,L, A,BLUE);
  tft.fillRect(62, 79, 5, 20, BLACK);

  //Botão Next
  tft.fillCircle(240, 90, 28, RED);
  tft.fillCircle(240, 90, 25, WHITE);
  //tft.fillTriangle( x2, y2,  x1, y1,  x0, y0, BLACK);
  tft.fillTriangle(240, 99, 240, 79, 255, 89, BLACK);
  tft.fillTriangle(225, 99, 225, 79, 240, 89, BLACK);
  tft.fillRect(255, 79, 5, 20, BLACK);

  //Barra de Volume
  tft.drawRoundRect(50, 180, 220, 20, 0, WHITE);
  tft.fillRect(51, 181, 218, 18, BLACK);

  //Botão Volume -
  tft.drawRoundRect(20, 180, 20, 20, 0, WHITE);
  tft.setCursor(25, 182);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("-");

  //Status da barra de volume
  volume = myDFPlayer.readVolume();
  volume = map(volume, 0, 30, 0, 218);
  tft.fillRect(51, 181, volume, 18, RED);

  //Botão Volume +

  tft.drawRoundRect(280, 180, 20, 20, 0, WHITE);
  tft.setCursor(285, 182);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("+");

  //Barra de Equalizer
  tft.fillRect(51, 21, 218, 18, BLACK);
  tft.setCursor(25, 22);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Equalizer: Normal");

  //Botão Volume + Equalizer

  tft.drawRoundRect(280, 20, 20, 20, 0, RED);
  tft.setCursor(285, 22);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("+");
}

int t1delay = 100;
void t1(void *params) // lcd - mp3
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(t1delay);
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    Serial.println("T1 Running");
    
    if(analogRead(lightSensor)>=500){
         //analogWrite(led_pin1, 255);
         //analogWrite(led_pin2, 255);
         txdata = 2;
     }
     else if(analogRead(lightSensor) >=250){
        //analogWrite(led_pin1, 50);
        ///analogWrite(led_pin2, 50);
        txdata = 1;
     } else {
        //analogWrite(led_pin1, 0);
        //analogWrite(led_pin2, 0);
        txdata = 0;
     }

  


    
    TSPoint p = ts.getPoint();
    pinMode(XM, OUTPUT);
    digitalWrite(XM, LOW);
    pinMode(YP, OUTPUT);
    digitalWrite(YP, HIGH);
    pinMode(YM, OUTPUT);
    digitalWrite(YM, LOW);
    pinMode(XP, OUTPUT);
    digitalWrite(XP, HIGH);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
    {
      p.x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
      p.y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
      Serial.print(" px: ");
      Serial.print(p.x);
      Serial.print("py: ");
      Serial.println(p.y);

      //Testa botao Play
      if (p.x > 100 & p.x<160 & p.y> 80 & p.y < 150)
      {
        if (valor_botao1 == 0)
        {
          tft.fillCircle(160, 90, 35, RED);
          tft.fillRect(148, 74, 10, 35, BLACK);
          tft.fillRect(163, 74, 10, 35, BLACK);
          valor_botao1 = !valor_botao1;
          MP3Running = 1;
          myDFPlayer.start();
          delay(100);
          tft.fillCircle(160, 90, 35, WHITE);
          tft.fillRect(148, 74, 10, 35, BLACK);
          tft.fillRect(163, 74, 10, 35, BLACK);
          faixa();
        }
        else
        {
          tft.fillCircle(160, 90, 35, RED);
          tft.fillTriangle(150, 104, 150, 74, 180, 89, BLACK);
          MP3Running = 0;
          myDFPlayer.pause();
          valor_botao1 = !valor_botao1;
          delay(100);
          tft.fillCircle(160, 90, 35, WHITE);
          tft.fillTriangle(150, 104, 150, 74, 180, 89, BLACK);
        }
      }

      //Testa botao Prev
      if (p.x > 30 & p.x<70 & p.y> 68 & p.y < 120)
      {
        if (valor_botao2 == 0 && MP3Running == 1)
        {
          tft.fillCircle(80, 90, 25, RED);
          tft.fillTriangle(82, 99, 82, 79, 67, 89, BLACK);
          tft.fillTriangle(97, 99, 97, 79, 82, 89, BLACK);
          tft.fillRect(62, 79, 5, 20, BLACK);
          myDFPlayer.previous();
          valor_botao2 = !valor_botao2;
          delay(100);
          tft.fillCircle(80, 90, 25, WHITE);
          tft.fillTriangle(82, 99, 82, 79, 67, 89, BLACK);
          tft.fillTriangle(97, 99, 97, 79, 82, 89, BLACK);
          tft.fillRect(62, 79, 5, 20, BLACK);
          faixa();
        }
        else
        {
          valor_botao1 == 0;
          valor_botao2 = !valor_botao2;
        }
      }

      //Testa botao Next
      if (p.x > 185 & p.x<239 & p.y> 68 & p.y < 120)
      {
        if (valor_botao3 == 0 && MP3Running == 1)
        {
          tft.fillCircle(240, 90, 25, RED);
          tft.fillTriangle(240, 99, 240, 79, 255, 89, BLACK);
          tft.fillTriangle(225, 99, 225, 79, 240, 89, BLACK);
          tft.fillRect(255, 79, 5, 20, BLACK);
          myDFPlayer.next();
          valor_botao3 = !valor_botao3;
          delay(100);
          tft.fillCircle(240, 90, 25, WHITE);
          tft.fillTriangle(240, 99, 240, 79, 255, 89, BLACK);
          tft.fillTriangle(225, 99, 225, 79, 240, 89, BLACK);
          tft.fillRect(255, 79, 5, 20, BLACK);
          faixa();
        }
        else
        {
          valor_botao1 == 0;
          valor_botao3 = !valor_botao3;
        }
      }

      //Testa botão volume +
      if (p.x > 249 & p.x<279 & p.y> 188 & p.y < 206)
      {
        if (valor_botao4 == 0)
        {
          myDFPlayer.volumeUp();
          tft.fillRect(281, 181, 18, 18, RED);
          volume = myDFPlayer.readVolume();
          volume = map(volume, 0, 30, 0, 218);
          valor_botao4 = !valor_botao4;
          v = volume;

          delay(100);
          tft.fillRect(281, 181, 18, 18, BLACK);
          tft.drawRoundRect(280, 180, 20, 20, 0, WHITE);
          tft.setCursor(285, 182);
          tft.setTextColor(WHITE);
          tft.setTextSize(2);
          tft.println("+");

          tft.fillRect(51, 181, 218, 18, BLACK);
          tft.fillRect(51, 181, v, 18, RED);
        }
        else
        {
          valor_botao1 == 0;
          valor_botao4 = !valor_botao4;
        }
      }

      //Testa botão volume -
      if (p.x > -13 & p.x<11 & p.y> 188 & p.y < 206)
      {
        if (valor_botao5 == 0)
        {
          myDFPlayer.volumeDown();
          tft.fillRect(21, 181, 18, 18, RED);
          volume = myDFPlayer.readVolume();
          volume = map(volume, 0, 30, 0, 218);
          valor_botao5 = !valor_botao5;
          delay(100);
          v = volume;

          tft.fillRect(21, 181, 18, 18, BLACK);
          tft.setCursor(25, 182);
          tft.setTextColor(WHITE);
          tft.setTextSize(2);
          tft.println("-");

          tft.fillRect(51, 181, 218, 18, BLACK);
          tft.fillRect(51, 181, v, 18, RED);
        }
        else
        {
          valor_botao1 == 0;
          valor_botao5 = !valor_botao5;
        }
      }

      //Testa botão volume + Equalizer
      if (p.x > 250 & p.x<277 & p.y> 20 & p.y < 42)
      {
        if (valor_botao6 == 0)
        {

          tft.fillRect(281, 21, 18, 18, RED);
          tft.setCursor(285, 22);
          tft.setTextColor(RED);
          tft.setTextSize(2);
          tft.println("+");
          valor_botao6 = !valor_botao6;
          delay(100);
          tft.fillRect(281, 21, 18, 18, BLACK);
          tft.drawRoundRect(280, 20, 20, 20, 0, RED);
          tft.setCursor(285, 22);
          tft.setTextColor(RED);
          tft.setTextSize(2);
          tft.println("+");

          equalizacao++;
          myDFPlayer.EQ(equalizacao);

          if (equalizacao == 6)
          {
            equalizacao = 0;
          }
          if (equalizacao == 0)
          {
            tft.fillRect(51, 21, 218, 18, BLACK);
            tft.setCursor(25, 22);
            tft.setTextColor(RED);
            tft.setTextSize(2);
            tft.println("Equalizer: Normal");
          }
          if (equalizacao == 1)
          {
            tft.fillRect(51, 21, 218, 18, BLACK);
            tft.setCursor(25, 22);
            tft.setTextColor(RED);
            tft.setTextSize(2);
            tft.println("Equalizer: POP");
          }
          if (equalizacao == 2)
          {
            tft.fillRect(51, 21, 218, 18, BLACK);
            tft.setCursor(25, 22);
            tft.setTextColor(RED);
            tft.setTextSize(2);
            tft.println("Equalizer: Rock");
          }
          if (equalizacao == 3)
          {
            tft.fillRect(51, 21, 218, 18, BLACK);
            tft.setCursor(25, 22);
            tft.setTextColor(RED);
            tft.setTextSize(2);
            tft.println("Equalizer: Jazz");
          }
          if (equalizacao == 4)
          {
            tft.fillRect(51, 21, 218, 18, BLACK);
            tft.setCursor(25, 22);
            tft.setTextColor(RED);
            tft.setTextSize(2);
            tft.println("Equalizer: Classic");
          }
          if (equalizacao == 5)
          {
            tft.fillRect(51, 21, 218, 18, BLACK);
            tft.setCursor(25, 22);
            tft.setTextColor(RED);
            tft.setTextSize(2);
            tft.println("Equalizer: Bass");
          }
        }
        else
        {
          valor_botao1 == 0;
          valor_botao6 = !valor_botao6;
        }
      }
    }
     vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

int t2delay = 20;
void t2(void *params) // 7segs - joystick
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(t2delay);
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    Serial.println("T2 Running");
    Serial.println(digitalRead(44));
    
    
    if (analogRead(A8) > 1000 && analogRead(A9) > 500)
    {
      digitalWrite(25, HIGH); //MID
      digitalWrite(27, LOW);  //LEFT TOP
      digitalWrite(29, LOW);  //TOP

      digitalWrite(31, LOW); //RIGHT TOP
      digitalWrite(33, LOW); //LEFT BOTTOM
      digitalWrite(35, LOW); //BOTTOM
      digitalWrite(37, LOW); //RIGHT BOTTOM
    }

    if (analogRead(A8) >= 0 && analogRead(A8) <= 50 && analogRead(A9) > 500)
    {
      digitalWrite(25, HIGH); //MID
      digitalWrite(27, LOW);  //LEFT TOP
      digitalWrite(29, LOW);  //TOP

      digitalWrite(31, HIGH); //RIGHT TOP
      digitalWrite(33, LOW);  //LEFT BOTTOM
      digitalWrite(35, LOW);  //BOTTOM
      digitalWrite(37, HIGH); //RIGHT BOTTOM
    }

    if (analogRead(A8) > 500 && analogRead(A9) >= 0 && analogRead(A9) <= 50)
    {
      digitalWrite(25, HIGH); //MID
      digitalWrite(27, HIGH); //LEFT TOP
      digitalWrite(29, HIGH); //TOP

      digitalWrite(31, HIGH); //RIGHT TOP
      digitalWrite(33, HIGH); //LEFT BOTTOM
      digitalWrite(35, LOW);  //BOTTOM
      digitalWrite(37, LOW);  //RIGHT BOTTOM
    }

    if (analogRead(A8) > 500 && analogRead(A9) > 1000)
    {
      digitalWrite(25, LOW);  //MID
      digitalWrite(27, HIGH); //LEFT TOP
      digitalWrite(29, HIGH); //TOP

      digitalWrite(31, HIGH); //RIGHT TOP
      digitalWrite(33, HIGH); //LEFT BOTTOM
      digitalWrite(35, HIGH); //BOTTOM
      digitalWrite(37, HIGH); //RIGHT BOTTOM
    }
    
     vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void loop()
{
  // empty
    Wire.beginTransmission( 5 );
    Wire.write(txdata) ;
    Wire.endTransmission(); 
    delay(200);
}

void faixa()
{

  tft.fillRect(109, 145, 180, 20, BLACK);
  tft.setCursor(100, 145);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Song: ");
  tft.print(myDFPlayer.readCurrentFileNumber());
  tft.print("/");
  tft.println(myDFPlayer.readFileCounts());
}
