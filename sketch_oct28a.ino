#include <Robojax_L298N_DC_motor.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
//--------- Flag structure --------------------------------------
typedef struct _vFlag
{
  uint8_t BTFlag = 0;
  uint8_t L298NFlag = 0;
  uint8_t HCSR04Flag = 1;
  uint8_t LEDFlag = 1;
  uint8_t ServoFlag = 0;
  uint8_t initial_Flag = 0;
  uint8_t FunctionFlag = 0;
  uint8_t back_light_Flag = 0;
  uint8_t front_light_Flag = 0;
} vFlag;
vFlag *flag_Ptr;Q1QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
vFlag flag;
//---------BT--------------------
BluetoothSerial SerialBT;
//---------------servo------------------------------
Servo myservo;  // create servo object to control a servo


// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define servoPin 13
//------LED------------------
#define LED_BUILTIN 2
//-------------L298---------------------------------------------------
// motor 1 settings
#define CHA 0
#define ENA 22 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 2
#define IN2 4
// motor 2 settings
#define IN3 18
#define IN4 19
#define ENB 23// this pin must be PWM enabled pin if Arduino board is used
#define CHB 1

const int CCW = 2; // do not change
const int CW  = 1; // do not change

#define motor1 1 // do not change
#define motor2 2 // do not change

// for two motors without debug information // Watch video instruciton for this line: https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor motors(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);
//------Bluetooth RC Controller Define ----
#define back_light 21
#define front_light 22
//-----hcsr04 sensor------------------
#define TRIGPIN_PIN 12
#define ECHO_PIN 14 
long duration;
unsigned long currentMillis = 0;
//----------global ------------------------
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // Sets speed of DC motors
int speedSet = 0;
int distance = 60;
int distanceR = 0;
int distanceL = 0;
//--------- uart structure --------------------------------------
//----------uart--------------
#define LINE_BUFFER_LENGTH 64
typedef struct _vUart
{
  char c;
  int lineIndex = 0;
  int line1Index = 0;
  int BTlineIndex = 0;
  bool lineIsComment; 
  bool lineSemiColon;
  //char *line;
  char line[128];
  //char line1[128];
  char BTline[20];
  //char R_line[20];
  //char L_line[20];
  String inputString;
  String BTinputString;
  String S1inputString;
  int V[16];
  char ctemp[30];
  char I2C_Data[80];
  int DC_Spped = 50;
  float Voltage[16];
  int Buffer[128];
  int StartCnt = 0;
  int ReadCnt = 0;
  int sensorValue = 0;
} vUart;
vUart *Uart_Ptr;
vUart Uart;
//-------------------------------------
TaskHandle_t huart;
TaskHandle_t hfunction;

void vUARTTask(void *pvParameters);
void vFunctionTask(void *pvParameters);
//------------------------------------------------------------------------------
void initial()
{
  Serial.println(F("Create Task"));
  //----------------------------------------------------------------------
  xTaskCreatePinnedToCore(
    vUARTTask, "UARTTask" // A name just for humans
    ,
    1024 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    &huart //handle
    ,
    0);

  xTaskCreatePinnedToCore(
    vFunctionTask, "FunctionTask"
    , 
    1024 // Stack size
    ,
    NULL, 1 // Priority
    ,
    &hfunction
    , 
    1);

  //----------------------------------------------------------------------

  //----------------------------------------------------------------------
}

void Forward() 
{  
  //motors.rotate(motor1, 60, CCW);
  //motors.rotate(motor2, 60, CCW);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 100);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 100);
}

void Reverse(){
  //motors.rotate(motor2, 70, CW);
  //motors.rotate(motor1, 70, CW);
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA, 100);
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
  analogWrite(ENB, 100);
}
void Left()
{
  //motors.rotate(motor1, 70, CW);
  //motors.rotate(motor2, 70, CCW); //LF
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, 100);
}
void Right()
{
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, 100);
  //motors.rotate(motor1, 70, CCW); //RF
  //motors.rotate(motor2, 70, CW);
}
void Stop() 
{
  motors.brake(1);
  motors.brake(2);
  //myservo.detach(); 
}
//-------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  Serial.println(F("init"));
  initial();
  SerialBT.begin("BT_L298N");
  myservo.setPeriodHertz(50);    
  myservo.attach(servoPin, 500, 2400); 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIGPIN_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(back_light, OUTPUT);
  pinMode(front_light, OUTPUT);
  motors.begin();
  myservo.write(90);
}
//-----------------------------------------
void loop() 
{
  Serial.print(F("Main at core:"));
  Serial.println(xPortGetCoreID());
  while(1)
  {
    if(flag.HCSR04Flag==1)
    {
      if(distance<=35)
      {
        Stop();
        delay(200);
        Reverse();
        delay(400);
        Stop();
        delay(100);
        flag.HCSR04Flag=2;
        delay(2000);
        flag.HCSR04Flag=3;
        delay(2000);
        flag.HCSR04Flag=1;
        if ((distanceR >= distanceL) ) 
        {
          Left();
          delay(700);
          Stop();
          delay(200);
          flag.HCSR04Flag=1;
        } 
        else 
        { 
          Right();
          delay(700);
          Stop();
          delay(200);
          flag.HCSR04Flag=1;
        }
        myservo.write(90);
        delay(1000);
      } 
      else 
      {
        flag.HCSR04Flag=1;
        Forward();
        delay(100);
        Stop();
        delay(30);
      }      
    }
    vTaskDelay(1);
  }
}
//----------------------------------------
void processCommand(char *data)
{
  int len, xlen, ylen, zlen, alen;
  int tempDIO;
  String stemp;

  len = Uart.inputString.length();
  //---------------------------------------
  if (strstr(data, "VER") != NULL)
  {
    Serial.println(F("ESP32_20230710"));
  }
  //-------------- HCSR04 --------------------
  if (strstr(data, "HCSR04_ON") != NULL)
  {
    flag.HCSR04Flag = 1;
    Serial.println(F("HCSR04_ON"));
  }
  if (strstr(data, "HCSR04_OFF") != NULL)
  {
    flag.HCSR04Flag = 0;
    Serial.println(F("HCSR04_OFF"));
  }  
  //----------------L298N-----------
  if (strstr(data, "F") != NULL)
  {
    Serial.println(F("Forward"));
    Forward();
    //forward();
    
  }
  if (strstr(data, "L") != NULL)
  {
    Serial.println(F("Left"));
    Left();
  }
  if (strstr(data, "R") != NULL)
  {
    Serial.println(F("Right"));
    Right();
  }
  if (strstr(data, "B") != NULL)
  {
    Serial.println(F("Reverse"));
    Reverse();
    //backward();
  }
  if (strstr(data, "S") != NULL)
  {
    Serial.println(F("Stop"));
    Stop();
  }
  //-----------------servo----------
  //-------------- Servo --------------------
  if (strstr(data, "SERVO_5")!= NULL)
  {
    Serial.println(F("SERVO_5"));
    myservo.write(5);
    //myservo.detach();
  }
  if (strstr(data, "SERVO_10")!= NULL)
  {
    Serial.println(F("SERVO_10"));
    myservo.write(10);
  }
  if (strstr(data, "SERVO_20")!= NULL)
  {
    Serial.println(F("SERVO_20"));
    myservo.write(20);
  }
  if (strstr(data, "SERVO_30")!= NULL)
  {
    Serial.println(F("SERVO_30"));
    myservo.write(30);
  }
  if (strstr(data, "SERVO_50")!= NULL)
  {
    Serial.println(F("SERVO_50"));
    myservo.write(50);
  }
  if (strstr(data, "SERVO_80")!= NULL)
  {
    Serial.println(F("SERVO_80"));
    myservo.write(80);
  }
  if (strstr(data, "SERVO_90")!= NULL)
  {
    Serial.println(F("SERVO_90"));
    myservo.write(90);
  }
  if (strstr(data, "SERVO_100")!= NULL)
  {
    Serial.println(F("SERVO_100"));
    myservo.write(100);
  }
  if (strstr(data, "SERVO_120")!= NULL)
  {
    Serial.println(F("SERVO_120"));
    myservo.write(120);
  }
  if (strstr(data, "SERVO_140")!= NULL)
  {
    Serial.println(F("SERVO_140"));
    myservo.write(140);
  }
  if (strstr(data, "SERVO_150")!= NULL)
  {
    Serial.println(F("SERVO_150"));
    myservo.write(150);
  }
}
//-----------------------------------------
//-------------------BT-----------------
void BTprocessCommand(String data)
{
  if (data =="FS")
  {
    Serial.println(F("Forward"));
    Forward();
  }
  if (data == "LS")
  {
    Serial.println(F("Left"));
    Left();
  }
  if (data == "RS")
  {
    Serial.println(F("Right"));
    Right();
  }
  if (data == "BS")
  {
    Serial.println(F("Reverse"));
    Reverse();
  }
  if (data == "S")
  {
    Serial.println(F("Stop"));
    Stop();
  }
  if (data == "X")
  {
    flag.HCSR04Flag=0;
    Serial.println(F("Stop"));
    Stop();
    flag.back_light_Flag=2;
    flag.HCSR04Flag=0;
  }
  if (data == "x")
  {
    Serial.println(F("Stop"));
    Stop();
    flag.back_light_Flag=0;
    flag.HCSR04Flag=1;
  }
  if (data == "FGFS")
  {
    //LF
    motors.rotate(motor1, 60, CCW);
    motors.rotate(motor2, 100, CCW);
  }
  if (data == "FIFS")
  {
    //RF
    motors.rotate(motor1, 100, CCW);
    motors.rotate(motor2, 60, CCW);
  }
  if (data == "BHBS")
  {
    //LB
    motors.rotate(motor1, 60, CW);
    motors.rotate(motor2, 100, CW);
  }
  if (data == "BJBS")
  {
    //RB
    motors.rotate(motor1, 100, CW);
    motors.rotate(motor2, 60, CW);
  }
  if (data == "U")
  {
    //backlight
    digitalWrite(back_light, HIGH);  
    flag.back_light_Flag=1;
    //Serial.println(F("light"));
  }
  if (data == "u")
  {
    //backlight
    digitalWrite(back_light, LOW);
    flag.back_light_Flag=0;
    //Serial.println(F("lightoff"));
  }
  if (data == "W")
  {
    digitalWrite(front_light, HIGH);  
    flag.front_light_Flag=1;
    //Serial.println(F("light"));
  }
  if (data == "w")
  {
    digitalWrite(front_light, LOW);
    flag.front_light_Flag=0;
    //Serial.println(F("lightoff"));
  }
}
//-------------------------------------------
void vUARTTask(void *pvParameters)
{
  (void)pvParameters;

  Serial.print(F("UARTTask at core:"));
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    while (Serial.available() > 0)
    {
      Uart.c = Serial.read();
  
      if ((Uart.c == '\n') || (Uart.c == '\r'))
      { // End of line reached
        if (Uart.lineIndex > 0)
        { // Line is complete. Then execute!
          Uart.line[Uart.lineIndex] = '\0'; // Terminate string
          //Serial.println( F("Debug") );
          //Serial.println( Uart.inputString );
          processCommand(Uart.line); 
          Uart.lineIndex = 0;
          Uart.inputString = "";
        }
        else
        {
          // Empty or comment line. Skip block.
        }
        Uart.lineIsComment = false;
        Uart.lineSemiColon = false;
        Serial.println(F("ok>"));
      }
      else
      {
        //Serial.println( c );
        if ((Uart.lineIsComment) || (Uart.lineSemiColon))
        {
          if (Uart.c == ')')
            Uart.lineIsComment = false; // End of comment. Resume line.
        }
        else
        {
          if (Uart.c == '/')
          { // Block delete not supported. Ignore character.
          }
          else if (Uart.c == '~')
          { // Enable comments flag and ignore all characters until ')' or EOL.
            Uart.lineIsComment = true;
          }
          else if (Uart.c == ';')
          {
            Uart.lineSemiColon = true;
          }
          else if (Uart.lineIndex >= LINE_BUFFER_LENGTH - 1)
          {
            Serial.println("ERROR - lineBuffer overflow");
            Uart.lineIsComment = false;
            Uart.lineSemiColon = false;
          }
          else if (Uart.c >= 'a' && Uart.c <= 'z')
          { // Upcase lowercase
            Uart.line[Uart.lineIndex] = Uart.c - 'a' + 'A';
            Uart.lineIndex = Uart.lineIndex + 1;
            Uart.inputString += (char)(Uart.c - 'a' + 'A');
          }
          else
          {
            Uart.line[Uart.lineIndex] = Uart.c;
            Uart.lineIndex = Uart.lineIndex + 1;
            Uart.inputString += Uart.c;
          }
        }
      }
    } //while (Serial.available() > 0)
    while (SerialBT.available())
    {
      flag.L298NFlag=1;
      String BTdata = SerialBT.readString();
      Stop();
      Serial.println(BTdata);
      BTprocessCommand(BTdata); // do something with the command
    }//while (BT.available())
    vTaskDelay(1);
  }
}
void vFunctionTask(void *pvParameters)
{
  (void)pvParameters;

  Serial.print(F("FunctionTask at core:"));
  Serial.println(xPortGetCoreID());
  for (;;) // A Task shall never return or exit.
  {
    if(flag.HCSR04Flag==1)
    {
      currentMillis = millis();
      myservo.write(90);
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10); 
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distance= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No pulse is from sensor");
      }
      else 
      {
        Serial.print("Ultrasonic sensor is shown distance:");
        Serial.print(distance);
        Serial.println("cm");
        Serial.print(distanceR-distanceL);
        Serial.println("cm");
      }
    }
    if(flag.HCSR04Flag==2)  //lookRight
    {
      myservo.write(20);
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10);  
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distanceR= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No pulse is from sensor");
      }
      else 
      {
        //Serial.print("Ultrasonic sensor is shown distanceR:");
        //Serial.print(distanceR);
        //Serial.println("cm");
      }
    }
    if(flag.HCSR04Flag==3)  //lookLeft
    {
      myservo.write(160);
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10);  
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distanceL= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No pulse is from sensor");
      }
      else 
      {
        //Serial.print("Ultrasonic sensor is shown distanceL:");
        //Serial.print(distanceL);
        //Serial.println("cm");
      }
    }
    vTaskDelay(1);
  }
}
