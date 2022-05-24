//OLED display
#include <Adafruit_ssd1306syp.h>
#define SDA_PIN 5
#define SCL_PIN 4
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);

#include <PID_v1.h>

//STATES
typedef enum VEGGIE_STATE
{
  VEGGIE_STATE_IDLE,//0
  VEGGIE_STATE_FILL,//1
  VEGGIE_STATE_FILL_HEAT,//2
  VEGGIE_STATE_PID_HEAT,//3
  VEGGIE_STATE_ERROR,
  VEGGIE_STATE_IDLE_STOP
} veggieState_t;
veggieState_t veggieState = VEGGIE_STATE_IDLE;

//PINS
const int analogInPin = A1;
const int pumpPin = 3;
const int ssrPin = 6;
const int eStop = 7;

//Pressure Sensor Variables
int rawIn = 0;        
int offset = 78; //calibrate for 0.5 as zero pressure
int fullScale = 819; // ~4.5volt max pressure (span) adjust (~798-840)
double pressure; // final pressure in psi
#define MAX_PRESSURE_LIMIT 2.7
#define MIN_PRESSURE_LIMIT 0.3
#define FILL_HANDOFF_HEAT 0.6
#define FILL_HANDOFF_PID 1.2
#define SET_PRESSURE 2.0

//PID Variables
#define PID_SAMPLE_TIME 500//200
#define WINDOW_SIZE_MAX 255
#define WINDOW_SIZE_MIN 0
double setpoint;
double output;
double kp = 500;
double ki = 0.025;
double kd = 3;

PID veggiePID(&pressure, &output, &setpoint, kp, ki, kd, DIRECT);

//timers
unsigned long displayTimer = 0;
const unsigned long DTIME = 500; //update every X'ms

//Pump Duty cycle
//int dutyCycleTotal = 5000; //~1KHz
//int dutyCycle = 0; //place to store the dutycycle


void displayTest() {
  display.initialize();
  display.clear();//Serial.println("debug display...");
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Welcome");
  display.update();
  delay(2000);
}

void readPSI(){
  rawIn = analogRead(analogInPin);
  pressure = (rawIn - offset) * 30.0 / (fullScale - offset); // pressure conversion
  /*Serial.print(rawIn);
  Serial.print(" ");
  Serial.print(pressure, 2); // one decimal place
  Serial.println(" psi");
  Serial.print(setpoint, 1); // one decimal place
  Serial.println("  set");
  Serial.print(output, 1); // one decimal place
  Serial.println("  out");*/

  if (pressure < 0) {
    pressure = 0.0;
  }
  display.clear();
  display.setCursor(0,0);
  display.setTextSize(3);
  display.print(pressure,1);
  display.println(" PSI");
  display.setTextSize(2);
  display.print(  veggieState);
  display.print("  ");
  display.print(  output);
  display.update();
}

void setup() {
  pinMode(pumpPin, OUTPUT);
  pinMode(ssrPin, OUTPUT);
  pinMode(eStop, INPUT_PULLUP);
  analogWrite(pumpPin, 0);
  
  Serial.begin(9600);
  displayTest();
  
  veggiePID.SetOutputLimits(WINDOW_SIZE_MIN, WINDOW_SIZE_MAX);
  veggiePID.SetSampleTime(PID_SAMPLE_TIME);
  //veggiePID.SetMode(AUTOMATIC);
}

void loop() {
  //check for eStop
  int stopCMD = digitalRead(eStop);
  if (!stopCMD){//active low
    veggieState = VEGGIE_STATE_IDLE_STOP;
  }
  
  if (millis() - displayTimer > DTIME) {
    readPSI(); //this updates pressure and displays it.
  }
  switch (veggieState) {
    //*******************************************************
    case VEGGIE_STATE_IDLE:
      analogWrite(pumpPin, 0);
      digitalWrite(ssrPin, LOW);
      setpoint = 0;
      veggiePID.SetMode(MANUAL);//turn on PID
      //Serial.println("IDLE");
      delay(2000);
      if (pressure < MAX_PRESSURE_LIMIT){
        veggieState = VEGGIE_STATE_FILL;
        break;
        if (pressure > MIN_PRESSURE_LIMIT){
          veggieState = VEGGIE_STATE_PID_HEAT;
          break;
        }
      }
      veggieState = VEGGIE_STATE_ERROR;
    break;
    //*******************************************************
    case VEGGIE_STATE_FILL:
      analogWrite(pumpPin, 255);
      digitalWrite(ssrPin, LOW);
      //Serial.println("FILL");
      if (pressure > FILL_HANDOFF_HEAT){
        veggieState = VEGGIE_STATE_FILL_HEAT;
      }
    break;
    //*******************************************************
    case VEGGIE_STATE_FILL_HEAT:
      analogWrite(pumpPin, 255);
      digitalWrite(ssrPin, HIGH);
      //Serial.println("FILL");
      if (pressure > FILL_HANDOFF_PID){
        veggieState = VEGGIE_STATE_PID_HEAT;
        veggiePID.SetMode(AUTOMATIC);//turn on PID
      }
      if (pressure < MIN_PRESSURE_LIMIT){//pressure drop turn off heater
        veggieState = VEGGIE_STATE_FILL;
      }
    break;
    //*******************************************************
    case VEGGIE_STATE_PID_HEAT:
      //Serial.println("PID HEAT");
      digitalWrite(ssrPin, HIGH);
      if (pressure > MAX_PRESSURE_LIMIT|| pressure < MIN_PRESSURE_LIMIT){
        veggieState = VEGGIE_STATE_ERROR;
      }
      setpoint = SET_PRESSURE;
      veggiePID.Compute();
      analogWrite(pumpPin, output);
      /*dutyCycle = map(output, 0, 100, 0, dutyCycleTotal);
      digitalWrite(pumpPin, LOW);
      delayMicroseconds(dutyCycleTotal - dutyCycle);
      digitalWrite(pumpPin, HIGH);
      delayMicroseconds(dutyCycle);*/
    break;    
    //*******************************************************
    case VEGGIE_STATE_ERROR:
      Serial.println("ERROR");
      analogWrite(pumpPin, 0);
      digitalWrite(ssrPin, LOW);
      display.clear();
      display.setCursor(0,0);
      display.setTextSize(3);
      display.println(" ERROR! ");
      display.update();
      delay(1000);
      readPSI(); //this updates pressure and displays it.
      delay(4000);
      //veggieState = VEGGIE_STATE_IDLE;
    break;
    //*******************************************************
    case VEGGIE_STATE_IDLE_STOP:
      Serial.println("IDLE_STOP");
      analogWrite(pumpPin, 0);
      digitalWrite(ssrPin, LOW);
      display.clear();
      display.setCursor(0,0);
      display.setTextSize(3);
      display.println("IDLE STOP");
      display.update();
      delay(2000);
      readPSI(); //this updates pressure and displays it.
      delay(4000);
    break;
  }
}
