//OLED display
#include <Adafruit_ssd1306syp.h>
#define SDA_PIN 5
#define SCL_PIN 4
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);

#include <PID_v1.h>

//STATES
typedef enum VEGGIE_STATE
{
  VEGGIE_STATE_IDLE,
  REFLOW_STATE_FILL,
  REFLOW_STATE_PID,
  REFLOW_STATE_PID_HEAT,
  REFLOW_STATE_ERROR
} veggieState_t;
veggieState_t veggieState = VEGGIE_STATE_IDLE;

//PINS
const int analogInPin = A1;
const int pumpPin = 3;
const int ssrPin = 6;

//Pressure Sensor Variables
int rawIn = 0;        
int offset = 78; //calibrate for 0.5 as zero pressure
int fullScale = 819; // ~4.5volt max pressure (span) adjust (~798-840)
double pressure; // final pressure in psi
#define MAX_PRESSURE_LIMIT 4.0
#define MIN_PRESSURE_LIMIT 1.0
#define SET_PRESSURE 2.0

//PID Variables
#define PID_SAMPLE_TIME 100
#define WINDOW_SIZE_MAX 255
#define WINDOW_SIZE_MIN 0
double setpoint;
double output;
double kp = 300;
double ki = 0.05;
double kd = 0;

PID veggiePID(&pressure, &output, &setpoint, kp, ki, kd, DIRECT);


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
  Serial.print(rawIn);
  Serial.print(" ");
  Serial.print(pressure, 1); // one decimal place
  Serial.println("  psi");
  Serial.print(setpoint, 1); // one decimal place
  Serial.println("  set");
  Serial.print(output, 1); // one decimal place
  Serial.println("  out");

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
  digitalWrite(pumpPin, LOW);
  
  Serial.begin(9600);
  displayTest();
  
  veggiePID.SetOutputLimits(WINDOW_SIZE_MIN, WINDOW_SIZE_MAX);
  veggiePID.SetSampleTime(PID_SAMPLE_TIME);
  //veggiePID.SetMode(AUTOMATIC);
}

void loop() {
  readPSI(); //this updates pressure and displays it.
  switch (veggieState) {
    //*******************************************************
    case VEGGIE_STATE_IDLE:
      analogWrite(pumpPin, 0);
      digitalWrite(ssrPin, LOW);
      setpoint = 0;
      veggiePID.SetMode(MANUAL);//turn on PID
      Serial.println("IDLE");
      delay(2000);
      if (pressure < MAX_PRESSURE_LIMIT){
        veggieState = REFLOW_STATE_FILL;
        break;
        if (pressure > MIN_PRESSURE_LIMIT){
          veggieState = REFLOW_STATE_PID;
          break;
        }
      }
      veggieState = REFLOW_STATE_ERROR;
    break;
    //*******************************************************
    case REFLOW_STATE_FILL:
      analogWrite(pumpPin, 255);
      Serial.println("FILL");
      if (pressure > MIN_PRESSURE_LIMIT){
        veggieState = REFLOW_STATE_PID;
        veggiePID.SetMode(AUTOMATIC);//turn on PID
      }
    break;
    //*******************************************************
    case REFLOW_STATE_PID:
      Serial.println("PID"); 
      if (pressure > MAX_PRESSURE_LIMIT|| pressure < MIN_PRESSURE_LIMIT){
        veggieState = REFLOW_STATE_ERROR;
      }
      setpoint = SET_PRESSURE;
      veggiePID.Compute();
      analogWrite(pumpPin, output);
    break;
    //*******************************************************  
    case REFLOW_STATE_PID_HEAT:
      Serial.println("PID HEAT");
      digitalWrite(ssrPin, HIGH);
      if (pressure > MAX_PRESSURE_LIMIT|| pressure < MIN_PRESSURE_LIMIT){
        veggieState = REFLOW_STATE_ERROR;
      }
      setpoint = SET_PRESSURE;
      veggiePID.Compute();
      analogWrite(pumpPin, output);
    break;    
    //*******************************************************
    case REFLOW_STATE_ERROR:
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
  }
}
