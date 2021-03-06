#include <Wire.h>
extern TwoWire Wire1; //// THIS IS NEW

/*
 * The logic here is as follows ...
 * The 5 sensor readings tell us that the temperature at the outside of the oven is always warmer then the temp at the center.
 * We believe this is true because the fan is pushing all the warm air to the outside when running at full speed.
 * If we slow down the fan we believe that we will even out the temperature.
 * The PID will adjust the firing delay on the fan so that the error between the two temps will be zero.
 */

//Inputs and outputs
gpio_num_t FAN_firing_pin = GPIO_NUM_32; // THIS IS FOR THE FAN TRIAC AS PER PCB LAYOUT
gpio_num_t zero_cross = GPIO_NUM_35; // THIS IS FOR THE ZERO CROSSING DETECTION AS PER PCB LAYOUT

const int ADDR = 0x40;
const int MeasureTemp = 0xE3;
int X0, X1, temp;
double X, X_out;

bool TempRequestSent = false; //?? THIS IS NEW

int last_CH1_state = 0;
bool zero_cross_detected = false;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int temp_read_Delay = 500;
float real_temperature = 0;
int FAN_firing_delay = 0;  // Initialize this to ZERO and we will adjust for different to see how the speed of the fan varies

//FAN PID variables
float FAN_PID_error = 0;
float FAN_previous_error = 0;
float elapsedTime, Time, timePrev;
int FAN_PID_value = 0;
int FAN_maximum_firing_delay = 7000; //TESTING THIS VALUE

//FAN PID constants
int FAN_kp = 500;   int FAN_ki = 1;     int FAN_kd = 1000;
int FAN_PID_p = 0;   int FAN_PID_i = 0;  int FAN_PID_d = 0;

//OVEN Temp values'
double Outer_Temp, Inner_Temp;  // These hold the values of the two temp sensors we will use for PID control.

//Zero Crossing Interrupt Function
void IRAM_ATTR zero_crossing()
{
  //If the last state was 0, then we have a state change...
  if (last_CH1_state == 0)
    zero_cross_detected = true; //We have detected a state change! We need both falling and rising edges
  //If pin 8 is LOW and the last state was HIGH then we have a state change
  else if (last_CH1_state == 1) {
    zero_cross_detected = true;    //We have detected a state change!  We need both falling and rising edges.
    last_CH1_state = 0;            //Store the current state into the last state for the next loop
  }
}

void setup() {
  Serial.begin(9600);
  pinMode (FAN_firing_pin, OUTPUT);
  pinMode (zero_cross, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, CHANGE);
  Wire.begin(18, 19, 50000);  //Inner sensor
  Wire1.begin(16, 17, 50000);  //Outer sensor
}

void loop()
{
  currentMillis = millis();           //Save the value of time before the loop

  // SEND RESUEST TO Si7021 SENSORS 10 milliceconds BEFORE we want to read them
  if (((currentMillis - previousMillis) >= (temp_read_Delay -10)) and !TempRequestSent) {
     Wire.beginTransmission(ADDR);
     Wire.write(MeasureTemp);
     Wire.endTransmission();
     Wire1.beginTransmission(ADDR);
     Wire1.write(MeasureTemp);
     Wire1.endTransmission();
     TempRequestSent = true;
  }
  
  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if (currentMillis - previousMillis >= temp_read_Delay) {
    //Update the firing delay value
        previousMillis += temp_read_Delay;              //Increase the previous time for next loop

    // Start new FAN PIC code

    Wire.requestFrom(ADDR, 2);
    if (Wire.available() <= 2); {
      X0 = Wire.read();
      X1 = Wire.read();
      X0 = X0 << 8;
      X_out = X0 + X1;
    }
    /**Calculate temperature**/
    X = (175.72 * X_out) / 65536;
    Inner_Temp = X - 46.85;

    Wire1.requestFrom(ADDR, 2);
    if (Wire1.available() <= 2); {
      X0 = Wire1.read();
      X1 = Wire1.read();
      X0 = X0 << 8;
      X_out = X0 + X1;
    }
    /**Calculate temperature**/
    X = (175.72 * X_out) / 65536;
    Outer_Temp = X - 46.85;

    FAN_PID_error = Outer_Temp - Inner_Temp;        //Calculate the pid ERROR as the difference between ths center and edge of oven

    // Print the firing delay and the temps of the five locations so we can graph them
    Serial.print(", Firing Delay=" + String(FAN_PID_value)); 
    Serial.print(", Error=" + String(FAN_PID_error));   // THIS IS THE DIFFERENCE IN TEMP BETWEEN THE OUTER AND INNER SENSOR THAT WE ARE TRYING TO REDUCE TO ZERO
    Serial.print(", Inner=" + String(Inner_Temp));
    Serial.print(", Outer=" + String(Outer_Temp));
    Serial.println();

    
    if(FAN_PID_error > 30)                              //integral constant will only affect errors below 30ºC             
      FAN_PID_i = 0;
    FAN_PID_p = FAN_kp * FAN_PID_error;                         //Calculate the P value
    FAN_PID_i = FAN_PID_i + (FAN_ki * FAN_PID_error);               //Calculate the I value
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = millis();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000;   
    FAN_PID_d = FAN_kd*((FAN_PID_error - FAN_previous_error)/elapsedTime); //Calculate the D value
    FAN_PID_value = FAN_PID_p + FAN_PID_i + FAN_PID_d; //Calculate total PID value
    if(FAN_PID_value < 0)      
      FAN_PID_value = 0;       
    if(FAN_PID_value > FAN_maximum_firing_delay)      
      FAN_PID_value = FAN_maximum_firing_delay;    
    FAN_previous_error = FAN_PID_error; //Remember to store the previous error.
    TempRequestSent = false;  // THIS IS REQUIRED
    //end new FAN PID code    
  }

  //If the zero cross interruption was detected we create the 100us firing pulse
  if (zero_cross_detected) {
    delayMicroseconds(FAN_PID_value); //This delay controls the power TO THE FAN
    digitalWrite(FAN_firing_pin, HIGH);
    delayMicroseconds(100);
    digitalWrite(FAN_firing_pin, LOW);
    zero_cross_detected = false;
  }
}
//End of void loop
