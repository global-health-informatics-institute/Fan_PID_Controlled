#include <Wire.h>

/*
 * The logic here is as follows ...
 * The 5 sensor readings tell us that the temperature at the outside of the oven is always warmer and the temp at the center.
 * We believe this is true because the fan is pushing all the warm air to the outside when running at full speed.
 * If we slow down the fan we believe that we will even out the temperature.
 * The PID will adjust the firing delay on the fan so that the error between the two temps will be zero.
 */

//Inputs and outputs
int FAN_firing_pin = 32; // THIS IS FOR THE FAN TRIAC AS PER PCB LAYOUT
int zero_cross = 35; // THIS IS FOR THE ZERO CROSSING DETECTION AS PER PCB LAYOUT

const int ADDR = 0x40;
const int MeasureTemp = 0xE3;
int X0, X1, temp;
double X, X_out;

int last_CH1_state = 0;
bool zero_cross_detected = false;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int temp_read_Delay = 500;
float real_temperature = 0;
int FAN_firing_delay = 0;  // Initialize this to ZERO and we will adjust for different to see how the speed of the fan varies

//PID variables
float FAN_PID_error = 0;
float FAN_previous_error = 0;
float elapsedTime, Time, timePrev;
int FAN_PID_value = 0;
int FAN_maximum_firing_delay = 7000; //TESTING THIS VALUE

//PID constants
int FAN_kp = 1000;   int FAN_ki = 0;     int FAN_kd = 0;
int FAN_PID_p = 0;   int FAN_PID_i = 0;  int FAN_PID_d = 0;

//Temp values
double Outer_Temp, Inner_Temp;

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
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, RISING);
}

double GetTemp(int SDA_Pin, int SLC_pin) {
  Wire.begin(SDA_Pin, SLC_pin, 5000);
  Wire.beginTransmission(ADDR);
  Wire.write(MeasureTemp);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 2);
  if (Wire.available() <= 2); {
    X0 = Wire.read();
    X1 = Wire.read();
    X0 = X0 << 8;
    X_out = X0 + X1;
  }
  /**Calculate temperature**/
  X = (175.72 * X_out) / 65536;
  X = X - 46.85;
  return X;
}

void loop()
{
  currentMillis = millis();           //Save the value of time before the loop
  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if (currentMillis - previousMillis >= temp_read_Delay) {
    //Update the firing delay value
        previousMillis += temp_read_Delay;              //Increase the previous time for next loop

// Start new FAN PIC code

    Inner_Temp = GetTemp(16, 18);
    Outer_Temp = GetTemp(17, 19);
    FAN_PID_error = Outer_Temp - Inner_Temp;        //Calculate the pid ERROR as the difference between ths center and edge of oven

    // Print the firing delay and the temps of the five locations so we can graph them
    Serial.print(", " + String(FAN_PID_value)); 
    Serial.print(", " + String(FAN_PID_error));   // THIS IS THE DIFFERENCE IN TEMP BETWEEN THE OUTER AND INNER SENSOR THAT WE ARE TRYING TO REDUCE TO ZERO
    Serial.print(", " + String(Inner_Temp));
    Serial.print(", " + String(Outer_Temp));
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
