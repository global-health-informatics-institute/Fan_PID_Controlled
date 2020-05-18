#include <Wire.h>

//Inputs and outputs
int firing_pin = 33; // THIS IS FOR THE FAN TRIAC AS PER PCB LAYOUT
int zero_cross = 25; // THIS IS FOR THE ZERO CROSSING DETECTION AS PER PCB LAYOUT

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
int setpoint = 75;
int firing_delay = 0;  // Initialize this to ZERO and we will adjust for different to see how the speed of the fan varies
int ticks = 0;  // this keeps the number of half-second ticks since the program started
int warm_up_ticks = 240;  // this is a TWO MINUTE delay before we start reducing the fan speed by 10% every 2 mins
int counter_zero_cross =0; // value tracks when zero crossing was detected 

//PID variables
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//PID constants
int kp = 1800;   int ki = 1.2;   int kd = 30000;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;


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
  counter_zero_cross += 1;
}

void setup() {
  Serial.begin(9600);
  pinMode (firing_pin, OUTPUT);
  pinMode (zero_cross, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, CHANGE);
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

int GetUpdatedFiringDelay() {
  if (ticks < warm_up_ticks)
    return 0;
  else
    if (ticks < warm_up_ticks + 240)  // Go to 90% power after warm-up time plus 2 mins
      return 2028;
    else    
      if (ticks < warm_up_ticks + 480)  // Go to 80% power after warm-up time plus 4 mins
        return 2944;
      else
        if (ticks < warm_up_ticks + 720)
          return 3680;
        else
           if (ticks < warm_up_ticks + 960)     
             return 4350;
           else 
             if (ticks < warm_up_ticks + 1200)     
               return 5000;
             else
               if (ticks < warm_up_ticks + 1440)   
                 return 5650;
               else
                 if (ticks < warm_up_ticks + 1680)
                   return 6320;
                 else
                   if (ticks < warm_up_ticks + 1920)
                     return 7056;
                   else
                     return 7972;
}

void loop()
{
  currentMillis = millis();           //Save the value of time before the loop
  // We create this if so we will read the temperature and change values each "temp_read_Delay"
  if (currentMillis - previousMillis >= temp_read_Delay) {
    //Update tick to keep track of how many half seconds have passed since the program started
    ticks += 1;
    
    //Update the firing delay value
    firing_delay = GetUpdatedFiringDelay();
    
    // Print the firing delay and the temps of the five locations so we can graph them
    Serial.print("," + String(firing_delay)); 
    Serial.print("," + String(counter_zero_cross)); 
    Serial.print("," + String(GetTemp(16, 18))); // Hinge Left
    Serial.print("," + String(GetTemp(17, 19))); // Front Left
    Serial.print("," + String(GetTemp(15, 4))); // Front Right
    Serial.print("," + String(GetTemp(14, 25))); // Hinge Right
    Serial.print("," + String(GetTemp(27, 33))); // Center
    Serial.println();
    previousMillis += temp_read_Delay;              //Increase the previous time for next loop
  }

  //If the zero cross interruption was detected we create the 100us firing pulse
  if (zero_cross_detected) {
    delayMicroseconds(firing_delay); //This delay controls the power TO THE FAN
    digitalWrite(firing_pin, HIGH);
    delayMicroseconds(100);
    digitalWrite(firing_pin, LOW);
    zero_cross_detected = false;
  }
}
//End of void loop
