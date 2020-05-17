#include <Wire.h>
//#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

const int ADDR = 0x40;
const int MeasureTemp = 0xE3;
int X0,X1,temp;
double X,X_out;

void setup() {
  Serial.begin(9600);
//  lcd.init();                      // initialize the lcd 
//  lcd.backlight();
//  lcd.clear();
}

double GetTemp(int SDA_Pin, int SLC_pin) {
  Wire.begin(SDA_Pin,SLC_pin,5000);
  Wire.beginTransmission(ADDR);
  Wire.write(MeasureTemp);
  Wire.endTransmission();
  Wire.requestFrom(ADDR,2);
  if(Wire.available()<=2);{
    X0 = Wire.read();
    X1 = Wire.read();
    X0 = X0<<8;
    X_out = X0+X1;
  }
  /**Calculate temperature**/
  X=(175.72*X_out)/65536;
  X=X-46.85;
  return X;
}

void loop() {
  double T;
  Serial.print("," +String(GetTemp(16,18))); // Hinge Left
  Serial.print("," +String(GetTemp(17,19))); // Front Left
  Serial.print("," +String(GetTemp(15,4))); // Center
  Serial.print("," +String(GetTemp(14,25))); // Hinge Right 
  Serial.print("," +String(GetTemp(27,33))); // Front Right
  Serial.println();
//  T=GetTemp(5,4);
//  Wire.begin(21,22,50000);
// lcd.setCursor(0,0);
//  lcd.print(T); 
  delay(1000);
}
