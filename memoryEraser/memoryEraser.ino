#include "Wire.h"

#define EEPROM_I2C_ADDRESS 0x50
int i;
  byte val = 255;
void setup()
{
  Wire.begin();
  Serial.begin(9600);

  int address = 0;
  byte val = 255;
 
  store_byte(address, val);
  byte readVal = load_byte(address);
 
  Serial.print("The returned value is ");
  Serial.println(readVal);

  delay(10000);

}

void loop()
{

  for (i; i < 513; i++) {
    store_byte(i, val);
    Serial.println(i);
  }
}

void store_byte(uint8_t eeaddress, uint32_t data) {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write(eeaddress);
  Wire.write(data);
  Wire.endTransmission();

  delay(20);
}

byte load_byte(uint8_t eeaddress) {
  byte data = 0xFF;

  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write(eeaddress);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_I2C_ADDRESS,1);

 if (Wire.available()) data = Wire.read();

  return data;
}