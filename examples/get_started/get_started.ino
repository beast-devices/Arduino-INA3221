#include <Wire.h>
#include <Beastdevices_INA3221.h>

#define SERIAL_SPEED 115200

// Set I2C address to 0x41 (0b1000001)
// and set the shunt resistance to 10 mOhm.
Beastdevices_INA3221 ina3221(0x41, 10);

void setup() {
  Serial.begin(SERIAL_SPEED);

  while (!Serial) {
      delay(1);
  }

  ina3221.begin();
  ina3221.reset();
}

void loop() {
  int32_t ch1_current;
  int32_t ch1_voltage;
  int32_t ch1_power;

  int32_t ch2_current;
  int32_t ch2_voltage;
  int32_t ch2_power;

  int32_t ch3_current;
  int32_t ch3_voltage;
  int32_t ch3_power;

  ch1_current = ina3221.Ch1().getCurrent();
  ch1_voltage = ina3221.Ch1().getVoltage();
  ch1_power = ina3221.Ch1().getPower();

  ch2_current = ina3221.Ch2().getCurrent();
  ch2_voltage = ina3221.Ch2().getVoltage();
  ch2_power = ina3221.Ch2().getPower();

  ch3_current = ina3221.Ch3().getCurrent();
  ch3_voltage = ina3221.Ch3().getVoltage();
  ch3_power = ina3221.Ch3().getPower();

  Serial.print("Channel 1: ");
  Serial.print(ch1_current);
  Serial.print("mA, ");
  Serial.print(ch1_voltage);
  Serial.print("mV, ");
  Serial.print(ch1_power);
  Serial.println("mW");

  Serial.print("Channel 2: ");
  Serial.print(ch2_current);
  Serial.print("mA, ");
  Serial.print(ch2_voltage);
  Serial.print("mV, ");
  Serial.print(ch2_power);
  Serial.println("mW");

  Serial.print("Channel 3: ");
  Serial.print(ch3_current);
  Serial.print("mA, ");
  Serial.print(ch3_voltage);
  Serial.print("mV, ");
  Serial.print(ch3_power);
  Serial.println("mW");

  delay(1000);
}
