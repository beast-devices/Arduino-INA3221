#include <Wire.h>
#include <Beastdevices_INA3221.h>

#define SERIAL_SPEED      115200
#define PRINT_DEC_POINTS  3

// Set I2C address to 0x41 (A0 pin -> VCC)
Beastdevices_INA3221 ina3221(INA3221_ADDR41_VCC);

void setup() {
  Serial.begin(SERIAL_SPEED);

  while (!Serial) {
      delay(1);
  }

  ina3221.begin();
  ina3221.reset();

  ina3221.setShuntRes(10, 10, 10);
  ina3221.setFilterRes(10, 10, 10);
  ina3221.setShuntConversionTime(INA3221_REG_CONF_CT_8244US);
}

void loop() {
  float current[3];
  float current_comp[3];
  float voltage[3];

  current[0] = ina3221.getCurrent(INA3221_CH1);
  current_comp[0] = ina3221.getCurrentCompensated(INA3221_CH1);
  voltage[0] = ina3221.getVoltage(INA3221_CH1);

  current[1] = ina3221.getCurrent(INA3221_CH2);
  current_comp[1] = ina3221.getCurrentCompensated(INA3221_CH2);
  voltage[1] = ina3221.getVoltage(INA3221_CH2);

  current[2] = ina3221.getCurrent(INA3221_CH3);
  current_comp[2] = ina3221.getCurrentCompensated(INA3221_CH3);
  voltage[2] = ina3221.getVoltage(INA3221_CH3);

  Serial.print("Channel 1: ");
  Serial.print(current[0], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(current_comp[0], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(voltage[0], PRINT_DEC_POINTS);
  Serial.println("V");

  Serial.print("Channel 2: ");
  Serial.print(current[1], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(current_comp[1], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(voltage[1], PRINT_DEC_POINTS);
  Serial.println("V");

  Serial.print("Channel 3: ");
  Serial.print(current[2], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(current_comp[2], PRINT_DEC_POINTS);
  Serial.print("A, ");
  Serial.print(voltage[2], PRINT_DEC_POINTS);
  Serial.println("V");


  delay(1000);
}
