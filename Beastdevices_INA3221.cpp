/*

	Arduino library for INA3221 current and voltage sensor.

	MIT License

	Copyright (c) 2020 Beast Devices, Andrejs Bondarevs

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

*/

#include "Beastdevices_INA3221.h"

void Beastdevices_INA3221::read(INA3221_REG reg, uint16_t *val) {
    i2c->beginTransmission(i2c_addr);
    i2c->write(reg); // Register
    i2c->endTransmission();

    i2c->requestFrom(i2c_addr, (uint8_t)2);

    if (i2c->available())
    {
      *val = ((i2c->read() << 8) | i2c->read());
    }
}

void Beastdevices_INA3221::write(INA3221_REG reg, uint16_t *val) {
    i2c->beginTransmission(i2c_addr);
    i2c->write(reg);                 // Register
    i2c->write((*val >> 8) & 0xFF); // Upper 8-bits
    i2c->write(*val & 0xFF);        // Lower 8-bits
    i2c->endTransmission();
}

void Beastdevices_INA3221::begin(TwoWire *theWire) {
    i2c = theWire;
    i2c->begin();
}

uint16_t Beastdevices_INA3221::getReg(INA3221_REG reg){
    uint16_t val;
    read(reg, &val);
    return val;
}

void Beastdevices_INA3221::reset(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.reset = 0x01;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setModePowerDown(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_bus_en = 0x00;
    conf_reg.mode_continious_en = 0x00;

    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setModeContinious(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_continious_en = 0x01;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setModeTriggered(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_continious_en = 0x00;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setShuntMeasEn(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_shunt_en = 0x01;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setShuntMeasDis(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_shunt_en = 0x00;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setBusMeasEn(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_bus_en = 0x01;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setBusMeasDis(){
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.mode_bus_en = 0x00;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setAvgMode(INA3221_AVG_MODE mode) {
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.avg_mode = mode;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setBusConvTime(INA3221_CONV_TIME convTime) {
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.bus_conv_time = convTime;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setShuntConvTime(INA3221_CONV_TIME convTime) {
    conf_reg_t conf_reg;

    read(INA3221_REG_CONF, (uint16_t*)&conf_reg);
    conf_reg.shunt_conv_time = convTime;
    write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::setPwrValidUpLim(int16_t voltagemV) {
    write(INA3221_REG_PWR_VALID_HI_LIM, (uint16_t*)&voltagemV);
}

void Beastdevices_INA3221::setPwrValidLowLim(int16_t voltagemV) {
    write(INA3221_REG_PWR_VALID_LO_LIM, (uint16_t*)&voltagemV);
}

void Beastdevices_INA3221::setShuntSumAlertLim(int32_t voltageuV) {
    int16_t val;
    val = voltageuV / 20;
    write(INA3221_REG_SHUNTV_SUM_LIM, (uint16_t*)&val);
}

void Beastdevices_INA3221::setCurrentSumAlertLim(int32_t currentmA) {
    int16_t shuntuV;
    shuntuV = currentmA*(int32_t)ch1.getShuntRes();
    setShuntSumAlertLim(shuntuV);
}

void Beastdevices_INA3221::setWarnAlertLatchEn() {
    masken_reg_t masken_reg;

    read(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    masken_reg.warn_alert_latch_en = 0x01;
    write(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    _masken_reg = masken_reg;
}

void Beastdevices_INA3221::setWarnAlertLatchDis() {
    masken_reg_t masken_reg;

    read(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    masken_reg.warn_alert_latch_en = 0x00;
    write(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    _masken_reg = masken_reg;
}

void Beastdevices_INA3221::setCritAlertLatchEn() {
    masken_reg_t masken_reg;

    read(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    masken_reg.crit_alert_latch_en = 0x01;
    write(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    _masken_reg = masken_reg;
}

void Beastdevices_INA3221::setCritAlertLatchDis() {
    masken_reg_t masken_reg;

    read(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    masken_reg.crit_alert_latch_en = 0x00;
    write(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    _masken_reg = masken_reg;
}

void Beastdevices_INA3221::readFlags() {
    read(INA3221_REG_MASK_ENABLE, (uint16_t*)&_masken_reg);
}

bool Beastdevices_INA3221::getTimingCtrlAlertFlag(){
    return _masken_reg.timing_ctrl_alert;
}

bool Beastdevices_INA3221::getPwrValidAlertFlag(){
    return _masken_reg.pwr_valid_alert;
}

bool Beastdevices_INA3221::getCurrentSumAlertFlag(){
    return _masken_reg.shunt_sum_alert;
}

bool Beastdevices_INA3221::getConvReadyFlag(){
    return _masken_reg.conv_ready;
}

uint16_t Beastdevices_INA3221::getManufID() {
    uint16_t id;
    read(INA3221_REG_MANUF_ID, &id);
    return id;
}

uint16_t Beastdevices_INA3221::getDieID() {
    uint16_t id;
    read(INA3221_REG_DIE_ID, &id);
    return id;
}

void Beastdevices_INA3221::Channel::setMeasEn() {
    conf_reg_t conf_reg;

    ina3221.read(INA3221_REG_CONF, (uint16_t*)&conf_reg);

    switch(chNum){
        case INA3221_CH1:
            conf_reg.ch1_en = 0x01;
            break;
        case INA3221_CH2:
            conf_reg.ch2_en = 0x01;
            break;
        case INA3221_CH3:
            conf_reg.ch3_en = 0x01;
            break;
    }

    ina3221.write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::Channel::setMeasDis() {
    conf_reg_t conf_reg;

    ina3221.read(INA3221_REG_CONF, (uint16_t*)&conf_reg);

    switch(chNum){
        case INA3221_CH1:
            conf_reg.ch1_en = 0x00;
            break;
        case INA3221_CH2:
            conf_reg.ch2_en = 0x00;
            break;
        case INA3221_CH3:
            conf_reg.ch3_en = 0x00;
            break;
    }

    ina3221.write(INA3221_REG_CONF, (uint16_t*)&conf_reg);
}

void Beastdevices_INA3221::Channel::setWarnAlertShuntLim(int32_t voltageuV) {
    INA3221_REG reg;
    int16_t val;

    switch(chNum){
        case INA3221_CH1:
            reg = INA3221_REG_CH1_WARNING_ALERT_LIM;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_WARNING_ALERT_LIM;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_WARNING_ALERT_LIM;
            break;
    }

    val = voltageuV / 5;
    ina3221.write(reg, (uint16_t*)&val);
}

void Beastdevices_INA3221::Channel::setCritAlertShuntLim(int32_t voltageuV) {
    INA3221_REG reg;
    int16_t val;

    switch(chNum){
        case INA3221_CH1:
            reg = INA3221_REG_CH1_CRIT_ALERT_LIM;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_CRIT_ALERT_LIM;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_CRIT_ALERT_LIM;
            break;
    }

    val = voltageuV / 5;
    ina3221.write(reg, (uint16_t*)&val);
}

void Beastdevices_INA3221::Channel::setWarnAlertCurrentLim(int32_t currentmA)
{
    int32_t shuntuV;
    shuntuV = currentmA*(int32_t)shuntRes;
    setWarnAlertShuntLim(shuntuV);
}

void Beastdevices_INA3221::Channel::setCritAlertCurrentLim(int32_t currentmA)
{
    int32_t shuntuV;
    shuntuV = currentmA*(int32_t)shuntRes;
    setCritAlertShuntLim(shuntuV);
}

void Beastdevices_INA3221::Channel::setCurrentSumEn() {
    masken_reg_t masken_reg;

    ina3221.read(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);

    switch(chNum){
        case INA3221_CH1:
            masken_reg.shunt_sum_en_ch1 = 0x01;
            break;
        case INA3221_CH2:
            masken_reg.shunt_sum_en_ch2 = 0x01;
            break;
        case INA3221_CH3:
            masken_reg.shunt_sum_en_ch3 = 0x01;
            break;
    }

    ina3221.write(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    ina3221._masken_reg = masken_reg;
}

void Beastdevices_INA3221::Channel::setCurrentSumDis() {
    masken_reg_t masken_reg;

    ina3221.read(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);

    switch(chNum){
        case INA3221_CH1:
            masken_reg.shunt_sum_en_ch1 = 0x00;
            break;
        case INA3221_CH2:
            masken_reg.shunt_sum_en_ch2 = 0x00;
            break;
        case INA3221_CH3:
            masken_reg.shunt_sum_en_ch3 = 0x00;
            break;
    }

    ina3221.write(INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    ina3221._masken_reg = masken_reg;
}

int32_t Beastdevices_INA3221::Channel::readShuntVoltage() {
    int32_t res;
    INA3221_REG reg;
    uint16_t val_raw;

    switch(chNum){
        case INA3221_CH1:
            reg = INA3221_REG_CH1_SHUNTV;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_SHUNTV;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_SHUNTV;
            break;
    }

    ina3221.read(reg, &val_raw);

    // 1 LSB = 5uV
    res = (int16_t)val_raw * 5;

    return res;
}

bool Beastdevices_INA3221::Channel::getWarnAlertFlag() {
    switch(chNum){
    case INA3221_CH1:
        return ina3221._masken_reg.warn_alert_ch1;
    case INA3221_CH2:
        return ina3221._masken_reg.warn_alert_ch2;
    case INA3221_CH3:
        return ina3221._masken_reg.warn_alert_ch3;
    }
}

bool Beastdevices_INA3221::Channel::getCritAlertFlag() {
    switch(chNum){
    case INA3221_CH1:
        return ina3221._masken_reg.crit_alert_ch1;
    case INA3221_CH2:
        return ina3221._masken_reg.crit_alert_ch2;
    case INA3221_CH3:
        return ina3221._masken_reg.crit_alert_ch3;
    }
}

int32_t Beastdevices_INA3221::Channel::getCurrent() {
    int32_t shunt_uV;
    int32_t current_mA;

    shunt_uV = readShuntVoltage();
    current_mA = shunt_uV/(int32_t)shuntRes;
    return current_mA;
}

int32_t Beastdevices_INA3221::Channel::getVoltage() {
    int32_t res;
    INA3221_REG reg;
    uint16_t val_raw;

    switch(chNum){
        case INA3221_CH1:
            reg = INA3221_REG_CH1_BUSV;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_BUSV;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_BUSV;
            break;
    }

    ina3221.read(reg, &val_raw);

    res = (int16_t)val_raw;

    return res;
}

int32_t Beastdevices_INA3221::Channel::getPower() {
    int32_t bus_mV;
    int32_t current_mA;
    int32_t power_mW;

    bus_mV = getVoltage();
    current_mA = getCurrent();
    power_mW = bus_mV*current_mA/1000;

    return power_mW;
}