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

#ifndef BEASTDEVICES_INA3221_H
#define BEASTDEVICES_INA3221_H

#include "Arduino.h"
#include "Wire.h"

// Channels
enum INA3221_CH {
    INA3221_CH1,
    INA3221_CH2,
    INA3221_CH3
};

// Registers
enum INA3221_REG {
    INA3221_REG_CONF,
    INA3221_REG_CH1_SHUNTV,
    INA3221_REG_CH1_BUSV,
    INA3221_REG_CH2_SHUNTV,
    INA3221_REG_CH2_BUSV,
    INA3221_REG_CH3_SHUNTV,
    INA3221_REG_CH3_BUSV,
    INA3221_REG_CH1_CRIT_ALERT_LIM,
    INA3221_REG_CH1_WARNING_ALERT_LIM,
    INA3221_REG_CH2_CRIT_ALERT_LIM,
    INA3221_REG_CH2_WARNING_ALERT_LIM,
    INA3221_REG_CH3_CRIT_ALERT_LIM,
    INA3221_REG_CH3_WARNING_ALERT_LIM,
    INA3221_REG_SHUNTV_SUM,
    INA3221_REG_SHUNTV_SUM_LIM,
    INA3221_REG_MASK_ENABLE,
    INA3221_REG_PWR_VALID_HI_LIM,
    INA3221_REG_PWR_VALID_LO_LIM,
    INA3221_REG_MANUF_ID = 0xFE,
    INA3221_REG_DIE_ID = 0xFF
};

// Conversion times
enum INA3221_CONV_TIME {
    INA3221_REG_CONF_CT_140US,
    INA3221_REG_CONF_CT_204US,
    INA3221_REG_CONF_CT_332US,
    INA3221_REG_CONF_CT_588US,
    INA3221_REG_CONF_CT_1100US,
    INA3221_REG_CONF_CT_2116US,
    INA3221_REG_CONF_CT_4156US,
    INA3221_REG_CONF_CT_8244US
};

// Averaging modes
enum INA3221_AVG_MODE {
    INA3221_REG_CONF_AVG_1,
    INA3221_REG_CONF_AVG_4,
    INA3221_REG_CONF_AVG_16,
    INA3221_REG_CONF_AVG_64,
    INA3221_REG_CONF_AVG_128,
    INA3221_REG_CONF_AVG_256,
    INA3221_REG_CONF_AVG_512,
    INA3221_REG_CONF_AVG_1024
};

class Beastdevices_INA3221 {

    // Configuration register
    typedef struct {
        uint16_t mode_shunt_en:1;
        uint16_t mode_bus_en:1;
        uint16_t mode_continious_en:1;
        uint16_t shunt_conv_time:3;
        uint16_t bus_conv_time:3;
        uint16_t avg_mode:3;
        uint16_t ch3_en:1;
        uint16_t ch2_en:1;
        uint16_t ch1_en:1;
        uint16_t reset:1;
    } conf_reg_t __attribute__((packed));

    // Mask/Enable register
    typedef struct {
        uint16_t conv_ready:1;
        uint16_t timing_ctrl_alert:1;
        uint16_t pwr_valid_alert:1;
        uint16_t warn_alert_ch3:1;
        uint16_t warn_alert_ch2:1;
        uint16_t warn_alert_ch1:1;
        uint16_t shunt_sum_alert:1;
        uint16_t crit_alert_ch3:1;
        uint16_t crit_alert_ch2:1;
        uint16_t crit_alert_ch1:1;
        uint16_t crit_alert_latch_en:1;
        uint16_t warn_alert_latch_en:1;
        uint16_t shunt_sum_en_ch3:1;
        uint16_t shunt_sum_en_ch2:1;
        uint16_t shunt_sum_en_ch1:1;
        uint16_t reserved:1;
    } masken_reg_t __attribute__((packed));

    class Channel {
        // Channel number
        INA3221_CH chNum;

        // Shunt resistance in mOhm
        uint32_t shuntRes;

        // Parent INA3221 class
        Beastdevices_INA3221 &ina3221;

    public:
        Channel(INA3221_CH chNum, Beastdevices_INA3221 &ina3221):
                    chNum{chNum}, ina3221{ina3221} {};

        // Sets shunt resistor vaue in mOhm
        void setShuntRes(uint32_t res) {shuntRes = res;}

        // Enables channel mesurements
        void setMeasEn();

        // Disables channel mesurements
        void setMeasDis();

        // Sets warning alert shunt voltage limit
        void setWarnAlertShuntLim(int32_t voltageuV);

        // Sets critical alert shunt voltage limit
        void setCritAlertShuntLim(int32_t voltageuV);

        // Sets warning alert current limit
        void setWarnAlertCurrentLim(int32_t currentmA);

        // Sets critical alert current limit
        void setCritAlertCurrentLim(int32_t currentmA);

        // Includes channel to fill Shunt-Voltage Sum register.
        void setCurrentSumEn();

        // Excludes channel from filling Shunt-Voltage Sum register.
        void setCurrentSumDis();

        // Gets value of shunt resistor in mOhm.
        uint32_t getShuntRes() {return shuntRes;}

        // Reads shunt voltage in uV.
        int32_t readShuntVoltage();

        // Gets warning alert flag.
        bool getWarnAlertFlag();

        // Gets critical alert flag.
        bool getCritAlertFlag();

        // Gets current in mA.
        int32_t getCurrent();

        // Gets bus voltage in mV.
        int32_t getVoltage();

        // Gets power in mW.
        int32_t getPower();
    };

    // Arduino's I2C library
    TwoWire *i2c;

    // I2C address
    uint8_t i2c_addr;

    Channel ch1;
    Channel ch2;
    Channel ch3;

    // Value of Mask/Enable register.
    masken_reg_t _masken_reg;

    // Reads 16 bytes from a register.
    void read(INA3221_REG reg, uint16_t *val);

    // Writes 16 bytes to a register.
    void write(INA3221_REG reg, uint16_t *val);

public:

    Beastdevices_INA3221(uint8_t addr, uint32_t shunt_res) :
        i2c_addr(addr),
        ch1(Channel(INA3221_CH1, *this)),
        ch2(Channel(INA3221_CH2, *this)),
        ch3(Channel(INA3221_CH3, *this))
    {
        ch1.setShuntRes(shunt_res);
        ch2.setShuntRes(shunt_res);
        ch3.setShuntRes(shunt_res);
    }

    Channel& Ch1() { return ch1; }
    Channel& Ch2() { return ch2; }
    Channel& Ch3() { return ch3; }

    // Initializes INA3221
    void begin(TwoWire *theWire = &Wire);

    // Sets I2C address of INA3221
    void setAddr(uint8_t addr) { i2c_addr = addr; }

    // Gets a register value.
    uint16_t getReg(INA3221_REG reg);

    // Resets INA3221
    void reset();

    // Sets operating mode to power-down
    void setModePowerDown();

    // Sets operating mode to continious
    void setModeContinious();

    // Sets operating mode to triggered (single-shot)
    void setModeTriggered();

    // Enables shunt-voltage measurement
    void setShuntMeasEn();

    // Disables shunt-voltage mesurement
    void setShuntMeasDis();

    // Enables bus-voltage measurement
    void setBusMeasEn();

    // Disables bus-voltage measureement
    void setBusMeasDis();

    // Sets averaging mode. Sets number of samples that are collected
    // and averaged togehter.
    void setAvgMode(INA3221_AVG_MODE mode);

    // Sets bus-voltage conversion time.
    void setBusConvTime(INA3221_CONV_TIME convTime);

    // Sets shunt-voltage conversion time.
    void setShuntConvTime(INA3221_CONV_TIME convTime);

    // Sets power-valid upper-limit voltage. The power-valid condition
    // is reached when all bus-voltage channels exceed the value set.
    // When the powervalid condition is met, the PV alert pin asserts high.
    void setPwrValidUpLim(int16_t voltagemV);

    // Sets power-valid lower-limit voltage. If any bus-voltage channel drops
    // below the power-valid lower-limit, the PV alert pin pulls low.
    void setPwrValidLowLim(int16_t voltagemV);

    // Sets the value that is compared to the Shunt-Voltage Sum register value
    // following each completed cycle of all selected channels to detect
    // for system overcurrent events.
    void setShuntSumAlertLim(int32_t voltagemV);

    // Sets the current value that is compared to the sum all currents.
    // This function is a helper for setShuntSumAlertLim(). It onverts current
    // value to shunt voltage value.
    void setCurrentSumAlertLim(int32_t currentmA);

    // Enables warning alert latch.
    void setWarnAlertLatchEn();

    // Disables warning alert latch.
    void setWarnAlertLatchDis();

    // Enables critical alert latch.
    void setCritAlertLatchEn();

    // Disables critical alert latch.
    void setCritAlertLatchDis();

    // Reads flags from Mask/Enable register.
    // When Mask/Enable register is read, flags are cleared.
    // Use getTimingCtrlAlertFlag(), getPwrValidAlertFlag(),
    // getCurrentSumAlertFlag() and getConvReadyFlag() to get flags after
    // readFlags() is called.
    void readFlags();

    // Gets timing-control-alert flag indicator.
    bool getTimingCtrlAlertFlag();

    // Gets power-valid-alert flag indicator.
    bool getPwrValidAlertFlag();

    // Gets summation-alert flag indicator.
    bool getCurrentSumAlertFlag();

    // Gets Conversion-ready flag.
    bool getConvReadyFlag();

    // Gets manufacturer ID.
    // Should read 0x5449.
    uint16_t getManufID();

    // Gets die ID.
    // Should read 0x3220.
    uint16_t getDieID();
};

#endif