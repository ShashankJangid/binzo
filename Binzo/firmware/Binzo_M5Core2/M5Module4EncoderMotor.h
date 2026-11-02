#ifndef _M5_MODULE_4ENCODERMOTOR_H_
#define _M5_MODULE_4ENCODERMOTOR_H_

#include "Arduino.h"
#include "I2C_Class.h"

#define MODULE_4ENCODERMOTOR_ADDR (0x24)

#define MODULE_4ENCODERMOTOR_SERVO_ANGLE_ADDR        (0x00)
#define MODULE_4ENCODERMOTOR_SERVO_PULSE_ADDR        (0x10)
#define MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR           (0x20)
#define MODULE_4ENCODERMOTOR_ENCODER_ADDR            (0x30)
#define MODULE_4ENCODERMOTOR_SPEED_ADDR              (0x40)
#define MODULE_4ENCODERMOTOR_ADC_8BIT_REG            (0xA0)
#define MODULE_4ENCODERMOTOR_ADC_12BIT_REG           (0xB0)
#define JUMP_TO_BOOTLOADER_REG                       (0xFD)
#define UPGRADE_BOOTLOADER_REG                       (0xE0)
#define MODULE_4ENCODERMOTOR_FIRMWARE_VERSION_ADDR   (0xFE)
#define MODULE_4ENCODERMOTOR_BOOTLOADER_VERSION_ADDR (0xFC)
#define MODULE_4ENCODERMOTOR_I2C_ADDRESS_ADDR        (0xFF)

/*
   |  0  |       1     |      2     |     3      |    4, 5, 6, 7  |          8
   |     9    |    10   |    11   |     12      | | mod |  position-p |
   position-i | position-d | position-point | position-max-speed |  speed-p |
   speed-i | speed-d | speed-point |
*/
#define MODULE_4ENCODERMOTOR_CONFIG_ADDR          (0x50)
#define MODULE_4ENCODERMOTOR_CURRENT_ADDR         (0x90)
#define MODULE_4ENCODERMOTOR_SOFT_START_STOP_ADDR (0xD1)

#define NORMAL_MODE            (0x00)
#define POSITION_MODE          (0x01)
#define SPEED_MODE             (0x02)
#define IAP_UPDATE_MODE        (0x03)
#define BOOTLOADER_UPDATE_MODE (0x04)

typedef enum { _8bit = 0, _12bit } hbridge_anolog_read_mode_t;

class M5Module4EncoderMotor {
   private:
    uint8_t checkIndex(uint8_t index);

    uint8_t _addr;
    I2C_Class _i2c;

   public:
    bool begin(TwoWire *wire = &Wire, uint8_t addr = MODULE_4ENCODERMOTOR_ADDR,
               uint8_t sda = 21, uint8_t scl = 22, long freq = 100000);
    void setMode(uint8_t index, uint8_t mode);
    int32_t getEncoderValue(uint8_t index);
    void setEncoderValue(uint8_t index, int32_t encode);

    void setMotorSpeed(uint8_t index, int8_t duty);
    int8_t getMotorSpeed(uint8_t index);

    int8_t getMotorSpeed20MS(uint8_t index);

    void setPositionPID(uint8_t index, uint8_t kp, uint8_t ki, uint8_t kd);
    void setPositionPoint(uint8_t index, int32_t position_point);
    void setPostionPIDMaxSpeed(uint8_t index, uint8_t max_pwm);

    void setSpeedPID(uint8_t index, uint8_t kp, uint8_t ki, uint8_t kd);
    void setSpeedPoint(uint8_t index, int8_t speed_point);

    void setSoftStartAndStop(uint8_t index, bool state);
    bool getSoftStartAndStop(uint8_t index);

    bool getFirewareVersion(uint8_t *fw);
    bool setI2CAddress(uint8_t addr);
    uint8_t getI2CAddress(void);
    float getMotorCurrent(void);
    void jumpBootloader(void);
    uint16_t getAnalogInput(hbridge_anolog_read_mode_t bit);
    bool getBootloaderVersion(uint8_t *fw);
};

#endif
