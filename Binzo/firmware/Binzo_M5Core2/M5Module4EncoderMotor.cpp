#include "M5Module4EncoderMotor.h"

#define MAX(in, max)            (((in) > (max)) ? (in) : (max))
#define MIN(in, min)            (((in) < (min)) ? (in) : (min))
#define CONSTRAIN(in, min, max) MAX(min, MIN(in, max))

bool M5Module4EncoderMotor::begin(TwoWire *wire, uint8_t addr, uint8_t sda,
                                  uint8_t scl, long freq) {
    _i2c.begin(wire, sda, scl, freq);
    _addr = addr;
    return _i2c.exist(addr);
}

uint8_t M5Module4EncoderMotor::checkIndex(uint8_t index) {
    // index=index - 1;
    index = CONSTRAIN(index, 0, 3);
    return index;
}

/**
 * @description:
 * @param index: 0 ~3
 * @param mode: NORMAL, POSITION LOCK, SPEED LOCK
 * @return: None
 */
void M5Module4EncoderMotor::setMode(uint8_t index, uint8_t mode) {
    index = checkIndex(index);
    _i2c.writeByte(_addr, MODULE_4ENCODERMOTOR_CONFIG_ADDR + (0x10 * index),
                   mode);
}

/**
 * @description:
 * @param index: 0 ~3
 * @return: encoder value
 */
int32_t M5Module4EncoderMotor::getEncoderValue(uint8_t index) {
    uint8_t addr;
    uint8_t read_buf[4] = {0, 0, 0, 0};

    index = checkIndex(index);
    addr  = MODULE_4ENCODERMOTOR_ENCODER_ADDR + 4 * index;
    _i2c.readBytes(_addr, addr, read_buf, 4);
    return (read_buf[0] << 24) | (read_buf[1] << 16) | (read_buf[2] << 8) |
           read_buf[3];
}

/**
 * @description:
 * @param index: 0 ~3
 * @param encoder: INT32_MIN ~ INT32_MAX
 * @return: None
 */
void M5Module4EncoderMotor::setEncoderValue(uint8_t index, int32_t encoder) {
    uint8_t addr;
    uint8_t write_buf[4] = {0, 0, 0, 0};

    index        = checkIndex(index);
    addr         = MODULE_4ENCODERMOTOR_ENCODER_ADDR + 4 * index;
    write_buf[0] = encoder >> 24;
    write_buf[1] = encoder >> 16;
    write_buf[2] = encoder >> 8;
    write_buf[3] = encoder & 0xff;

    _i2c.writeBytes(_addr, addr, write_buf, 4);
}

/**
 * @description:
 * @param index: 0 ~3
 * @param duty: set motor speed, -127 ~ 127
 * @return:
 */
void M5Module4EncoderMotor::setMotorSpeed(uint8_t index, int8_t duty) {
    uint8_t addr;
    index = checkIndex(index);
    addr  = MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR + index;

    _i2c.writeByte(_addr, addr, duty);
}

/**
 * @description:
 * @param index: 0 ~3
 * @return Motor run speed, -127 ~ 127:
 */
int8_t M5Module4EncoderMotor::getMotorSpeed(uint8_t index) {
    uint8_t read_data;
    uint8_t addr;
    index = checkIndex(index);
    addr  = MODULE_4ENCODERMOTOR_PWM_DUTY_ADDR + index;

    _i2c.readBytes(_addr, addr, &read_data, 1);
    return read_data;
}

/**
 * @description:
 * @param index: 0 ~3
 * @return: Motor encoder increments every 20 ms
 */
int8_t M5Module4EncoderMotor::getMotorSpeed20MS(uint8_t index) {
    uint8_t read_data;
    uint8_t addr;
    index = checkIndex(index);
    addr  = MODULE_4ENCODERMOTOR_SPEED_ADDR + index;

    _i2c.readBytes(_addr, addr, &read_data, 1);
    return read_data;
}

void M5Module4EncoderMotor::setPositionPID(uint8_t index, uint8_t kp,
                                           uint8_t ki, uint8_t kd) {
    uint8_t write_buf[3] = {0, 0, 0};
    uint8_t addr;
    index = checkIndex(index);

    addr         = MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x01;
    write_buf[0] = kp;
    write_buf[1] = ki;
    write_buf[2] = kd;

    _i2c.writeBytes(_addr, addr, write_buf, 3);
}

/**
 * @description:
 * @param index: 0 ~3
 * @param position_point: in POSITION mode, motor will lock in this value,
 * INT32_MIN ~ INT32_MAX
 * @return: None
 */
void M5Module4EncoderMotor::setPositionPoint(uint8_t index,
                                             int32_t position_point) {
    uint8_t addr;
    uint8_t write_buf[4] = {0, 0, 0, 0};

    index        = checkIndex(index);
    addr         = MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x04;
    write_buf[0] = position_point & 0xff;
    write_buf[1] = position_point >> 8;
    write_buf[2] = position_point >> 16;
    write_buf[3] = position_point >> 24;

    // Serial.printf(" %x %x %x %x \r\n", write_buf[0], write_buf[1],
    // write_buf[2], write_buf[3]);
    _i2c.writeBytes(_addr, addr, write_buf, 4);
}

/**
 * @description:
 * @param  index: 0 ~3
 * @param  max_pwm: 0 ~ 127, POSITION mode, max speed
 * @return:
 */
void M5Module4EncoderMotor::setPostionPIDMaxSpeed(uint8_t index,
                                                  uint8_t max_pwm) {
    uint8_t addr;
    index = checkIndex(index);
    addr  = MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x08;

    _i2c.writeByte(_addr, addr, max_pwm);
}

void M5Module4EncoderMotor::setSpeedPID(uint8_t index, uint8_t kp, uint8_t ki,
                                        uint8_t kd) {
    uint8_t write_buf[3] = {0, 0, 0};
    uint8_t addr;
    index = checkIndex(index);

    addr         = MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x09;
    write_buf[0] = kp;
    write_buf[1] = ki;
    write_buf[2] = kd;

    _i2c.writeBytes(_addr, addr, write_buf, 3);
}

/**
 * @description:
 * @param index: 0 ~3
 * @param speed_point: speed_point is lock getMotorSpeed20MS(), not
 * getMotorSpeed(), maybe -20 ~ 20
 * @return: None
 */
void M5Module4EncoderMotor::setSpeedPoint(uint8_t index, int8_t speed_point) {
    uint8_t addr;
    index = checkIndex(index);
    addr  = MODULE_4ENCODERMOTOR_CONFIG_ADDR + index * 0x10 + 0x0c;

    _i2c.writeByte(_addr, addr, (uint8_t)speed_point);
}

/**
 * @description:
 * @param fw: pointer to the fw version
 * @return true or false
 */
bool M5Module4EncoderMotor::getFirewareVersion(uint8_t *fw) {
    uint8_t read_data;
    uint8_t addr;
    addr = MODULE_4ENCODERMOTOR_FIRMWARE_VERSION_ADDR;

    if (_i2c.readBytes(_addr, addr, &read_data, 1)) {
        *fw = read_data;
        return true;
    } else {
        return false;
    }
}

bool M5Module4EncoderMotor::getBootloaderVersion(uint8_t *fw) {
    uint8_t read_data;
    uint8_t addr;
    addr = MODULE_4ENCODERMOTOR_BOOTLOADER_VERSION_ADDR;

    if (_i2c.readBytes(_addr, addr, &read_data, 1)) {
        *fw = read_data;
        return true;
    } else {
        return false;
    }
}

/**
 * @description:
 * @param addr: 0 ~ 127
 * @return: None
 */
bool M5Module4EncoderMotor::setI2CAddress(uint8_t addr) {
    uint8_t reg;
    uint8_t write_buf = 0;

    if (addr > 127) addr = 127;

    reg       = MODULE_4ENCODERMOTOR_I2C_ADDRESS_ADDR;
    write_buf = addr;
    if (_i2c.writeByte(_addr, reg, write_buf)) {
        _addr = addr;
        return true;
    }
    return false;
}

/**
 * @description:
 * @return I2C Address
 */
uint8_t M5Module4EncoderMotor::getI2CAddress(void) {
    uint8_t read_data;
    uint8_t addr;
    addr = MODULE_4ENCODERMOTOR_I2C_ADDRESS_ADDR;

    _i2c.readBytes(_addr, addr, &read_data, 1);
    return read_data;
}

float M5Module4EncoderMotor::getMotorCurrent(void) {
    uint8_t data[4];
    float c;
    uint8_t *p;

    _i2c.readBytes(_addr, MODULE_4ENCODERMOTOR_CURRENT_ADDR, data, 4);
    p = (uint8_t *)&c;
    memcpy(p, data, 4);

    return c;
}

void M5Module4EncoderMotor::jumpBootloader(void) {
    uint8_t value = 1;

    _i2c.writeBytes(_addr, JUMP_TO_BOOTLOADER_REG, (uint8_t *)&value, 1);
}

uint16_t M5Module4EncoderMotor::getAnalogInput(hbridge_anolog_read_mode_t bit) {
    if (bit == _8bit) {
        uint8_t data;
        _i2c.readBytes(_addr, MODULE_4ENCODERMOTOR_ADC_8BIT_REG, &data, 1);
        return data;
    } else {
        uint8_t data[2];
        _i2c.readBytes(_addr, MODULE_4ENCODERMOTOR_ADC_12BIT_REG, data, 2);
        return (data[0] | (data[1] << 8));
    }
}

void M5Module4EncoderMotor::setSoftStartAndStop(uint8_t index, bool state) {
    uint8_t buf;
    index = checkIndex(index);
    _i2c.readBytes(_addr, MODULE_4ENCODERMOTOR_SOFT_START_STOP_ADDR, &buf, 1);
    buf = buf & ~(1 << index);
    buf = buf | (state << index);
    _i2c.writeByte(_addr, MODULE_4ENCODERMOTOR_SOFT_START_STOP_ADDR, buf);
}

bool M5Module4EncoderMotor::getSoftStartAndStop(uint8_t index) {
    uint8_t data;
    index = checkIndex(index);
    _i2c.readBytes(_addr, MODULE_4ENCODERMOTOR_SOFT_START_STOP_ADDR, &data, 1);
    return data & (1 << index);
}
