#include <Wire.h>

#include "module_RDA5807m.h"

// ============================================================
// I2C — direct access (0x11)
// ============================================================

bool CLASS_RDA5807M_DRIVER::begin(int sda, int scl) {
    Wire.begin(sda, scl);
    Wire.setClock(100000);

    _wireReady = true;
    uint16_t id = 0;
    if (!readReg(RDA5807M_REG_CHIP_ID, id)) {
        _wireReady = false;
        DEBUGRDA("I2C 0x%02X not found (SDA=%d SCL=%d)\r\n",
                 RDA5807M_I2C_ADDR, sda, scl);
        return false;
    }

    DEBUGRDA("chip ID 0x%04X (SDA=%d SCL=%d)\r\n", id, sda, scl);
    return true;
}

bool CLASS_RDA5807M_DRIVER::isWireReady() const { return _wireReady; }
bool CLASS_RDA5807M_DRIVER::isPowered() const { return _powered; }

bool CLASS_RDA5807M_DRIVER::readReg(uint8_t reg, uint16_t& val) {
    if (!_wireReady) { return false; }

    Wire.beginTransmission(RDA5807M_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { return false; }
    if (Wire.requestFrom((int)RDA5807M_I2C_ADDR, 2) != 2) { return false; }
    if (Wire.available() < 2) { return false; }

    uint8_t hi = (uint8_t)Wire.read();
    uint8_t lo = (uint8_t)Wire.read();
    val = ((uint16_t)hi << 8) | lo;
    return true;
}

bool CLASS_RDA5807M_DRIVER::writeReg(uint8_t reg, uint16_t val) {
    if (!_wireReady) { return false; }

    Wire.beginTransmission(RDA5807M_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)((val >> 8) & 0xFF));
    Wire.write((uint8_t)(val & 0xFF));
    return (Wire.endTransmission() == 0);
}

// ============================================================
// Питание и настройка
// ============================================================

bool CLASS_RDA5807M_DRIVER::powerUp() {
    // ENABLE + NEW_METHOD + RDS_EN + DHIZ, DMUTE=0 (audio mute).
    uint16_t ctrl = RDA5807M_CTRL_ENABLE |
                    RDA5807M_CTRL_NEW_METHOD |
                    RDA5807M_CTRL_RDS_EN |
                    RDA5807M_CTRL_DHIZ;
    if (!writeReg(RDA5807M_REG_CTRL, ctrl)) { return false; }

    // De-emphasis 50 us + soft mute; RDS FIFO не используем.
    uint16_t audio = RDA5807M_AUDIO_DE | RDA5807M_AUDIO_SOFTMUTE;
    if (!writeReg(RDA5807M_REG_AUDIO, audio)) { return false; }

    // LNA_PORT_SEL=2 (LNAP), SEEKTH=8, VOLUME=0.
    uint16_t vol = ((uint16_t)2 << RDA5807M_VOLUME_LNA_PORT_SHIFT) |
                   ((uint16_t)8 << RDA5807M_VOLUME_SEEKTH_SHIFT);
    if (!writeReg(RDA5807M_REG_VOLUME, vol)) { return false; }

    uint16_t tune = ((uint16_t)(_band & 0x03) << 2) |
                    (uint16_t)spaceFromStep(_stepKhz);
    if (!writeReg(RDA5807M_REG_TUNE, tune)) { return false; }

    uint16_t ctrlReadback = 0;
    if (readReg(RDA5807M_REG_CTRL, ctrlReadback)) {
        DEBUGRDA("ctrl readback 0x%04X\r\n", ctrlReadback);
    }

    _powered = true;
    return true;
}

bool CLASS_RDA5807M_DRIVER::powerDown() {
    _powered = false;
    if (!_wireReady) { return true; }
    return writeReg(RDA5807M_REG_CTRL, 0);
}

bool CLASS_RDA5807M_DRIVER::setBand(uint8_t band) {
    if (band > 3) { return false; }
    _band = band;
    return true;
}

bool CLASS_RDA5807M_DRIVER::setStep(uint16_t stepKhz) {
    if (stepKhz != 100 && stepKhz != 50 && stepKhz != 200 && stepKhz != 25) {
        return false;
    }
    _stepKhz = stepKhz;
    return true;
}

bool CLASS_RDA5807M_DRIVER::tune(uint32_t freq) {
    if (!_powered) { return false; }
    uint16_t channel = channelFromFreq(freq, _band, _stepKhz);
    if (channel == 0xFFFF) { return false; }

    uint16_t reg = ((uint16_t)channel << 6) |
                   ((uint16_t)(_band & 0x03) << 2) |
                   (uint16_t)spaceFromStep(_stepKhz) |
                   RDA5807M_TUNE_TUNE;
    if (!writeReg(RDA5807M_REG_TUNE, reg)) { return false; }

    _freq = freq;
    return true;
}

uint32_t CLASS_RDA5807M_DRIVER::currentFreq() const { return _freq; }
uint8_t  CLASS_RDA5807M_DRIVER::currentBand() const { return _band; }
uint16_t CLASS_RDA5807M_DRIVER::currentStepKhz() const { return _stepKhz; }

// ============================================================
// Статус и RDS-блоки
// ============================================================

bool CLASS_RDA5807M_DRIVER::readStatus(uint16_t& statusA, uint16_t& statusB) {
    if (!readReg(RDA5807M_REG_STATUS_A, statusA)) { return false; }
    if (!readReg(RDA5807M_REG_STATUS_B, statusB)) { return false; }
    return true;
}

bool CLASS_RDA5807M_DRIVER::readRdsBlocks(uint16_t& blockA, uint16_t& blockB,
                                          uint16_t& blockC, uint16_t& blockD) {
    if (!readReg(RDA5807M_REG_RDS_A, blockA)) { return false; }
    if (!readReg(RDA5807M_REG_RDS_B, blockB)) { return false; }
    if (!readReg(RDA5807M_REG_RDS_C, blockC)) { return false; }
    if (!readReg(RDA5807M_REG_RDS_D, blockD)) { return false; }
    return true;
}

uint8_t CLASS_RDA5807M_DRIVER::groupType(uint16_t blockB) {
    return (uint8_t)((blockB >> 12) & 0x0F);
}

uint8_t CLASS_RDA5807M_DRIVER::groupVersion(uint16_t blockB) {
    return (uint8_t)((blockB >> 11) & 0x01);
}

char CLASS_RDA5807M_DRIVER::rdsChar(uint8_t c) {
    if (c >= 0x20 && c <= 0x7E) { return (char)c; }
    return ' ';
}

bool CLASS_RDA5807M_DRIVER::decodeClock(uint16_t blockB, uint16_t blockC,
                                        uint16_t blockD, time_t& utc,
                                        int16_t& offsetMin) {
    if (groupType(blockB) != 4 || groupVersion(blockB) != 0) { return false; }

    uint32_t mjd = ((uint32_t)(blockB & 0x0003) << 15) | ((uint32_t)blockC >> 1);
    uint8_t hour = (uint8_t)(((blockC & 0x0001) << 4) | (blockD >> 12));
    uint8_t minute = (uint8_t)((blockD >> 6) & 0x3F);
    uint8_t offsetSign = (uint8_t)((blockD >> 5) & 0x01);
    uint8_t offsetHalf = (uint8_t)(blockD & 0x1F);

    // 2020-01-01 .. 2100-01-01; epoch ограничиваем 32-битным time_t.
    if (mjd < 58849 || mjd > 88069) { return false; }
    if (hour > 23 || minute > 59 || offsetHalf > 31) { return false; }

    int64_t epoch = (int64_t)(mjd - 40587) * 86400 +
                    (int64_t)hour * 3600 +
                    (int64_t)minute * 60;
    if (epoch < 1577836800LL || epoch > 2147483647LL) { return false; }

    utc = (time_t)epoch;
    offsetMin = (int16_t)((offsetSign != 0) ? -1 : 1) * (int16_t)offsetHalf * 30;
    return true;
}

// ============================================================
// Диапазоны и каналы
// ============================================================

uint32_t CLASS_RDA5807M_DRIVER::bandBaseFreq(uint8_t band) {
    switch (band) {
        case 1:  return 76000;
        case 2:  return 76000;
        case 3:  return 65000;
        default: return 87000;
    }
}

uint32_t CLASS_RDA5807M_DRIVER::bandScanStart(uint8_t band) {
    if (band == 0) { return 87500; }
    return bandBaseFreq(band);
}

uint32_t CLASS_RDA5807M_DRIVER::bandScanEnd(uint8_t band) {
    switch (band) {
        case 1:  return 91000;
        case 2:  return 108000;
        case 3:  return 76000;
        default: return 108000;
    }
}

uint8_t CLASS_RDA5807M_DRIVER::spaceFromStep(uint16_t stepKhz) {
    switch (stepKhz) {
        case 200: return 1;
        case 50:  return 2;
        case 25:  return 3;
        default:  return 0; // 100 кГц
    }
}

uint16_t CLASS_RDA5807M_DRIVER::channelFromFreq(uint32_t freq, uint8_t band,
                                                 uint16_t stepKhz) {
    uint32_t base = bandBaseFreq(band);
    if (freq < base) { return 0xFFFF; }
    if (stepKhz == 0) { return 0xFFFF; }

    uint32_t channel = (freq - base) / stepKhz;
    if (channel > 0x3FF) { return 0xFFFF; }
    return (uint16_t)channel;
}
