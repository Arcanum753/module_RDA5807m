#ifndef _MODULE_RDA5807M_DRIVER_h
#define _MODULE_RDA5807M_DRIVER_h

// ============================================================
// module_RDA5807m_driver.h — низкоуровневый драйвер RDA5807M.
// Только I2C и регистры микросхемы; без привязки к web/core_state.
// ============================================================

#include "module_RDA5807m_types.h"

class CLASS_RDA5807M_DRIVER {
public:
    bool begin(int sda, int scl);
    bool isWireReady() const;
    bool isPowered() const;

    bool powerUp();
    bool powerDown();

    bool setBand(uint8_t band);
    bool setStep(uint16_t stepKhz);
    bool tune(uint32_t freq);

    bool readStatus(uint16_t& statusA, uint16_t& statusB);
    bool readRdsBlocks(uint16_t& blockA, uint16_t& blockB,
                       uint16_t& blockC, uint16_t& blockD);
    bool readReg(uint8_t reg, uint16_t& val);
    bool writeReg(uint8_t reg, uint16_t val);

    uint32_t currentFreq() const;
    uint8_t  currentBand() const;
    uint16_t currentStepKhz() const;

    // Чистые функции декодирования RDS (используются engine и тестами).
    static uint8_t  groupType(uint16_t blockB);
    static uint8_t  groupVersion(uint16_t blockB);
    static bool     decodeClock(uint16_t blockB, uint16_t blockC, uint16_t blockD,
                                time_t& utc, int16_t& offsetMin);
    static char     rdsChar(uint8_t c);
    static uint32_t bandBaseFreq(uint8_t band);   // база формулы CHAN, кГц
    static uint32_t bandScanStart(uint8_t band);  // начало скана, кГц
    static uint32_t bandScanEnd(uint8_t band);    // конец скана, кГц
    static uint8_t  spaceFromStep(uint16_t stepKhz);
    static uint16_t channelFromFreq(uint32_t freq, uint8_t band, uint16_t stepKhz);

private:
    bool _wireReady = false;
    bool _powered = false;
    uint8_t _band = 0;
    uint16_t _stepKhz = 100;
    uint32_t _freq = 0;
};

#endif // _MODULE_RDA5807M_DRIVER_h
