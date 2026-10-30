#ifndef _MODULE_RDA5807M_TYPES_h
#define _MODULE_RDA5807M_TYPES_h

// ============================================================
// module_RDA5807m_types.h — типы, структуры и define'ы модуля RDA5807M.
// Реализация: module_RDA5807m_driver.cpp (драйвер микросхемы),
// module_RDA5807m.cpp (шаблонный блок) и module_RDA5807m_engine.cpp
// (автомат сканирования, список станций, источник времени).
// ============================================================

#include <Arduino.h>
#include <stdint.h>
#include <time.h>

#define CONFIG_FILE_RDA5807M            "/config_rda5807m.json"

// I2C direct-access адрес. Полный доступ 0x10 не используется: он конфликтует
// с VEML7700 и другими устройствами на 0x10.
#define RDA5807M_I2C_ADDR               0x11

// Регистры RDA5807M (16-битные, старший байт передаётся первым)
#define RDA5807M_REG_CHIP_ID            0x00
#define RDA5807M_REG_CTRL               0x02
#define RDA5807M_REG_TUNE               0x03
#define RDA5807M_REG_AUDIO              0x04
#define RDA5807M_REG_VOLUME             0x05
#define RDA5807M_REG_OPEN               0x06
#define RDA5807M_REG_BLEND              0x07
#define RDA5807M_REG_STATUS_A           0x0A
#define RDA5807M_REG_STATUS_B           0x0B
#define RDA5807M_REG_RDS_A              0x0C
#define RDA5807M_REG_RDS_B              0x0D
#define RDA5807M_REG_RDS_C              0x0E
#define RDA5807M_REG_RDS_D              0x0F

// Биты регистра 0x02 (CTRL)
#define RDA5807M_CTRL_ENABLE            (1u << 0)
#define RDA5807M_CTRL_SOFT_RESET        (1u << 1)
#define RDA5807M_CTRL_NEW_METHOD        (1u << 2)
#define RDA5807M_CTRL_RDS_EN            (1u << 3)
#define RDA5807M_CTRL_SEEK              (1u << 8)
#define RDA5807M_CTRL_SEEKUP            (1u << 9)
#define RDA5807M_CTRL_BASS              (1u << 12)
#define RDA5807M_CTRL_MONO              (1u << 13)
#define RDA5807M_CTRL_DMUTE             (1u << 14)
#define RDA5807M_CTRL_DHIZ              (1u << 15)

// Биты регистра 0x03 (TUNE)
#define RDA5807M_TUNE_TUNE              (1u << 4)
#define RDA5807M_TUNE_DIRECT            (1u << 5)

// Биты регистра 0x04 (AUDIO)
#define RDA5807M_AUDIO_SOFTMUTE         (1u << 9)
#define RDA5807M_AUDIO_DE               (1u << 11)
#define RDA5807M_AUDIO_RDS_FIFO_EN      (1u << 12)

// Биты регистра 0x05 (VOLUME)
#define RDA5807M_VOLUME_LNA_PORT_SHIFT  6
#define RDA5807M_VOLUME_SEEKTH_SHIFT    8

// Биты регистра 0x0A (STATUS_A)
#define RDA5807M_STATUS_A_ST            (1u << 10)
#define RDA5807M_STATUS_A_RDSS          (1u << 12)
#define RDA5807M_STATUS_A_SF            (1u << 13)
#define RDA5807M_STATUS_A_STC           (1u << 14)
#define RDA5807M_STATUS_A_RDSR          (1u << 15)

// Биты регистра 0x0B (STATUS_B)
#define RDA5807M_STATUS_B_BLERB_SHIFT   0
#define RDA5807M_STATUS_B_BLERA_SHIFT   2
#define RDA5807M_STATUS_B_ABCD_E        (1u << 4)
#define RDA5807M_STATUS_B_FM_READY      (1u << 7)
#define RDA5807M_STATUS_B_FM_TRUE       (1u << 8)
#define RDA5807M_STATUS_B_RSSI_SHIFT    9

// Пределы и таймауты
#define RDA_RDS_MAX_STATIONS            12
#define RDA_DEFAULT_MIN_RSSI            28
#define RDA_DEFAULT_SCAN_INTERVAL_S     3600UL
#define RDA_SCAN_START_DELAY_MS         5000UL
#define RDA_TICK_MS                     20UL
#define RDA_TUNE_TIMEOUT_MS             300UL
#define RDA_NAME_LISTEN_MS              4000UL
#define RDA_NAME_RDS_GRACE_MS           1500UL
#define RDA_TIME_LISTEN_MS              65000UL
#define RDA_SCAN_GLOBAL_TIMEOUT_MS      (15UL * 60UL * 1000UL)
#define RDA_TIME_STALE_MIN_S            3600UL
#define RDA_I2C_RETRY_MAX               3

// Режим модуля (ресурс rda5807m.mode)
#define RDA_MODE_AUTO                   0
#define RDA_MODE_OFF                    1

// Фазы автомата сканирования
enum RdaScanPhase : uint8_t {
    RDA_PHASE_IDLE = 0,
    RDA_PHASE_POWER_WAIT,
    RDA_PHASE_BAND_TUNE,
    RDA_PHASE_BAND_WAIT,
    RDA_PHASE_NAMES_TUNE,
    RDA_PHASE_NAMES_LISTEN,
    RDA_PHASE_TIME_TUNE,
    RDA_PHASE_TIME_LISTEN
};

// Пины по умолчанию (в конфиге -1 = платформенный дефолт)
#if defined(ESP32)
#define RDA_DEFAULT_SDA                 21
#define RDA_DEFAULT_SCL                 22
#endif

#if defined(ESP8266)
#define RDA_DEFAULT_SDA                 4
#define RDA_DEFAULT_SCL                 5
#endif

// ============================================================
// Конфиг — сохраняется в /config_rda5807m.json
// ============================================================
typedef struct {
    uint8_t  mode;             // RDA_MODE_AUTO / RDA_MODE_OFF
    int16_t  sda;              // -1 = платформенный дефолт
    int16_t  scl;              // -1 = платформенный дефолт
    uint8_t  minRssi;          // программный порог RSSI, 0..127
    uint8_t  band;             // 0..3
    uint16_t stepKhz;          // 100/50/200/25
    uint32_t scanIntervalS;    // период планового скана, сек
    bool     scanOnBoot;       // запускать скан после старта
} strRda5807mConfig;

// ============================================================
// Станция и результат RDS
// ============================================================
typedef struct {
    uint32_t freq;             // частота в кГц (87500 = 87.50 МГц)
    uint8_t  rssi;             // 0..127
    uint8_t  bler;             // 0..3 (максимум BLERA/BLERB)
    bool     stereo;           // бит ST
    uint16_t pi;               // PI-код станции
    char     name[9];          // PS: 8 символов + NUL
    uint8_t  nameMask;         // маска принятых сегментов PS (4 бита)
    time_t   time;             // UTC начала минуты из группы 4A
    int16_t  offsetMin;        // местное смещение станции, минуты
    bool     timeValid;        // время получено и прошло проверку
} strRdaStation;

#endif // _MODULE_RDA5807M_TYPES_h
