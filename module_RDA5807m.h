#ifndef _MODULE_RDA5807M_h
#define _MODULE_RDA5807M_h

#include "main.h"

#include "mod_context.h"

#if defined(DEBUG_RDA5807M)
#define DEBUGRDA(...) DBG_MOD("[M_RDA5807M] ", __VA_ARGS__)
#endif

#if !defined(DEBUG_RDA5807M)
#define DEBUGRDA(...)
#endif

#if defined(ESP32)
#include <LittleFS.h>
#endif

#if defined(ESP8266)
#include <LittleFS.h>
#endif

#include "module_RDA5807m_types.h"
#include "module_RDA5807m_driver.h"

// Полные определения — в соответствующих заголовках ядра; здесь нужны только
// для сигнатур методов и trampoline'ов.
struct BusValue;
class AsyncWebServerRequest;

// ============================================================
// module_RDA5807m — FM/RDS-приёмник RDA5807M.
// Роли: драйвер микросхемы и источник времени системы (RDS CT, группа 4A).
// Управление: auto/off через rda5807m.mode; телеметрия на шину не публикуется.
// ============================================================
class CLASS_MODULE_RDA5807M {
public:
    CLASS_MODULE_RDA5807M();

#if defined(ESP32)
    void setFs(fs::LittleFSFS* fs);
#endif
#if defined(ESP8266)
    void setFs(FS* fs);
#endif

    void begin();
    void begin(ModContext& ctx);
    void web_Init();
    void register_resources();
    void registerTimeSource();
    void loop();

    // ===== Публичное API =====
    void startScan();
    void stopScan();
    void setMode(uint8_t mode);
    uint8_t getMode() const;
    bool isScanning() const;

    // Time Source Provider API
    bool timeGet(time_t& out);
    const char* timeStatus() const;

private:
    // Версионные методы
    String getVersionStr();
    String getGeneratedTime();
    String getCommitDateStr();
    void html_ver_get(AsyncWebServerRequest *request);

    // Веб-обработчики
    void handleInfo(AsyncWebServerRequest *request);
    void handleScan(AsyncWebServerRequest *request);
    void handleSave(AsyncWebServerRequest *request);

    // Конфиг
    void defaultConfig();
    bool loadConfig();
    bool saveConfig();
    bool saveModeConfig();
    void saveNow();
    int resolvedSda() const;
    int resolvedScl() const;
    uint32_t staleMs() const;

    // Автомат сканирования
    void scanTick(uint32_t now);
    void resetScanState();
    void tuneCurrentBandChannel(uint32_t now);
    void advanceBandChannel(uint32_t now);
    void startNamesPhase(uint32_t now);
    void tuneListenStation(uint8_t index, uint8_t tunePhase, uint32_t now);
    void startListen(uint8_t index, uint8_t listenPhase, uint32_t listenMs, uint32_t now);
    void nextNameStation(uint32_t now);
    void startTimeHunt(uint32_t now);
    void finishScan(uint32_t now);
    bool pollRds(uint8_t listenPhase, uint32_t now);
    bool processRdsGroup(uint16_t blockA, uint16_t blockB, uint16_t blockC,
                         uint16_t blockD, uint16_t statusA, uint16_t statusB,
                         strRdaStation *station);
    void updateStationQuality(strRdaStation *station, uint16_t statusA, uint16_t statusB);
    void considerStation(uint32_t freq, uint16_t statusA, uint16_t statusB);
    void sortStationsByFreq();
    int  strongestStationIndex() const;
    bool ensureDriverReady();
    void noteI2cResult(bool ok);
    const char* phaseName() const;
    uint8_t scanProgress() const;

    // Trampoline'ы resource-bus
    static int s_cbMode(void* user, int argc, const BusValue* argv, BusValue& result);
    static int s_cbSave(void* user, int argc, const BusValue* argv, BusValue& result);
    static int s_cbSaveMode(void* user, int argc, const BusValue* argv, BusValue& result);
    static void s_deferredSave();

protected:
#if defined(ESP32)
    fs::LittleFSFS* _fs;
#endif
#if defined(ESP8266)
    FS*             _fs;
#endif

    strRda5807mConfig _config;
    CLASS_RDA5807M_DRIVER _driver;

    uint8_t _mode;
    bool _scanning;

    uint8_t _phase;
    uint32_t _phaseDeadline;
    uint32_t _lastTickMs;
    uint32_t _nextScanAt;
    uint32_t _scanStartMs;
    uint32_t _scanFreq;
    uint32_t _listenStartMs;
    bool _tuneOk;

    uint16_t _detectedCount;
    uint8_t _scanMaxRssi;
    uint8_t _stationCount;
    uint8_t _listenIndex;
    uint8_t _timeIndex;
    bool _ctFound;
    bool _rdsSeen;

    strRdaStation _stations[RDA_RDS_MAX_STATIONS];

    time_t _lastRdsEpoch;
    uint32_t _lastRdsMs;

    bool _i2cFault;
    uint8_t _i2cFailCount;
    bool _pendingSave;
};

extern CLASS_MODULE_RDA5807M module_rda5807m;

#endif // _MODULE_RDA5807M_h
