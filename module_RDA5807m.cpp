#include "core_web/FSWebServerLib.h"

#include "core_json/core_json.h"
#include "core_state/core_state.h"
#include "core_sys/core_sys.h"

#include "module_RDA5807m.h"
#include "common/common.h"
#include "module_RDA5807m_version.h"
#include "core_sys/eertos.h"

#include <stdio.h>
#include <string.h>

// Имена значений ENUM-ресурса rda5807m.mode.
static const char* const kRdaModeNames[] = { "auto", "off" };

CLASS_MODULE_RDA5807M module_rda5807m;

// ============================================================
// Колбэки источника времени
// ============================================================

static bool rdaTimeGet(time_t& out) {
    return module_rda5807m.timeGet(out);
}

static const char* rdaTimeStatus() {
    return module_rda5807m.timeStatus();
}

// ============================================================
// Колбэки resource-bus
// ============================================================

int CLASS_MODULE_RDA5807M::s_cbMode(void* user, int argc, const BusValue* argv,
                                     BusValue& result) {
    (void)result;
    CLASS_MODULE_RDA5807M* self = (CLASS_MODULE_RDA5807M*)user;
    if (self == nullptr || argc < 1) { return BUS_ERR_BAD_VALUE; }
    int mode = (int)argv[0].i;
    if (mode != RDA_MODE_AUTO && mode != RDA_MODE_OFF) { return BUS_ERR_BAD_VALUE; }
    self->setMode((uint8_t)mode);
    return BUS_OK;
}

int CLASS_MODULE_RDA5807M::s_cbSave(void* user, int argc, const BusValue* argv,
                                     BusValue& result) {
    (void)argc; (void)argv; (void)result;
    CLASS_MODULE_RDA5807M* self = (CLASS_MODULE_RDA5807M*)user;
    if (self == nullptr) { return BUS_ERR_INTERNAL; }
    self->saveNow();
    return BUS_OK;
}

int CLASS_MODULE_RDA5807M::s_cbSaveMode(void* user, int argc, const BusValue* argv,
                                         BusValue& result) {
    (void)argc; (void)argv; (void)result;
    CLASS_MODULE_RDA5807M* self = (CLASS_MODULE_RDA5807M*)user;
    if (self == nullptr) { return BUS_ERR_INTERNAL; }
    return self->saveModeConfig() ? BUS_OK : BUS_ERR_INTERNAL;
}

void CLASS_MODULE_RDA5807M::s_deferredSave() {
    module_rda5807m._pendingSave = false;
    module_rda5807m.saveConfig();
}

// ============================================================
// Конструктор
// ============================================================

CLASS_MODULE_RDA5807M::CLASS_MODULE_RDA5807M() {
    _fs = nullptr;
    _mode = RDA_MODE_AUTO;
    _scanning = false;
    _phase = RDA_PHASE_IDLE;
    _phaseDeadline = 0;
    _lastTickMs = 0;
    _nextScanAt = 0;
    _scanStartMs = 0;
    _scanFreq = 0;
    _listenStartMs = 0;
    _tuneOk = false;
    _detectedCount = 0;
    _scanMaxRssi = 0;
    _stationCount = 0;
    _listenIndex = 0;
    _timeIndex = 0;
    _ctFound = false;
    _rdsSeen = false;
    _lastRdsEpoch = 0;
    _lastRdsMs = 0;
    _i2cFault = false;
    _i2cFailCount = 0;
    _pendingSave = false;

    for (uint8_t i = 0; i < RDA_RDS_MAX_STATIONS; i++) {
        memset(&_stations[i], 0, sizeof(_stations[i]));
    }

    defaultConfig();
}

#if defined(ESP32)
void CLASS_MODULE_RDA5807M::setFs(fs::LittleFSFS* fs)
#endif
#if defined(ESP8266)
void CLASS_MODULE_RDA5807M::setFs(FS* fs)
#endif
{
    _fs = fs;
}

// ============================================================
// begin()
// ============================================================

void CLASS_MODULE_RDA5807M::begin() {
    DEBUGRDA("%s\r\n", __FUNCTION__);

    defaultConfig();
    if (loadConfig() == false) { saveConfig(); }

    _lastRdsEpoch = 0;
    _lastRdsMs = 0;
    _i2cFault = false;
    _i2cFailCount = 0;

    // register_resources() уже вызван registry-генератором до begin().
    core_state.signal("rda5807m.mode", BusValue::en((int32_t)_mode));

    if (_mode == RDA_MODE_OFF) {
        _nextScanAt = 0;
    } else if (_config.scanOnBoot) {
        _nextScanAt = millis() + RDA_SCAN_START_DELAY_MS;
    } else {
        _nextScanAt = millis() + (_config.scanIntervalS * 1000UL);
    }

    DEBUGRDA("mode=%s sda=%d scl=%d minRssi=%u band=%u step=%u interval=%lu boot=%d\r\n",
             (_mode == RDA_MODE_OFF) ? "off" : "auto",
             resolvedSda(), resolvedScl(), _config.minRssi, _config.band,
             _config.stepKhz, (unsigned long)_config.scanIntervalS,
             _config.scanOnBoot);
}

void CLASS_MODULE_RDA5807M::begin(ModContext& ctx) {
    _fs = ctx.fs;
    begin();
}

// ============================================================
// web_Init()
// ============================================================

void CLASS_MODULE_RDA5807M::web_Init() {
    DEBUGRDA("%s\r\n", __FUNCTION__);

    ESPHTTPServer.on("/rda/info", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!ESPHTTPServer.checkAuth(request)) { return request->requestAuthentication(); }
        this->handleInfo(request);
    });

    ESPHTTPServer.on("/rda/scan", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!ESPHTTPServer.checkAuth(request)) { return request->requestAuthentication(); }
        this->handleScan(request);
    });

    ESPHTTPServer.on("/rda/save", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!ESPHTTPServer.checkAuth(request)) { return request->requestAuthentication(); }
        this->handleSave(request);
    });

    ESPHTTPServer.on("/rda/ver", HTTP_GET, [this](AsyncWebServerRequest *request) {
        this->html_ver_get(request);
    });
}

// ============================================================
// register_resources() и registerTimeSource()
// ============================================================

void CLASS_MODULE_RDA5807M::register_resources() {
    DEBUGRDA("%s\r\n", __FUNCTION__);

    core_state.regEnum("mode", 2, kRdaModeNames, "режим модуля (auto/off)");

    core_state.regFunc("mode", "->", "сменить режим (auto/off)", s_cbMode, this);
    core_state.regFunc("save", "->", "сохранить /config_rda5807m.json", s_cbSave, this);
    core_state.regFunc("save_mode", "->", "сохранить только режим модуля", s_cbSaveMode, this);

    core_state.regFuncCode("rda5807m.mode", BUS_ERR_BAD_VALUE,
                           "значение вне диапазона (0=auto, 1=off)");
}

void CLASS_MODULE_RDA5807M::registerTimeSource() {
    DEBUGRDA("%s\r\n", __FUNCTION__);
    // Источник read-only, приоритет ниже NTP/DS3231 и выше HTTP.
    core_sys.addTimeSource("rda5807m", 40, rdaTimeGet, nullptr, rdaTimeStatus);
}

// ============================================================
// Веб-обработчики
// ============================================================

void CLASS_MODULE_RDA5807M::handleInfo(AsyncWebServerRequest *request) {
    DEBUGRDA("%s\r\n", __FUNCTION__);

    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    resp->print("{\"mode\":");
    resp->print((int)_mode);
    resp->print(",\"modeName\":\"");
    resp->print((_mode == RDA_MODE_OFF) ? "off" : "auto");
    resp->print("\",\"scanning\":");
    resp->print(_scanning ? "true" : "false");
    resp->print(",\"phase\":\"");
    resp->print(phaseName());
    resp->print("\",\"progress\":");
    resp->print((int)scanProgress());
    resp->print(",\"freq\":");
    resp->print((unsigned)_scanFreq);
    resp->print(",\"detected\":");
    resp->print((unsigned)_detectedCount);
    resp->print(",\"maxRssi\":");
    resp->print((unsigned)_scanMaxRssi);
    resp->print(",\"stationCount\":");
    resp->print((unsigned)_stationCount);
    resp->print(",\"stationMax\":");
    resp->print((int)RDA_RDS_MAX_STATIONS);
    resp->print(",\"sda\":");
    resp->print(resolvedSda());
    resp->print(",\"scl\":");
    resp->print(resolvedScl());
    resp->print(",\"minRssi\":");
    resp->print((unsigned)_config.minRssi);
    resp->print(",\"i2cFault\":");
    resp->print(_i2cFault ? "true" : "false");
    resp->print(",\"lastSync\":");
    resp->print((long)_lastRdsEpoch);
    resp->print(",\"lastSyncAgeS\":");
    if (_lastRdsEpoch > 0) {
        resp->print((unsigned long)((millis() - _lastRdsMs) / 1000UL));
    } else {
        resp->print(0);
    }
    resp->print(",\"timeStatus\":\"");
    resp->print(escapeJson(String(timeStatus())));
    resp->print("\",\"stations\":[");

    for (uint8_t i = 0; i < _stationCount; i++) {
        strRdaStation& st = _stations[i];
        if (i > 0) { resp->print(","); }
        resp->print("{\"freq\":");
        resp->print((unsigned)st.freq);
        resp->print(",\"rssi\":");
        resp->print((unsigned)st.rssi);
        resp->print(",\"bler\":");
        resp->print((unsigned)st.bler);
        resp->print(",\"stereo\":");
        resp->print(st.stereo ? "true" : "false");
        resp->print(",\"name\":\"");
        resp->print(escapeJson(String(st.name)));
        resp->print("\",\"pi\":\"0x");
        char pi[5];
        snprintf(pi, sizeof(pi), "%04X", st.pi);
        resp->print(pi);
        resp->print("\",\"time\":");
        resp->print(st.timeValid ? (long)st.time : 0L);
        resp->print(",\"offsetMin\":");
        resp->print((int)st.offsetMin);
        resp->print(",\"timeValid\":");
        resp->print(st.timeValid ? "true" : "false");
        resp->print("}");
    }

    resp->print("]}");
    request->send(resp);
}

void CLASS_MODULE_RDA5807M::handleScan(AsyncWebServerRequest *request) {
    DEBUGRDA("%s\r\n", __FUNCTION__);

    if (_mode == RDA_MODE_OFF) {
        request->send(409, "text/plain", "mode off");
        return;
    }

    String action = "start";
    for (uint8_t i = 0; i < request->args(); i++) {
        if (request->argName(i) == "action") {
            action = request->arg(i);
            break;
        }
    }

    if (action == "stop") {
        stopScan();
        request->send(200, "text/plain", "OK");
        return;
    }

    if (action == "start") {
        startScan();
        request->send(200, "text/plain", "OK");
        return;
    }

    request->send(400, "text/plain", "bad action");
}

void CLASS_MODULE_RDA5807M::handleSave(AsyncWebServerRequest *request) {
    DEBUGRDA("%s\r\n", __FUNCTION__);

    if (request->args() == 0) {
        request->send(200, "text/plain", "OK");
        return;
    }

    bool modeChanged = false;
    for (uint8_t i = 0; i < request->args(); i++) {
        String name = request->argName(i);
        String value = request->arg(i);

        if (name == "mode") {
            int mode = value.toInt();
            if (mode != RDA_MODE_AUTO && mode != RDA_MODE_OFF) {
                request->send(400, "text/plain", "bad mode");
                return;
            }
            setMode((uint8_t)mode);
            modeChanged = true;
            continue;
        }
        if (name == "minRssi") {
            _config.minRssi = (uint8_t)constrain(value.toInt(), 0, 127);
            continue;
        }
        if (name == "scanIntervalS") {
            _config.scanIntervalS = (uint32_t)constrain(value.toInt(), 300, 86400);
            continue;
        }
        if (name == "scanOnBoot") {
            _config.scanOnBoot = (value == "true" || value == "1");
            continue;
        }
    }

    if (!saveConfig()) {
        request->send(500, "text/plain", "save failed");
        return;
    }
    if (modeChanged) {
        DEBUGRDA("mode saved: %s\r\n", (_mode == RDA_MODE_OFF) ? "off" : "auto");
    }
    request->send(200, "text/plain", "OK");
}

// ============================================================
// Конфиг
// ============================================================

void CLASS_MODULE_RDA5807M::defaultConfig() {
    _config.mode = RDA_MODE_AUTO;
    _config.sda = -1;
    _config.scl = -1;
    _config.minRssi = RDA_DEFAULT_MIN_RSSI;
    _config.band = 0;
    _config.stepKhz = 100;
    _config.scanIntervalS = RDA_DEFAULT_SCAN_INTERVAL_S;
    _config.scanOnBoot = true;
}

bool CLASS_MODULE_RDA5807M::loadConfig() {
    DEBUGRDA("%s\r\n", __FUNCTION__);
    JsonDocument doc;
    if (core_json.jsonFileLoadDoc(CONFIG_FILE_RDA5807M, doc) == false) {
        return false;
    }

    _config.mode = doc["mode"].is<int>() ? (uint8_t)doc["mode"].as<int>() : RDA_MODE_AUTO;
    if (_config.mode != RDA_MODE_AUTO && _config.mode != RDA_MODE_OFF) {
        _config.mode = RDA_MODE_AUTO;
    }

    _config.sda = doc["sda"].is<int>() ? (int16_t)doc["sda"].as<int>() : -1;
    _config.scl = doc["scl"].is<int>() ? (int16_t)doc["scl"].as<int>() : -1;
    if (_config.sda < -1 || _config.sda > 48) { _config.sda = -1; }
    if (_config.scl < -1 || _config.scl > 48) { _config.scl = -1; }

    _config.minRssi = doc["minRssi"].is<int>()
                        ? (uint8_t)constrain(doc["minRssi"].as<int>(), 0, 127)
                        : RDA_DEFAULT_MIN_RSSI;

    _config.band = doc["band"].is<int>()
                     ? (uint8_t)constrain(doc["band"].as<int>(), 0, 3)
                     : 0;

    int step = doc["stepKhz"].is<int>() ? doc["stepKhz"].as<int>() : 100;
    if (step != 100 && step != 50 && step != 200 && step != 25) { step = 100; }
    // band 2 (76..108 МГц) с шагом 25 кГц не влезает в 10-битный CHAN.
    if (_config.band == 2 && step == 25) { step = 50; }
    _config.stepKhz = (uint16_t)step;

    _config.scanIntervalS = doc["scanIntervalS"].is<unsigned long>()
                              ? (uint32_t)doc["scanIntervalS"].as<unsigned long>()
                              : RDA_DEFAULT_SCAN_INTERVAL_S;
    if (_config.scanIntervalS < 300 || _config.scanIntervalS > 86400) {
        _config.scanIntervalS = RDA_DEFAULT_SCAN_INTERVAL_S;
    }

    _config.scanOnBoot = doc["scanOnBoot"].is<bool>()
                           ? doc["scanOnBoot"].as<bool>()
                           : true;

    _mode = _config.mode;
    DEBUGRDA("config loaded: mode=%u sda=%d scl=%d minRssi=%u band=%u step=%u interval=%lu boot=%d\r\n",
             _config.mode, _config.sda, _config.scl, _config.minRssi, _config.band,
             _config.stepKhz, (unsigned long)_config.scanIntervalS, _config.scanOnBoot);
    return true;
}

bool CLASS_MODULE_RDA5807M::saveConfig() {
    DEBUGRDA("%s\r\n", __FUNCTION__);
    JsonDocument doc;
    core_json.jsonFileLoadDoc(CONFIG_FILE_RDA5807M, doc);

    doc["mode"] = _config.mode;
    doc["sda"] = _config.sda;
    doc["scl"] = _config.scl;
    doc["minRssi"] = _config.minRssi;
    doc["band"] = _config.band;
    doc["stepKhz"] = _config.stepKhz;
    doc["scanIntervalS"] = _config.scanIntervalS;
    doc["scanOnBoot"] = _config.scanOnBoot;

    return core_json.jsonFileSaveDoc(CONFIG_FILE_RDA5807M, doc);
}

bool CLASS_MODULE_RDA5807M::saveModeConfig() {
    JsonDocument doc;
    if (core_json.jsonFileLoadDoc(CONFIG_FILE_RDA5807M, doc) == false) {
        // Файла нет — сохраняем полный конфиг с текущими значениями.
        return saveConfig();
    }
    doc["mode"] = _config.mode;
    return core_json.jsonFileSaveDoc(CONFIG_FILE_RDA5807M, doc);
}

void CLASS_MODULE_RDA5807M::saveNow() {
    if (_pendingSave) { return; }
    _pendingSave = true;
    SetTask(&CLASS_MODULE_RDA5807M::s_deferredSave);
}

int CLASS_MODULE_RDA5807M::resolvedSda() const {
    return (_config.sda >= 0) ? (int)_config.sda : RDA_DEFAULT_SDA;
}

int CLASS_MODULE_RDA5807M::resolvedScl() const {
    return (_config.scl >= 0) ? (int)_config.scl : RDA_DEFAULT_SCL;
}

uint32_t CLASS_MODULE_RDA5807M::staleMs() const {
    uint32_t interval = _config.scanIntervalS;
    if (interval < 1800UL) { interval = 1800UL; }
    uint32_t stale = interval * 2UL;
    if (stale < RDA_TIME_STALE_MIN_S) { stale = RDA_TIME_STALE_MIN_S; }
    return stale * 1000UL;
}

// ============================================================
// Версионные методы
// ============================================================

String CLASS_MODULE_RDA5807M::getVersionStr() {
    return String(MODULE_RDA5807M_VERSION);
}

String CLASS_MODULE_RDA5807M::getGeneratedTime() {
    return String(MODULE_RDA5807M_GENERATED_TIME);
}

String CLASS_MODULE_RDA5807M::getCommitDateStr() {
    return String(MODULE_RDA5807M_COMMIT_DATE_STR);
}

void CLASS_MODULE_RDA5807M::html_ver_get(AsyncWebServerRequest *request) {
    DEBUGRDA("%s\r\n", __FUNCTION__);
    String values = "";
    values += "rdaversion|"  + getVersionStr()    + "|div\n";
    values += "rdagentime|"  + getGeneratedTime() + "|div\n";
    values += "rdagendate|"  + getCommitDateStr() + "|div\n";
    request->send(200, "text/plain", values);
}
