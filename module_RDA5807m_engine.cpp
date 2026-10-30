#include "core_web/FSWebServerLib.h"

#include "core_state/core_state.h"
#include "core_sys/core_sys.h"

#include "module_RDA5807m.h"

#include <string.h>

// ============================================================
// Вспомогательные функции
// ============================================================

static uint8_t rdaRssi(uint16_t statusB) {
    return (uint8_t)((statusB >> RDA5807M_STATUS_B_RSSI_SHIFT) & 0x7F);
}

static uint8_t rdaBler(uint16_t statusB) {
    uint8_t blerA = (uint8_t)((statusB >> RDA5807M_STATUS_B_BLERA_SHIFT) & 0x03);
    uint8_t blerB = (uint8_t)((statusB >> RDA5807M_STATUS_B_BLERB_SHIFT) & 0x03);
    return (blerA > blerB) ? blerA : blerB;
}

static void rdaClearStation(strRdaStation& st) {
    memset(&st, 0, sizeof(st));
    st.name[0] = '\0';
}

#if defined(DEBUG_RDA5807M)
// Диагностика прохода диапазона: видно, есть ли вообще STC/FM_TRUE/RSSI.
static uint16_t sDbgChCount = 0;
static uint16_t sDbgStcCount = 0;
static uint16_t sDbgFmCount = 0;
static uint8_t  sDbgMaxRssi = 0;
#endif

// ============================================================
// Публичный API
// ============================================================

void CLASS_MODULE_RDA5807M::loop() {
    uint32_t now = millis();
    if ((uint32_t)(now - _lastTickMs) < RDA_TICK_MS) { return; }
    _lastTickMs = now;

    if (_mode == RDA_MODE_OFF) {
        if (_scanning) { stopScan(); }
        return;
    }

    if (_scanning) {
        scanTick(now);
        return;
    }

    if (_nextScanAt != 0 && (int32_t)(now - _nextScanAt) >= 0) {
        startScan();
    }
}

void CLASS_MODULE_RDA5807M::startScan() {
    if (_mode == RDA_MODE_OFF || _scanning) { return; }

    uint32_t now = millis();
    resetScanState();

    _scanning = true;
    _scanStartMs = now;
    _phase = RDA_PHASE_POWER_WAIT;
    _phaseDeadline = now + 120UL;

    if (!ensureDriverReady()) {
        _i2cFault = true;
        DEBUGRDA("scan aborted: I2C not ready\r\n");
        finishScan(now);
        return;
    }

    _driver.setBand(_config.band);
    _driver.setStep(_config.stepKhz);

    if (!_driver.powerUp()) {
        noteI2cResult(false);
        _i2cFault = true;
        DEBUGRDA("scan aborted: power-up failed\r\n");
        finishScan(now);
        return;
    }
    noteI2cResult(true);

#if defined(DEBUG_RDA5807M)
    sDbgChCount = 0;
    sDbgStcCount = 0;
    sDbgFmCount = 0;
    sDbgMaxRssi = 0;
#endif

    DEBUGRDA("scan started: band=%u step=%u minRssi=%u\r\n",
             _config.band, _config.stepKhz, _config.minRssi);
}

void CLASS_MODULE_RDA5807M::stopScan() {
    uint32_t now = millis();

    if (_driver.isWireReady()) {
        _driver.powerDown();
    }

    if (_scanning) {
        DEBUGRDA("scan stopped manually\r\n");
        _scanning = false;
        _phase = RDA_PHASE_IDLE;
        _nextScanAt = now + (_config.scanIntervalS * 1000UL);
    }
}

void CLASS_MODULE_RDA5807M::setMode(uint8_t mode) {
    if (mode != RDA_MODE_AUTO && mode != RDA_MODE_OFF) { return; }
    if (mode == _mode) { return; }

    _mode = mode;
    _config.mode = mode;

    if (_mode == RDA_MODE_OFF) {
        stopScan();
        _nextScanAt = 0;
    } else {
        _i2cFault = false;
        _i2cFailCount = 0;
        _nextScanAt = millis();
    }

    core_state.signal("rda5807m.mode", BusValue::en((int32_t)_mode));
    DEBUGRDA("mode=%s\r\n", (_mode == RDA_MODE_OFF) ? "off" : "auto");
}

uint8_t CLASS_MODULE_RDA5807M::getMode() const { return _mode; }
bool CLASS_MODULE_RDA5807M::isScanning() const { return _scanning; }

// ============================================================
// Источник времени
// ============================================================

bool CLASS_MODULE_RDA5807M::timeGet(time_t& out) {
    if (_mode == RDA_MODE_OFF || _i2cFault || _lastRdsEpoch == 0) {
        return false;
    }

    uint32_t age = millis() - _lastRdsMs;
    if (age > staleMs()) { return false; }

    out = _lastRdsEpoch + (time_t)(age / 1000UL);
    return true;
}

const char* CLASS_MODULE_RDA5807M::timeStatus() const {
    if (_mode == RDA_MODE_OFF) { return "module off"; }
    if (_i2cFault) { return "i2c error"; }
    if (_lastRdsEpoch == 0) { return "no rds time"; }
    if ((millis() - _lastRdsMs) > staleMs()) { return "rds time stale"; }
    return "";
}

// ============================================================
// Автомат сканирования
// ============================================================

void CLASS_MODULE_RDA5807M::scanTick(uint32_t now) {
    if ((uint32_t)(now - _scanStartMs) > RDA_SCAN_GLOBAL_TIMEOUT_MS) {
        DEBUGRDA("scan timeout\r\n");
        finishScan(now);
        return;
    }

    switch (_phase) {
        case RDA_PHASE_POWER_WAIT: {
            if ((int32_t)(now - _phaseDeadline) < 0) { return; }
            _scanFreq = CLASS_RDA5807M_DRIVER::bandScanStart(_config.band);
            tuneCurrentBandChannel(now);
            break;
        }

        case RDA_PHASE_BAND_WAIT: {
            uint16_t statusA = 0;
            uint16_t statusB = 0;
            if (!_driver.readStatus(statusA, statusB)) {
                noteI2cResult(false);
                if (_i2cFault) { finishScan(now); }
                return;
            }
            noteI2cResult(true);

            bool stc = (statusA & RDA5807M_STATUS_A_STC) != 0;
            if (!stc && (int32_t)(now - _phaseDeadline) < 0) { return; }

            uint8_t bandRssi = rdaRssi(statusB);
            if (bandRssi > _scanMaxRssi) { _scanMaxRssi = bandRssi; }

#if defined(DEBUG_RDA5807M)
            sDbgChCount++;
            if (stc) { sDbgStcCount++; }
            uint8_t dbgRssi = rdaRssi(statusB);
            if (dbgRssi > sDbgMaxRssi) { sDbgMaxRssi = dbgRssi; }
            if (statusB & RDA5807M_STATUS_B_FM_TRUE) { sDbgFmCount++; }
            uint8_t dbgLogRssi = (_config.minRssi > 10) ? _config.minRssi : 10;
            if (sDbgChCount <= 5 || (statusB & RDA5807M_STATUS_B_FM_TRUE) ||
                dbgRssi >= dbgLogRssi) {
                DEBUGRDA("ch %lu A=0x%04X B=0x%04X rssi=%u fm=%u stc=%u\r\n",
                         (unsigned long)(_config.stepKhz ?
                             (CLASS_RDA5807M_DRIVER::bandBaseFreq(_config.band) +
                              (uint32_t)(statusA & 0x3FF) * _config.stepKhz) / 1000UL : 0UL),
                         statusA, statusB, dbgRssi,
                         (statusB & RDA5807M_STATUS_B_FM_TRUE) ? 1 : 0,
                         stc ? 1 : 0);
            }
#endif

            if (stc) {
                uint32_t base = CLASS_RDA5807M_DRIVER::bandBaseFreq(_config.band);
                uint32_t ch = statusA & 0x03FF;
                uint32_t freq = base + ch * _config.stepKhz;
                if ((statusB & RDA5807M_STATUS_B_FM_TRUE) &&
                    rdaRssi(statusB) >= _config.minRssi) {
                    considerStation(freq, statusA, statusB);
                }
            }
            advanceBandChannel(now);
            break;
        }

        case RDA_PHASE_NAMES_TUNE: {
            if ((int32_t)(now - _phaseDeadline) < 0) { return; }
            if (!_tuneOk) {
                if (_i2cFault) { finishScan(now); }
                else { nextNameStation(now); }
                return;
            }
            startListen(_listenIndex, RDA_PHASE_NAMES_LISTEN, RDA_NAME_LISTEN_MS, now);
            break;
        }

        case RDA_PHASE_NAMES_LISTEN: {
            if (pollRds(RDA_PHASE_NAMES_LISTEN, now)) {
                if (_i2cFault) { finishScan(now); }
                else { nextNameStation(now); }
            }
            break;
        }

        case RDA_PHASE_TIME_TUNE: {
            if ((int32_t)(now - _phaseDeadline) < 0) { return; }
            if (!_tuneOk) {
                finishScan(now);
                return;
            }
            startListen(_listenIndex, RDA_PHASE_TIME_LISTEN, RDA_TIME_LISTEN_MS, now);
            break;
        }

        case RDA_PHASE_TIME_LISTEN: {
            if (pollRds(RDA_PHASE_TIME_LISTEN, now)) {
                finishScan(now);
            }
            break;
        }

        default:
            finishScan(now);
            break;
    }
}

void CLASS_MODULE_RDA5807M::resetScanState() {
    for (uint8_t i = 0; i < RDA_RDS_MAX_STATIONS; i++) {
        rdaClearStation(_stations[i]);
    }
    _detectedCount = 0;
    _scanMaxRssi = 0;
    _stationCount = 0;
    _listenIndex = 0;
    _timeIndex = 0;
    _ctFound = false;
    _rdsSeen = false;
    _tuneOk = false;
    _listenStartMs = 0;
    _scanFreq = CLASS_RDA5807M_DRIVER::bandScanStart(_config.band);
}

void CLASS_MODULE_RDA5807M::tuneCurrentBandChannel(uint32_t now) {
    uint32_t end = CLASS_RDA5807M_DRIVER::bandScanEnd(_config.band);
    if (_scanFreq > end) {
        startNamesPhase(now);
        return;
    }

    if (CLASS_RDA5807M_DRIVER::channelFromFreq(_scanFreq, _config.band,
                                                 _config.stepKhz) == 0xFFFF) {
        advanceBandChannel(now);
        return;
    }

    if (!_driver.tune(_scanFreq)) {
        noteI2cResult(false);
        if (_i2cFault) {
            finishScan(now);
            return;
        }
        // Разовый сбой I2C/перестройки: пропускаем канал, не читая старый статус.
        advanceBandChannel(now);
        return;
    }

    _phase = RDA_PHASE_BAND_WAIT;
    _phaseDeadline = now + RDA_TUNE_TIMEOUT_MS;
}

void CLASS_MODULE_RDA5807M::advanceBandChannel(uint32_t now) {
    uint32_t step = _config.stepKhz;
    if (step == 0) { step = 100; }
    _scanFreq += step;

    if (_scanFreq > CLASS_RDA5807M_DRIVER::bandScanEnd(_config.band)) {
        startNamesPhase(now);
        return;
    }
    tuneCurrentBandChannel(now);
}

void CLASS_MODULE_RDA5807M::startNamesPhase(uint32_t now) {
    sortStationsByFreq();

    if (_stationCount == 0) {
        DEBUGRDA("no stations found\r\n");
        finishScan(now);
        return;
    }

    _listenIndex = 0;
    tuneListenStation(_listenIndex, RDA_PHASE_NAMES_TUNE, now);
}

void CLASS_MODULE_RDA5807M::tuneListenStation(uint8_t index, uint8_t tunePhase,
                                               uint32_t now) {
    _listenIndex = index;
    _tuneOk = false;

    if (index < _stationCount) {
        _tuneOk = _driver.tune(_stations[index].freq);
        if (!_tuneOk) { noteI2cResult(false); }
    }

    _phase = tunePhase;
    _phaseDeadline = now + ((_tuneOk) ? RDA_TUNE_TIMEOUT_MS : 0UL);
}

void CLASS_MODULE_RDA5807M::startListen(uint8_t index, uint8_t listenPhase,
                                         uint32_t listenMs, uint32_t now) {
    _listenIndex = index;
    _phase = listenPhase;
    _listenStartMs = now;
    _phaseDeadline = now + listenMs;
    _rdsSeen = false;
}

void CLASS_MODULE_RDA5807M::nextNameStation(uint32_t now) {
    _listenIndex++;
    if (_listenIndex >= _stationCount) {
        if (_ctFound) {
            finishScan(now);
        } else {
            startTimeHunt(now);
        }
        return;
    }
    tuneListenStation(_listenIndex, RDA_PHASE_NAMES_TUNE, now);
}

void CLASS_MODULE_RDA5807M::startTimeHunt(uint32_t now) {
    int strongest = strongestStationIndex();
    if (strongest < 0) {
        finishScan(now);
        return;
    }

    _timeIndex = (uint8_t)strongest;
    DEBUGRDA("time hunt on %lu kHz\r\n", (unsigned long)_stations[_timeIndex].freq);
    tuneListenStation(_timeIndex, RDA_PHASE_TIME_TUNE, now);
}

void CLASS_MODULE_RDA5807M::finishScan(uint32_t now) {
    if (_driver.isWireReady()) {
        _driver.powerDown();
    }

    _scanning = false;
    _phase = RDA_PHASE_IDLE;
    _rdsSeen = false;
    _nextScanAt = now + (_config.scanIntervalS * 1000UL);

    DEBUGRDA("scan finished: detected=%u stations=%u ct=%s\r\n",
             _detectedCount, _stationCount, _ctFound ? "yes" : "no");
#if defined(DEBUG_RDA5807M)
    DEBUGRDA("band stats: ch=%u stc=%u fm_true=%u max_rssi=%u\r\n",
             sDbgChCount, sDbgStcCount, sDbgFmCount, sDbgMaxRssi);
#endif
}

bool CLASS_MODULE_RDA5807M::pollRds(uint8_t listenPhase, uint32_t now) {
    if ((int32_t)(now - _phaseDeadline) >= 0) { return true; }

    uint16_t statusA = 0;
    uint16_t statusB = 0;
    if (!_driver.readStatus(statusA, statusB)) {
        noteI2cResult(false);
        return _i2cFault;
    }
    noteI2cResult(true);

    strRdaStation* station = (_listenIndex < _stationCount) ? &_stations[_listenIndex] : nullptr;
    if (station != nullptr) {
        updateStationQuality(station, statusA, statusB);
    }

    if (!(statusA & RDA5807M_STATUS_A_RDSR)) {
        if (listenPhase == RDA_PHASE_NAMES_LISTEN && !_rdsSeen &&
            (uint32_t)(now - _listenStartMs) >= RDA_NAME_RDS_GRACE_MS) {
            return true;
        }
        return false;
    }

    _rdsSeen = true;

    if (!(statusA & RDA5807M_STATUS_A_RDSS)) {
        return false;
    }

    uint16_t blockA = 0;
    uint16_t blockB = 0;
    uint16_t blockC = 0;
    uint16_t blockD = 0;
    if (!_driver.readRdsBlocks(blockA, blockB, blockC, blockD)) {
        noteI2cResult(false);
        return _i2cFault;
    }

    if (processRdsGroup(blockA, blockB, blockC, blockD, statusA, statusB, station)) {
        if (listenPhase == RDA_PHASE_TIME_LISTEN) { return true; }
    }

    if (listenPhase == RDA_PHASE_NAMES_LISTEN) {
        return (station != nullptr && station->nameMask == 0x0F);
    }
    return (station != nullptr && station->timeValid);
}

bool CLASS_MODULE_RDA5807M::processRdsGroup(uint16_t blockA, uint16_t blockB,
                                             uint16_t blockC, uint16_t blockD,
                                             uint16_t statusA, uint16_t statusB,
                                             strRdaStation* station) {
    if (station == nullptr) { return false; }

    uint8_t group = CLASS_RDA5807M_DRIVER::groupType(blockB);
    uint8_t version = CLASS_RDA5807M_DRIVER::groupVersion(blockB);
    uint8_t blerA = (uint8_t)((statusB >> RDA5807M_STATUS_B_BLERA_SHIFT) & 0x03);
    uint8_t blerB = (uint8_t)((statusB >> RDA5807M_STATUS_B_BLERB_SHIFT) & 0x03);

    if (group == 0) {
        if (station->pi == 0) {
            station->pi = blockA;
        } else if (blockA != 0 && station->pi != blockA) {
            station->pi = blockA;
            station->nameMask = 0;
            memset(station->name, 0, sizeof(station->name));
        }

        if (blerA > 1 || blerB > 1) { return false; }

        uint8_t addr = (uint8_t)(blockB & 0x03);
        station->name[addr * 2]     = CLASS_RDA5807M_DRIVER::rdsChar((uint8_t)(blockD >> 8));
        station->name[addr * 2 + 1] = CLASS_RDA5807M_DRIVER::rdsChar((uint8_t)(blockD & 0xFF));
        station->name[8] = '\0';
        station->nameMask |= (uint8_t)(1u << addr);
        return false;
    }

    if (group == 4 && version == 0) {
        if (blerA != 0 || blerB != 0) { return false; }
        if (!(statusA & RDA5807M_STATUS_A_RDSS)) { return false; }

        time_t utc = 0;
        int16_t offsetMin = 0;
        if (!CLASS_RDA5807M_DRIVER::decodeClock(blockB, blockC, blockD, utc, offsetMin)) {
            return false;
        }

        station->time = utc;
        station->offsetMin = offsetMin;
        station->timeValid = true;

        if (_lastRdsEpoch == 0 || utc >= _lastRdsEpoch) {
            _lastRdsEpoch = utc;
            _lastRdsMs = millis();
        }
        _ctFound = true;
        DEBUGRDA("RDS time on %lu kHz: epoch=%ld offset=%d min\r\n",
                 (unsigned long)station->freq, (long)utc, (int)offsetMin);
        return true;
    }

    return false;
}

void CLASS_MODULE_RDA5807M::updateStationQuality(strRdaStation* station,
                                                  uint16_t statusA,
                                                  uint16_t statusB) {
    if (station == nullptr) { return; }
    station->rssi = rdaRssi(statusB);
    station->bler = rdaBler(statusB);
    station->stereo = (statusA & RDA5807M_STATUS_A_ST) != 0;
}

void CLASS_MODULE_RDA5807M::considerStation(uint32_t freq, uint16_t statusA,
                                             uint16_t statusB) {
    uint8_t rssi = rdaRssi(statusB);
    uint8_t bler = rdaBler(statusB);
    bool stereo = (statusA & RDA5807M_STATUS_A_ST) != 0;

    for (uint8_t i = 0; i < _stationCount; i++) {
        if (_stations[i].freq == freq) {
            _stations[i].rssi = rssi;
            _stations[i].bler = bler;
            _stations[i].stereo = stereo;
            return;
        }
    }

    _detectedCount++;

    if (_stationCount < RDA_RDS_MAX_STATIONS) {
        strRdaStation& st = _stations[_stationCount];
        rdaClearStation(st);
        st.freq = freq;
        st.rssi = rssi;
        st.bler = bler;
        st.stereo = stereo;
        _stationCount++;
        return;
    }

    uint8_t weakest = 0;
    for (uint8_t i = 1; i < _stationCount; i++) {
        if (_stations[i].rssi < _stations[weakest].rssi) { weakest = i; }
    }
    if (rssi <= _stations[weakest].rssi) { return; }

    strRdaStation& st = _stations[weakest];
    rdaClearStation(st);
    st.freq = freq;
    st.rssi = rssi;
    st.bler = bler;
    st.stereo = stereo;
}

void CLASS_MODULE_RDA5807M::sortStationsByFreq() {
    for (uint8_t i = 1; i < _stationCount; i++) {
        strRdaStation key = _stations[i];
        int j = (int)i - 1;
        while (j >= 0 && _stations[j].freq > key.freq) {
            _stations[j + 1] = _stations[j];
            j--;
        }
        _stations[j + 1] = key;
    }
}

int CLASS_MODULE_RDA5807M::strongestStationIndex() const {
    if (_stationCount == 0) { return -1; }
    uint8_t best = 0;
    for (uint8_t i = 1; i < _stationCount; i++) {
        if (_stations[i].rssi > _stations[best].rssi) { best = i; }
    }
    return (int)best;
}

bool CLASS_MODULE_RDA5807M::ensureDriverReady() {
    if (_driver.isWireReady() && !_i2cFault) {
        _driver.setBand(_config.band);
        _driver.setStep(_config.stepKhz);
        return true;
    }

    if (!_driver.begin(resolvedSda(), resolvedScl())) {
        noteI2cResult(false);
        return false;
    }

    noteI2cResult(true);
    _driver.setBand(_config.band);
    _driver.setStep(_config.stepKhz);
    return true;
}

void CLASS_MODULE_RDA5807M::noteI2cResult(bool ok) {
    if (ok) {
        _i2cFailCount = 0;
        _i2cFault = false;
        return;
    }

    if (_i2cFailCount < 255) { _i2cFailCount++; }
    if (_i2cFailCount >= RDA_I2C_RETRY_MAX) { _i2cFault = true; }
}

const char* CLASS_MODULE_RDA5807M::phaseName() const {
    switch (_phase) {
        case RDA_PHASE_POWER_WAIT:   return "power";
        case RDA_PHASE_BAND_TUNE:
        case RDA_PHASE_BAND_WAIT:    return "band";
        case RDA_PHASE_NAMES_TUNE:
        case RDA_PHASE_NAMES_LISTEN: return "names";
        case RDA_PHASE_TIME_TUNE:
        case RDA_PHASE_TIME_LISTEN:  return "time";
        default:                     return "idle";
    }
}

uint8_t CLASS_MODULE_RDA5807M::scanProgress() const {
    if (!_scanning) { return 0; }

    if (_phase <= RDA_PHASE_BAND_WAIT) {
        uint32_t start = CLASS_RDA5807M_DRIVER::bandScanStart(_config.band);
        uint32_t end = CLASS_RDA5807M_DRIVER::bandScanEnd(_config.band);
        if (end <= start) { return 0; }
        uint32_t pos = (_scanFreq > start) ? (_scanFreq - start) : 0;
        uint32_t total = end - start;
        if (pos > total) { pos = total; }
        return (uint8_t)((pos * 60UL) / total);
    }

    if (_phase <= RDA_PHASE_NAMES_LISTEN) {
        if (_stationCount == 0) { return 60; }
        return (uint8_t)(60UL + ((uint32_t)_listenIndex * 30UL) / _stationCount);
    }

    return 90;
}
