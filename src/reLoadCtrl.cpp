#include "reLoadCtrl.h"
#include <string.h>
#include "reNvs.h"
#include "reEvents.h"
#include "reMqtt.h"
#include "reEsp32.h"
#include "rLog.h"
#include "rStrings.h"
#include "def_sntp.h"

static const char* logTAG = "LOAD";

#define ERR_LOAD_CHECK(err, str) if (err != ESP_OK) { rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err)); return false; };
#define ERR_GPIO_SET_LEVEL "Failed to change GPIO level"
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- rLoadController ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rLoadController::rLoadController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space,
  uint32_t* cycle_duration, uint32_t* cycle_interval, timeintv_t cycle_type,
  cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed,
  cb_load_publish_t cb_mqtt_publish)
{
  // Configure GPIO
  _pin = pin;
  _level_on = level_on;
  _nvs_space = nvs_space;
  _state = false;
  _last_on = 0;
  _last_off = 0;
  _cycle_duration = cycle_duration;
  _cycle_interval = cycle_interval;
  _cycle_type = cycle_type;

   // Reset pointers
  _period_start = nullptr;
  _mqtt_topic = nullptr;
  _mqtt_publish = nullptr;
  _timer_on = nullptr;
  _timer_free = !use_timer;
  _timer_cycle = nullptr;

  // Callbacks
  _gpio_before = cb_gpio_before;
  _gpio_after = cb_gpio_after;
  _state_changed = cb_state_changed;
  _mqtt_publish = cb_mqtt_publish;

  // Clear counters
  countersReset();
}

rLoadController::~rLoadController()
{
  cycleFree();
  timerFree();
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Parameters -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rLoadController::setPeriodStartDay(uint8_t* mday)
{
  _period_start = mday;
}

void rLoadController::setCallbacks(cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed)
{
  _gpio_before = cb_gpio_before;
  _gpio_after = cb_gpio_after;
  _state_changed = cb_state_changed;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- Load --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rLoadController::loadInit(bool init_state)
{
  if (!_timer_free) timerCreate();
  return loadInitGPIO() && loadSetState(init_state, true, false);
}

bool rLoadController::loadSetStatePriv(bool new_state)
{
  bool phy_level = new_state ? _level_on : !_level_on;
  if (_gpio_before) { 
    _gpio_before(this, phy_level, 0); 
  };
  bool ret = loadSetStateGPIO(phy_level);
  if (_gpio_after) { 
    _gpio_after(this, phy_level, 0); 
  };
  return ret;
}

bool rLoadController::loadSetState(bool new_state, bool forced, bool publish)
{
  if (forced || (_state != new_state)) {
    bool change_ok = false;
    if ((_cycle_duration) && (*_cycle_duration > 0) && (_cycle_interval) && (*_cycle_interval > 0)) {
      // Activate cycle timer
      change_ok = cycleSetCyclePriv(new_state);
    } else {
      // Set physical level to GPIO
      cycleFree();
      change_ok = loadSetStatePriv(new_state);
    };

    // If the change level was successful
    if (change_ok && (_state != new_state)) {
      _state = new_state;
      if (_state) {
        _last_on = time(nullptr);
        _durations.durLast = 0;
        _counters.cntTotal++;
        _counters.cntToday++;
        _counters.cntWeekCurr++;
        _counters.cntMonthCurr++;
        _counters.cntPeriodCurr++;
        _counters.cntYearCurr++;
        rlog_i(logTAG, "Load on GPIO %d is ON", _pin);
      } else {
        _last_off = time(nullptr);
        timerStop();
        // Calculate turn-on duration
        if (((_last_on <= 1000000000) && (_last_off <= 1000000000)) || ((_last_on > 1000000000) && (_last_off > 1000000000))) {
          // Сrutch: timezone correction 
          // When the first time stamp occurred before the zone was applied, and the second after it, and a negative time interval is obtained
          if (_last_on > _last_off) {
            if ((_last_on - _last_off) < CONFIG_SNTP_TIMEZONE_SECONDS) {
              _last_off =+ CONFIG_SNTP_TIMEZONE_SECONDS;
            };
          };
          if (_last_on < _last_off) {
            _durations.durLast = _last_off - _last_on;
            _durations.durTotal = _durations.durTotal + _durations.durLast;
            _durations.durToday = _durations.durToday + _durations.durLast;
            _durations.durWeekCurr = _durations.durWeekCurr + _durations.durLast;
            _durations.durMonthCurr = _durations.durMonthCurr + _durations.durLast;
            _durations.durPeriodCurr = _durations.durPeriodCurr + _durations.durLast;
            _durations.durYearCurr = _durations.durYearCurr + _durations.durLast;
          };
        };
        rlog_i(logTAG, "Load on GPIO %d is OFF", _pin);
      };

      // Publish status and counters
      if (publish) {
        mqttPublish();
      };

      // Call external callback
      if (_state_changed) { 
        _state_changed(this, _state, _durations.durLast); 
      };
      return true;
    };
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Cycle --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void loadControllerCycleEnd(void* arg)
{
  if (arg) {
    rLoadController* ctrl = (rLoadController*)arg;
    ctrl->cycleToggle();
  };
}

bool rLoadController::cycleCreate()
{
  if (_timer_cycle == nullptr) {
    esp_timer_create_args_t cfg;
    memset(&cfg, 0, sizeof(esp_timer_create_args_t));
    cfg.name = "load_ctrl_cycle";
    cfg.callback = loadControllerCycleEnd;
    cfg.arg = this;
    RE_OK_CHECK(esp_timer_create(&cfg, &_timer_cycle), return false);
  };
  return true;
}

bool rLoadController::cycleFree()
{
  if (_timer_cycle != nullptr) {
    if (esp_timer_is_active(_timer_cycle)) {
      esp_timer_stop(_timer_cycle);
    };
    if (_timer_free) {
      _timer_cycle = nullptr;
    };
  };
  return true;
}

bool rLoadController::cycleToggle()
{
  if (_timer_cycle && _cycle_duration && _cycle_interval) {
    // Stop timer if active
    if (esp_timer_is_active(_timer_cycle)) {
      esp_timer_stop(_timer_cycle);
    };
    // Switching the load
    bool new_state = !_cycle_state;
    if (loadSetStatePriv(new_state)) {
      // Calculate timer duration
      uint64_t duration = 1000 * (uint64_t)(new_state ? *_cycle_duration : *_cycle_interval);
      switch (_cycle_type) {
        case TI_SECONDS:
          duration = duration * 1000;
          break;
        case TI_MINUTES:
          duration = duration * 1000 * 60;
          break;
        case TI_HOURS:
          duration = duration * 1000 * 60 * 60;
          break;
        case TI_DAYS:
          duration = duration * 1000 * 60 * 60 * 24;
          break;
        default:
          break;
      }
      // Starting the timer
      if (duration > 0) {
        if (esp_timer_start_once(_timer_cycle, duration) == ESP_OK) {
          _cycle_state = new_state;
          return true;
        } else {
          loadSetStatePriv(_cycle_state);
          return false;
        };
      } else {
        _cycle_state = new_state;
        return true;
      };
    };
  };
  return false;
}

bool rLoadController::cycleSetCyclePriv(bool new_state)
{
  _cycle_state = false;
  if (new_state) {
    if (cycleCreate()) {
      return cycleToggle();
    };
  } else {
    cycleFree();
    return loadSetStatePriv(_cycle_state);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Timer --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void loadControllerTimerEnd(void* arg)
{
  if (arg) {
    rLoadController* ctrl = (rLoadController*)arg;
    ctrl->loadSetState(false, false, true);
  };
}

bool rLoadController::loadSetTimer(uint32_t duration_ms)
{
  if (_timer_on == nullptr) timerCreate();
  if (_timer_on != nullptr) {
    if (esp_timer_is_active(_timer_on)) {
      esp_timer_stop(_timer_on);
    };
    RE_OK_CHECK(esp_timer_start_once(_timer_on, (uint64_t)(duration_ms)*1000), return false);
    if (getState() || loadSetState(true, false, true)) {
      return true;
    } else {
      esp_timer_stop(_timer_on);
      esp_timer_delete(_timer_on);
      _timer_on = nullptr;
    };
  };
  return false;
}

bool rLoadController::timerCreate()
{
  if (_timer_on == nullptr) {
    esp_timer_create_args_t cfg;
    memset(&cfg, 0, sizeof(esp_timer_create_args_t));
    cfg.name = "load_ctrl_on";
    cfg.callback = loadControllerTimerEnd;
    cfg.arg = this;
    RE_OK_CHECK(esp_timer_create(&cfg, &_timer_on), return false);
  };
  return true;
}

bool rLoadController::timerIsActive()
{
  return (_timer_on != nullptr) && esp_timer_is_active(_timer_on);
}

bool rLoadController::timerStop()
{
  if (_timer_on != nullptr) {
    if (esp_timer_is_active(_timer_on)) {
      esp_timer_stop(_timer_on);
    };
    if (_timer_free) {
      RE_OK_CHECK(esp_timer_delete(_timer_on), return false);
      _timer_on = nullptr;
    };
  };
  return true;
}

bool rLoadController::timerFree()
{
  if (_timer_on != nullptr) {
    if (esp_timer_is_active(_timer_on)) {
      esp_timer_stop(_timer_on);
    };
    RE_OK_CHECK(esp_timer_delete(_timer_on), return false);
    _timer_on = nullptr;
  };
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Get data -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rLoadController::getState()
{
  return _state;
}

time_t rLoadController::getLastOn()
{
  return _last_on;
}

time_t rLoadController::getLastOff()
{
  return _last_off;
}

time_t rLoadController::getLastDuration()
{
  return _durations.durLast;
}

char* rLoadController::getLastDurationStr()
{
  return malloc_timespan_hms(_durations.durLast);
}

re_load_counters_t rLoadController::getCounters()
{
  return _counters;
}

re_load_durations_t rLoadController::getDurations()
{
  return _durations;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- MQTT ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rLoadController::mqttSetCallback(cb_load_publish_t cb_publish)
{
  _mqtt_publish = cb_publish;
}

char* rLoadController::mqttTopicGet()
{
  return _mqtt_topic;
}

bool rLoadController::mqttTopicSet(char* topic)
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = topic;
  return (_mqtt_topic != nullptr);
}

bool rLoadController::mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3)
{
  return mqttTopicSet(mqttGetTopicDevice(primary, local, topic1, topic2, topic3));
}

void rLoadController::mqttTopicFree()
{
  if (_mqtt_topic) free(_mqtt_topic);
  _mqtt_topic = nullptr;
}

bool rLoadController::mqttPublish()
{
  if ((_mqtt_topic) && (_mqtt_publish)) {
    return _mqtt_publish(this, _mqtt_topic, getJSON(), false, true);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- JSON ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

char* rLoadController::getTimestampsJSON()
{
  char _time_on[CONFIG_LOADCTRL_TIMESTAMP_BUF_SIZE];
  char _time_off[CONFIG_LOADCTRL_TIMESTAMP_BUF_SIZE];

  time2str_empty( CONFIG_LOADCTRL_TIMESTAMP_FORMAT, &_last_on, &_time_on[0], sizeof(_time_on));
  time2str_empty( CONFIG_LOADCTRL_TIMESTAMP_FORMAT, &_last_off, &_time_off[0], sizeof(_time_off));

  return malloc_stringf("{\"" CONFIG_LOADCTRL_ON "\":\"%s\",\"" CONFIG_LOADCTRL_OFF "\":\"%s\"}", _time_on, _time_off);
}

char* rLoadController::getCountersJSON()
{
  return malloc_stringf("{\"" CONFIG_LOADCTRL_TOTAL "\":%d,\"" CONFIG_LOADCTRL_TODAY "\":%d,\"" CONFIG_LOADCTRL_YESTERDAY "\":%d,\"" CONFIG_LOADCTRL_WEEK_CURR "\":%d,\"" CONFIG_LOADCTRL_WEEK_PREV "\":%d,\"" CONFIG_LOADCTRL_MONTH_CURR "\":%d,\"" CONFIG_LOADCTRL_MONTH_PREV "\":%d,\"" CONFIG_LOADCTRL_PERIOD_CURR "\":%d,\"" CONFIG_LOADCTRL_PERIOD_PREV "\":%d,\"" CONFIG_LOADCTRL_YEAR_CURR "\":%d,\"" CONFIG_LOADCTRL_YEAR_PREV "\":%d}", 
    _counters.cntTotal, 
    _counters.cntToday, _counters.cntYesterday, 
    _counters.cntWeekCurr, _counters.cntWeekPrev, 
    _counters.cntMonthCurr, _counters.cntMonthPrev, 
    _counters.cntPeriodCurr, _counters.cntPeriodPrev, 
    _counters.cntYearCurr, _counters.cntYearPrev);
}

char* rLoadController::getDurationsJSON()
{
  uint32_t durCurr = 0;
  if (_state && (_last_on > 1000000000)) {
    durCurr = time(nullptr) - _last_on;
  };
  return malloc_stringf("{\"" CONFIG_LOADCTRL_LAST "\":%d,\"" CONFIG_LOADCTRL_TOTAL "\":%d,\"" CONFIG_LOADCTRL_TODAY "\":%d,\"" CONFIG_LOADCTRL_YESTERDAY "\":%d,\"" CONFIG_LOADCTRL_WEEK_CURR "\":%d,\"" CONFIG_LOADCTRL_WEEK_PREV "\":%d,\"" CONFIG_LOADCTRL_MONTH_CURR "\":%d,\"" CONFIG_LOADCTRL_MONTH_PREV "\":%d,\"" CONFIG_LOADCTRL_PERIOD_CURR "\":%d,\"" CONFIG_LOADCTRL_PERIOD_PREV "\":%d,\"" CONFIG_LOADCTRL_YEAR_CURR "\":%d,\"" CONFIG_LOADCTRL_YEAR_PREV "\":%d}", 
    _state ? durCurr : _durations.durLast, _durations.durTotal + durCurr, 
    _durations.durToday + durCurr, _durations.durYesterday, 
    _durations.durWeekCurr + durCurr, _durations.durWeekPrev, 
    _durations.durMonthCurr + durCurr, _durations.durMonthPrev, 
    _durations.durPeriodCurr + durCurr, _durations.durPeriodPrev, 
    _durations.durYearCurr + durCurr, _durations.durYearPrev);
}

char* rLoadController::getJSON()
{
  char* _json = nullptr;

  char* _json_time = getTimestampsJSON();
  char* _json_counters = getCountersJSON();
  char* _json_durations = getDurationsJSON();
  
  if ((_json_time) && (_json_counters) && (_json_durations)) {
    _json = malloc_stringf("{\"" CONFIG_LOADCTRL_STATUS "\":%d,\"" CONFIG_LOADCTRL_TIMESTAMP "\":%s,\"" CONFIG_LOADCTRL_DURATIONS "\":%s,\"" CONFIG_LOADCTRL_COUNTERS "\":%s}",
      _state, _json_time, _json_durations, _json_counters);
  };

  if (_json_time) free(_json_time);
  if (_json_counters) free(_json_counters);
  if (_json_durations) free(_json_durations);

  return _json;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------ Reading and saving counters from flash memory ------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rLoadController::countersReset()
{
  memset((void*)&_counters, 0, sizeof(re_load_counters_t));
  memset((void*)&_durations, 0, sizeof(re_load_durations_t));
}

void rLoadController::countersNvsRestore()
{
  if (_nvs_space) {
    // Number of days since UNIX epoch, discarding time
    uint32_t daysNow = (uint32_t)(time(nullptr) / 86400);
    uint32_t daysNvs = daysNow;
    nvs_handle_t nvs_handle;
    if (nvsOpen(_nvs_space, NVS_READONLY, &nvs_handle)) {
      nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_DAYS, &daysNvs);
      nvs_close(nvs_handle);
    };

    re_load_counters_t _nvsCnt;
    bool _nvsCntEnabled = false;
    char* nmsp_cnt = malloc_stringf("%s.cnt", _nvs_space);
    if (nmsp_cnt) {
      nvs_handle_t nvs_handle;
      if (nvsOpen(nmsp_cnt, NVS_READONLY, &nvs_handle)) {
        _nvsCntEnabled = true;
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TOTAL, &_nvsCnt.cntTotal);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TODAY, &_nvsCnt.cntToday);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YESTERDAY, &_nvsCnt.cntYesterday);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_CURR, &_nvsCnt.cntWeekCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_PREV, &_nvsCnt.cntWeekPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_CURR, &_nvsCnt.cntMonthCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_PREV, &_nvsCnt.cntMonthPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_CURR, &_nvsCnt.cntPeriodCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_PREV, &_nvsCnt.cntPeriodPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_CURR, &_nvsCnt.cntYearCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_PREV, &_nvsCnt.cntYearPrev);
        nvs_close(nvs_handle);
      };
      free(nmsp_cnt);
    };

    re_load_durations_t _nvsDur;
    bool _nvsDurEnabled = false;
    char* nmsp_dur = malloc_stringf("%s.dur", _nvs_space);
    if (nmsp_dur) {
      nvs_handle_t nvs_handle;
      if (nvsOpen(nmsp_dur, NVS_READONLY, &nvs_handle)) {
        _nvsDurEnabled = true;
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_LAST, &_nvsDur.durLast);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TOTAL, &_nvsDur.durTotal);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TODAY, &_nvsDur.durToday);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YESTERDAY, &_nvsDur.durYesterday);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_CURR, &_nvsDur.durWeekCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_PREV, &_nvsDur.durWeekPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_CURR, &_nvsDur.durMonthCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_PREV, &_nvsDur.durMonthPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_CURR, &_nvsDur.durPeriodCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_PREV, &_nvsDur.durPeriodPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_CURR, &_nvsDur.durYearCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_PREV, &_nvsDur.durYearPrev);
        nvs_close(nvs_handle);
      };
      free(nmsp_dur);
    };

    // Restore data
    if (daysNow == daysNvs) {
      // Data was saved today
      if (_nvsCntEnabled) {
        _counters = _nvsCnt;
      };
      if (_nvsDurEnabled) {
        _durations = _nvsDur;
      };
    } else {
      // Restore total counters
      if (_nvsCntEnabled) {
        _counters.cntTotal  = _nvsCnt.cntTotal;
      };
      if (_nvsDurEnabled) {
        _durations.durLast = _nvsDur.durLast;
        _durations.durTotal = _nvsDur.durTotal;
      };

      // Decode week, month, period, and year
      time_t timeNow = (time_t)daysNow * 86400 + 1;
      time_t timeNvs = (time_t)daysNvs * 86400 + 1;
      uint32_t weekNow = (daysNow + 3) / 7;
      uint32_t weekNvs = (daysNvs + 3) / 7;
      struct tm tmNow;
      struct tm tmNvs;
      localtime_r(&timeNow, &tmNow);
      localtime_r(&timeNvs, &tmNvs);
      uint8_t pmNow = 0;
      uint8_t pmNvs = 0;
      uint16_t pyNow = 0;
      uint16_t pyNvs = 0;
      if ((_period_start) && (*_period_start > 0)) {
        pyNow = tmNow.tm_year;
        if (tmNow.tm_mday < *_period_start) {
          pmNow = tmNow.tm_mon;
        } else {
          pmNow = tmNow.tm_mon + 1;
          if (pmNow > 11) {
            pyNow++;
            pmNow = 0;
          };
        };
        pyNvs = tmNvs.tm_year;
        if (tmNvs.tm_mday < *_period_start) {
          pmNvs = tmNvs.tm_mon;
        } else {
          pmNvs = tmNvs.tm_mon + 1;
          if (pmNvs > 11) {
            pyNvs++;
            pmNvs = 0;
          };
        };
      };

      // Data was saved on the previous day
      if (daysNow == daysNvs + 1) {
        if (_nvsCntEnabled) {
          _counters.cntToday = 0;
          _counters.cntYesterday = _nvsCnt.cntToday;
        };
        if (_nvsDurEnabled) {
          _durations.durToday = 0;
          _durations.durYesterday = _nvsDur.durToday;
        };
      };

      // Data was saved on the current week
      if (weekNow == weekNvs) {
        if (_nvsCntEnabled) {
          _counters.cntWeekCurr = _nvsCnt.cntWeekCurr;
          _counters.cntWeekPrev = _nvsCnt.cntWeekPrev;
        };
        if (_nvsDurEnabled) {
          _durations.durWeekCurr = _nvsDur.durWeekCurr;
          _durations.durWeekPrev = _nvsDur.durWeekPrev;
        };
      }
      // Data was saved on the previous week
      else if (weekNow == weekNvs + 1) {
        if (_nvsCntEnabled) {
          _counters.cntWeekCurr = 0;
          _counters.cntWeekPrev = _nvsCnt.cntWeekCurr;
        };
        if (_nvsDurEnabled) {
          _durations.durWeekCurr = 0;
          _durations.durWeekPrev = _nvsDur.durWeekCurr;
        };
      };

      // Data was saved on the current month
      if ((tmNow.tm_year == tmNvs.tm_year) && (tmNow.tm_mon == tmNvs.tm_mon)) {
        if (_nvsCntEnabled) {
          _counters.cntMonthCurr = _nvsCnt.cntMonthCurr;
          _counters.cntMonthPrev = _nvsCnt.cntMonthPrev;
        };
        if (_nvsDurEnabled) {
          _durations.durMonthCurr = _nvsDur.durMonthCurr;
          _durations.durMonthPrev = _nvsDur.durMonthPrev;
        };
      }
      // Data was saved on the previous month
      else if (((tmNow.tm_year == tmNvs.tm_year) && (tmNow.tm_mon == tmNvs.tm_mon + 1)) 
            || ((tmNow.tm_year == tmNvs.tm_year + 1) && (tmNow.tm_mon == 0) && (tmNvs.tm_mon == 11))) {
        if (_nvsCntEnabled) {
          _counters.cntMonthCurr = 0;
          _counters.cntMonthPrev = _nvsCnt.cntMonthCurr;
        };
        if (_nvsDurEnabled) {
          _durations.durMonthCurr = 0;
          _durations.durMonthPrev = _nvsDur.durMonthCurr;
        };
      };

      // Data was saved on the current period
      if ((pyNow == pyNvs) && (pmNow == pmNvs)) {
        if (_nvsCntEnabled) {
          _counters.cntPeriodCurr = _nvsCnt.cntPeriodCurr;
          _counters.cntPeriodPrev = _nvsCnt.cntPeriodPrev;
        };
        if (_nvsDurEnabled) {
          _durations.durPeriodCurr = _nvsDur.durPeriodCurr;
          _durations.durPeriodPrev = _nvsDur.durPeriodPrev;
        };
      }
      // Data was saved on the previous period
      else if (((pyNow == pyNvs) && (pmNow == pmNvs + 1)) 
            || ((pyNow == pyNvs + 1) && (pmNow == 0) && (pmNvs == 11))) {
        if (_nvsCntEnabled) {
          _counters.cntPeriodCurr = 0;
          _counters.cntPeriodPrev = _nvsCnt.cntPeriodCurr;
        };
        if (_nvsDurEnabled) {
          _durations.durPeriodCurr = 0;
          _durations.durPeriodPrev = _nvsDur.durPeriodCurr;
        };
      };

      // Data was saved on the current year
      if (tmNow.tm_year == tmNvs.tm_year) {
        if (_nvsCntEnabled) {
          _counters.cntYearCurr = _nvsCnt.cntYearCurr;
          _counters.cntYearPrev = _nvsCnt.cntYearPrev;
        };
        if (_nvsDurEnabled) {
          _durations.durYearCurr = _nvsDur.durYearCurr;
          _durations.durYearPrev = _nvsDur.durYearPrev;
        };
      } 
      // Data was saved on the previous year
      else if (tmNow.tm_year == tmNvs.tm_year + 1) {
        if (_nvsCntEnabled) {
          _counters.cntYearCurr = 0;
          _counters.cntYearPrev = _nvsCnt.cntYearCurr;
        };
        if (_nvsDurEnabled) {
          _durations.durYearCurr = 0;
          _durations.durYearPrev = _nvsDur.durYearCurr;
        };
      };
    };
  };
}

void rLoadController::countersNvsStore()
{
  if (_nvs_space && (_counters.cntTotal > 0)) {
    nvs_handle_t nvs_handle;
    if (nvsOpen(_nvs_space, NVS_READWRITE, &nvs_handle)) {
      // Number of days since UNIX epoch, discarding time
      uint32_t days = (uint32_t)(time(nullptr) / 86400);
      nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_DAYS, days);
      nvs_commit(nvs_handle);
      nvs_close(nvs_handle);
    };

    char* nmsp_cnt = malloc_stringf("%s.cnt", _nvs_space);
    if (nmsp_cnt) {
      nvs_handle_t nvs_handle;
      if (nvsOpen(nmsp_cnt, NVS_READWRITE, &nvs_handle)) {
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_TOTAL, _counters.cntTotal);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_TODAY, _counters.cntToday);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_YESTERDAY, _counters.cntYesterday);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_CURR, _counters.cntWeekCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_PREV, _counters.cntWeekPrev);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_CURR, _counters.cntMonthCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_PREV, _counters.cntMonthPrev);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_CURR, _counters.cntPeriodCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_PREV, _counters.cntPeriodPrev);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_CURR, _counters.cntYearCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_PREV, _counters.cntYearPrev);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
      };
      free(nmsp_cnt);
    };

    char* nmsp_dur = malloc_stringf("%s.dur", _nvs_space);
    if (nmsp_dur) {
      nvs_handle_t nvs_handle;
      if (nvsOpen(nmsp_dur, NVS_READWRITE, &nvs_handle)) {
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_LAST, _durations.durLast);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_TOTAL, _durations.durTotal);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_TODAY, _durations.durToday);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_YESTERDAY, _durations.durYesterday);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_CURR, _durations.durWeekCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_PREV, _durations.durWeekPrev);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_CURR, _durations.durMonthCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_PREV, _durations.durMonthPrev);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_CURR, _durations.durPeriodCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_PREV, _durations.durPeriodPrev);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_CURR, _durations.durYearCurr);
        nvs_set_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_PREV, _durations.durYearPrev);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
      };
      free(nmsp_dur);
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Event handlers ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void rLoadController::countersTimeEventHandler(int32_t event_id, void* event_data)
{
  // Start of the day
  if (event_id == RE_TIME_START_OF_DAY) {
    _counters.cntYesterday = _counters.cntToday;
    _counters.cntToday = 0;
    _durations.durYesterday = _durations.durToday;
    _durations.durToday = 0;

    if ((event_data) && (_period_start)) {
      int* mday = (int*)event_data;
      if (*mday == *_period_start) {
        _counters.cntPeriodPrev = _counters.cntPeriodCurr;
        _counters.cntPeriodCurr = 0;
        _durations.durPeriodPrev = _durations.durPeriodCurr;
        _durations.durPeriodCurr = 0;
      };
    };
  }
  // Beginning of the week
  else if (event_id == RE_TIME_START_OF_WEEK) {
    _counters.cntWeekPrev = _counters.cntWeekCurr;
    _counters.cntWeekCurr = 0;
    _durations.durWeekPrev = _durations.durWeekCurr;
    _durations.durWeekCurr = 0;
  }
  // Beginning of the month
  else if (event_id == RE_TIME_START_OF_MONTH) {
    _counters.cntMonthPrev = _counters.cntMonthCurr;
    _counters.cntMonthCurr = 0;
    _durations.durMonthPrev = _durations.durMonthCurr;
    _durations.durMonthCurr = 0;
  }
  // Beginning of the year
  else if (event_id == RE_TIME_START_OF_YEAR) {
    _counters.cntYearPrev = _counters.cntYearCurr;
    _counters.cntYearCurr  = 0;
    _durations.durYearPrev = _durations.durYearCurr;
    _durations.durYearCurr  = 0;
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- rLoadGpioController -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rLoadGpioController::rLoadGpioController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space,
  uint32_t* cycle_duration, uint32_t* cycle_interval, timeintv_t cycle_type,
  cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
  cb_load_publish_t cb_mqtt_publish)
:rLoadController(pin, level_on, use_timer, nvs_space, 
  cycle_duration, cycle_interval, cycle_type,
  cb_gpio_before, cb_gpio_after, cb_state_changed, cb_mqtt_publish)
{
}

rLoadGpioController::rLoadGpioController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space)
:rLoadController(pin, level_on, use_timer, nvs_space, nullptr, nullptr, TI_MILLISECONDS, nullptr, nullptr, nullptr, nullptr)
{
}

rLoadGpioController::rLoadGpioController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space, 
  cb_load_change_t cb_state_changed, cb_load_publish_t cb_mqtt_publish)
:rLoadController(pin, level_on, use_timer, nvs_space, nullptr, nullptr, TI_MILLISECONDS, nullptr, nullptr, cb_state_changed, cb_mqtt_publish)
{
}

bool rLoadGpioController::loadInitGPIO()
{
  // Configure internal GPIO to output
  gpio_reset_pin((gpio_num_t)_pin);
  ERR_LOAD_CHECK(gpio_set_direction((gpio_num_t)_pin, GPIO_MODE_OUTPUT), ERR_GPIO_SET_MODE);
  return true;
}

bool rLoadGpioController::loadSetStateGPIO(uint8_t physical_level)
{
  ERR_LOAD_CHECK(gpio_set_level((gpio_num_t)_pin, (uint32_t)physical_level), ERR_GPIO_SET_LEVEL);
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- rLoadIoExpController ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rLoadIoExpController::rLoadIoExpController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space,
  uint32_t* cycle_duration, uint32_t* cycle_interval, timeintv_t cycle_type,
  cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change,
  cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
  cb_load_publish_t cb_mqtt_publish)
:rLoadController(pin, level_on, use_timer, nvs_space, 
  cycle_duration, cycle_interval, cycle_type,
  cb_gpio_before, cb_gpio_after, cb_state_changed, cb_mqtt_publish)
{
  _gpio_init = cb_gpio_init;
  _gpio_change = cb_gpio_change;
}

rLoadIoExpController::rLoadIoExpController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space,
  cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change)
:rLoadController(pin, level_on, use_timer, nvs_space, nullptr, nullptr, TI_MILLISECONDS, nullptr, nullptr, nullptr, nullptr)
{
  _gpio_init = cb_gpio_init;
  _gpio_change = cb_gpio_change;
}

rLoadIoExpController::rLoadIoExpController(uint8_t pin, uint8_t level_on, bool use_timer, const char* nvs_space,
  cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change, cb_load_change_t cb_state_changed, cb_load_publish_t cb_mqtt_publish)
:rLoadController(pin, level_on, use_timer, nvs_space, nullptr, nullptr, TI_MILLISECONDS, nullptr, nullptr, cb_state_changed, cb_mqtt_publish)
{
  _gpio_init = cb_gpio_init;
  _gpio_change = cb_gpio_change;
}

bool rLoadIoExpController::loadInitGPIO()
{
  if (_gpio_init) {
    return _gpio_init(this, _pin, _level_on);
  };
  return true;
}

bool rLoadIoExpController::loadSetStateGPIO(uint8_t physical_level)
{
  if (_gpio_change) {
    return _gpio_change(this, _pin, physical_level);
  };
  return false;
}

