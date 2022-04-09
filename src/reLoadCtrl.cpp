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
// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- rLoadController ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rLoadController::rLoadController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
  cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed,
  cb_load_publish_t cb_mqtt_publish)
{
  // Configure GPIO
  _pin = pin;
  _level_on = level_on;
  _use_pullup = use_pullup;
  _nvs_space = nvs_space;
  _state = false;
  _last_on = 0;
  _last_off = 0;

   // Reset pointers
  _period_start = nullptr;
  _mqtt_topic = nullptr;
  _mqtt_publish = nullptr;

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
  return loadInitGPIO() && loadSetState(init_state, true, false);
}

bool rLoadController::loadSetState(bool new_state, bool forced, bool publish)
{
  if (forced || (_state != new_state)) {
    // Determine which level needs to be written to the GPIO
    bool phy_level;
    new_state ? phy_level = _level_on : phy_level = !_level_on;

    // Set physical level to GPIO
    if (_gpio_before) { _gpio_before(this, phy_level, 0); };
    bool change_ok = loadSetStateGPIO(phy_level);
    if (_gpio_after) { _gpio_after(this, phy_level, 0); };

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
        mqttPublish(forced);
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
// ------------------------------------------------------ Get data -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rLoadController::getState()
{
  return _state;
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

bool rLoadController::mqttPublish(bool forced)
{
  if ((_mqtt_topic) && (_mqtt_publish)) {
    return _mqtt_publish(this, _mqtt_topic, getJSON(), forced, false, true);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- JSON ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

char* rLoadController::getTimestampsJSON()
{
  struct tm _ti_on;
  char _time_on[CONFIG_LOADCTRL_TIMESTAMP_BUF_SIZE];
  memset(&_time_on, 0, sizeof(_time_on));
  if (_last_on > 0) {
    localtime_r(&_last_on, &_ti_on);
    strftime(_time_on, sizeof(_time_on), CONFIG_LOADCTRL_TIMESTAMP_FORMAT, &_ti_on);
  } else {
    strcpy(_time_on, CONFIG_FORMAT_EMPTY_DATETIME);
  };

  struct tm _ti_off;
  char _time_off[CONFIG_LOADCTRL_TIMESTAMP_BUF_SIZE];
  memset(&_time_off, 0, sizeof(_time_off));
  if (_last_off > 0) {
    localtime_r(&_last_off, &_ti_off);
    strftime(_time_off, sizeof(_time_off), CONFIG_LOADCTRL_TIMESTAMP_FORMAT, &_ti_off);
  } else {
    strcpy(_time_off, CONFIG_FORMAT_EMPTY_DATETIME);
  };

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
    char* nmsp_cnt = malloc_stringf("%s.cnt", _nvs_space);
    if (nmsp_cnt) {
      nvs_handle_t nvs_handle;
      if (nvsOpen(nmsp_cnt, NVS_READONLY, &nvs_handle)) {
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TOTAL, &_counters.cntTotal);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TODAY, &_counters.cntToday);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YESTERDAY, &_counters.cntYesterday);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_CURR, &_counters.cntWeekCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_PREV, &_counters.cntWeekPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_CURR, &_counters.cntMonthCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_PREV, &_counters.cntMonthPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_CURR, &_counters.cntPeriodCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_PREV, &_counters.cntPeriodPrev);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_CURR, &_counters.cntYearCurr);
        nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_PREV, &_counters.cntYearPrev);
        nvs_close(nvs_handle);
      };
      free(nmsp_cnt);
    };

    char* nmsp_dur = malloc_stringf("%s.dur", _nvs_space);
    if (nmsp_dur) {
      nvs_handle_t nvs_handle;
      if (nvsOpen(nmsp_cnt, NVS_READONLY, &nvs_handle)) {
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_LAST, &_durations.durLast);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TOTAL, &_durations.durTotal);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_TODAY, &_durations.durToday);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YESTERDAY, &_durations.durYesterday);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_CURR, &_durations.durWeekCurr);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_WEEK_PREV, &_durations.durWeekPrev);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_CURR, &_durations.durMonthCurr);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_MONTH_PREV, &_durations.durMonthPrev);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_CURR, &_durations.durPeriodCurr);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_PERIOD_PREV, &_durations.durPeriodPrev);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_CURR, &_durations.durYearCurr);
          nvs_get_u32(nvs_handle, CONFIG_LOADCTRL_YEAR_PREV, &_durations.durYearPrev);
        nvs_close(nvs_handle);
      };
      free(nmsp_dur);
    };
  };
}

void rLoadController::countersNvsStore()
{
  if (_nvs_space && (_counters.cntTotal > 0)) {
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
      if (nvsOpen(nmsp_cnt, NVS_READWRITE, &nvs_handle)) {
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

    if (event_data && _period_start) {
      int* mday = (int*)event_data;
      if (*mday == *_period_start) {
        _counters.cntPeriodPrev = _counters.cntPeriodCurr;
        _counters.cntPeriodCurr = 0;
        _durations.durPeriodPrev = _durations.durPeriodCurr;
        _durations.durPeriodCurr = 0;
      };
    };

    countersNvsStore();
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

rLoadGpioController::rLoadGpioController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
  cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
  cb_load_publish_t cb_mqtt_publish)
:rLoadController(pin, level_on, use_pullup, nvs_space, 
  cb_gpio_before, cb_gpio_after, cb_state_changed, cb_mqtt_publish)
{
}

rLoadGpioController::rLoadGpioController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space)
:rLoadController(pin, level_on, use_pullup, nvs_space, nullptr, nullptr, nullptr, nullptr)
{
}

bool rLoadGpioController::loadInitGPIO()
{
  // Configure internal GPIO to output
  gpio_pad_select_gpio((gpio_num_t)_pin);
  ERR_LOAD_CHECK(gpio_set_direction((gpio_num_t)_pin, GPIO_MODE_OUTPUT), ERR_GPIO_SET_MODE);
  if (_use_pullup) {
    if (_level_on) {
      ERR_LOAD_CHECK(gpio_set_pull_mode((gpio_num_t)_pin, GPIO_PULLDOWN_ONLY), ERR_GPIO_SET_MODE);
    } else {
      ERR_LOAD_CHECK(gpio_set_pull_mode((gpio_num_t)_pin, GPIO_PULLUP_ONLY), ERR_GPIO_SET_MODE);
    };
  };
  return true;
}

bool rLoadGpioController::loadSetStateGPIO(bool physical_level)
{
  ERR_LOAD_CHECK(gpio_set_level((gpio_num_t)_pin, (uint32_t)physical_level), ERR_GPIO_SET_LEVEL);
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- rLoadIoExpController ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rLoadIoExpController::rLoadIoExpController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
  cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change,
  cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
  cb_load_publish_t cb_mqtt_publish)
:rLoadController(pin, level_on, use_pullup, nvs_space, 
  cb_gpio_before, cb_gpio_after, cb_state_changed, cb_mqtt_publish)
{
  _gpio_init = cb_gpio_init;
  _gpio_change = cb_gpio_change;
}

rLoadIoExpController::rLoadIoExpController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
  cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change)
:rLoadController(pin, level_on, use_pullup, nvs_space, nullptr, nullptr, nullptr, nullptr)
{
  _gpio_init = cb_gpio_init;
  _gpio_change = cb_gpio_change;
}

bool rLoadIoExpController::loadInitGPIO()
{
  if (_gpio_init) {
    return _gpio_init(this, _pin, _level_on, _use_pullup);
  };
  return true;
}

bool rLoadIoExpController::loadSetStateGPIO(bool physical_level)
{
  if (_gpio_change) {
    return _gpio_change(this, _pin, physical_level);
  };
  return false;
}

