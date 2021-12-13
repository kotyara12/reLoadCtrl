#include "reLoadCtrl.h"
#include <string.h>
#include "reNvs.h"
#include "reEvents.h"
#include "reMqtt.h"
#include "rLog.h"
#include "rStrings.h"

static const char* logTAG = "LOAD";

#define ERR_LOAD_CHECK(err, str) if (err != ESP_OK) rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err));
#define ERR_GPIO_SET_LEVEL "Failed to change GPIO level"
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Constructor -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

rLoadController::rLoadController(uint8_t gpio_num, bool gpio_on, const char* nvs_space)
{
  // Configure GPIO
  _gpio_num = gpio_num;
  _gpio_on = gpio_on;
  _nvs_space = nvs_space;
  _state = false;
  _last_on = 0;
  _last_off = 0;

  // Reset pointers
  _period_start = nullptr;
  _mqtt_topic = nullptr;
  _mqtt_publish = nullptr;

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

void rLoadController::setCallbackGPIO(cb_load_control_t cb_before, cb_load_control_t cb_after)
{
  _gpio_before = cb_before;
  _gpio_after = cb_after;
}

void rLoadController::setCallbackState(cb_load_control_t cb_state)
{
  _load_state = cb_state;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- Load --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rLoadController::loadInitGPIO(bool init_state)
{
  // Configure GPIO to output
  gpio_pad_select_gpio((gpio_num_t)_gpio_num);
  ERR_LOAD_CHECK(gpio_set_direction((gpio_num_t)_gpio_num, GPIO_MODE_OUTPUT), ERR_GPIO_SET_MODE);
  if (_gpio_on) {
    ERR_LOAD_CHECK(gpio_set_pull_mode((gpio_num_t)_gpio_num, GPIO_PULLDOWN_ONLY), ERR_GPIO_SET_MODE);
  } else {
    ERR_LOAD_CHECK(gpio_set_pull_mode((gpio_num_t)_gpio_num, GPIO_PULLUP_ONLY), ERR_GPIO_SET_MODE);
  };
  return loadSetStateGPIO(init_state, true, true);
}

bool rLoadController::loadSetStateGPIO(bool new_state, bool forced, bool publish)
{
  if (forced || (_state != new_state)) {
    // Determine which level needs to be written to the GPIO
    uint32_t new_level = 0;
    if (new_state) {
      _gpio_on ? new_level = 1 : new_level = 0;
    } else {
      _gpio_on ? new_level = 0 : new_level = 1;
    };

    // Set physical level to GPIO
    if (_gpio_before) { _gpio_before(this, new_state); };
    ERR_LOAD_CHECK(gpio_set_level((gpio_num_t)_gpio_num, new_level), ERR_GPIO_SET_LEVEL);
    if (_gpio_after) { _gpio_after(this, new_state); };

    // If the change level was successful
    if (_state != new_state) {
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
        rlog_i(logTAG, "Load on GPIO %d is ON", _gpio_num);
      } else {
        _last_off = time(nullptr);
        if (((_last_on <= 1000000000) && (_last_off <= 1000000000)) || ((_last_on > 1000000000) && (_last_off > 1000000000))) {
          _durations.durLast = _last_off - _last_on;
          _durations.durTotal = _durations.durTotal + _durations.durLast;
          _durations.durToday = _durations.durToday + _durations.durLast;
          _durations.durWeekCurr = _durations.durWeekCurr + _durations.durLast;
          _durations.durMonthCurr = _durations.durMonthCurr + _durations.durLast;
          _durations.durPeriodCurr = _durations.durPeriodCurr + _durations.durLast;
          _durations.durYearCurr = _durations.durYearCurr + _durations.durLast;
        };
        rlog_i(logTAG, "Load on GPIO %d is OFF", _gpio_num);
      };

      // Publish status and counters
      if (publish) {
        mqttPublish(true);
      };

      // Call external callback
      if (_load_state) { 
        _load_state(this, _state); 
      };
      return true;
    };
  };
  return false;
}

bool rLoadController::loadSetState(bool new_state, bool publish)
{
  return loadSetStateGPIO(new_state, false, publish);
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
  char* _json_time = nullptr;
  char* _json_time_on = nullptr;
  char* _json_time_off = nullptr;
  
  if (_last_on == 0) {
    _json_time_on = malloc_stringf(CONFIG_LOADCTRL_TIMESTAMP_NULL);
  } else {
    _json_time_on = malloc_timestr(CONFIG_FORMAT_TIMESTAMP_L, _last_on);
  };
  
  if (_last_off == 0) {
    _json_time_off = malloc_stringf(CONFIG_LOADCTRL_TIMESTAMP_NULL);
  } else {
    _json_time_off = malloc_timestr(CONFIG_FORMAT_TIMESTAMP_L, _last_off);
  };

  if ((_json_time_on) && (_json_time_off)) {
    _json_time = malloc_stringf("{\"" CONFIG_LOADCTRL_ON "\":\"%s\",\"" CONFIG_LOADCTRL_OFF "\":\"%s\"}", _json_time_on, _json_time_off);
  };

  if (_json_time_on) free(_json_time_on);
  if (_json_time_off) free(_json_time_off);

  return _json_time;
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
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_TOTAL, OPT_TYPE_U32, &_counters.cntTotal);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_TODAY, OPT_TYPE_U32, &_counters.cntToday);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_YESTERDAY, OPT_TYPE_U32, &_counters.cntYesterday);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_WEEK_CURR, OPT_TYPE_U32, &_counters.cntWeekCurr);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_WEEK_PREV, OPT_TYPE_U32, &_counters.cntWeekPrev);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_MONTH_CURR, OPT_TYPE_U32, &_counters.cntMonthCurr);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_MONTH_PREV, OPT_TYPE_U32, &_counters.cntMonthPrev);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_PERIOD_CURR, OPT_TYPE_U32, &_counters.cntPeriodCurr);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_PERIOD_PREV, OPT_TYPE_U32, &_counters.cntPeriodPrev);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_YEAR_CURR, OPT_TYPE_U32, &_counters.cntYearCurr);
      nvsRead(nmsp_cnt, CONFIG_LOADCTRL_YEAR_PREV, OPT_TYPE_U32, &_counters.cntYearPrev);
      free(nmsp_cnt);
    };

    char* nmsp_dur = malloc_stringf("%s.dur", _nvs_space);
    if (nmsp_dur) {
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_LAST, OPT_TYPE_U32, &_durations.durLast);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_TOTAL, OPT_TYPE_U32, &_durations.durTotal);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_TODAY, OPT_TYPE_U32, &_durations.durToday);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_YESTERDAY, OPT_TYPE_U32, &_durations.durYesterday);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_WEEK_CURR, OPT_TYPE_U32, &_durations.durWeekCurr);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_WEEK_PREV, OPT_TYPE_U32, &_durations.durWeekPrev);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_MONTH_CURR, OPT_TYPE_U32, &_durations.durMonthCurr);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_MONTH_PREV, OPT_TYPE_U32, &_durations.durMonthPrev);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_PERIOD_CURR, OPT_TYPE_U32, &_durations.durPeriodCurr);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_PERIOD_PREV, OPT_TYPE_U32, &_durations.durPeriodPrev);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_YEAR_CURR, OPT_TYPE_U32, &_durations.durYearCurr);
      nvsRead(nmsp_dur, CONFIG_LOADCTRL_YEAR_PREV, OPT_TYPE_U32, &_durations.durYearPrev);
      free(nmsp_dur);
    };
  };
}

void rLoadController::countersNvsStore()
{
  if (_nvs_space) {
    char* nmsp_cnt = malloc_stringf("%s.cnt", _nvs_space);
    if (nmsp_cnt) {
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_TOTAL, OPT_TYPE_U32, &_counters.cntTotal);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_TODAY, OPT_TYPE_U32, &_counters.cntToday);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_YESTERDAY, OPT_TYPE_U32, &_counters.cntYesterday);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_WEEK_CURR, OPT_TYPE_U32, &_counters.cntWeekCurr);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_WEEK_PREV, OPT_TYPE_U32, &_counters.cntWeekPrev);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_MONTH_CURR, OPT_TYPE_U32, &_counters.cntMonthCurr);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_MONTH_PREV, OPT_TYPE_U32, &_counters.cntMonthPrev);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_PERIOD_CURR, OPT_TYPE_U32, &_counters.cntPeriodCurr);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_PERIOD_PREV, OPT_TYPE_U32, &_counters.cntPeriodPrev);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_YEAR_CURR, OPT_TYPE_U32, &_counters.cntYearCurr);
      nvsWrite(nmsp_cnt, CONFIG_LOADCTRL_YEAR_PREV, OPT_TYPE_U32, &_counters.cntYearPrev);
      free(nmsp_cnt);
    };

    char* nmsp_dur = malloc_stringf("%s.dur", _nvs_space);
    if (nmsp_dur) {
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_LAST, OPT_TYPE_U32, &_durations.durLast);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_TOTAL, OPT_TYPE_U32, &_durations.durTotal);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_TODAY, OPT_TYPE_U32, &_durations.durToday);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_YESTERDAY, OPT_TYPE_U32, &_durations.durYesterday);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_WEEK_CURR, OPT_TYPE_U32, &_durations.durWeekCurr);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_WEEK_PREV, OPT_TYPE_U32, &_durations.durWeekPrev);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_MONTH_CURR, OPT_TYPE_U32, &_durations.durMonthCurr);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_MONTH_PREV, OPT_TYPE_U32, &_durations.durMonthPrev);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_PERIOD_CURR, OPT_TYPE_U32, &_durations.durPeriodCurr);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_PERIOD_PREV, OPT_TYPE_U32, &_durations.durPeriodPrev);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_YEAR_CURR, OPT_TYPE_U32, &_durations.durYearCurr);
      nvsWrite(nmsp_dur, CONFIG_LOADCTRL_YEAR_PREV, OPT_TYPE_U32, &_durations.durYearPrev);
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
