/* 
   EN: Class for controlling a load (e.g. a relay) with calculation of operating time and energy consumption
   RU: Класс для управления нагрузкой (например реле) с подсчетом времени работы и потребляемой энергии
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Страница проекта: https://github.com/kotyara12/reLoadCtrl
*/

#ifndef __RE_LOADCTRL_H__
#define __RE_LOADCTRL_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <driver/gpio.h>
#include "project_config.h"
#include "def_consts.h"

typedef struct {
  uint32_t cntTotal      = 0;
  uint32_t cntToday      = 0;
  uint32_t cntYesterday  = 0;
  uint32_t cntWeekCurr   = 0;
  uint32_t cntWeekPrev   = 0;
  uint32_t cntMonthCurr  = 0;
  uint32_t cntMonthPrev  = 0;
  uint32_t cntPeriodCurr = 0;
  uint32_t cntPeriodPrev = 0;
  uint32_t cntYearCurr   = 0;
  uint32_t cntYearPrev   = 0;
} re_load_counters_t;

typedef struct {
  time_t durLast        = 0;
  time_t durTotal       = 0;
  time_t durToday       = 0;
  time_t durYesterday   = 0;
  time_t durWeekCurr    = 0;
  time_t durWeekPrev    = 0;
  time_t durMonthCurr   = 0;
  time_t durMonthPrev   = 0;
  time_t durPeriodCurr  = 0;
  time_t durPeriodPrev  = 0;
  time_t durYearCurr    = 0;
  time_t durYearPrev    = 0;
} re_load_durations_t;

class rLoadController;

typedef bool (*cb_load_publish_t) (rLoadController *ctrl, char* topic, char* payload, bool forced, bool free_topic, bool free_payload);
typedef void (*cb_load_control_t) (rLoadController *ctrl, bool state);

#ifdef __cplusplus
extern "C" {
#endif

class rLoadController {
  public:
    rLoadController(uint8_t gpio_num, bool gpio_on, const char* nvs_space);
    ~rLoadController();

    // Load switching
    bool loadInitGPIO(bool init_state);
    bool loadSetState(bool new_state, bool publish = false);

    // Get current data
    bool getState();
    time_t getLastDuration();
    char*  getLastDurationStr();
    re_load_counters_t getCounters();
    re_load_durations_t getDurations();
    char* getTimestampsJSON();
    char* getCountersJSON();
    char* getDurationsJSON();
    char* getJSON();

    // MQTT
    void mqttSetCallback(cb_load_publish_t cb_publish);
    char* mqttTopicGet();
    bool mqttTopicSet(char* topic);
    bool mqttTopicCreate(bool primary, bool local, const char* topic1, const char* topic2, const char* topic3);
    void mqttTopicFree();
    bool mqttPublish(bool forced);
    
    // Saving the state of counters
    void countersReset();
    void countersNvsRestore();
    void countersNvsStore();

    // Event handlers
    void countersTimeEventHandler(int32_t event_id, void* event_data);

    // Other parameters
    void setPeriodStartDay(uint8_t* mday);
    void setCallbackGPIO(cb_load_control_t cb_before, cb_load_control_t cb_after);
    void setCallbackState(cb_load_control_t cb_state);
  protected:
    virtual bool loadSetStateGPIO(bool new_state, bool forced, bool publish);
  private:
    uint8_t     _gpio_num = 0;                 // MCU pin number
    bool        _gpio_on = true;               // MCU output level at which the load is considered to be on
    bool        _state = false;                // Current load state
    time_t      _last_on = 0;                  // The last time the load was turned on
    time_t      _last_off = 0;                 // Time of last load disconnection
    uint8_t*    _period_start = nullptr;       // Day of month at the beginning of the billing period (for example, sending meter readings)
    re_load_counters_t  _counters;             // Counters of the number of load switching
    re_load_durations_t _durations;            // Load operating time counters
    const char* _nvs_space = nullptr;          // Namespace to store counter values 
    char*       _mqtt_topic = nullptr;         // MQTT topic

    cb_load_control_t _gpio_before = nullptr;  // Pointer to the callback function to be called before set physical level to GPIO
    cb_load_control_t _gpio_after = nullptr;   // Pointer to the callback function to be called after set physical level to GPIO
    cb_load_control_t _load_state = nullptr;   // Pointer to the callback function to be called after load switching
    cb_load_publish_t _mqtt_publish = nullptr; // Pointer to the publish callback function
};

#ifdef __cplusplus
}
#endif

#endif // __RE_LOADCTRL_H__