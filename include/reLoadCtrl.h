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
#include <esp_err.h>
#include <driver/gpio.h>
#include "project_config.h"
#include "def_consts.h"

typedef struct {
  uint32_t cntTotal       = 0;
  uint32_t cntToday       = 0;
  uint32_t cntYesterday   = 0;
  uint32_t cntWeekCurr    = 0;
  uint32_t cntWeekPrev    = 0;
  uint32_t cntMonthCurr   = 0;
  uint32_t cntMonthPrev   = 0;
  uint32_t cntPeriodCurr  = 0;
  uint32_t cntPeriodPrev  = 0;
  uint32_t cntYearCurr    = 0;
  uint32_t cntYearPrev    = 0;
} re_load_counters_t;

// Maximum duration for a year: 60 * 60 * 24 * 366 = 31 622 400 = 0x01e28500 < 32bit
typedef struct {
  uint32_t durLast        = 0;
  uint32_t durTotal       = 0;
  uint32_t durToday       = 0;
  uint32_t durYesterday   = 0;
  uint32_t durWeekCurr    = 0;
  uint32_t durWeekPrev    = 0;
  uint32_t durMonthCurr   = 0;
  uint32_t durMonthPrev   = 0;
  uint32_t durPeriodCurr  = 0;
  uint32_t durPeriodPrev  = 0;
  uint32_t durYearCurr    = 0;
  uint32_t durYearPrev    = 0;
} re_load_durations_t;


class rLoadController;

typedef bool (*cb_load_publish_t) (rLoadController *ctrl, char* topic, char* payload, bool forced, bool free_topic, bool free_payload);
typedef void (*cb_load_change_t) (rLoadController *ctrl, bool state, time_t duration);
typedef bool (*cb_load_gpio_init_t) (rLoadController *ctrl, uint8_t pin, bool level_on, bool use_pullup);
typedef bool (*cb_load_gpio_change_t) (rLoadController *ctrl, uint8_t pin, bool physical_level);

#ifdef __cplusplus
extern "C" {
#endif

class rLoadController {
  public:
    rLoadController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
      cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
      cb_load_publish_t cb_mqtt_publish);
    ~rLoadController();

    // Load switching
    bool loadInit(bool init_value);
    bool loadSetState(bool new_state, bool forced, bool publish);

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
    void setCallbacks(cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed);
  protected:
    uint8_t     _pin = 0;                       // Pin number
    bool        _level_on = true;               // Output level at which the load is considered to be on
    bool        _use_pullup = true;             // Use internal pin pullup

    virtual bool loadInitGPIO() = 0;
    virtual bool loadSetStateGPIO(bool physical_level) = 0; 
  private:
    bool        _state = false;                 // Current load state
    time_t      _last_on = 0;                   // The last time the load was turned on
    time_t      _last_off = 0;                  // Time of last load disconnection
    uint8_t*    _period_start = nullptr;        // Day of month at the beginning of the billing period (for example, sending meter readings)
    re_load_counters_t  _counters;              // Counters of the number of load switching
    re_load_durations_t _durations;             // Load operating time counters
    const char* _nvs_space = nullptr;           // Namespace to store counter values 
    char*       _mqtt_topic = nullptr;          // MQTT topic

    cb_load_change_t _gpio_before = nullptr;    // Pointer to the callback function to be called before set physical level to GPIO
    cb_load_change_t _gpio_after = nullptr;     // Pointer to the callback function to be called after set physical level to GPIO
    cb_load_change_t _state_changed = nullptr;  // Pointer to the callback function to be called after load switching
    cb_load_publish_t _mqtt_publish = nullptr;  // Pointer to the publish callback function
};

class rLoadGpioController: public rLoadController {
  public:
    rLoadGpioController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
      cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
      cb_load_publish_t cb_mqtt_publish);
    rLoadGpioController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space);
  protected:
    bool loadInitGPIO() override;
    bool loadSetStateGPIO(bool physical_level) override; 
};

class rLoadIoExpController: public rLoadController {
  public:
    rLoadIoExpController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
      cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change,
      cb_load_change_t cb_gpio_before, cb_load_change_t cb_gpio_after, cb_load_change_t cb_state_changed, 
      cb_load_publish_t cb_mqtt_publish);
    rLoadIoExpController(uint8_t pin, bool level_on, bool use_pullup, const char* nvs_space,
      cb_load_gpio_init_t cb_gpio_init, cb_load_gpio_change_t cb_gpio_change);
  protected:
    bool loadInitGPIO() override;
    bool loadSetStateGPIO(bool physical_level) override; 
  private:
    cb_load_gpio_init_t _gpio_init = nullptr;
    cb_load_gpio_change_t _gpio_change = nullptr;
};

#ifdef __cplusplus
}
#endif

#endif // __RE_LOADCTRL_H__