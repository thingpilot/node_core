
/**
  * @file    NodeFlow.h
  * @version 1.0.0
  * @author  
  * @brief   Header file of the SX1276 driver module. 
  * A library which implements a software flow for thingpilot Node Core powered modules. 
  * Controls the device configuration, OTA config updates, security, sensor read timing, 
  * telemetry transmit timing and provides pure virtual functions for the application developer 
  * to implement application device specific logic
  */

#ifndef MBED_NODEFLOW_H
#define MBED_NODEFLOW_H

/** Includes 
 */
#include "TPL5010.h"
#include "tformatter.h"
#include "DataManager.h"
#include "SaraN2.h"
#include "LorawanTP.h"
#include "rtc_api_hal.h"

/** Enum
 */            
enum WakeupType {
   WAKEUP_RESET,
   WAKEUP_TIMER,
   WAKEUP_PIN
};
/**
 * Type of sensors. This will be used for 
 */
typedef enum {
    TEMP_C, HMDTY_RH, LGHT_L, AUX, AUX_2, AUX_3, AUX_4
} sensor;

/** Base class for the Nodeflow
 */      
class NodeFlow {

    public:
    /**
     * Constructor
     */
    NodeFlow();  
    
    /**
     * Destructor
     */
    virtual ~NodeFlow();
    
    /**
     * Generic Initialise device, init_eeprom, set flag to debug mode
     */
    virtual void initialise(); 
     /**
     * Start
     */
    virtual void start(); 
    /**
     * Sends an alive beat. The device is on and working kindoff beat (can be skipped)
     */
    bool alivebeat();
    
    /**
     * Return WakeupType. 
     */
     static WakeupType get_wakeup_type();

     /**
     * Reset. 
     */
     virtual void reset();
     /**
     *  Sets the device in standby mode. Handles the times, 
     *  @param periodic_intervals   Sets periodic time intervals for device to wakeup
     *  periodic times must equal with the highest common factor
     *  @param interrupt_sleep_time Sets time that the interrupt will sleep after first wakeup
     *  ex. enter_standby(periodic_intervals, interrupt_sleep_time);
     */

     void enter_standby(int periodic_intervals=NULL);
     //void standby(int seconds, bool wkup_one, bool wkup_two);


    /**
     * Defines how long the device will ignore the interrupt after an interrupt goes off
     *
     */
     #if defined (MBED_INTERRUPTIN_H)  //if interrupt is defined 
     void set_max_interrupt_sleep(int interrupt_sleep_time=NULL);
     #endif

    /**
     *
     */
    void set_sensors(char my_node_id_1=NULL,char my_node_id_2=NULL,char my_node_id_3=NULL);
    
    /** The sensors should present themselves before they start reporting sensor data to the controller. 
     *  @param childSensorId - The unique child id you want to choose for the sensor connected to the thing pilot. Range 0-254.
     *  @param sensorType - The sensor type you want to create.
     *  @param description An optional textual description of the attached sensor.
     *  @param echo - Set this to true if you want destination node to echo the message back to this node. 
     *  Default is not to request echo. If set to true, the final destination will echo back the contents of the message, 
     *  triggering the receive() function on the original node with a copy of the message, with message.
     *  isEcho() set to true and sender/destination switched.
     *
     *  Returns true if sensor is initialised
     */
     virtual bool present(uint8_t childSensorId=NULL, uint8_t sensorType=NULL,uint8_t readingTime=NULL,
                         const char *description=NULL, bool echo=false);
    /**
     * Calculates the next sensor reading timer
     * 
     */
     int next_reading_time();

     /**
      *
      */
     int SensorsHandler();

     void TimeTriggerHandler();


    void   SystemPower_Config();
    static void rtc_set_wake_up_timer_s(uint32_t delta);
    void clear_uc_wakeup_flags();
    /**/

    //watchdog driver
    int ModemSend(int periodic_intervals);

    private:

    static RTC_HandleTypeDef RtcHandle;
    TPL5010 wdg;
    void _init_rtc();
    
    
    
    /* Actions */
   // void init_eeprom(); from DataManager
    // void get_periodic_sensors(); //Sensors driver
    // void get_trigger_sensors(); //Sensors driver
    
     float get_supply_voltage(); //treat as a sensor 

     


};


#endif