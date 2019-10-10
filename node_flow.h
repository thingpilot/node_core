
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


enum WakeupType {
    WAKEUP_RESET,
    WAKEUP_TIMER,
    WAKEUP_PIN
    };

/**/    
 

/** Base class for the Nodeflow
 */      
class NodeFlow {

    public:
   /** Enum
    */            
    
    enum {
        NODE_FLOW_OK=0
    };
   /**
    * Type of sensors. This will be used for 
    */
    typedef enum {
        TEMP_C, HMDTY_RH, LIGHT_LX, PRESS_PA, AUX_2, AUX_3, AUX_4
    } sensor;
        
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
     * Return WakeupType. 
     */
     static WakeupType get_wakeup_type();

     

     /**
     * Reset. Reset the eeprom to default 
     */
     virtual void reset();

    /**
     *  Configure Sleep
     */
     /**
     *  Sets the device in standby mode. Handles the times, 
     *  @param periodic_intervals   Sets periodic time intervals for device to wakeup
     *  periodic times must equal with the highest common factor
     *  @param interrupt_sleep_time Sets time that the interrupt will sleep after first wakeup
     *  ex. enter_standby(periodic_intervals, interrupt_sleep_time);
     */

     void enter_standby(int intervals=NULL);
     //void standby(int seconds, bool wkup_one, bool wkup_two);
    int8_t _sleep	(const uint32_t 	sleepingM, const uint8_t interrupt1 =NULL);	
    /**
     * Use this in case of constant periodic sensing from all sensors
     * 
     */
     void setReportIntervalMinutes(int periodic_intervals=NULL);
    /**
     * Defines how long the device will ignore the interrupt after an interrupt goes off
     *
     */
     #if defined (MBED_INTERRUPTIN_H)  //if interrupt is defined 
     void set_max_interrupt_sleep(int interrupt_sleep_time=NULL);
     #endif
    
    
    /** This should only be use on first wakeup or reset device as it will ovewrite time and sensor Ids 
     *  The sensors should present themselves before they start reporting sensor data to the controller. 
     *  @param SensorId - The unique id you want to choose for the sensor connected to the thing pilot. (ex Range 0-254)
     *  @param sensorType - The sensor type you want to create.
     *
     *  Returns true if sensor is initialised
     */
    virtual bool registerSensor(const uint8_t sensorId=NULL, const uint8_t readingTime=NULL);
    
    int set_reading_time(int arr[],int n);
    /**
     * Calculates the next sensor reading timer
     * 
     */
     //int next_reading_time();

    /**
     * Wait without setting the device to standby
     */
     void wait (const uint32_t waitingMS);

    /** Check wich Sensors are set to be read
     *  Return which sensors to be read, return next reading time for each
     */
     int  SensorsHandler();

     void TimeTriggerHandler();


    void   SystemPower_Config();
    static void rtc_set_wake_up_timer_s(uint32_t delta);
    void clear_uc_wakeup_flags();
    /**/

    
    int ModemSend(uint8_t port,const uint8_t *data, uint8_t length,int periodic_intervals,int flags);


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