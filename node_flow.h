
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

/** Includes 
 */

#ifndef MBED_NODEFLOW_H
#define MBED_NODEFLOW_H


#include "TPL5010.h"
#include "tformatter.h"
#include "DataManager.h"
#include "SaraN2.h"
#include "LorawanTP.h"


        
struct tpflags {

        //3-bit flag, diff compination deferent flag code
        unsigned int flag : 3; 
        // 0.in_reset_mode=   000
        // 1.in_debug_mode=   001
        // 2.in_read_mode=    010
        // 3.in_trigger_mode= 011
        // 4.in_sleep_mode=   100

};


/** Base class for the Nodeflow
 */      
class NodeFlow {
    public:
    /*Constructor*/
    NodeFlow();  //Node flow as an InterruptHandler
    
    /*Deconstructor*/
    ~NodeFlow();
    
    /*Interrupts*/
    void NodeFlowStart(); //kick wdg, read flags
    void Init(); //initialise device, init_eeprom, set flag to debug
    void PeriodicSenseHandler(); //
    void TriggerSenseHandler();
    
    private:
    /* Actions */
    void init_eeprom(); 
    void get_periodic_sensors(); //periodic
    void get_trigger_sensors(); //Sensors driver
    void enter_standby(); //watchdog driver

    float get_supply_voltage();

    TPL5010 tpl5010(PinName done);
    
    

};


#endif