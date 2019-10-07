/**
  * @file    NodeFlow.cpp
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
#include "node_flow.h"
TPL5010 tpl5010(PinName done);

NodeFlow::NodeFlow(){

}
// 

/* Kick wdg, read flags*/
void NodeFlow::Init()
{
   init_eeprom();
   
   
   tpflags a={1}; 
   a.flag; //set state in debug return value from eeprom, default 0
}

void NodeFlow::NodeFlowStart()
{
   
   tpflags a={1}; 
}

void NodeFlow::PeriodicSenseHandler()
{
    tpflags a={2};
}

void NodeFlow::TriggerSenseHandler()
{
    tpflags a={3};
}

void NodeFlow::init_eeprom()
{
    //
}






