/**
  * @file    NodeFlow.cpp
  * @version 1.0.0
  * @author  
  * @brief   C++ file of the NodeFlow.cpp. 
  * A library which implements a software flow for thingpilot Node Core powered modules. 
  * Controls the device configuration, OTA config updates, security, sensor read timing, 
  * telemetry transmit timing and provides pure virtual functions for the application developer 
  * to implement application device specific logic
  */


/** Includes 
 */
#include "node_flow.h"

/**
 *  Initialise device, sets default configuration where needed,
 *  Initialise rtc and get_wakeup_type
 *  Must be called by the user ex. mydevice.Initialise();
 */
  void NodeFlow::start()
{   
    wdg.kick(); 
    _init_rtc();
    
    }

/**
 *
 */   
 void NodeFlow::initialise()
{   
    wdg.kick(); 
    _init_rtc();
    
    }

/**
 * This should only be use on first wakeup or reset device as it will ovewrite time and sensor Ids
 */
bool NodeFlow::registerSensor(const uint8_t sensorId=NULL,const uint8_t seadingTime=NULL)
{
    //Create an array of sensor ids with sensortypes in eeprom, with their reading time

}

/** 
    if its the first time (in case of reset set the value to the smallest directly)
    for each sensor calculate next reading time get the smallest 
    and set it as the next reading timer
 */ 

int NodeFlow::set_reading_time(int arr[],int sizeofarray){

//int array[]={0,0};   //each time the user creates a new sensor  
    
    int time_comparator=0; 
    int temp=0;

    
    int sizeofarray=sizeof(arr)/sizeof(arr[0]);
    for (int i=0; i<=sizeofarray; i++){
        //int temp=arr[i];
        if (temp>=arr[i] && arr[i]!=0){
            temp=arr[i];   
        }
    time_comparator=temp;
    //printf("\r\nMin: %d , %d",time_comparator, sizeofarray);
    //printf("\r\n %d",time_comparator);
    } 
    return time_comparator;
}
/**
 * 
 */
int NodeFlow::SensorHandler(){

    



}
void NodeFlow::reset(){

}


// void ModemSend(int time)
// {
    
// }

/**
 *
 */

   union DeviceConfig
{
    struct 
    {
        uint16_t device_id;
        int timestamp;
        uint16_t mode;
        uint16_t property;
        uint8_t flag;
        uint8_t cool;
    } parameters;

    char data[sizeof(DeviceConfig::parameters)];
};

union SensorA
{
    struct 
    {
        float temp;
        int timestamp;
    } parameters;

    char data[sizeof(SensorA::parameters)];
};

union SensorB
{
    struct 
    {
        uint16_t hum;
        int timestamp;
    } parameters;

    char data[sizeof(SensorB::parameters)];
};

enum Filenames
{
    DeviceConfig_n = 0,
    SensorA_n      = 1,
    SensorB_n      = 2
};


 RTC_HandleTypeDef RtcHandle;

void NodeFlow::_init_rtc() {
   PlatformMutex *mtx = new PlatformMutex;
   mtx->lock();
   rtc_init();
   mtx->unlock();
   delete(mtx);
}
void NodeFlow::SystemPower_Config() {
   HAL_Init();
   GPIO_InitTypeDef GPIO_InitStructure;
   __HAL_RCC_PWR_CLK_ENABLE();
   HAL_PWREx_EnableUltraLowPower();
   HAL_PWREx_EnableFastWakeUp();
   __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOH_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();
   GPIO_InitStructure.Pin = GPIO_PIN_All;
   GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStructure.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
   __HAL_RCC_GPIOA_CLK_DISABLE();
   __HAL_RCC_GPIOB_CLK_DISABLE();
   __HAL_RCC_GPIOC_CLK_DISABLE();
   __HAL_RCC_GPIOD_CLK_DISABLE();
   __HAL_RCC_GPIOH_CLK_DISABLE();
   __HAL_RCC_GPIOE_CLK_DISABLE();
}


 void NodeFlow::rtc_set_wake_up_timer_s(uint32_t delta) {
   uint32_t clock = RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
   // HAL_RTCEx_SetWakeUpTimer_IT will assert that delta is 0xFFFF at max
   if (delta > 0xFFFF) {
       delta -= 0x10000;
       clock = RTC_WAKEUPCLOCK_CK_SPRE_17BITS;
   }
   RtcHandle.Instance = RTC;
   HAL_StatusTypeDef status = HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, delta, clock);
   if (status != HAL_OK) {
       NVIC_SystemReset();
    }
}
void NodeFlow::clear_uc_wakeup_flags() {
   __HAL_RCC_CLEAR_RESET_FLAGS();
   SET_BIT(PWR->CR, PWR_CR_CWUF);
}
static WakeupType get_wakeup_type() {
   if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
       return WAKEUP_RESET;
   }
   if(READ_BIT(RTC->ISR, RTC_ISR_WUTF)) {
       return WAKEUP_TIMER;
   }
   if(READ_BIT(PWR->CSR, PWR_CSR_WUF)) {
       return WAKEUP_PIN;
   }
   return WAKEUP_RESET;
}

void NodeFlow::standby(int seconds, bool wkup_one, bool wkup_two) {
   SystemPower_Config();
   core_util_critical_section_enter();
   clear_uc_wakeup_flags();
   // Enable wakeup timer.
   rtc_set_wake_up_timer_s(seconds);
   if(wkup_one) {
       HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
   }
   else {
       HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
   }
   if(wkup_two) {
       HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
   }
   else {
       HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
   }
   HAL_PWR_EnterSTANDBYMode();
   // this should not happen...
   //rtc_deactivate_wake_up_timer();
   core_util_critical_section_exit();
   // something went wrong, let's reset
   NVIC_SystemReset();
}

