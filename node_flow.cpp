/**
  * @file    NodeFlow.cpp
  * @version 0.1.0
  * @author  Rafaella Neofytou, Adam Mitchell
  * @brief   C++ file of the NodeFlow. Provides a very lightweight filesystem to facilitate the
  *          storage of arbitrary file types
  */

  /** Includes
 */
#include "node_flow.h"


/** Use the SX1276 Lora radio pins
 *
 */
SX1276_LoRaRadio myradio(PA_7,PA_6,PA_5,PB_13,D7,PC_6,PC_7,PC_8,PB_14,PB_15,PC_9,
                            NC,NC,NC,NC,NC,NC,NC);


/**
 */
Serial pc(PC_TXD, PC_RXD);

/**Use the watchdog 
 */
TPL5010 wdg(PA_10);

/**Initialise the wakeup flag as UNKNOWN
 */
int wkp=WAKEUP_UNKNOWN;
int status = 1;

/** Constructor. Create a NodeFlow interface, connected to the pins specified 
 *  operating at the specified frequency
 * 
 * @param write_control GPIO to enable or disable write functionality
 * @param sda I2C data line pin
 * @param scl I2C clock line pin
 * @param frequency_hz The bus frequency in hertz
 */
NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz): 
 DataManager(write_control, sda, scl, frequency_hz) {

 }

/** Destructor. 
 */
 NodeFlow::~NodeFlow() {

 }

/** Eeprom configuration. 
 * @param DeviceConfig. Device specifics- send with the message payload.
 * @param SensorConfig. Each sensor is registered in the Sensor config file.
 * @param SensorA..SensorD. Each sensor will be able to store a specific amount of values (to be specified).
 * @param BatteryVoltage
 *
 */
    union DeviceConfig
{
    struct 
    {
        uint16_t device_id;
        int timestamp;
        uint16_t modulation; 
        uint16_t snr;
        
    } parameters;

    char data[sizeof(DeviceConfig::parameters)];
};

    union SensorConfig
{
    struct 
    {   uint8_t device_id;
        uint8_t device_type;
        uint16_t time_comparator; 
       
    
    } parameters;

    char data[sizeof(SensorConfig::parameters)];
};

union SensorA
{
    struct 
    {   
        uint8_t device_id;
        uint8_t device_type;
        uint16_t value;
        int timestamp;
    } parameters;

    char data[sizeof(SensorA::parameters)];
};

union SensorB
{
    struct 
    {
        uint16_t value;
        uint16_t device_type;
        int timestamp;
    } parameters;

    char data[sizeof(SensorB::parameters)];
};
union BatteryVoltage
{
    struct 
    {
        uint16_t device_type;
        int timestamp;


    } parameters;

    char data[sizeof(BatteryVoltage::parameters)];
};
union TimeConfig
{
    struct 
    {
        uint16_t time_comparator;

        
    } parameters;

    char data[sizeof(TimeConfig::parameters)];
};

   union TempSensorConfig
{
    struct 
    {   uint8_t  device_id;
        uint16_t time_comparator; 
       
    
    } parameters;

    char data[sizeof(TempSensorConfig::parameters)];
};

   union TempConfig
{
    struct 
    {   uint8_t  device_id;
        uint16_t time_comparator; 
       
    
    } parameters;

    char data[sizeof(TempConfig::parameters)];
};

/** Each filename in the eeprom hold a unique number
 */

enum Filenames
{
    DeviceConfig_n          = 0,
    SensorA_n               = 1, 
    SensorB_n               = 2,
    SensorC_n               = 3,
    SensorD_n               = 4,
    SensorE_n               = 5,
    SensorF_n               = 6,
    SensorG_n               = 7,
    SensorConfig_n          = 8,
    BatteryVoltage_n        = 9,  
    TimeConfig_n            = 10,
    TempSensorConfig_n      = 11,
    TempConfig_n            = 12
   
};

/**Types of Sensors. The user will be given the types to choose, he could use a custom type of sensor (ex. custom for voltage)
 */
typedef enum {
    TEMP_C          =0,
    HUM_P           =1,
    LIGHT_LX        =2,
    CUSTOM_V        =3,
    CUSTOM_U        =4
}ValueType;


/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 *  
 * @return wkp. Indicates the wakeup type so the user will be able to change the specific logic.
 */
int NodeFlow::start(string device_id){

  wdg.kick();
 _init_rtc();
 
 wkp=get_wakeup_type();

 if (wkp==WAKEUP_PIN) {
    pc.printf("\r\n--------------------PIN WAKEUP--------------------\r\n");
    return wkp; 
 }
 if (wkp==WAKEUP_TIMER) {
    pc.printf("\r\n-------------------TIMER WAKEUP-------------------\r\n");
    return wkp; 
 }

  if (wkp==WAKEUP_RESET) {
    //set time here
    set_time(1571753420);
    pc.printf("\r\n-------------------THING PILOT--------------------\r\n");;
    
    initialise();
    pc.printf("\r\nWelcome!\r\nYour device id: %s\r\n",device_id.c_str());
    status=add_time_config_file();
    TimeConfig t_conf;
    t_conf.parameters.time_comparator=0;
    status= DataManager::overwrite_file_entries(TimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Time Config failed to overwrite: %i\r\n", status);
        return status;
    }
    return wkp; 
 }
  if (wkp==WAKEUP_SOFTWARE) {
    pc.printf("\r\n--------------------SOFTWARE WAKEUP--------------------\r\n");
     return wkp;   
 }
 if (wkp==WAKEUP_LOWPOWER) {
    pc.printf("\r\n--------------------LOW POWER WAKEUP--------------------\r\n");
     return wkp;
 }
 
 if (wkp==WAKEUP_UNKNOWN) {
    pc.printf("\r\n--------------------UNKNOWN--------------------\r\n"); 
     return wkp;
 }
   
   return status;
}



bool NodeFlow::isReadingTime(int device_id){


return true;
}

/** Initialise the eeprom
 * @return Status
 */
int NodeFlow::initialise(){
    
    pc.printf("\r\n------------------INITIALISATION------------------\r\n");
    status=DataManager::init_filesystem();
    bool initialised = false;
    status=DataManager::is_initialised(initialised);

    if(status!=0){
    pc.printf("Filesystem initialisation failed. status: %i, is initialised: %i\r\n", status, initialised);  
 }

 //get_global_stats();
return status;
}
/** Get global stats
 * @return Status
 */
int NodeFlow::get_global_stats() {
    DataManager_FileSystem::GlobalStats_t g_stats;
    status = DataManager::get_global_stats(g_stats.data);
    DataManager::print_global_stats(pc, g_stats);
 
 return status;
}

/** Initialise the time config file
 * @return Status
 */
int NodeFlow::add_time_config_file(){
    
    DataManager_FileSystem::File_t TimeConfig_File_t;
    TimeConfig_File_t.parameters.filename = TimeConfig_n;
    TimeConfig_File_t.parameters.length_bytes = sizeof(TimeConfig::parameters);
    status=DataManager::add_file(TimeConfig_File_t, 1);

    if (status!=0){
        pc.printf("Time Config failed: %i\r\n", status);
        
    }
    return status;

}


int NodeFlow::add_data_config_file(uint16_t entries_to_store,uint16_t device_id,int timestamp,
                            uint16_t mode, uint16_t property, uint8_t flag,uint8_t cool){

    // DataManager_FileSystem::File_t DeviceConfig_File_t;
    // DeviceConfig_File_t.parameters.filename = DeviceConfig_n;
    // DeviceConfig_File_t.parameters.length_bytes = sizeof(DeviceConfig::parameters);

    
    //  if(DataManager::add_file(DeviceConfig_File_t, entries_to_store)!=0){
        
    //      pc.printf("Unsuccess! status: %i\r\n", status);
    //  }

    //  else{
    //     pc.printf("\r\nadd_file status: %i\r\n", status);
        
    //     DeviceConfig w_conf;
    //     w_conf.parameters.device_id = device_id;
    //     w_conf.parameters.timestamp = timestamp;
    //     w_conf.parameters.flag = mode;
    //     w_conf.parameters.mode = property;
    //     w_conf.parameters.property = flag;
    //     w_conf.parameters.cool = cool;
    //     for(int i = 0; i < 2; i++)
    // {
    //     w_conf.parameters.device_id = i;
    //     status = DataManager::append_file_entry(DeviceConfig_n, w_conf.data, sizeof(w_conf.parameters));
    //     pc.printf("append_file_entry No: %i status: %i\r\n", i, status);
    // }
    //  }
    
return status;
}
// int NodeFlow::add_sensor_config_file(uint16_t entries_to_store){

//     // DataManager_FileSystem::File_t SensorConfig_File_t;
//     // SensorConfig_File_t.parameters.filename = SensorConfig_n;
//     // SensorConfig_File_t.parameters.length_bytes = sizeof(SensorConfig::parameters);
//     // status=DataManager::add_file(SensorConfig_File_t, entries_to_store);

//     //  if(status!=0){
        
//     //      pc.printf("Unsuccess! status: %i\r\n", status);
//     //  }

//     //  else{
//     //     pc.printf("\r\nadd_file status: %i\r\n", status);
//     //  }
    
// return status;
// }

/*Device config, Sensor_1-8*/
//  int NodeFlow::get_file_parameters(uint8_t filename, DataManager_FileSystem::File_t &file){
    
//     // status = DataManager::get_file_by_name(filename, file);
//     // DataManager::print_file(pc, file);
//     return status;
//  }

 int NodeFlow::add_sensors( uint8_t device_id[],uint8_t device_type[],uint16_t reading_time[],
                             size_t number_of_sensors) {
  
  
  if (wkp==WAKEUP_RESET) {
  pc.printf("\r\n-------------------ADD SENSORS--------------------\r\n");
   // get_global_stats();
    if (number_of_sensors>7){
        status=-1; //change 
        pc.printf("Error more than 7 sensors. Status %d \r\n", status);   
        
    }

    else {
        
    DataManager_FileSystem::File_t SensorConfig_File_t;
    SensorConfig_File_t.parameters.filename = SensorConfig_n;
    SensorConfig_File_t.parameters.length_bytes = sizeof(SensorConfig::parameters);
    status=DataManager::add_file(SensorConfig_File_t, number_of_sensors);
     if (status!=0){
        pc.printf("Add file failed: %i\r\n", status);
        return status;
    }

    //at initialisation
    DataManager_FileSystem::File_t TempSensorConfig_File_t;
    TempSensorConfig_File_t.parameters.filename = TempSensorConfig_n;
    TempSensorConfig_File_t.parameters.length_bytes = sizeof(TempSensorConfig::parameters);
    status = DataManager::add_file(TempSensorConfig_File_t, number_of_sensors);
   
        if(status!=0){
             pc.printf("Add file failed: %i\r\n", status);
            }

        else{
                   
            for (int i=0; i<number_of_sensors; i++){
                SensorConfig s_conf;
                //s_conf.parameters.device_id=i;
                s_conf.parameters.device_id = device_id[i];
                s_conf.parameters.device_type =device_type[i];
                s_conf.parameters.time_comparator=reading_time[i];
               
                status=DataManager::append_file_entry(SensorConfig_n, s_conf.data, sizeof(s_conf.parameters));
                if(status!=0){
                    pc.printf("Add file failed: %i\r\n", status);
                }

                //temporary reading times
                TempSensorConfig ts_conf;
                ts_conf.parameters.device_id = device_id[i];
                ts_conf.parameters.time_comparator=reading_time[i];
        
                status = DataManager::append_file_entry(TempSensorConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
                if(status!=0){
                    pc.printf("Error append_file_entries No: %i status: %i\r\n", i, status);
                
                }
                else{
            
                status = DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
                if(status!=0){
                    pc.printf("Read file failed: %i\r\n", status);
                }
                pc.printf("%d. Sensor device id: %i, wake up every: %u Seconds\r\n",i, ts_conf.parameters.device_id,ts_conf.parameters.time_comparator);
               
                }
            }
        }
     }
    
    pc.printf("--------------------------------------------------\r\n");
   
    return status;
  }

   //  get_global_stats();
    return 0; 
}

int NodeFlow::set_reading_time(uint16_t arr[], int n){
 

 pc.printf("\r\n-----------------NEXT READING TIME----------------\r\n");

 TempSensorConfig ts_conf;
 TimeConfig t_conf;
 TempConfig tm_conf;
 SensorConfig s_conf;

 status=DataManager::read_file_entry(TimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
 if (status!=0){
     pc.printf("Error read_file_entry TimeConfig. status: %i\r\n", status);
     return 2;
 }
 int time_comparator=t_conf.parameters.time_comparator; 

 if (time_comparator==0){
    DataManager_FileSystem::File_t TempConfig_File_t;
    TempConfig_File_t.parameters.filename = TempConfig_n;
    TempConfig_File_t.parameters.length_bytes = sizeof(TempConfig::parameters);
    status = DataManager::add_file(TempConfig_File_t, n);
    if (status!=0){
            pc.printf("Add file error. status: %i\r\n", status);
            return 2;
         }
 }
 pc.printf("\r\nTime comparator equals %d (should be zero at first)\r\n",time_comparator);
 status=DataManager::read_file_entry(TempSensorConfig_n, 0, ts_conf.data, sizeof(ts_conf.parameters));
 if (status!=0){
     pc.printf("Error read_file_entry Temporary Config. status: %i\r\n", status);
     return 2;
 }
 int temp=ts_conf.parameters.time_comparator; //time_left for first sensor
 //pc.printf("\r\nTemp equals %d\r\n",temp);

   // int sizeofarray=sizeof(arr)/sizeof(arr[0]);
 for (int i=0; i<n; i++){

        status=DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if (status!=0){
            pc.printf("Error read_file_entry Temporary Config. status: %i\r\n", status);
            return 2;
         }
        int dev_id=ts_conf.parameters.device_id;
        int time_comparator_now= ts_conf.parameters.time_comparator;//(temp1);//-time_comparator;

        if (temp>=time_comparator_now && time_comparator_now!=0){
            temp=time_comparator_now; 
             
        }

    } 
     time_comparator=temp;
   // pc.printf("\r\nTime comparator equals %d \r\n",time_comparator);
    t_conf.parameters.time_comparator=time_comparator;
    status=DataManager::overwrite_file_entries(TimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
     if (status!=0){
            pc.printf("Error overwrite time config. status: %i\r\n", status);
            return 2;
         }
   // pc.printf("Overwrite status: %i\r\n", status);
    

   
    for (int i=0; i<n; i++){

        status=DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if (status!=0){
            pc.printf("Error read temporary time sensor config. status: %i\r\n", status);
            return 2;
         }
        
        int dev_id=ts_conf.parameters.device_id;
        int time_comp= ts_conf.parameters.time_comparator-time_comparator;
        
        tm_conf.parameters.device_id=dev_id;
        tm_conf.parameters.time_comparator=time_comp;
        if (time_comp==0){
            status=DataManager::read_file_entry(SensorConfig_n, i, s_conf.data, sizeof(s_conf.parameters));
            if (status!=0){
            pc.printf("Error read sensor config. status: %i\r\n", status);
            return 2;
             }
            tm_conf.parameters.time_comparator=s_conf.parameters.time_comparator;

        }
       //i can't overwrite while reading the pointer is setting me at the first so i created a temporary config 
        if(i==0){
        status=DataManager::overwrite_file_entries(TempConfig_n, tm_conf.data, sizeof(tm_conf.parameters));  
         
        }
        else{
        status=DataManager::append_file_entry(TempConfig_n, tm_conf.data, sizeof(tm_conf.parameters));
        }
        if (status!=0){
            pc.printf("Error Temp config. status: %i\r\n", status);
            return 2;
             }
    }
    for (int i=0; i<n; i++){
    status=DataManager::read_file_entry(TempConfig_n, i, tm_conf.data, sizeof(tm_conf.parameters));
    if (status!=0){
            pc.printf("Error read temp config. status: %i\r\n", status);
            return 2;
             }
    pc.printf("%d. Sensor device id: %i, next reading: %u seconds\r\n",i, tm_conf.parameters.device_id,tm_conf.parameters.time_comparator);
   
    ts_conf.parameters.device_id=tm_conf.parameters.device_id;
    ts_conf.parameters.time_comparator=tm_conf.parameters.time_comparator;

    if(i==0){
        status=DataManager::overwrite_file_entries(TempSensorConfig_n, ts_conf.data, sizeof(ts_conf.parameters)); 
          
        }
    else{
        status=DataManager::append_file_entry(TempSensorConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
    }
    if (status!=0){
            pc.printf("Error write temp sensor config. status: %i\r\n", status);
            return 2;
             }
    }

    pc.printf("\r\nNext Reading time %d ,No of sensors: %d\r\n",time_comparator, n);
    pc.printf("--------------------------------------------------\r\n");

    return time_comparator;
}

RTC_HandleTypeDef RtcHandle;
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

time_t timestamp;
struct tm currTime;

void NodeFlow::SystemClock_Config(){
 
//  HAL_Init();
// //Initialise the calendar 
// //Disable rtc registers write protection
// RTC->WPR |= 0xCA;
// RTC->WPR |= 0x53;
// //enter initialization mode
// RTC->ISR |= 1;

// while(!(RTC->ISR & RTC_ISR_INITF))
// {
//  RTC->ISR |=RTC_ISR_INIT;

// }
// //nomize en lathos
// RTC->PRER = 0x007f00ff; 

// //load time and date values in the shadow reg
// RTC->TR |=;

}
/* Code to get timestamp 
*
*  You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values 
*  in the higher-order calendar shadow registers to ensure consistency between the time and date values.
*  Reading RTC current time locks the values in calendar shadow registers until Current date is read
*  to ensure consistency between the time and date values.
*/
int NodeFlow::get_timestamp(){

// HAL_RTCEx_GetTimeStamp(&RtcHandle,&currentTime,&currentDate,RTC_FORMAT_BIN);

/* Enable the power interface clock by setting the PWREN bits in the RCC_APB1ENR
register*/    
// RCC->APB1ENR |= RCC_APB1ENR_PWREN;

// PWR->CR |= PWR_CR_DBP;
// RCC->CSR |= (1 << 16);
// RCC->CSR |= RCC_CSR_RTCEN;
/* 
HAL_RTCEx_GetTimeStamp (&RtcHandle, &currentTime, &currentDate,  RTC_FORMAT_BCD);

// HAL_RTC_GetTime(&RtcHandle, &currentTime, RTC_FORMAT_BIN);
// HAL_RTC_GetDate(&RtcHandle, &currentDate, RTC_FORMAT_BIN);

currTime.tm_year = currentDate.Year + 100;  // In fact: 2000 + 18 - 1900
currTime.tm_mday = currentDate.Date;
currTime.tm_mon  = currentDate.Month- 1;

currTime.tm_hour = currentTime.Hours;
currTime.tm_min  = currentTime.Minutes;
currTime.tm_sec  = currentTime.Seconds;

timestamp = mktime(&currTime);
//time_comparator=difftime(time_t time1, time_t time0); measure the difference

pc.printf("Timestamp: %d",timestamp);
//ThisThread::sleep_for(1); 
*/
time_t seconds = time(NULL);
        
pc.printf("Time as seconds since January 1, 1970 = %d\n", seconds);
        
pc.printf("Time as a basic string = %s", ctime(&seconds));
ThisThread::sleep_for(5);
return timestamp;
}

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

int NodeFlow::get_wakeup_type(){
 if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
       return WAKEUP_RESET;
   }
   if(READ_BIT(RTC->ISR, RTC_ISR_WUTF)) {
       return WAKEUP_TIMER;
   }
   if(READ_BIT(PWR->CSR, PWR_CSR_WUF)) {
       return WAKEUP_PIN;
   }

   if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
       return WAKEUP_SOFTWARE;
   }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
       return WAKEUP_LOWPOWER;
   }
   return WAKEUP_UNKNOWN;
    
}


int NodeFlow:: enter_standby(int intervals) {
return 0;
}
void NodeFlow::standby(int seconds, bool wkup_one, bool wkup_two) { 
   //ThisThread::sleep_for(1);
   //seconds=seconds-1;
   if (seconds<1){
       seconds=1;
   } 
   myradio.sleep();
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








