/**
 ******************************************************************************
 * @file    NodeFlow.cpp
 * @version 0.1.0
 * @author  Rafaella Neofytou, Adam Mitchell
 * @brief   C++ file of the Wright || Earheart node from Think Pilot. 
 ******************************************************************************
 **/

/** Includes
 */
#include "node_flow.h"
/**
 */
Serial pc(PC_10, PC_11);


/**Use the watchdog 
 */
TPL5010 wdg(PA_10); //PA_5 for earhart

/**Initialise the wakeup flag as UNKNOWN
 */
int wkp=WAKEUP_UNKNOWN;
int flags=FLAG_UNKNOWN;
int status = 1;


/** Constructor. Create a NodeFlow interface, connected to the pins specified 
 *  operating at the specified frequency
 * 
 * @param write_control GPIO to enable or disable write functionality
 * @param sda I2C data line pin
 * @param scl I2C clock line pin
 * @param frequency_hz The bus frequency in hertz. */

 NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz): 
 DataManager(write_control, sda, scl, frequency_hz), LorawanTP() {

 }

/** Destructor. 
 */
 NodeFlow::~NodeFlow() {
 }

/** Eeprom configuration. 
 *
 * @param DeviceConfig. Device specifics- send with the message payload.
 *
 * @param SensorConfig. Each sensor is registered in the Sensor config file.
 *
 * @param SensorA..SensorD. Each sensor will be able to store a specific amount of values (to be specified).
 *
 * @param BatteryVoltage
 **/
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
        uint16_t time_comparator; 
       
    
    } parameters;

    char data[sizeof(SensorConfig::parameters)];
};

union SensorA
{
    struct 
    {   
        uint8_t device_id;
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
        int timestamp;
    } parameters;

    char data[sizeof(SensorB::parameters)];
};
union BatteryVoltage
{
    struct 
    {
        uint16_t value;
        int timestamp;

    } parameters;

    char data[sizeof(BatteryVoltage::parameters)];
};

/** Holds only the next sleeping time (time difference)
 */
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

/**Program specific flags*/
   union FlagsConfig
{
    struct 
    {    uint8_t  sensing_time; //both sending&sensing time for lora
         uint8_t  clock_synch; 
         uint8_t  kick_wdg;
         uint8_t  sending_time; //only for NBIOT
       
    } parameters;

    char data[sizeof(FlagsConfig::parameters)];
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
    FlagsConfig_n           = 7,
    SensorConfig_n          = 8,
    BatteryVoltage_n        = 9,  
    TimeConfig_n            = 10,
    TempSensorConfig_n      = 11,
    TempConfig_n            = 12
              
   
};


/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 *  
 * @return wkp. Indicates the wakeup type so the user will be able to change the specific logic.
 */
int NodeFlow::start(string device_id){

  wdg.kick();
 _init_rtc();
 
 wkp=get_wakeup_type();
 flags=get_flags();
 if (wkp==WAKEUP_PIN) {
    pc.printf("\r\n--------------------PIN WAKEUP--------------------\r\n");
    return wkp; 
 }
 if (wkp==WAKEUP_TIMER) {
    pc.printf("\r\n-------------------TIMER WAKEUP-------------------\r\n");
    return wkp; 
 }
if (wkp==WAKEUP_RESET) {
    pc.printf("\r\n-------------------THING PILOT--------------------\r\nWelcome!\r\nYour device id: %s\r\n",device_id.c_str());
   
    initialise();
    time_now();
    time_config_init();
    flags_config_init();
     
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


/** Initialise the eeprom.Retry 
 * @return Status
 */
int NodeFlow::initialise(){    
    status=DataManager::init_filesystem();
    if(status!=DATA_MANAGER_OK){
        status=DataManager::init_filesystem();
    }
    bool initialised = false;
    status=DataManager::is_initialised(initialised);
    if(status!=DATA_MANAGER_OK){
       status=DataManager::is_initialised(initialised);
    }
    if(status!=0){
        pc.printf("Filesystem initialisation failed. status: %i, is initialised: %i\r\n", status, initialised);
    }
    
    
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
int NodeFlow::time_config_init(){
    DataManager_FileSystem::File_t TimeConfig_File_t;
    TimeConfig_File_t.parameters.filename = TimeConfig_n;
    TimeConfig_File_t.parameters.length_bytes = sizeof(TimeConfig::parameters);
    status=DataManager::add_file(TimeConfig_File_t, 1);

    if (status!=0){
        pc.printf("Time Config failed: %i\r\n", status);  
    }

    status=set_time_config(0);
    return status;
}

int NodeFlow:: set_time_config(int time_comparator){
    TimeConfig t_conf;
    t_conf.parameters.time_comparator=time_comparator;
    status= DataManager::overwrite_file_entries(TimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Time Config failed to overwrite: %i\r\n", status);
    }
    
    return status;
}

int NodeFlow::flags_config_init(){
    
    DataManager_FileSystem::File_t FlagsConfig_File_t;
    FlagsConfig_File_t.parameters.filename = FlagsConfig_n;
    FlagsConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);
    status=DataManager::add_file(FlagsConfig_File_t, 1);

    if (status!=0){
        pc.printf("FLAGS Config failed: %i\r\n", status);  
    }
    
    status=set_flags_config(false,false,true);
   
    return status;
}

int NodeFlow:: set_flags_config(bool kick_wdg, bool sense_time, bool clock_synch){
 
    FlagsConfig f_conf;
    f_conf.parameters.kick_wdg=kick_wdg;
    f_conf.parameters.sensing_time=sense_time;
    f_conf.parameters.clock_synch=clock_synch;
    f_conf.parameters.sending_time=0;
    status= DataManager::overwrite_file_entries(FlagsConfig_n, f_conf.data, sizeof(f_conf.parameters));
    if (status!=0){
        pc.printf("Flags Config failed to overwrite: %i\r\n", status);  
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

 int NodeFlow::add_sensors( uint8_t device_id[],uint16_t reading_time[],
                             size_t number_of_sensors) {
  
  if (wkp==WAKEUP_RESET) {
    pc.printf("\r\n-------------------ADD SENSORS--------------------\r\n");
    
    if (number_of_sensors>7){
        status=-1; //change 
        pc.printf("Error more than 7 sensors. Status %d \r\n", status);   
        return status;
    }
    else {
    /** Files Initialisation
     */    
    DataManager_FileSystem::File_t SensorConfig_File_t;
    SensorConfig_File_t.parameters.filename = SensorConfig_n;
    SensorConfig_File_t.parameters.length_bytes = sizeof(SensorConfig::parameters);
    status=DataManager::add_file(SensorConfig_File_t, number_of_sensors);
        if (status!=0){
            pc.printf("Add file failed: %i\r\n", status);
            return status; }
    
    DataManager_FileSystem::File_t TempSensorConfig_File_t;
    TempSensorConfig_File_t.parameters.filename = TempSensorConfig_n;
    TempSensorConfig_File_t.parameters.length_bytes = sizeof(TempSensorConfig::parameters);
    status = DataManager::add_file(TempSensorConfig_File_t, number_of_sensors);
        if(status!=0){
             pc.printf("Add file failed: %i\r\n", status);
             return status;}
        
        for (int i=0; i<number_of_sensors; i++){
             SensorConfig s_conf;
             s_conf.parameters.device_id = device_id[i];
             s_conf.parameters.time_comparator=reading_time[i];
               
             status=DataManager::append_file_entry(SensorConfig_n, s_conf.data, sizeof(s_conf.parameters));
             if(status!=0){
                pc.printf("Add file failed: %i\r\n", status);
                return status;}

            //temporary reading times
             TempSensorConfig ts_conf;
             ts_conf.parameters.device_id = device_id[i];
             ts_conf.parameters.time_comparator=reading_time[i];
        
             status = DataManager::append_file_entry(TempSensorConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
             if(status!=0){
                 pc.printf("Error append_file_entries No: %i status: %i\r\n", i, status);
                 return status;}
             
             status = DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
             if(status!=0){
                 pc.printf("Read file failed: %i\r\n", status);
                 return status;}
             pc.printf("%d. Sensor device id: %i, wake up every: %u Seconds\r\n",i, ts_conf.parameters.device_id,ts_conf.parameters.time_comparator);
                  
            }
     }
    pc.printf("--------------------------------------------------\r\n");
   
    return status;
  }

    return 0; 
}

/**Returns seconds until next reading
 */
int NodeFlow::set_scheduler(float reading_specific_time_h_m[],size_t length){
    pc.printf("\r\n-----------------NEXT READING TIME----------------\r\n");
    
    if (wkp==WAKEUP_RESET){
      for (int i=0; i<length; i++){
          float time=reading_specific_time_h_m[i];
          uint16_t time_remainder=(((int(time))*3600)+((fmod(time,1))*6000))/2;
         //store those values to eeprom
          
      }  
         
     }
    if (wkp==WAKEUP_RESET ){// || flags==FLAG_CLOCK_SYNCH)){
        get_timestamp();
        }
    uint32_t time_remainder=this->time_now();
    //LOOKING FOR CLOCKSYNCH HOUR
    //uint32_t clock_synch=0;
    uint32_t timediff_temp=86400;
    //Remainding until clock synch
    int32_t timediff=6600;
    
    for (int i=0; i<length; i++){
        float time=reading_specific_time_h_m[i];
        uint32_t t=((int(time))*3600)+((fmod(time,1))*6000); 

        timediff=t-time_remainder;
        if (timediff<0){
            timediff=timediff+86400;
        }
        if (timediff<timediff_temp){
            timediff_temp=timediff;
        }
    }
    // if (timediff_temp>clock_synch){
        //set_flags_config(false, false, true);
    // }
    if (timediff_temp>7200)  {
        timediff_temp=6600;
        //set_flags_config(bool kick_wdg, bool sense_time, bool clock_synch)
        set_flags_config(true, false, false);
    }


    // if (timediff_temp<clock_synch && wkp!=WAKEUP_RESET){
    //         timediff_temp=clock_synch;
    //     }
    pc.printf("Next Reading in minutes %d\r\n", timediff_temp/60);
   // ThisThread::sleep_fo50000);
    return timediff_temp; 
}



int NodeFlow::set_reading_time(uint16_t arr[], size_t n){
 
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
            return status;
         }
    }
    pc.printf("\r\nTime comparator equals %d (should be zero at first)\r\n",time_comparator);
    status=DataManager::read_file_entry(TempSensorConfig_n, 0, ts_conf.data, sizeof(ts_conf.parameters));
    if (status!=0){
        pc.printf("Error read_file_entry Temporary Config. status: %i\r\n", status);
        return status;
    }
    int temp=ts_conf.parameters.time_comparator; //time_left for first sensor
    
    for (int i=0; i<n; i++){

        status=DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if (status!=0){
            pc.printf("Error read_file_entry Temporary Config. status: %i\r\n", status);
            return status;
         }
        int dev_id=ts_conf.parameters.device_id;
        int time_comparator_now= ts_conf.parameters.time_comparator;//(temp1);//-time_comparator;

        if (temp>=time_comparator_now && time_comparator_now!=0){
            temp=time_comparator_now;      
            }
        } 
     time_comparator=temp;
   
    status=set_time_config(time_comparator);
     
    for (int i=0; i<n; i++){

        status=DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if (status!=0){
            pc.printf("Error read temporary time sensor config. status: %i\r\n", status);
            return status;
         }
        
        int dev_id=ts_conf.parameters.device_id;
        int time_comp= ts_conf.parameters.time_comparator-time_comparator;
        
        tm_conf.parameters.device_id=dev_id;
        tm_conf.parameters.time_comparator=time_comp;
        if (time_comp==0){
            status=DataManager::read_file_entry(SensorConfig_n, i, s_conf.data, sizeof(s_conf.parameters));
            if (status!=0){
            pc.printf("Error read sensor config. status: %i\r\n", status);
            return status;
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
            return status;
             }
    }
    for (int i=0; i<n; i++){
    status=DataManager::read_file_entry(TempConfig_n, i, tm_conf.data, sizeof(tm_conf.parameters));
    if (status!=0){
            pc.printf("Error read temp config. status: %i\r\n", status);
            return status;
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
            return status;
             }
    }

    pc.printf("\r\nNext Reading time %d ,No of sensors: %d\r\n",time_comparator, n);
    pc.printf("--------------------------------------------------\r\n");

    return time_comparator;
}


/** Timestamp. Send ClockSync message, wait for a response from ttn if it fails don't change the time
    * @param num_timestamp_retries  The number of retries to get the Timestamp
    * 

 */
uint8_t NodeFlow::get_timestamp(){
#if defined (BOARD) && (BOARD == DEVELOPMENT_BOARD_V1_1_0)
    {
        joinTTN();
        int64_t timestamp=0;
        uint8_t dummy[1]={0};
        uint8_t port= 223;
        pc.printf("Horrayy,setting the time, bear with me\r\n");
        uint8_t retcode=LorawanTP::send_message(port, dummy, sizeof(dummy));
        if(retcode<=0){pc.printf("Failed to send\r\n");}

        LorawanTP::receive_message();
        ThisThread::sleep_for(1000);
        for(int i=0; (timestamp<=0) && (i<4); i++) {
        pc.printf("\r\n%i. Waiting for a server message dude \r\n",i);
        ThisThread::sleep_for(5000);
        retcode=LorawanTP::send_message(port, dummy, sizeof(dummy));
        if(retcode<0){
            pc.printf("Failed to send\r\n");
            }
        timestamp=LorawanTP::receive_message().received_value[0];
        
        pc.printf("\"Timestamp\"(it will be wrong: %llu",timestamp);
        }
    }
    #endif /* #if defined (BOARD) && (BOARD == DEVELOPMENT_BOARD_V1_1_0) */
    
 return 0;   
}


uint32_t NodeFlow::time_now() {
    time_t time_now=time(NULL);
    uint32_t timestamp=time_now; 
    uint32_t remainder_time=(timestamp%86400);
    double_t t_value=(remainder_time/3600.000000);
    double_t minutes_f=fmod(t_value,1);
    uint8_t hours= t_value- minutes_f ;
    double_t minutes=minutes_f*60; 
    double_t seconds=(fmod(minutes,1))*60;
    pc.printf("Time now(HH:MM:SS)= %02d:%02d:%02d.\r\n", hours, int(minutes),int(seconds));

    return remainder_time;
}

/**LorawanTP
 */
int NodeFlow::joinTTN(){
    
    int retcode=LorawanTP::join();
        if(retcode<0){
            pc.printf("Failed to join\r\n");}
    
    return retcode;
}
int NodeFlow::sendTTN(uint8_t port, uint8_t payload[], uint16_t length){
    int retcode=0;
    //will change with flag isReadingTime
    if (wkp==WAKEUP_TIMER){
        joinTTN();
        pc.printf("---------------------SENDING----------------------\r\n");
        time_now();

        retcode=LorawanTP::send_message(port, payload, length);
        if (retcode<0){
            pc.printf("\r\nError Sending %d bytes\r\n", retcode);
        }
        else{
        pc.printf("\r\nSending %d bytes\r\n", retcode);   }
        pc.printf("--------------------------------------------------\r\n");
    }
    return retcode;
}

uint64_t NodeFlow::receiveTTN(){
    uint16_t decValue=0;
    if (wkp==WAKEUP_TIMER){
        decValue=LorawanTP::receive_message().received_value[0];
       uint8_t port=LorawanTP::receive_message().port;
        if ( decValue<=0){
            pc.printf("No Uplink\r\n");
            }
        else{
            pc.printf("Rx decimal value %d\r\n", decValue);
            pc.printf("port %d\r\n", port);
        }
    }
    return decValue;
}
RTC_HandleTypeDef RtcHandle;
/**Standby
 */
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


int NodeFlow::get_flags(){
FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if (status!=0){
        pc.printf("FlagsConfig. status: %i\r\n", status);
        return status;
    }
     if (f_conf.parameters.clock_synch==1) {
         return FLAG_CLOCK_SYNCH;
     }
    if (f_conf.parameters.kick_wdg==1) {
         return FLAG_WDG;
     }
     if (f_conf.parameters.sensing_time==1) {
         return FLAG_SENSING;
     }
     return FLAG_UNKNOWN;

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


void NodeFlow::standby(int seconds, bool wkup_one, bool wkup_two) { 
   if (seconds<60){
       seconds=60;
   } 
   int retcode=LorawanTP::sleep();
   if(retcode!=LORAWAN_STATUS_OK){
       pc.printf("\r\nLora not on sleep?!\r\n");     
    }
   ThisThread::sleep_for(100);
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








