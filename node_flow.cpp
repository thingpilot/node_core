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
#include "mbed_debug.h"
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

/** Define the target 
 */ 
int NodeFlow::getPlatform(){
#if defined (TP_DEVELOPMENT_BOARD_V1_1_0)
    {
        pc.printf("DEVELOPMENT_BOARD_V1_1_0")
        return DEVELOPMENT_BOARD_V1_1_0;
    }
#endif

#if defined (TP_WRIGHT_V1_0_0)
    {
        return WRIGHT_V1_0_0;
    }
#endif

#if defined (TP_EARHART_V1_0_0)
    {
        return EARHART_V1_0_0;
    }
#endif

return 0;
}

/** Eeprom configuration. 
 *
 * @param DeviceConfig. Device specifics- send with the message payload.
 *
 * @param SensorDataConfig. Each sensor will be able to store a specific amount of values (to be specified).
 *
 * @param SchedulerConfig. Holds the scheduled times by the user.
 *
 * @param SensorConfig. Each sensor is registered in the Sensor config file.
 *
 **/
    union DeviceConfig
{
    struct 
    {
        uint32_t device_sn; //Device unique id?! our unique id?
        uint8_t modulation; //defined 0 or 1 for lora, nbiot respectively, 
        
    } parameters;

    char data[sizeof(DeviceConfig::parameters)];
};

/**We need to agree on what this shoud be, data formatter? 
 * I think 
 */
union SensorDataConfig
{
    struct 
    {   
        uint8_t sensor_id;
        uint16_t value;
        uint32_t timestamp;
    } parameters;

    char data[sizeof(SensorDataConfig::parameters)];
};

/**The User can define MAX_BUFFER_READING_TIMES */
union SchedulerConfig
{
    struct 
    {   
        uint16_t time_comparator; //fisrt value holds status, second holds length of the array
        
    } parameters;

    char data[sizeof(SchedulerConfig::parameters)];
};
union ClockSynchConfig
{
    struct 
    {  
        uint16_t time_comparator;
        bool clockSynchOn;
        
    } parameters;

    char data[sizeof(ClockSynchConfig::parameters)];
};
/**Program specific flags*/
   union FlagsConfig
{
    struct 
    {    bool  sensing_time; //both sending&sensing time for lora
         bool  clock_synch; 
         bool  kick_wdg;
         bool  sending_time; //only for NBIOT
       
    } parameters;

    char data[sizeof(FlagsConfig::parameters)];
};

   union IncrementConfig
{
    struct 
    {    
        uint16_t  increment; 
    } parameters;

    char data[sizeof(IncrementConfig::parameters)];
};

/**Sensor Config,TempSensorConfig, Time Config&&TempConfig be used in later version 
 * if the user wants to "register" each sensor for different reading times */
   union SensorConfig
{
    struct 
    {   uint8_t device_id;
        uint16_t time_comparator; 
       
    
    } parameters;

    char data[sizeof(SensorConfig::parameters)];
};

   union TempSensorConfig
{
    struct 
    {   uint8_t  device_id;
        uint16_t time_comparator; 
       
    
    } parameters;

    char data[sizeof(TempSensorConfig::parameters)];
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
    SensorDataConfig        = 1, 
    SchedulerConfig_n       = 2,
    ClockSynchConfig_n      = 3,
    FlagsConfig_n           = 4, //wdg,clock synch, sensing ,sending
    IncrementConfig_n       = 5,
    SensorConfig_n          = 6, //not using for now
    TimeConfig_n            = 7,
    TempSensorConfig_n      = 8,
    TempConfig_n            = 9
};

/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 *  
 * @return wkp. Indicates the wakeup type so the user will be able to change the specific logic.
 */
int NodeFlow::start(){
    wdg.kick();
    _init_rtc();
    wkp=get_wakeup_type();
 
    if (wkp==WAKEUP_PIN) {
        pc.printf("\r\n--------------------PIN WAKEUP--------------------\r\n");
        time_now();
        HandleInterrupt();
        
    }
    else if (wkp==WAKEUP_TIMER) {
        pc.printf("\r\n-------------------TIMER WAKEUP-------------------\r\n");
        time_now();
        HandleModem();
        
    }
    else if (wkp==WAKEUP_RESET) {
        pc.printf("\r\n                      __|__       \n               --@--@--(_)--@--@--\n-------------------THING PILOT--------------------\r\n");
        pc.printf("\nDevice Unique ID: %08X %08X %08X \r", STM32_UID[0], STM32_UID[1], STM32_UID[2]);
        initialise();
        time_now();
        
        if(getPlatform()==DEVELOPMENT_BOARD_V1_1_0){
            overwrite_sched_config(SCHEDULER,0);
            overwrite_sched_config(SCHEDULER_SIZE,1);
            if (SCHEDULER==true){
                get_timestamp();
                for (int i=0; i<SCHEDULER_SIZE; i++){
                uint16_t time_remainder=(((int(scheduler[i]))*3600)+((fmod(scheduler[i],1))*6000))/2; 
                timetodate(time_remainder*2);
                overwrite_sched_config(time_remainder,i+2);
                }
            }
            else if(SCHEDULER==false){
                overwrite_sched_config(NEXT_TIME,3);
            }    
        }
    }
        else {
            pc.printf("\r\n---------------------UNKNOWN---------------------\r\n");
        }
    flags=get_flags();
    uint16_t next_time=set_scheduler();
    standby(next_time,true,true);  
    return wkp;
}

int NodeFlow::HandleModem(){
    if(getPlatform()==DEVELOPMENT_BOARD_V1_1_0){
        
        uint16_t length=0;
        uint8_t *payload=0;
        payload=HandlePeriodic(length);
        sendTTN(1, payload, length);            
        receiveTTN();

 }
  return 0;
}
/** Initialise the EEPROM
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
    config_init();
       
return status;
}

int NodeFlow::config_init(){
/**DeviceConfig */
    DataManager_FileSystem::File_t DeviceConfig_File_t;
    DeviceConfig_File_t.parameters.filename = DeviceConfig_n;
    DeviceConfig_File_t.parameters.length_bytes = sizeof( DeviceConfig::parameters);
    status=DataManager::add_file(DeviceConfig_File_t, 1); 
    if (status!=0){
        pc.printf("Device Config failed: %i\r\n", status); 
        return status;  
    }

    DeviceConfig w_conf;
    w_conf.parameters.device_sn = STM32_UID[0];
    w_conf.parameters.modulation = getPlatform();
    status = DataManager::append_file_entry(DeviceConfig_n, w_conf.data, sizeof(w_conf.parameters));
    if (status!=0){
        pc.printf("DeviceConfig error: %i status: %i\r\n", 0, status);
        return status; 
    }
/**SchedulerConfig */
    DataManager_FileSystem::File_t SchedulerConfig_File_t;
    SchedulerConfig_File_t.parameters.filename = SchedulerConfig_n;
    SchedulerConfig_File_t.parameters.length_bytes = sizeof( SchedulerConfig::parameters);
    status=DataManager::add_file(SchedulerConfig_File_t, MAX_BUFFER_READING_TIMES); 
    if (status!=0){
        pc.printf("Scheduler Config failed: %i\r\n", status);
        return status;   
    }
  /**ClockSynchConfig */
    DataManager_FileSystem::File_t ClockSynchConfig_File_t;
    ClockSynchConfig_File_t.parameters.filename = ClockSynchConfig_n;
    ClockSynchConfig_File_t.parameters.length_bytes = sizeof( ClockSynchConfig::parameters);
    status=DataManager::add_file(ClockSynchConfig_File_t, 1); 
    if (status!=0){
        pc.printf("ClockSynchConfig failed: %i\r\n", status);
        return status;   
    }
    overwrite_clock_synch_config(CLOCK_SYNCH_TIME/2,CLOCK_SYNCH);
    
/**FlagsConfig*/
    DataManager_FileSystem::File_t FlagsConfig_File_t;
    FlagsConfig_File_t.parameters.filename = FlagsConfig_n;
    FlagsConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);
    status=DataManager::add_file(FlagsConfig_File_t, 1);
    if (status!=0){
        pc.printf("FLAGS Config failed: %i\r\n", status);
        return status;   
    }
    status=set_flags_config(false, true, false);  //sensing true
/**IncrementConfig*/
    DataManager_FileSystem::File_t IncrementConfig_File_t;
    IncrementConfig_File_t.parameters.filename = IncrementConfig_n;
    IncrementConfig_File_t.parameters.length_bytes = sizeof(IncrementConfig::parameters);
    status=DataManager::add_file(IncrementConfig_File_t, 1);
    if (status!=0){
        pc.printf("FLAGS Config failed: %i\r\n", status);
        return status;   
    }
    IncrementConfig i_conf;
    i_conf.parameters.increment=0;
    status= DataManager::append_file_entry(IncrementConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=0){
        pc.printf("IncrementConfig failed to overwrite: %i\r\n", status);
        return status; 
    }

return 0;
}
/**How the user will erase the value?! daily, after sending?  */
int NodeFlow::read_increment(){
    IncrementConfig i_conf;
    status = DataManager::read_file_entry(IncrementConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    int increment=i_conf.parameters.increment;
    pc.printf("Increment value: %d\r\n",increment);
    return increment;
}
int NodeFlow::increment(int i){
    IncrementConfig i_conf;
    int incrementt=read_increment();
    i_conf.parameters.increment=i+incrementt;
    status= DataManager::overwrite_file_entries(IncrementConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=0){
        pc.printf("IncrementConfig failed to overwrite: %i\r\n", status);   
    }
    status = DataManager::read_file_entry(IncrementConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    int increment=i_conf.parameters.increment;
    
    return increment;
}

int NodeFlow::sensor_config_init(int length){

    DataManager_FileSystem::File_t SensorConfig_File_t;
    SensorConfig_File_t.parameters.filename = SensorConfig_n;
    SensorConfig_File_t.parameters.length_bytes = sizeof(SensorConfig::parameters);
    status=DataManager::add_file(SensorConfig_File_t, length);
        if (status!=0){
            pc.printf("Add file failed: %i\r\n", status);
            return status; 
            }
    
    DataManager_FileSystem::File_t TempSensorConfig_File_t;
    TempSensorConfig_File_t.parameters.filename = TempSensorConfig_n;
    TempSensorConfig_File_t.parameters.length_bytes = sizeof(TempSensorConfig::parameters);
    status = DataManager::add_file(TempSensorConfig_File_t, length);
        if(status!=0){
             pc.printf("Add file failed: %i\r\n", status);
             return status;
             }
    return 0;   
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
 *  @return Status
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

int NodeFlow::set_time_config(int time_comparator){
    TimeConfig t_conf;
    t_conf.parameters.time_comparator=time_comparator;
    status= DataManager::overwrite_file_entries(TimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Time Config failed to overwrite: %i\r\n", status);
    }
    
    return status;
}
/** Scheduler Config overwrite in case of a received_message, should be less than the MAX_BUFFER_READING_TIMES
 */
 
int NodeFlow::overwrite_sched_config(int time_comparator,int i){
    SchedulerConfig t_conf;
    
    if (i==0) {
        t_conf.parameters.time_comparator=time_comparator;
        status= DataManager::overwrite_file_entries(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if (status!=0){
            pc.printf("Scheduler Config failed to overwrite: %i\r\n", status);
            }
         }
    else if (i==1) {
         t_conf.parameters.time_comparator=time_comparator;
         status= DataManager::append_file_entry(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if (status!=0){
            pc.printf("Scheduler Config failed to append: %i\r\n", status);
            }
    }
    else {
        t_conf.parameters.time_comparator=time_comparator;
        status= DataManager::append_file_entry(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if (status!=0){
            pc.printf("Scheduler Config failed to append: %i\r\n", status);
            }
    }
    return status;
}
//change so the user will choose
int NodeFlow::overwrite_clock_synch_config(int time_comparator,bool clockSynchOn){
    ClockSynchConfig c_conf;
    c_conf.parameters.clockSynchOn=clockSynchOn;
    c_conf.parameters.time_comparator=time_comparator;
    status= DataManager::overwrite_file_entries(ClockSynchConfig_n, c_conf.data, sizeof(c_conf.parameters));
    if (status!=0){
        pc.printf("Scheduler Config failed to overwrite: %i\r\n", status);
        } 
    return status;
}

uint16_t NodeFlow::read_clock_synch_config(bool &clockSynchOn){
       ClockSynchConfig c_conf;
       status = DataManager::read_file_entry(ClockSynchConfig_n, 0, c_conf.data, sizeof(c_conf.parameters));
       uint16_t time_remainder=c_conf.parameters.time_comparator;
       clockSynchOn=c_conf.parameters.clockSynchOn;
       return time_remainder;

 }

 uint16_t NodeFlow::read_sched_config(int i){
       SchedulerConfig r_conf;
       status = DataManager::read_file_entry(SchedulerConfig_n, i, r_conf.data, sizeof(r_conf.parameters));
      
       uint16_t time_remainder=r_conf.parameters.time_comparator;
       return time_remainder;

 }
/**Sets the flags, for just kicking the watchdog, sensing time,clock synch time, or sending time(NOT YET) */
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
/**Ignore this for now */
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
    sensor_config_init(number_of_sensors);
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

/**Returns seconds until next reading. Sets the flags for the next event in case of timer wakeup
 */
int NodeFlow::set_scheduler(){
    pc.printf("\r\n-----------------NEXT READING TIME----------------\r\n");

    bool schedulerOn=read_sched_config(0);
    if (schedulerOn==true){
        if (flags==FLAG_CLOCK_SYNCH){
          //  pc.printf("%d Flag synch\r",length);
            get_timestamp();
        }
    uint32_t time_remainder=this->time_now();
    //Read the schedule time config, find the next time
    uint32_t timediff_temp=86400;
    int32_t timediff=6600;
    uint32_t next_sch_time=0;
    uint16_t length=read_sched_config(1);
    pc.printf("%d Scheduled times\r",length);
    for (int i=0; i<length; i++){
        uint32_t scheduled_times=read_sched_config(i+2)*2;
        //Make sure that the time does not exceeds 24 hours(user mistake)
        scheduled_times=scheduled_times%86400;

        timediff=scheduled_times-time_remainder;
        if (timediff<0){
            timediff=timediff+86400;
        }
        if (timediff<timediff_temp){
            timediff_temp=timediff; //holds the smallest different form time_now
            next_sch_time=scheduled_times;
            set_flags_config(false, true, false);    
        }  
    }
    pc.printf("\nNext sensing time,");
    timetodate(next_sch_time);
    //the clock synch should be send in 2 bytes, so half the value)
    bool clockSynchOn=0;
    uint16_t cs_time=(2*read_clock_synch_config(clockSynchOn))-time_remainder;
    if(clockSynchOn==true){
        if (cs_time<0){
                cs_time=cs_time+86400;
            }
    }
    
    if(cs_time<timediff_temp){
        timediff_temp=cs_time;
        set_flags_config(false, false, true);
    }
    //Check that its not more than 2 hours
    if (timediff_temp>7200)  {
        timediff_temp=6600;
        //set_flags_config(bool kick_wdg, bool sense_time, bool clock_synch)
        set_flags_config(true, false, false);
    }
    
    return timediff_temp; 


   }
else{
    return NEXT_TIME;
}
    
}



int NodeFlow::set_reading_time(uint16_t arr[], size_t n){
 
    pc.printf("\r\n-----------------NEXT READING TIME----------------\r\n");
    if (wkp==WAKEUP_RESET){
        time_config_init();
    }
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
    //have to change that for EARHART_V1_0_0 or TARGET NAME
    if (getPlatform()==DEVELOPMENT_BOARD_V1_1_0){
        joinTTN();
        int64_t timestamp=0;
        uint8_t dummy[1]={1};
        uint8_t port=0;
        pc.printf("Horrayy,setting the time, bear with me\r\nRetries are set to %d\r\n",MAX_RETRY_CLOCK_SYNCH);
        ThisThread::sleep_for(1000);
        uint8_t retcode=LorawanTP::send_message(223, dummy, sizeof(dummy));
        if(retcode<=0){
            pc.printf("Failed to send\r\n");
            }
        LorawanTP::receive_message(false);
        ThisThread::sleep_for(1000);
        for(int i=0; ((port!=CLOCK_SYNCH_PORT) && (i<MAX_RETRY_CLOCK_SYNCH));i++) {
        pc.printf("%i. Waiting for a server message dude \r\n",i);
        ThisThread::sleep_for(5000);
        retcode=LorawanTP::send_message(223, dummy, sizeof(dummy));
        
        if(retcode<0){
            pc.printf("Failed to send\r\n");
            }
        timestamp=LorawanTP::receive_message(false).received_value[0];
        port=LorawanTP::receive_message(true).port;

        }
    }


    return 0;   
}


uint32_t NodeFlow::time_now() {
   
    time_t time_now=time(NULL);
    uint32_t timestamp=time_now; 
    uint32_t remainder_time=(timestamp%86400);

    timetodate(remainder_time);
    return remainder_time;
}

uint8_t NodeFlow::timetodate(uint32_t remainder_time){
    double_t t_value=(remainder_time/3600.000000);
    double_t minutes_f=fmod(t_value,1);
    uint8_t hours= t_value- minutes_f ;
    double_t minutes=minutes_f*60; 
    double_t seconds=(fmod(minutes,1))*60;
    pc.printf("Time(HH:MM:SS):   %02d:%02d:%02d\r\n", hours, int(minutes),int(seconds));
    return 0;
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
    
    // if (wkp==WAKEUP_TIMER && flags==FLAG_SENSING){
        joinTTN();
        pc.printf("---------------------SENDING----------------------\r\n");
        time_now();

        retcode=LorawanTP::send_message(port, payload, length);
        if (retcode<0){
            pc.printf("\r\nError Sending %d bytes", retcode);
        }
        else{
        pc.printf("\r\nSuccesfully sending %d bytes", retcode);  
         }
        
        pc.printf("\r\n--------------------------------------------------\r\n");
    //}
    return retcode;
}

uint64_t NodeFlow::receiveTTN(){
    uint16_t decValue=0;
    if (wkp==WAKEUP_TIMER && flags==FLAG_SENSING){
        decValue=LorawanTP::receive_message(false).received_value[0]; //checking for a 
        uint8_t port=LorawanTP::receive_message(true).port;
        if ( port==SCHEDULER_PORT){
            uint8_t retcode=LorawanTP::receive_message(true).retcode;
            overwrite_sched_config(true,0);
            overwrite_sched_config(retcode/2,1);

            for (int i=0; i<retcode/2; i++){
                decValue=LorawanTP::receive_message(true).received_value[i];
                pc.printf("%i.RX scheduler: %d(10)\r\n",i, decValue);
                overwrite_sched_config(decValue,i+2);
                }
            }
        
        if(port==CLOCK_SYNCH_PORT){
            bool clockSynchOn=false;
            uint8_t retcode=LorawanTP::receive_message(true).retcode;
            if(retcode==1){
                overwrite_clock_synch_config(decValue,false);
            }
           if(retcode==2){
               overwrite_clock_synch_config(decValue,true);
           }
        }
        if(port==0){
            pc.printf("No Rx available\r\n");
            
        }
        else{
            pc.printf("Rx: %d(10)\r\n", decValue);
            pc.printf("Port: %d\r\n", port);
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
     if (f_conf.parameters.clock_synch==true) {
         return FLAG_CLOCK_SYNCH;
     }
    if (f_conf.parameters.kick_wdg==true) {
         return FLAG_WDG;
     }
     if (f_conf.parameters.sensing_time==true) {
         return FLAG_SENSING;
     }
     return FLAG_SENDING;

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
   if (seconds<2){
       seconds=2;
   } 
   if (seconds>6600){
       seconds=6600;
       set_flags_config(true, false, false);
   }
   int retcode=LorawanTP::sleep();
   if(retcode!=LORAWAN_STATUS_OK){
       pc.printf("\r\nLora not on sleep?!\r\n");     
    }
   //Without this delay it breaks..?!
   ThisThread::sleep_for(1);
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








