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
Serial pc(TP_PC_TXU, TP_PC_RXU);

#if BOARD == EARHART_V1_0_0
LorawanTP lpwan; 
#endif /* #if BOARD == EARHART_V1_0_0 */

/**Initialise the wakeup flag as UNKNOWN
 */
int flags=NodeFlow::FLAG_UNKNOWN;
int status = 1;

/** Constructor. Create a NodeFlow interface, connected to the pins specified 
 *  operating at the specified frequency
 * 
 * @param write_control GPIO to enable or disable write functionality
 * @param sda I2C data line pin
 * @param scl I2C clock line pin
 * @param frequency_hz The bus frequency in hertz. */
#if BOARD == EARHART_V1_0_0
NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz, PinName done): 
DataManager(write_control, sda, scl, frequency_hz), watchdog(done) 
{

}
#endif /* #if BOARD == EARHART_V1_0_0 */

#if BOARD == WRIGHT_V1_0_0
NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz,
                   PinName txu, PinName rxu, PinName cts, PinName rst, 
                   PinName vint, PinName gpio, int baud, PinName done) :
                   DataManager(write_control, sda, scl, frequency_hz), _radio(txu, rxu, cts, rst, vint, gpio, baud), watchdog(done)
{

}
#endif /* #if BOARD == WRIGHT_V1_0_0 */

/** Destructor. 
 */
 NodeFlow::~NodeFlow() {
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
         bool  pin_wakeup;
         bool  flag;
       
    } parameters;

    char data[sizeof(FlagsConfig::parameters)];
};


union NextTimeConfig
{
    struct 
    {
        uint16_t time_comparator;
        
    } parameters;

    char data[sizeof(NextTimeConfig::parameters)];
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
    FlagsConfig_n           = 4, //wdg,clock synch, sensing,sending
    IncrementConfig_n       = 5,
    SensorConfig_n          = 6, //not using for now
    TimeConfig_n            = 7,
    TempSensorConfig_n      = 8,
    TempConfig_n            = 9,
    NextTimeConfig_n        = 10
 };

/** Attempt to connect to NB-IoT network with default parameters
 *  described in tp_nbiot_interface.h. The function blocks and will
 *  time out after 5 minutes at which point the NB-IoT modem will 
 *  regress to minimum functionality in order to conserve power whilst
 *  the application decides what to do
 */
#if BOARD == WRIGHT_V1_0_0
int NodeFlow::initialise_nbiot()
{
    int status = _radio.start();
    if(status != NodeFlow::NODEFLOW_OK)
    {
        return status;
    }

    return NodeFlow::NODEFLOW_OK;
}
#endif /* #if BOARD == WRIGHT_V1_0_0 */

/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 */
void NodeFlow::start(){
    uint16_t next_time=0;
    watchdog.kick();

    TP_Sleep_Manager::WakeupType_t wkp = sleep_manager.get_wakeup_type();

    if (wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_PIN) {
        pc.printf("\r\n--------------------PIN WAKEUP--------------------\r\n");
        //timetodate(time_now());
        HandleInterrupt();
        next_time=get_interrupt_latency();
        if (next_time>INTERRUPT_DELAY){
            set_wakeup_pin_flag(true);
            enter_standby(INTERRUPT_DELAY,false);
        }
         else{
            enter_standby(next_time,false);
        }
    }
    else if (wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_TIMER) {
        if(delay_pin_wakeup()==NodeFlow::FLAG_WAKEUP_PIN){
            set_wakeup_pin_flag(false);
            next_time=get_interrupt_latency();
        }
        else{
         pc.printf("\r\n-------------------TIMER WAKEUP-------------------\r\n");
            timetodate(time_now());
            HandleModem();
            flags=get_flags();
            next_time=set_scheduler();
        }
         
    }
    else if (wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_RESET) {
        pc.printf("\r\n                      __|__       \n               --@--@--(_)--@--@--\n-------------------THING PILOT--------------------\r\n");
        pc.printf("\nDevice Unique ID: %08X %08X %08X \r", STM32_UID[0], STM32_UID[1], STM32_UID[2]);
        initialise();
        setup();
        get_timestamp();
        timetodate(time_now());
    
        overwrite_sched_config(SCHEDULER,SCHEDULER_SIZE);
        
        if (read_sched_config(0)==true){
            for (int i=0; i<SCHEDULER_SIZE; i++){
                uint16_t time_remainder=DIVIDE(((int(scheduler[i]))*HOURINSEC)+((fmod(scheduler[i],1))*6000)); 
                timetodate(time_remainder*2);
                append_sched_config(time_remainder);
            }
        }
        else{
            if (SCHEDULER_SIZE==1){
                append_sched_config(periodic[0]);
            }
            //Each sensor has a different time
            else{
                time_config_init();
                add_sensors();
                for (int i=0; i<SCHEDULER_SIZE; i++){ 
                    append_sched_config(periodic[i]); }
            }
        }
        flags=get_flags();
        next_time=set_scheduler();      
    }
    pc.printf("\nGoing to sleep for %d",next_time);
    enter_standby(next_time,true);  
}

int NodeFlow::HandleModem(){
    uint16_t length=0;
    uint8_t *payload=0;
    payload=HandlePeriodic(length);
#if BOARD == EARHART_V1_0_0
        sendTTN(1, payload, length);            
        receiveTTN();
#endif /* #if BOARD == EARHART_V1_0_0 */

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
    
    w_conf.parameters.modulation =1;
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
    overwrite_clock_synch_config(DIVIDE(CLOCK_SYNCH_TIME),CLOCK_SYNCH);
    
/**FlagsConfig*/
    DataManager_FileSystem::File_t FlagsConfig_File_t;
    FlagsConfig_File_t.parameters.filename = FlagsConfig_n;
    FlagsConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);
    status=DataManager::add_file(FlagsConfig_File_t, 1);
    if (status!=0){
        pc.printf("FLAGS Config failed: %i\r\n", status);
        return status;   
    }
    status=set_flags_config(false, false, false);  //sensing true
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
    //pc.printf("Increment value: %d\r\n",increment);
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
 
int NodeFlow::overwrite_sched_config(uint16_t code,uint16_t length){
    SchedulerConfig t_conf;
    //code,length,values
    t_conf.parameters.time_comparator=code;
    status= DataManager::overwrite_file_entries(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Scheduler Config failed to overwrite: %i\r\n", status);
        }
    t_conf.parameters.time_comparator=length;
    status= DataManager::append_file_entry(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Scheduler Config failed to append: %i\r\n", status);
        }
    return status;
}
int NodeFlow::append_sched_config(uint16_t time_comparator){
    SchedulerConfig t_conf;
    t_conf.parameters.time_comparator=time_comparator;
    status= DataManager::append_file_entry(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Scheduler Config failed to append: %i\r\n", status);
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
    f_conf.parameters.pin_wakeup=0;
    status= DataManager::overwrite_file_entries(FlagsConfig_n, f_conf.data, sizeof(f_conf.parameters));
    if (status!=0){
        pc.printf("Flags Config failed to overwrite: %i\r\n", status);  
    }
    return status;
}


int NodeFlow::set_wakeup_pin_flag(bool wakeup_pin){
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    uint8_t kick_wdg=f_conf.parameters.kick_wdg;
    uint8_t sensing_flag=f_conf.parameters.sensing_time;
    uint8_t cs_flag=f_conf.parameters.clock_synch;
    uint8_t send_flag=f_conf.parameters.sending_time;
    f_conf.parameters.kick_wdg=kick_wdg;
    f_conf.parameters.sensing_time=sensing_flag;
    f_conf.parameters.clock_synch=cs_flag;
    f_conf.parameters.sending_time=send_flag;
    f_conf.parameters.pin_wakeup=wakeup_pin;
    
    status= DataManager::overwrite_file_entries(FlagsConfig_n, f_conf.data, sizeof(f_conf.parameters));
    if (status!=0){
        pc.printf(" wakeup Flags Config failed to overwrite: %i\r\n", status);  
    }
    return status;
}

/**Ignore this for now */
int NodeFlow::add_sensors() { //uint8_t device_id[],uint16_t reading_time[],size_t number_of_sensors
    pc.printf("\r\n-------------------ADD SENSORS--------------------\r\n");
    /** Files Initialisation
     */
    sensor_config_init(SCHEDULER_SIZE);
    for (int i=0; i<SCHEDULER_SIZE; i++){
        SensorConfig s_conf;
        s_conf.parameters.device_id = i;
        s_conf.parameters.time_comparator=periodic[i];
        
        status=DataManager::append_file_entry(SensorConfig_n, s_conf.data, sizeof(s_conf.parameters));
        if(status!=0){
            pc.printf("Add file failed: %i\r\n", status);
        return status;}

        //temporary reading times
        TempSensorConfig ts_conf;
        ts_conf.parameters.device_id = i;
        ts_conf.parameters.time_comparator=periodic[i];
    
        status = DataManager::append_file_entry(TempSensorConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
        if(status!=0){
            pc.printf("Error append_file_entries No: %i status: %i\r\n", i, status);
        return status;}
            
        status = DataManager::read_file_entry(TempSensorConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if(status!=0){
            pc.printf("Read file failed: %i\r\n", status);
            return status;}
        pc.printf("%d. Sensing group id: %i, wake up every: %u Seconds\r\n",i, ts_conf.parameters.device_id,ts_conf.parameters.time_comparator);
                
        }
     
    pc.printf("--------------------------------------------------\r\n");
   
    return status;
  

    return 0; 
}

/**Returns seconds until next reading. Sets the flags for the next event in case of timer wakeup
 */
int NodeFlow::set_scheduler(){

    pc.printf("\r\n-----------------NEXT READING TIME----------------\r\n");

    bool schedulerOn=read_sched_config(0);
    uint16_t length=read_sched_config(1);
    uint32_t timediff_temp=DAYINSEC;
    if (schedulerOn==true){
        if (flags==NodeFlow::FLAG_CLOCK_SYNCH){
            get_timestamp();
        }
    uint32_t time_remainder=this->time_now();
    //Read the schedule time config, find the next time
    int32_t timediff=6600;
    uint32_t next_sch_time=0;
    
    pc.printf("%d Scheduled times\r",length);
    for (int i=0; i<length; i++){
        uint32_t scheduled_times=read_sched_config(i+2)*2;
        //Make sure that the time does not exceeds 24 hours(user mistake)
        scheduled_times=scheduled_times%DAYINSEC;

        timediff=scheduled_times-time_remainder;
        if (timediff<0){
            timediff=timediff+DAYINSEC;
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
    if(clockSynchOn){
        if (cs_time<0){
                cs_time=cs_time+DAYINSEC;
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
   }
else if(schedulerOn==false){
    timediff_temp=set_reading_time();  
} 



ovewrite_wakeup_timestamp(timediff_temp);

return timediff_temp;   
}

int NodeFlow::set_reading_time(){ 
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
        status = DataManager::add_file(TempConfig_File_t, SCHEDULER_SIZE);
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
    
    for (int i=0; i<SCHEDULER_SIZE; i++){

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
     
    for (int i=0; i<SCHEDULER_SIZE; i++){

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
    for (int i=0; i<SCHEDULER_SIZE; i++){
    status=DataManager::read_file_entry(TempConfig_n, i, tm_conf.data, sizeof(tm_conf.parameters));
    if (status!=0){
            pc.printf("Error read temp config. status: %i\r\n", status);
            return status;
             }
    pc.printf("%d. Sensing group id: %i, next reading: %u seconds\r\n",i, tm_conf.parameters.device_id,tm_conf.parameters.time_comparator);
   
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

    pc.printf("\r\nNext Reading time %d ,No of sensors: %d\r\n",time_comparator, SCHEDULER_SIZE);
    pc.printf("--------------------------------------------------\r\n");

    return time_comparator;
}

uint16_t NodeFlow::get_interrupt_latency(){
    NextTimeConfig t_conf;

    status=DataManager::read_file_entry(NextTimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf("Error read_file_entry TimeConfig. status: %i\r\n", status);
        return 2;
    }

    uint16_t next_sch_time=t_conf.parameters.time_comparator;
    next_sch_time=next_sch_time-time_now();
    return next_sch_time;
}

int NodeFlow::ovewrite_wakeup_timestamp(uint16_t time_remainder){
    uint16_t next_sch_time=time_now()+time_remainder;

    DataManager_FileSystem::File_t NextTimeConfig_File_t;
    NextTimeConfig_File_t.parameters.filename =  NextTimeConfig_n;
    NextTimeConfig_File_t.parameters.length_bytes = sizeof( NextTimeConfig::parameters);
    status=DataManager::add_file(NextTimeConfig_File_t, 1);

    if (status!=0){
        pc.printf("NextTime Config failed: %i\r\n", status);  
    }

     NextTimeConfig t_conf;
    t_conf.parameters.time_comparator=next_sch_time;
    status= DataManager::overwrite_file_entries( NextTimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status!=0){
        pc.printf(" NextTime Config failed to overwrite: %i\r\n", status);
    }

    return 0;
}
/** Timestamp. Send ClockSync message, wait for a response from ttn if it fails don't change the time
    * @param num_timestamp_retries  The number of retries to get the Timestamp
    * 

 */
uint8_t NodeFlow::get_timestamp(){
   #if BOARD == EARHART_V1_0_0
        joinTTN();
        int64_t timestamp=0;
        uint8_t dummy[1]={1};
        uint8_t port=0;
        pc.printf("Horrayy,setting the time, bear with me\r\nRetries are set to %d\r\n",MAX_RETRY_CLOCK_SYNCH);
        uint8_t retcode=lpwan.send_message(223, dummy, sizeof(dummy));
        if(retcode<=0){
            pc.printf("Failed to send\r\n");
            }
        lpwan.receive_message(false);
        ThisThread::sleep_for(1000);
        for(int i=0; ((port!=CLOCK_SYNCH_PORT) && (i<MAX_RETRY_CLOCK_SYNCH));i++) {
        pc.printf("%i. Waiting for a server message dude \r\n",i);
        ThisThread::sleep_for(5000);
        retcode=lpwan.send_message(223, dummy, sizeof(dummy));
        if(retcode<0){
            pc.printf("Failed to send\r\n");
            }
        timestamp=lpwan.receive_message(false).received_value[0];
        port=lpwan.receive_message(true).port;

        }
    
    #endif /* BOARD == EARHART_V1_0_0 */

    return 0;   
}

uint32_t NodeFlow::time_now() {
    time_t time_now=time(NULL);
    uint32_t timestamp=time_now; 
    uint32_t remainder_time=(timestamp%DAYINSEC);
   // timetodate(remainder_time);
    return remainder_time;
}

uint8_t NodeFlow::timetodate(uint32_t remainder_time){
    double_t t_value=(remainder_time/float(HOURINSEC));
    double_t minutes_f=fmod(t_value,1);
    uint8_t hours= t_value- minutes_f;
    double_t minutes=minutes_f*MINUTEINSEC; 
    double_t seconds=(fmod(minutes,1))*MINUTEINSEC;
    pc.printf("Time(HH:MM:SS):   %02d:%02d:%02d\r\n", hours, int(minutes),int(seconds));
    return 0;
}

/**LorawanTP
 */
#if BOARD == EARHART_V1_0_0
int NodeFlow::joinTTN(){
    
    int retcode=lpwan.join();
        if(retcode<0){
            pc.printf("Failed to join\r\n");}
    
    return retcode;
}
int NodeFlow::sendTTN(uint8_t port, uint8_t payload[], uint16_t length){
    int retcode=0;
    
    // if (wkp==WAKEUP_TIMER && flags==NodeFlow::FLAG_SENSING){
        joinTTN();
        pc.printf("---------------------SENDING----------------------\r\n");
        timetodate(time_now());

        retcode=lpwan.send_message(port, payload, length);
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
    decValue=lpwan.receive_message(false).received_value[0]; //checking for a 
    uint8_t port=lpwan.receive_message(true).port;
    if ( port==SCHEDULER_PORT){
        uint8_t retcode=lpwan.receive_message(true).retcode;
        overwrite_sched_config(true, DIVIDE(retcode));
        for (int i=0; i<DIVIDE(retcode); i++){
            decValue=lpwan.receive_message(true).received_value[i];
            pc.printf("%i.RX scheduler: %d(10)\r\n",i, decValue);
            append_sched_config(decValue);
            }
        }
    
    if(port==CLOCK_SYNCH_PORT){
        bool clockSynchOn=false;
        uint8_t retcode=lpwan.receive_message(true).retcode;
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
    
    return decValue;
}
#endif /* #if BOARD == EARHART_V1_0_0 */


int NodeFlow::get_flags(){    
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if (status!=0){
        pc.printf("FlagsConfig. status: %i\r\n", status);
        return status;
    }
     if (f_conf.parameters.clock_synch) {
         return NodeFlow::FLAG_CLOCK_SYNCH;
     }
    if (f_conf.parameters.kick_wdg) {
         return NodeFlow::FLAG_WDG;
     }
     if (f_conf.parameters.sensing_time) {
         return NodeFlow::FLAG_SENSING;
     }
     return NodeFlow::FLAG_SENDING;

}

int NodeFlow::delay_pin_wakeup(){    
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if (status!=0){
        pc.printf("FlagsConfig. status: %i\r\n", status);
        return status;
    }
     if (f_conf.parameters.pin_wakeup) {
         return NodeFlow::FLAG_WAKEUP_PIN;
     }
     return 0;
}

void NodeFlow::enter_standby(int seconds, bool wkup_one) 
{ 
    if(seconds < 2)
    {
        seconds = 2;
    } 
    else if(seconds > 6600)
    {
        seconds = 6600;
        set_flags_config(true, false, false);
    }

    #if BOARD == EARHART_V1_0_0
        int retcode=lpwan.sleep();
    #endif /* BOARD == EARHART_V1_0_0 */

    //Without this delay it breaks..?!
    ThisThread::sleep_for(2);

    sleep_manager.standby(seconds, wkup_one);
}




