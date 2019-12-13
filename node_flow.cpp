/**
 ******************************************************************************
 * @file    NodeFlow.cpp
 * @version 0.3.0
 * @author  Rafaella Neofytou, Adam Mitchell
 * @brief   C++ file of the Wright || Earheart node from Think Pilot. 
 ******************************************************************************
 **/

/** Includes
 */
#include "node_flow.h"


/**Initialise the wakeup flag as UNKNOWN
 */
int flags=NodeFlow::FLAG_UNKNOWN;
int status = -1;
int written_entries=0;
#if(SCHEDULER)
    float scheduler[1];
#endif


/** Constructor. Create a NodeFlow interface, connected to the pins specified 
 *  operating at the specified frequency
 * 
 * @param write_control GPIO to enable or disable write functionality
 * @param sda I2C data line pin
 * @param scl I2C clock line pin
 * @param frequency_hz The bus frequency in hertz. */
#if BOARD == EARHART_V1_0_0
NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz,PinName mosi,PinName miso,PinName sclk,PinName nss,
                   PinName reset,PinName dio0,PinName dio1,PinName dio2,PinName dio3,PinName dio4,PinName dio5,PinName rf_switch_ctl1,
                   PinName rf_switch_ctl2,PinName txctl,PinName rxctl,PinName ant_switch,PinName pwr_amp_ctl,PinName tcxo, PinName done): 
                   DataManager(write_control, sda, scl, frequency_hz), _radio(mosi, miso, sclk, nss, reset, dio0, dio1, 
                   dio2,dio3,dio4,dio5,rf_switch_ctl1,rf_switch_ctl2,txctl,rxctl,ant_switch,pwr_amp_ctl,tcxo),watchdog(done)
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
 NodeFlow::~NodeFlow() 
 {
 }

#if BOARD == WRIGHT_V1_0_0
int NodeFlow::initialise_nbiot()
{
    if(_comms_stack == Comms_Radio_Stack::NBIOT)
    {
        status=_radio.ready(); 
        if(status == NodeFlow::NODEFLOW_OK)
        {
            status = _radio.start();
            if(status != NodeFlow::NODEFLOW_OK)
            {
                return status;
            }  
        }
    }
    return status;
}
#endif
/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 */
void NodeFlow::start()
{
    uint32_t next_time=0;
    watchdog.kick();
    TP_Sleep_Manager::WakeupType_t wkp = sleep_manager.get_wakeup_type();

    if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_PIN)
    {   
        pc.printf("\r\n--------------------PIN WAKEUP--------------------\r\n");
        HandleInterrupt(); /**Pure virtual function */
        status=get_interrupt_latency(&next_time);
        if (status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"get_interrupt_latency", status,__PRETTY_FUNCTION__);
        }

        if(next_time>INTERRUPT_DELAY)
        {
            next_time=INTERRUPT_DELAY;
            status=set_wakeup_pin_flag(true);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"set_wakeup_pin_flag", status,__PRETTY_FUNCTION__);
            }
        }
        
        enter_standby(next_time,false);
        
    }
    else if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_TIMER) 
    {
        flags=get_flags();
        if(delay_pin_wakeup()==NodeFlow::FLAG_WAKEUP_PIN)
        {
            status=set_wakeup_pin_flag(false);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"set_wakeup_pin_flag", status,__PRETTY_FUNCTION__);
            }

            status=get_interrupt_latency(&next_time);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"get_interrupt_latency", status,__PRETTY_FUNCTION__);
            }
        }
        else
        {
            pc.printf("\r\n-------------------TIMER WAKEUP-------------------\r\n");
            timetodate(time_now());
            HandleModem();
            status=set_scheduler(&next_time);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"set_scheduler", status,__PRETTY_FUNCTION__);
            }
           
        }
         
    }
    else if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_RESET || wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_SOFTWARE) 
    {
        pc.printf("\r\n                      __|__       \n               --@--@--(_)--@--@--\n-------------------THING PILOT--------------------\r\n");
        pc.printf("\nDevice Unique ID: %08X %08X %08X \r", STM32_UID[0], STM32_UID[1], STM32_UID[2]);
        status=initialise();
        if (initialise() != NODEFLOW_OK)
        { 
            NVIC_SystemReset(); 
        }
        #if BOARD == WRIGHT_V1_0_0
            initialise_nbiot();
        #endif
         
        setup(); /**Pure virtual by the user */
       
        if(CLOCK_SYNCH)
        {
            get_timestamp();
           
        }
        timetodate(time_now()); 
        status=init_sched_config();
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"init_sched_config", status,__PRETTY_FUNCTION__);
        }

        #if(SEND_SCHEDULER)
            status=init_send_sched_config();
            
            if (status!=NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"init_send_sched_config", status,__PRETTY_FUNCTION__);
            }   
        #endif
        flags=get_flags();
        status=set_scheduler(&next_time); 
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"set_scheduler", status,__PRETTY_FUNCTION__);
        }  
         
    }
    pc.printf("\nGoing to sleep for %d ",next_time);
    timetodate(time_now());
    enter_standby(next_time,true);
}

/** Initialise the EEPROM
 * @return Status
 */
int NodeFlow::initialise()
{    
    status=DataManager::init_filesystem();
    if(status != NODEFLOW_OK)
    {
       ErrorHandler(__LINE__,"init_filesystem", status,__PRETTY_FUNCTION__);
       return DATA_MANAGER_FAIL;
    }

    bool initialised = false;
    status=DataManager::is_initialised(initialised);
    if(status != NODEFLOW_OK)
    {
       ErrorHandler(__LINE__,"is_initialised", status,__PRETTY_FUNCTION__);
       return DATA_MANAGER_FAIL;
    }

    DataManager_FileSystem::File_t ErrorConfig_File_t;
    ErrorConfig_File_t.parameters.filename = ErrorConfig_n;
    ErrorConfig_File_t.parameters.length_bytes = sizeof(ErrorConfig::parameters);

    status=DataManager::add_file(ErrorConfig_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    ErrorConfig e_conf;
    e_conf.parameters.errCnt=0;
    status= DataManager::overwrite_file_entries(ErrorConfig_n, e_conf.data, sizeof(e_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"ERROR IN ERROR OH SHIT",status,__PRETTY_FUNCTION__); 
        return status;    
    }

    /** SchedulerConfig */
    DataManager_FileSystem::File_t SchedulerConfig_File_t;
    SchedulerConfig_File_t.parameters.filename = SchedulerConfig_n;  
    SchedulerConfig_File_t.parameters.length_bytes = sizeof( SchedulerConfig::parameters);

    status=DataManager::add_file(SchedulerConfig_File_t, MAX_BUFFER_READING_TIMES+2); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    SchedulerConfig s_conf;
    s_conf.parameters.time_comparator=0;
    status= DataManager::overwrite_file_entries(SchedulerConfig_n, s_conf.data, sizeof(s_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__); 
        return status;    
    }

    /** SendSchedulerConfig 
     */
    DataManager_FileSystem::File_t SendSchedulerConfig_File_t;
    SendSchedulerConfig_File_t.parameters.filename = SendSchedulerConfig_n;  
    SendSchedulerConfig_File_t.parameters.length_bytes = sizeof( SendSchedulerConfig::parameters);
    #if(SEND_SCHEDULER)
        status=DataManager::add_file(SendSchedulerConfig_File_t, MAX_BUFFER_SENDING_TIMES+2); 
    #endif
    #if(!SEND_SCHEDULER)
        status=DataManager::add_file(SendSchedulerConfig_File_t, 2);
    #endif
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SendSchedulerConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    SendSchedulerConfig ss_conf;
    ss_conf.parameters.time_comparator=0;
    status= DataManager::overwrite_file_entries(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SendSchedulerConfig",status,__PRETTY_FUNCTION__);  
        return status;    
    }
    
    /** ClockSynchConfig 
     */
    DataManager_FileSystem::File_t ClockSynchConfig_File_t;
    ClockSynchConfig_File_t.parameters.filename = ClockSynchConfig_n;
    ClockSynchConfig_File_t.parameters.length_bytes = sizeof( ClockSynchConfig::parameters);

    status=DataManager::add_file(ClockSynchConfig_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"ClockSynchConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }
    #if (CLOCK_SYNCH)
        status=overwrite_clock_synch_config(DIVIDE(CLOCK_SYNCH_TIME),CLOCK_SYNCH);
    #endif
    #if (!CLOCK_SYNCH)
        status=overwrite_clock_synch_config(0,CLOCK_SYNCH);
    #endif
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"Overwrite_clock_synch_config",status,__PRETTY_FUNCTION__);
        return status;
    }   
   
    /** FlagsConfig
     */
    DataManager_FileSystem::File_t FlagsConfig_File_t;
    FlagsConfig_File_t.parameters.filename = FlagsConfig_n;
    FlagsConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);

    status=DataManager::add_file(FlagsConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagsConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    status=set_flags_config(0);  //TODO: CHECK INIT FLAGS (IF HEART BEAT IS TRUE THEN CHANGE ALL FLAGS TO TRUE?)

    /** IncrementConfig
     */
    DataManager_FileSystem::File_t IncrementConfig_File_t;
    IncrementConfig_File_t.parameters.filename = IncrementConfig_n;
    IncrementConfig_File_t.parameters.length_bytes = sizeof(IncrementConfig::parameters);

    status=DataManager::add_file(IncrementConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    IncrementConfig i_conf;
    i_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementConfig",status,__PRETTY_FUNCTION__);
        return status; 
    }

    DataManager_FileSystem::File_t NextTimeConfig_File_t;
    NextTimeConfig_File_t.parameters.filename = NextTimeConfig_n;
    NextTimeConfig_File_t.parameters.length_bytes = sizeof(NextTimeConfig::parameters);

    status=DataManager::add_file(NextTimeConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"NextTimeConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    DataManager_FileSystem::File_t MetricGroupConfig_File_t;
    MetricGroupConfig_File_t.parameters.filename = MetricGroupConfig_n;
    MetricGroupConfig_File_t.parameters.length_bytes = sizeof(MetricGroupConfig::parameters);

    status = DataManager::add_file(MetricGroupConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    DataManager_FileSystem::File_t SensorDataConfig_File_t;
    SensorDataConfig_File_t.parameters.filename = SensorDataConfig_n;
    SensorDataConfig_File_t.parameters.length_bytes = sizeof(SensorDataConfig::parameters);

    status = DataManager::add_file(SensorDataConfig_File_t, 10*MAX_BUFFER_READING_TIMES); //TODO: what is the maximum size supported?!
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SensorDataConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
       
    return status;
}

int NodeFlow::HandleModem()
{
    uint16_t sched_length;
    status=read_sched_config(1,&sched_length);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }

    if(flags==NodeFlow::FLAG_SENSING ||flags==NodeFlow::FLAG_SENSE_SEND||
        flags==NodeFlow::FLAG_SENSE_SEND_SYNCH||flags==NodeFlow::FLAG_SENSE_SYNCH)
    {
        
        if (sched_length>1)
        {   
            uint8_t mg_flag;
            get_metric_flags(&mg_flag);
            bitset<8> flags(mg_flag);
            pc.printf("MetricGroupA: %d, MetricGroupB: %d, MetricGroupC: %d, MetricGroupC: %d \r\n",flags.test(0),flags.test(1),flags.test(2),flags.test(3));
            
            if(flags.test(0)==1)
            {
                MetricGroupA(); 
            }

            if(flags.test(1)==1)
            {
                MetricGroupB(); 
            }
            if(flags.test(2)==1)
            {   
                MetricGroupC(); 
            }
            if(flags.test(3)==1)
            {
                MetricGroupD(); 
            }
        }
        else
        {
            MetricGroupA(); 
        }   
              
    }

    if(flags==NodeFlow::FLAG_SENDING||flags==NodeFlow::FLAG_SENSE_SEND||
        flags==NodeFlow::FLAG_SEND_SYNCH||flags==NodeFlow::FLAG_SENSE_SEND_SYNCH)
    { 
        status= DataManager::get_total_written_file_entries(SensorDataConfig_n, written_entries);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
            return status;
        }

        int response_code=-1;
        uint8_t payload[written_entries];
        uint8_t nbiot_payload[written_entries+4];
        uint32_t sn=STM32_UID[0];
        *(time_t *)(nbiot_payload)=sn;
      
        pc.printf("Written entries %d\r\n",written_entries);
        pc.printf("Buffer(LSB)  = 0x");
        SensorDataConfig d_conf;
        for (int i=0; i<written_entries; i++) 
        {
            status = DataManager::read_file_entry(SensorDataConfig_n, i, d_conf.data, sizeof(d_conf.parameters));
            pc.printf(" %02x", d_conf.parameters.byte);   
            payload[i]=d_conf.parameters.byte;
        }
        printf("\r\n");
        /**TODO: NBIOT send */
        #if BOARD == WRIGHT_V1_0_0
            memcpy(nbiot_payload+4,payload,written_entries); /*Adds a part of the serial number 4 bytes*/
            pc.printf("\r\nNBIOT Buffer(LSB)  = 0x");
            for (int i=0; i<written_entries+4; i++) 
            {
                pc.printf(" %02x",nbiot_payload[i]);   
            }
            pc.printf("\r\n");
            //TODO: CHECK WITH NBIOT
            char recv_data[512];
            
            status=_radio.coap_post((char*)nbiot_payload, recv_data, SaraN2::TEXT_PLAIN, response_code);
            if (response_code != 0 || response_code != 2 ) //TODO: Check
            {
                 ErrorHandler(__LINE__,"Error Sending NBIOT",status,__PRETTY_FUNCTION__); //TODO: remove that as an error because it will restart
            } 
            else
            {
                _clear_after_send();
            }
        #endif

        #if BOARD == EARHART_V1_0_0
            response_code=sendTTN(1, payload, written_entries);
            if (response_code > NODEFLOW_OK) 
            {
                _clear_after_send();
            } 
        #endif 

         
    }

    return NODEFLOW_OK;
}

template <typename DataType>
void NodeFlow::add_record(DataType data)
{
    uint8_t bytes[sizeof(DataType)];
    *(DataType *)(bytes)=data;

    if(written_entries == 0)
    {
        uint8_t mg_flag;
        get_metric_flags(&mg_flag);
        add_sensing_entry(mg_flag); 

        #if BOARD == WRIGHT_V1_0_0
            time_t time_now=time(NULL);
            uint8_t time_bytes[4];
            *(time_t *)(time_bytes)=time_now;
            for (int i=0; i<4; i++)
            {
                pc.printf("[ 0x%.2x]", time_bytes[i]);
                add_sensing_entry(time_bytes[i]);
            }
        pc.printf("\r\n");
        #endif
        written_entries=5;
    }
    pc.printf("Bytes = ");
    for (int i=0; i<sizeof(DataType); i++)
    {
        pc.printf("[ 0x%.2x]", bytes[i]);
        add_sensing_entry(bytes[i]);
    }
    pc.printf("\r\n");
}

template void NodeFlow::add_record<float>(float data);
template void NodeFlow::add_record<int>(int data);
template void NodeFlow::add_record<int8_t>(int8_t data);
template void NodeFlow::add_record<int16_t>(int16_t data);
template void NodeFlow::add_record<int64_t>(int64_t data);
template void NodeFlow::add_record<uint8_t>(uint8_t data);
template void NodeFlow::add_record<uint16_t>(uint16_t data);
template void NodeFlow::add_record<uint32_t>(uint32_t data);
template void NodeFlow::add_record<uint64_t>(uint64_t data);

int NodeFlow::add_sensing_entry(uint8_t value)
{
    SensorDataConfig t_conf;
    t_conf.parameters.byte=value;
    status= DataManager::append_file_entry(SensorDataConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        status= DataManager::append_file_entry(SensorDataConfig_n, t_conf.data, sizeof(t_conf.parameters));
        ErrorHandler(__LINE__,"SensorDataConfig",status,__PRETTY_FUNCTION__);
    }
    return status;
}



/** How the user will erase the value?! daily, after sending?  
 */
int NodeFlow::read_increment(int *increment_value)
{
    IncrementConfig i_conf;
    status = DataManager::read_file_entry(IncrementConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementConfig",status,__PRETTY_FUNCTION__);   
    }
    *increment_value=i_conf.parameters.increment;

    return status;
}

int NodeFlow::increment(int i)
{
    int increment_value;
    status=read_increment(&increment_value);
    if (status == NODEFLOW_OK)
    {
        IncrementConfig i_conf;
        i_conf.parameters.increment=i+increment_value;
        status= DataManager::overwrite_file_entries(IncrementConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementConfig",status,__PRETTY_FUNCTION__); 
        }
    }
    return status;
}

int NodeFlow::_clear_increment()
{
    IncrementConfig i_conf;
    i_conf.parameters.increment=0;
   
    status= DataManager::overwrite_file_entries(IncrementConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementConfig",status,__PRETTY_FUNCTION__);   
    }
    return status;
}
/**TODO: Check for configuration from the server, those need to initialised with the other at start?! */
int NodeFlow::sensor_config_init(int length)
{
    DataManager_FileSystem::File_t SensingGroupConfig_File_t;
    SensingGroupConfig_File_t.parameters.filename = SensingGroupConfig_n;
    SensingGroupConfig_File_t.parameters.length_bytes = sizeof(SensingGroupConfig::parameters);

    status=DataManager::add_file(SensingGroupConfig_File_t, length);
    if (status != NODEFLOW_OK) 
    {
        ErrorHandler(__LINE__,"SensingGroupConfig",status,__PRETTY_FUNCTION__);
        return status; 
    }
    
    DataManager_FileSystem::File_t TempSensingGroupConfig_File_t;
    TempSensingGroupConfig_File_t.parameters.filename = TempSensingGroupConfig_n;
    TempSensingGroupConfig_File_t.parameters.length_bytes = sizeof(TempSensingGroupConfig::parameters);

    status = DataManager::add_file(TempSensingGroupConfig_File_t, length);
    if(status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    return status;   
}


/** Initialise the time config file
 *
 *  @return Status
 */
int NodeFlow::time_config_init()
{
    DataManager_FileSystem::File_t TimeConfig_File_t;
    TimeConfig_File_t.parameters.filename = TimeConfig_n;
    TimeConfig_File_t.parameters.length_bytes = sizeof(TimeConfig::parameters);
    status = DataManager::add_file(TimeConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TimeConfig",status,__PRETTY_FUNCTION__);  
        return status;
    }
    
    status = set_time_config(0);
    
    return status;
}

int NodeFlow::set_time_config(int time_comparator)
{
    TimeConfig t_conf;
    t_conf.parameters.time_comparator=time_comparator;

    status= DataManager::overwrite_file_entries(TimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TimeConfig",status,__PRETTY_FUNCTION__);
    }
    
    return status;
}



/** Scheduler Config overwrite in case of a received_message, should be less than the MAX_BUFFER_READING_TIMES
 */ 
int NodeFlow::init_sched_config()
{
    #if(!SCHEDULER)
        if(SCHEDULER_SIZE>4)
        {
            pc.printf("\nWARNING!! Scheduler size too big,\nonly 1 interval time is associated with each metric group\n");
            #define SCHEDULER_SIZE 4 
        }
         
    #endif
    status=overwrite_sched_config(SCHEDULER,SCHEDULER_SIZE);
    while (status != NODEFLOW_OK)
    {
        status=overwrite_sched_config(SCHEDULER,SCHEDULER_SIZE);
    }
    uint16_t schedulerOn;
    status=read_sched_config(0,&schedulerOn);
    if(schedulerOn)
    {   
        #if (SCHEDULER_A)
            pc.printf("\r\n---------------ADD SENSING TIMES GA---------------\r\n");
            for(int i=0; i<SCHEDULER_A_SIZE; i++)
            {
                status=timetoseconds(schedulerA[i],1);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"timetoseconds",status,__PRETTY_FUNCTION__);
                    return status;
                }

            }
        #endif     
        #if (SCHEDULER_B)
            pc.printf("\r\n---------------ADD SENSING TIMES GB---------------\r\n");
            for(int i=0; i<SCHEDULER_B_SIZE; i++)
            {
                status=timetoseconds(schedulerB[i],2);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"timetoseconds",status,__PRETTY_FUNCTION__);
                    return status;
                }
            }      
        #endif
        #if (SCHEDULER_C)
            pc.printf("\r\n---------------ADD SENSING TIMES GC---------------\r\n");       
            for(int i=0; i<SCHEDULER_C_SIZE; i++)
            {
                status=timetoseconds(schedulerC[i],4);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"timetoseconds",status,__PRETTY_FUNCTION__);
                    return status;
                }
            }
        #endif

        #if (SCHEDULER_D)
            pc.printf("\r\n--------------ADD SENSING TIMES GD---------------\r\n");
            for(int i=0; i<SCHEDULER_D_SIZE; i++)
            {
                status=timetoseconds(schedulerD[i],8);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"timetoseconds",status,__PRETTY_FUNCTION__);
                    return status;
                }
            }
        #endif
    }   
    else
    {
        if(schedulerOn==1)
        {
            #if (!SCHEDULER)
                status=append_sched_config(scheduler[0],0);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
                    return status;
                }
            #endif
        }
        /**Each sensor has a different time*/
        else
        {
            status=time_config_init();
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"time_config_init",status,__PRETTY_FUNCTION__);  
                return status;
            }
            status=add_sensing_groups();
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"add_sensing_groups",status,__PRETTY_FUNCTION__);  
                return status;
            }

            for(int i=0; i<SCHEDULER_SIZE; i++)
            { 
                #if (!SCHEDULER)
                    status=append_sched_config(scheduler[i],0);
                    if(status != NODEFLOW_OK)
                    {
                        ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
                        return status;
                    }
                #endif
            }
        }
    }
return NODEFLOW_OK;
}

int NodeFlow::timetoseconds(float scheduler_time, uint8_t group_id)
{
    uint16_t time_remainder=DIVIDE(((int(scheduler_time))*HOURINSEC)+((fmod(scheduler_time,1))*6000));
    status=append_sched_config(time_remainder,group_id); /**group_id dec for 0001,0010,0100,1000: 1,2,4,8*/
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    
    timetodate(time_remainder*2);

    return status;

}

int NodeFlow::overwrite_sched_config(uint16_t code,uint16_t length)
{
    SchedulerConfig s_conf;
    s_conf.parameters.time_comparator=code;

    status= DataManager::overwrite_file_entries(SchedulerConfig_n, s_conf.data, sizeof(s_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    s_conf.parameters.time_comparator=length;

    status = DataManager::append_file_entry(SchedulerConfig_n, s_conf.data, sizeof(s_conf.parameters));
    
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    return status;
}

int NodeFlow::append_sched_config(uint16_t time_comparator,uint8_t group_id)
{
    SchedulerConfig t_conf;
    t_conf.parameters.time_comparator=time_comparator;
    t_conf.parameters.group_id=group_id;

    status= DataManager::append_file_entry(SchedulerConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}


/** SendScheduler Config TODO:overwrite in case of an NBIOT received_message, should be less than the MAX_BUFFER_SENDING_TIMES
 */

int NodeFlow::init_send_sched_config()
{
    pc.printf("\r\n---------------ADD SENDING TIMES-----------------\r\n");
    #if(SEND_SCHEDULER)
        status=overwrite_send_sched_config(SEND_SCHEDULER,SEND_SCHEDULER_SIZE);
    #endif
    #if(!SEND_SCHEDULER)
        status=overwrite_send_sched_config(SEND_SCHEDULER,0);
    #endif
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"overwrite_send_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint16_t sendschedulerOn;
    status=read_send_sched_config(0,&sendschedulerOn);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_send_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint16_t send_sched_length;
    uint16_t time_remainder=0;
    if(sendschedulerOn)
    {
        read_send_sched_config(1,&send_sched_length);
        for(int i=0; i<send_sched_length; i++)
        {   
            #if(SEND_SCHEDULER)
                time_remainder=DIVIDE(((int(nbiot_send_scheduler[i]))*HOURINSEC)+((fmod(nbiot_send_scheduler[i],1))*6000));
            #endif
            status=append_send_sched_config(time_remainder);
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"append_send_sched_config",status,__PRETTY_FUNCTION__);
                return status;
            }
            pc.printf("%d. Sending ",i);
            timetodate(time_remainder*2);
        }
       
    }

    else
    {  
        uint16_t sched_length;
        status=read_sched_config(1,&sched_length);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);
            return status;
        }
        if(sched_length)
        {
            status=append_send_sched_config(scheduler[0]);
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"append_send_sched_config",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
    }
return status;
}

int NodeFlow::overwrite_send_sched_config(uint16_t code,uint16_t length)
{
    SendSchedulerConfig ss_conf;
    ss_conf.parameters.time_comparator=code;

    status= DataManager::overwrite_file_entries(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SendSchedulerConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    ss_conf.parameters.time_comparator=length;

    status = DataManager::append_file_entry(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SendSchedulerConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    return status;
}


int NodeFlow::append_send_sched_config(uint16_t time_comparator)
{
    SendSchedulerConfig ss_conf;
    ss_conf.parameters.time_comparator=time_comparator;

    status= DataManager::append_file_entry(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"SendSchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}

int NodeFlow::read_send_sched_config(int i, uint16_t* time)
{
    SendSchedulerConfig ss_conf;
    status = DataManager::read_file_entry(SendSchedulerConfig_n, i, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"SendSchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    *time=ss_conf.parameters.time_comparator;

    return status;
}

//change so the user will choose
int NodeFlow::overwrite_clock_synch_config(int time_comparator, bool clockSynchOn)
{
    ClockSynchConfig c_conf;
    c_conf.parameters.clockSynchOn=clockSynchOn;
    c_conf.parameters.time_comparator=time_comparator;
    
    status = DataManager::overwrite_file_entries(ClockSynchConfig_n, c_conf.data, sizeof(c_conf.parameters));
    int count=0;
    while(status != NODEFLOW_OK && count<MAX_OVERWRITE_RETRIES) 
    {
        ErrorHandler(__LINE__,"ClockSynchConfig",status,__PRETTY_FUNCTION__);
        status = DataManager::overwrite_file_entries(ClockSynchConfig_n, c_conf.data, sizeof(c_conf.parameters));
        ++count;
    } 

    return status;
}

int NodeFlow::read_clock_synch_config(uint16_t* time, bool &clockSynchOn)
{
    ClockSynchConfig c_conf;
    status = DataManager::read_file_entry(ClockSynchConfig_n, 0, c_conf.data, sizeof(c_conf.parameters));
    if (status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"ClockSyncConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    
    clockSynchOn=c_conf.parameters.clockSynchOn;
    *time=c_conf.parameters.time_comparator;

    return status;
}

int NodeFlow::read_sched_config(int i, uint16_t* time_comparator)
{
    SchedulerConfig r_conf;
    status = DataManager::read_file_entry(SchedulerConfig_n, i, r_conf.data, sizeof(r_conf.parameters));
    *time_comparator=r_conf.parameters.time_comparator;
    if (status!=NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}

int NodeFlow::read_sched_group_id(int i, uint8_t* group_id)
{
    SchedulerConfig r_conf;
    status = DataManager::read_file_entry(SchedulerConfig_n, i, r_conf.data, sizeof(r_conf.parameters));
    *group_id=r_conf.parameters.group_id;
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}

/**Sets the flags, for just kicking the watchdog, sensing time,clock synch time, or sending time(NOT YET) */
int NodeFlow:: set_flags_config(uint8_t ssck_flag)
{
    FlagsConfig f_conf;
    f_conf.parameters.ssck_flag=ssck_flag;
    f_conf.parameters.pin_wakeup=0;

    status= DataManager::overwrite_file_entries(FlagsConfig_n, f_conf.data, sizeof(f_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagsConfig",status,__PRETTY_FUNCTION__); 
    }

    return status;
}

int NodeFlow::set_wakeup_pin_flag(bool wakeup_pin)
{
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));

    f_conf.parameters.pin_wakeup=wakeup_pin;
    status= DataManager::overwrite_file_entries(FlagsConfig_n, f_conf.data, sizeof(f_conf.parameters));

    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagsConfig",status,__PRETTY_FUNCTION__);  
        status= DataManager::overwrite_file_entries(FlagsConfig_n, f_conf.data, sizeof(f_conf.parameters));
    }
    return status;
}

/**Ignore this for now 
 */
int NodeFlow::add_sensing_groups() {
    pc.printf("\r\n---------------ADD METRIC GROUPS-----------------\r\n");
    sensor_config_init(SCHEDULER_SIZE);
    for (int i=0; i<SCHEDULER_SIZE; i++)
    {
        SensingGroupConfig sg_conf;
        sg_conf.parameters.group_id = false;
        sg_conf.parameters.time_comparator=scheduler[i];
        
        status=DataManager::append_file_entry(SensingGroupConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
        if(status!=0)
        {
            ErrorHandler(__LINE__,"SensingGroupConfig",status,__PRETTY_FUNCTION__); 
            return status;
        }

        TempSensingGroupConfig ts_conf;
        ts_conf.parameters.group_id = false;
        ts_conf.parameters.time_comparator=scheduler[i];
    
        status = DataManager::append_file_entry(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
        if(status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__); 
            return status;
        }
            
        status = DataManager::read_file_entry(TempSensingGroupConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__); 
            return status;
        }
        pc.printf("%d. Sensing group id: %i, wake up every: %u Seconds\r\n",i, ts_conf.parameters.group_id,ts_conf.parameters.time_comparator);
                
        }
     
    pc.printf("--------------------------------------------------\r\n");
   
    return status;
  
}

/**Returns seconds until next reading. Sets the flags for the next event in case of timer wakeup
 */
int NodeFlow::set_scheduler(uint32_t* next_timediff)
{
   
    uint8_t mybit_int;
    bitset<8> ssck_flag(0b0000'0000);

    uint16_t schedulerOn;
    status=read_sched_config(0,&schedulerOn);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__); 
        return status;
    }

    uint16_t sendschedulerOn;
    status=read_send_sched_config(0,&sendschedulerOn);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_send_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint16_t length;
    status=read_sched_config(1,&length);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint32_t timediff_temp=DAYINSEC;
    uint32_t timediff_temp_send=DAYINSEC;

    if(flags == NodeFlow::FLAG_CLOCK_SYNCH||flags == NodeFlow::FLAG_SENSE_SYNCH 
        ||flags == NodeFlow::FLAG_SEND_SYNCH ||flags == NodeFlow::FLAG_SENSE_SEND_SYNCH)
    {
        get_timestamp();
    }
    pc.printf("\r\n-----------------NEXT READING TIME----------------\r\n");
    uint32_t time_remainder=this->time_now();
    int32_t timediff=0;
    uint32_t next_sch_time=0;
    uint32_t scheduled_times=0;
    uint16_t times;
    uint8_t group_id=0;
    uint8_t temp_group_id=0;
    if(schedulerOn)
    {
        for (int i=0; i<length; i++)
        {
            
            status=read_sched_config(i+2,&times);
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);
                return status;
            }
            scheduled_times=times*2;
            //Make sure that the time does not exceeds 24 hours(user mistake)
            scheduled_times=scheduled_times%DAYINSEC;
            timediff=scheduled_times-time_remainder;

            if(timediff<0)
            {
                timediff=timediff+DAYINSEC;
            }
            if (timediff <= timediff_temp)
            {
                if (timediff < timediff_temp)
                {
                    read_sched_group_id(i+2,&group_id);
                    

                }
                if (timediff == timediff_temp)
                {
                    read_sched_group_id(i+2,&temp_group_id);
                    if(temp_group_id!=group_id)
                    {
                         group_id=group_id+temp_group_id;
                    }             
                }
                
                timediff_temp=timediff; //holds the smallest different form time_now
                next_sch_time=scheduled_times;
                
                status=overwrite_metric_flags(group_id);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"overwrite_metric_flags",status,__PRETTY_FUNCTION__);
                    return status;
                }
            }  
        }
        pc.printf("Group id:  %d\r\n",group_id);
        pc.printf("Next Sensing ");
        timetodate(next_sch_time);
       
    }
    else if(schedulerOn == false && length==1)
    {
        
        status=read_sched_config(2,&times);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);
            return status;
        }
        timediff_temp=times;
    }
    else if(schedulerOn == false && length>1)
    {
       set_reading_time(&timediff_temp);  
    } 
    ssck_flag.set(0);
  
    #if BOARD == EARHART_V1_0_0
        ssck_flag.set(1);
    #endif

    if(sendschedulerOn)
    {
        uint16_t send_length;
        uint16_t sched_time_temp;
        read_send_sched_config(1,&send_length);
        for (int i=0; i<send_length; i++)
        {   
            read_send_sched_config(i+2,&sched_time_temp);
            scheduled_times=sched_time_temp*2;
            //Make sure that the time does not exceeds 24 hours(user mistake)
            scheduled_times=scheduled_times%DAYINSEC;
            timediff=scheduled_times-time_remainder;

            if(timediff<0)
            {
                timediff=timediff+DAYINSEC;
            }
            if (timediff<timediff_temp_send)
            {
                timediff_temp_send=timediff; //holds the smallest different form time_now
                next_sch_time=scheduled_times;
            }  
        }
        pc.printf("Next Sending ");
        timetodate(next_sch_time);

         if(timediff_temp_send<=timediff_temp)
        {
           
            ssck_flag.set(1);
            if(schedulerOn == false && length>1)
            {
                fix_sensing_group_time(timediff_temp_send);
            }
               
            if(timediff_temp_send<timediff_temp)
            {
                timediff_temp=timediff_temp_send;
                ssck_flag.reset(0);
            } 
        }
    }
   
    //the clock synch should be send in 2 bytes, so half the value)
    bool clockSynchOn=0;
    uint16_t time;
    status=read_clock_synch_config(&time,clockSynchOn);
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_clock_synch_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint32_t cs_time=(2*time)-time_remainder;

    if(clockSynchOn)
    {
        status=read_clock_synch_config(&time,clockSynchOn);
        if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"read_clock_synch_config",status,__PRETTY_FUNCTION__);
                return status;
            }
        pc.printf("Next ClkSync ");
        timetodate(2*time);
        if(cs_time < 0)
        {
            cs_time=cs_time+DAYINSEC;
        }
        if(cs_time<=timediff_temp)
        {   
            
            ssck_flag.set(2);
            if(schedulerOn == false && length>1)
            {
                fix_sensing_group_time(cs_time);
            }
            if(cs_time<timediff_temp)
            {
                timediff_temp=cs_time;
                ssck_flag.reset(0);
                ssck_flag.reset(1);
                
            } 
        }
    }
    
    /**Check that its not more than 2 hours*/
    if (timediff_temp>6600)  
    {
        if(schedulerOn == false && length>1)
        {
            fix_sensing_group_time(timediff_temp);
        }
        timediff_temp=6600;
        ssck_flag.reset(0);
        ssck_flag.reset(1);
        ssck_flag.reset(2);
        ssck_flag.set(3);
       
    } 
    mybit_int = int(ssck_flag.to_ulong());

    pc.printf("\r\nSense flag: %d, Send flag: %d,Clock: %d, Kick flag: %d\n", ssck_flag.test(0), ssck_flag.test(1), ssck_flag.test(2), ssck_flag.test(3));
    ThisThread::sleep_for(100);
    set_flags_config(mybit_int);
    ovewrite_wakeup_timestamp(timediff_temp); 

    *next_timediff=timediff_temp;
    return NODEFLOW_OK;   
}



int NodeFlow::fix_sensing_group_time(uint32_t time){
    uint16_t sched_length;
    read_sched_config(1,&sched_length);
    uint8_t temp[sched_length];
    TempSensingGroupConfig ts_conf;
    SensingGroupConfig sg_conf;

    for(int i=0; i<sched_length; i++)
    {
        status = DataManager::read_file_entry(TempSensingGroupConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        ts_conf.parameters.time_comparator= ts_conf.parameters.time_comparator-time;
        temp[i]=ts_conf.parameters.time_comparator;
    }
    for(int i=0; i<sched_length; i++)
    {   
        ts_conf.parameters.time_comparator=temp[i];
        if(i == 0)
        {
            status=DataManager::overwrite_file_entries(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));  
        }
        else
        {
            status=DataManager::append_file_entry(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                status=DataManager::append_file_entry(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
                ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
                return status;
            }
        }

    }
return NODEFLOW_OK;
}
int NodeFlow::set_reading_time(uint32_t* time)
{ 
    TempSensingGroupConfig ts_conf;
    TimeConfig t_conf;
    SensingGroupConfig sg_conf;
    
    uint16_t temp_time[SCHEDULER_SIZE];
    uint8_t mybit_int;
    bitset<8> flags(0b0000'0000);

    status = DataManager::read_file_entry(TimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TimeConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    int time_comparator=t_conf.parameters.time_comparator; 
    
    status = DataManager::read_file_entry(TempSensingGroupConfig_n, 0, ts_conf.data, sizeof(ts_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    int temp=ts_conf.parameters.time_comparator; //time_left for first sensor
    
    for (int i=0; i<SCHEDULER_SIZE; i++)
    {
        status=DataManager::read_file_entry(TempSensingGroupConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
            return status;
        }
      
        int time_comparator_now= ts_conf.parameters.time_comparator;

        if (temp >= time_comparator_now && time_comparator_now != 0)
        {
            flags.set(i);                    
            if(temp>time_comparator_now)
            {
                for(int y=0; y<i; y++)
                {
                    flags.reset(y);
                }
            }

            temp=time_comparator_now; 
        }
    }

    time_comparator=temp;
    mybit_int = int(flags.to_ulong());
    overwrite_metric_flags(mybit_int);
    status=set_time_config(time_comparator);
    
    for(int i=0; i<SCHEDULER_SIZE; i++)
    {
        status=DataManager::read_file_entry(TempSensingGroupConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
            return status;
        }
        int time_comp=ts_conf.parameters.time_comparator-time_comparator;
        temp_time[i]=time_comp;
       
        if(time_comp == 0)
        {
            status=DataManager::read_file_entry(SensingGroupConfig_n, i, sg_conf.data, sizeof(sg_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"SensingGroupConfig",status,__PRETTY_FUNCTION__);
                return status;
            }
           
            temp_time[i]=sg_conf.parameters.time_comparator;
        }

    }

    for(int i=0; i<SCHEDULER_SIZE; i++)
    {
        pc.printf("%d. Sensing group id: %i, next reading: %u seconds\r\n",i,i,temp_time[i]);
        ts_conf.parameters.group_id=i;
        ts_conf.parameters.time_comparator=temp_time[i];

        if(i == 0)
        {
            status=DataManager::overwrite_file_entries(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters)); 
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
        else
        {
            status=DataManager::append_file_entry(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                status=DataManager::append_file_entry(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
                ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
    }
    
    pc.printf("\r\nNext Reading ");
    timetodate(time_comparator+time_now());
    pc.printf("GroupA: %d, GroupB: %d, GroupC: %d, GroupD: %d\n", flags.test(0), flags.test(1), flags.test(2), flags.test(3));
    *time=time_comparator;

    return status;
}
int NodeFlow::overwrite_metric_flags(uint8_t mybit_int)
{
    MetricGroupConfig mg_conf;
    mg_conf.parameters.metric_group_id=mybit_int;
    status=DataManager::overwrite_file_entries(MetricGroupConfig_n, mg_conf.data, sizeof(mg_conf.parameters));
    if(status != NODEFLOW_OK)
    {   
        status=DataManager::overwrite_file_entries(MetricGroupConfig_n, mg_conf.data, sizeof(mg_conf.parameters));
        ErrorHandler(__LINE__,"MetricGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    return status;

}

int NodeFlow:: get_metric_flags(uint8_t *flag)
{
    MetricGroupConfig mg_conf;
    status=DataManager::read_file_entry(MetricGroupConfig_n, 0, mg_conf.data, sizeof(mg_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    } 
    *flag=mg_conf.parameters.metric_group_id;
    return status;
}
int NodeFlow::get_interrupt_latency(uint32_t *next_sch_time)
{
    NextTimeConfig t_conf;
    status=DataManager::read_file_entry(NextTimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"NextTimeConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    
    uint32_t temp=0;
    *next_sch_time=t_conf.parameters.time_comparator-time_now();
    
    return status;
}

int NodeFlow::ovewrite_wakeup_timestamp(uint16_t time_remainder){
    
    NextTimeConfig t_conf;
    t_conf.parameters.time_comparator=time_now()+time_remainder;
    status=DataManager::overwrite_file_entries( NextTimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK){
        ErrorHandler(__LINE__,"NextTimeConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}
/** Timestamp. Send ClockSync message, wait for a response from ttn if it fails don't change the time
    * @param num_timestamp_retries  The number of retries to get the Timestamp
    * 

 */
int NodeFlow::get_timestamp()
{

   time_t unix_time=0;

   #if BOARD == EARHART_V1_0_0
        int retcode=_radio.join(CLASS_A);
        if(retcode<0)
        {
            ErrorHandler(__LINE__,"_radio.join",retcode,__PRETTY_FUNCTION__);
            return retcode;
        }
        uint8_t dummy[1]={1};
        uint8_t port=0;
        uint32_t rx_dec_buffer[MAX_BUFFER_READING_TIMES];
        pc.printf("Horrayy,setting the time, bear with me\r\nRetries are set to %d\r\n",MAX_RETRY_CLOCK_SYNCH);
        retcode=_radio.send_message(223, dummy, sizeof(dummy));
        if(retcode<=0)
        {
            ErrorHandler(__LINE__,"FAILED TO SEND",retcode,__PRETTY_FUNCTION__);
            return retcode;
        }
        _radio.receive_message(rx_dec_buffer,&port,&retcode);
        port=0;
        ThisThread::sleep_for(1000);
        for(int i=0; ((port!=CLOCK_SYNCH_PORT) && (i<MAX_RETRY_CLOCK_SYNCH));i++) 
        {
            pc.printf("%i. Waiting for a server message dude \r\n",i);
            ThisThread::sleep_for(5000);
            retcode=_radio.send_message(223, dummy, sizeof(dummy));
            if(status<0)
            {
                ErrorHandler(__LINE__,"FAILED TO SEND",retcode,__PRETTY_FUNCTION__);
            }
            _radio.receive_message(rx_dec_buffer,&port,&retcode);
            if(port == CLOCK_SYNCH_PORT && retcode>0)
            {
                unix_time=rx_dec_buffer[0];
            }
        }
    
    #endif /* BOARD == EARHART_V1_0_0 */

    #if BOARD == WRIGHT_V1_0_0
        //_radio.get_unix_time(&unix_time);
    #endif

    time_t time_now=time(NULL);
    if (unix_time>time_now)
    {
        pc.printf("Received value: %d\r\n",unix_time);
        set_time(unix_time);
    }

    return NODEFLOW_OK;   
}

uint32_t NodeFlow::time_now() 
{
    time_t time_now=time(NULL);
    uint32_t timestamp=time_now; 
    uint32_t remainder_time=(timestamp%DAYINSEC);
    return remainder_time;
}

int NodeFlow::timetodate(uint32_t remainder_time)
{
    double_t t_value=(remainder_time/float(HOURINSEC));
    double_t minutes_f=fmod(t_value,1);
    uint8_t hours= t_value- minutes_f;
    double_t minutes=minutes_f*MINUTEINSEC; 
    double_t seconds=(fmod(minutes,1))*MINUTEINSEC;
    pc.printf("Time(HH:MM:SS):   %02d:%02d:%02d\r\n", hours, int(minutes),int(seconds));
    return NODEFLOW_OK;
}

/**LorawanTP
 */
#if BOARD == EARHART_V1_0_0

int NodeFlow::sendTTN(uint8_t port, uint8_t payload[], uint16_t length)
{
    status=_radio.join(CLASS_A);
    if(status < NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FAILED TO JOIN",status,__PRETTY_FUNCTION__);
        return status;
    }
    pc.printf("---------------------SENDING----------------------\r\n");
    timetodate(time_now());
    
    status=_radio.send_message(port, payload, length);
    if (status < NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FAILED TO SEND",status,__PRETTY_FUNCTION__);
        return status;
    }
   
    pc.printf("\r\nSuccesfully sending %d bytes",status);  
    pc.printf("\r\n--------------------------------------------------\r\n");
    return status;
}

int NodeFlow::receiveTTN(uint32_t* rx_message, uint8_t* rx_port)
{   uint8_t port=0;
    int retcode=0;
    uint32_t rx_dec_buffer[MAX_BUFFER_READING_TIMES];
    _radio.receive_message(rx_dec_buffer,&port,&retcode); 
    
    if (port==SCHEDULER_PORT)
    {
        status=overwrite_sched_config(true, DIVIDE(retcode));
        if (status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"overwrite_sched_config",status,__PRETTY_FUNCTION__);
            return status;
        }

        for (int i=0; i<DIVIDE(retcode); i++)
        {
            pc.printf("%i.RX scheduler: %d(10)\r\n",i, rx_dec_buffer[i]);
            status=append_sched_config(rx_dec_buffer[i]/2,1); //TODO: CHANGE GROUP ID- depends on data
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
    }
    
    if(port==CLOCK_SYNCH_ACK_PORT)
    {
        bool clockSynchOn=false;
        uint16_t time;
        status=read_clock_synch_config(&time,clockSynchOn);
        if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"read_clock_synch_config",status,__PRETTY_FUNCTION__);
                return status;
            }
        if(retcode==1 && clockSynchOn)
        {
            status=overwrite_clock_synch_config(rx_dec_buffer[0]/2,false);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"overwrite_clock_synch_config",status,__PRETTY_FUNCTION__);
                return status;
            }

        }
        else
        {
            status=overwrite_clock_synch_config(rx_dec_buffer[0]/2,true);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"overwrite_clock_synch_config",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
    }
    if(port==0)
    {
        pc.printf("No Rx available\r\n"); 
    }
    else
    {
        pc.printf("Rx: %d(10)\r\n", rx_dec_buffer[0]);
        pc.printf("Port: %d\r\n", port);
    }
    _radio.sleep();
    *rx_message=rx_dec_buffer[0];
    *rx_port=port;
    
    return status;
}
#endif /* #if BOARD == EARHART_V1_0_0 */


int NodeFlow::get_flags()
{    
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagsConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    bitset<8> ssck_flag(f_conf.parameters.ssck_flag);
    if(ssck_flag.test(0) && ssck_flag.test(1) && ssck_flag.test(2))
    {
        return NodeFlow::FLAG_SENSE_SEND_SYNCH;
    }

    if(ssck_flag.test(1) && ssck_flag.test(2))
    {
        return NodeFlow::FLAG_SEND_SYNCH;
    }
    if(ssck_flag.test(0) && ssck_flag.test(1))
    {
        return NodeFlow::FLAG_SENSE_SEND;
    }
    if(ssck_flag.test(0) && ssck_flag.test(2))
    {
        return NodeFlow::FLAG_SENSE_SYNCH;
    }
    if(ssck_flag.test(2))
    {
        return NodeFlow::FLAG_CLOCK_SYNCH;
    }
    if(ssck_flag.test(3))
    {
        return NodeFlow::FLAG_WDG;
    }
    if(ssck_flag.test(0))
    {
        return NodeFlow::FLAG_SENSING;
    }

    if(ssck_flag.test(1))
    {
        return NodeFlow::FLAG_SENDING;
    }

    return NodeFlow::FLAG_SENDING; 
}

int NodeFlow::delay_pin_wakeup()
{    
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagsConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagsConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
     if (f_conf.parameters.pin_wakeup)
     {
         return NodeFlow::FLAG_WAKEUP_PIN;
     }
     return NODEFLOW_OK;
}

/** Clear whatever needed i.e increments, eeprom stuff and other
 */
int NodeFlow::_clear_after_send()
{
    SensorDataConfig sd_conf;
    sd_conf.parameters.byte=0;
    status= DataManager::delete_file_entries(SensorDataConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SensorDataConfig",status,__PRETTY_FUNCTION__); 
    }
    
   _clear_increment();
    
 return NODEFLOW_OK;
}

/** Manage device sleep times before calling sleep_manager.standby().
 *  Ensure that the maximum time the device can sleep for is 6600 seconds,
 *  this is due to the watchdog timer timeout, set at 7200 seconds
 *
 * @param seconds Number of seconds to sleep for 
 * @param wkup_one If true the device will respond to rising edge interrupts
 *                 on WKUP_PIN1
 * @return None 
 */
void NodeFlow::enter_standby(int seconds, bool wkup_one) 
{ 
    if(seconds < 2)
    {
        seconds = 2;
    } 
    
    #if BOARD == EARHART_V1_0_0
        int retcode=_radio.sleep();
    #endif /* BOARD == EARHART_V1_0_0 */

    //Without this delay it breaks..?!
    ThisThread::sleep_for(2);

    sleep_manager.standby(seconds, wkup_one);
}

void NodeFlow::ErrorHandler(int line, const char* str1, int status,const char* str2) 
{
   pc.printf("Error in line No = %d, %s,Status = %d,Function name = %s\r\n",line, str1, status, str2);
   //check for concecutive line and status errors error reporting
   int errCnt=0;
   error_increment(&errCnt);
   pc.printf("Errors %d",errCnt);
   if(errCnt>=STATUS_ERROR_TOLERANCE+1)
   {
       NVIC_SystemReset();
   }
}

void NodeFlow:: error_increment(int *errCnt)
{
    ErrorConfig e_conf;
    status = DataManager::read_file_entry(ErrorConfig_n, 0, e_conf.data, sizeof(e_conf.parameters));
    if (status == NODEFLOW_OK)
    {
        e_conf.parameters.errCnt=1+e_conf.parameters.errCnt;
        status= DataManager::overwrite_file_entries(ErrorConfig_n, e_conf.data, sizeof(e_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementConfig",status,__PRETTY_FUNCTION__); 
        }
    }
    *errCnt=e_conf.parameters.errCnt;

}




