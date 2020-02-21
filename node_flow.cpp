/**
 ******************************************************************************
 * @file    NodeFlow.cpp
 * @version 0.4.0
 * @author  Rafaella Neofytou, Adam Mitchell
 * @brief   C++ file of the Wright || Earheart node from Think Pilot. 
 ******************************************************************************
 **/

/** Includes
 */
#include "node_flow.h"


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
    #if(SCHEDULER)
        scheduler=new float[1];
    #endif /* #if(SCHEDULER) */
    
}
#endif /* #if BOARD == EARHART_V1_0_0 */

#if BOARD == WRIGHT_V1_0_0
NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz,
                   PinName txu, PinName rxu, PinName cts, PinName rst, 
                   PinName vint, PinName gpio, int baud, PinName done) :
                   DataManager(write_control, sda, scl, frequency_hz), _radio(txu, rxu, cts, rst, vint, gpio, baud), watchdog(done)
{
    #if(SCHEDULER)
        scheduler=new float[1];
    #endif /* #if(SCHEDULER) */
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
            char ipv4[] = "68.183.254.233";
            uint16_t port= 5683;
            char uri[] = "coap://68.183.254.233:5683/";
            uint8_t uri_length=27;
            status=_radio.configure_coap(ipv4, port, uri, uri_length);
            if(status != NodeFlow::NODEFLOW_OK)
            {
                debug("\nCoap server not configured %d\r\n",status);
                return status;
            }

            status = _radio.start(5);
            if(status != NodeFlow::NODEFLOW_OK)
            {
                debug("\nRadio not initialised %d\r\n",status);
                //return status;
            }
        }
       
        
    }
    return status;      
}
#endif /* #if BOARD == WRIGHT_V1_0_0 */


void NodeFlow::_oob_enter_test()
{
    INIT_STATE = Init_State::TEST;
}

void NodeFlow::_oob_gpio_test_handler()
{
    TEST_STATE = Test_State::GPIO_TEST;
}

void NodeFlow::_oob_enter_prov()
{
    INIT_STATE = Init_State::PROV;
}

void NodeFlow::_oob_end_handler()
{
    TEST_STATE = Test_State::END;
    PROV_STATE = Prov_State::END;
}

void NodeFlow::_run()
{
    INIT_STATE = Init_State::RUN;
}


/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 */
void NodeFlow::start()
{
    uint32_t next_time=0;
    watchdog.kick();
    TP_Sleep_Manager::WakeupType_t wkp = sleep_manager.get_wakeup_type();
    
    time_t start_time=time_now();
    if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_PIN)
    {   
        #if (INTERRUPT_ON)
            debug("\r\n--------------------PIN WAKEUP--------------------\r\n");
            tformatter.setup();
            HandleInterrupt(); /**Pure virtual function */
            uint16_t c_entries;
            tformatter.get_entries(c_entries);
            if (c_entries > 1)
            {
                #if(!SEND_SCHEDULER)
                    mg_counter(1);
                    _send(true);
                #endif /* #if(!SEND_SCHEDULER) */

                #if(SEND_SCHEDULER)
                    mg_counter(0);
                    add_payload_data(0);
                    if( is_overflow())
                    {
                        _send(false);
                    }
                #endif /* #if(SEND_SCHEDULER) */
            }
            get_interrupt_latency(next_time);
            if(next_time>INTERRUPT_DELAY)
            {
                next_time=INTERRUPT_DELAY;
                set_wakeup_pin_flag(true);
            }
        
            enter_standby(next_time,false);
        #endif
    }
    else if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_TIMER) 
    {
        if(is_delay_pin_wakeup_flag()==NodeFlow::FLAG_WAKEUP_PIN)
        {
            set_wakeup_pin_flag(false);
            get_interrupt_latency(next_time);
        }
        else
        {   
            debug("\r\n-------------------TIMER WAKEUP-------------------\r\n");
            timetodate(time_now());
            tformatter.setup();
            uint8_t wakeup_flag=get_wakeup_flags();

            if(wakeup_flag==NodeFlow::FLAG_SENSING || wakeup_flag==NodeFlow::FLAG_SENSE_SEND ||
                wakeup_flag==NodeFlow::FLAG_SENSE_SEND_SYNCH || wakeup_flag==NodeFlow::FLAG_SENSE_SYNCH)
            {
                _sense();
            }
            if(wakeup_flag==NodeFlow::FLAG_SENDING || wakeup_flag==NodeFlow::FLAG_SENSE_SEND ||
                wakeup_flag==NodeFlow::FLAG_SEND_SYNCH || wakeup_flag==NodeFlow::FLAG_SENSE_SEND_SYNCH)
            { 
                _send(false);
            }

            time_t end_time=time_now();
            int latency=end_time-start_time;
            set_scheduler(latency, next_time);

            //todo: check this because the latency changes if you get a timestamp 
            if(wakeup_flag == NodeFlow::FLAG_CLOCK_SYNCH || wakeup_flag == NodeFlow::FLAG_SENSE_SYNCH 
                ||wakeup_flag == NodeFlow::FLAG_SEND_SYNCH || wakeup_flag== NodeFlow::FLAG_SENSE_SEND_SYNCH)
            {
                get_timestamp();
                set_scheduler(0, next_time);//todo: CHECK
            }
            timetodate(time_now());
           
        }
    }
    else if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_RESET || wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_SOFTWARE) 
    {
        debug("\r\n                      __|__\n               --+--+--(_)--+--+--\n-------------------THING PILOT--------------------\r\n");
        debug("\nDevice Unique ID: %08X %08X %08X \r", STM32_UID[0], STM32_UID[1], STM32_UID[2]);
        wait_us(1000);
        status=initialise();
        if (status != NODEFLOW_OK)
        { 
            NVIC_SystemReset(); 
        }
        
        wait_us(300000);

        {
            ATCmdParser *_parser;

            _parser = new ATCmdParser(mbed_file_handle(STDOUT_FILENO));
            _parser->set_delimiter("\r\n");
            _parser->set_timeout(1000);

            _parser->oob("TEST", callback(this, &NodeFlow::_oob_enter_test));
            _parser->oob("PROV", callback(this, &NodeFlow::_oob_enter_prov));
            _parser->oob("AT+END", callback(this, &NodeFlow::_oob_end_handler));

            _parser->send("AT+CTRL");
            if(_parser->recv("OK"))
            {
                switch(INIT_STATE)
                {
                    case Init_State::TEST: 
                    {
                        _parser->oob("AT+GPIO", callback(this, &NodeFlow::_oob_gpio_test_handler));

                        while(INIT_STATE == Init_State::TEST)
                        {
                            switch(TEST_STATE)
                            {
                                case Test_State::GPIO_TEST: 
                                {
                                    int pin, state = 0;
                                    _parser->recv("=%i,%i", &pin, &state);

                                    DigitalOut test_pin((PinName)pin, state);                    
                                    _parser->send("GPIO: %i, %i", pin, state);
                                    
                                    if(_parser->recv("OK"))
                                    {
                                        TEST_STATE = Test_State::WFC;
                                        break;
                                    }

                                    TEST_STATE = Test_State::WFC;
                                    break;
                                }
                                case Test_State::WFC: 
                                {
                                    _parser->process_oob();
                                    break;
                                }
                                case Test_State::END:
                                {
                                    _parser->send("OK");
                                    _run();
                                    break;
                                }    
                            }
                        }

                        break;
                    }
                    case Init_State::PROV:
                    {
                        while(INIT_STATE == Init_State::PROV)
                        {
                            switch(PROV_STATE)
                            {
                                case Prov_State::PROVISION: 
                                {
                                    _parser->process_oob();
                                    break;
                                }
                                case Prov_State::WFC: 
                                {
                                    _parser->process_oob();
                                    break;
                                }
                                case Prov_State::END: 
                                {
                                    _parser->send("OK");
                                    _run();
                                    break;
                                }
                            }
                        }

                        break;
                    }
                }
            }

            delete _parser;
        }

        #if BOARD == WRIGHT_V1_0_0
            debug("\r\nInitialising NB-IOT..");
            initialise_nbiot();
        #endif /* #if BOARD == WRIGHT_V1_0_0 */
        setup(); /** Pure virtual by the user */

        if(CLOCK_SYNCH) //todo: time synch everyday? every two/three user can choose?
        {
            get_timestamp();
        }
        timetodate(time_now());
        start_time=time_now();
        init_sched_config();
        init_send_sched_config();
        set_scheduler(0,next_time); 
    }

    debug("\nGoing to sleep for %d ",next_time);
    timetodate(time_now());
    
    #if (INTERRUPT_ON) //todo: bug
        if (next_time<15) //to prevent more delays
        {
            enter_standby(next_time,false);
        }
        else
        {
            enter_standby(next_time,true);
        }
    #endif
    #if (!INTERRUPT_ON)
        enter_standby(next_time,false);
    #endif

}


/** Initialise the EEPROM
 * @return Status
 */
int NodeFlow::initialise()
{    
    status=DataManager::init_filesystem();
    if(status != NODEFLOW_OK)
    {
        debug("\r\nInit error. Line: %d,status: %d, %s",__LINE__, status,__PRETTY_FUNCTION__);
        return status;
    }

    bool initialised = false;
    status=DataManager::is_initialised(initialised);
    if(status != NODEFLOW_OK)
    {
        debug("\r\nInit error. Line: %d,status: %d, %s",__LINE__, status,__PRETTY_FUNCTION__);
        return status;
    }

    DataManager_FileSystem::File_t ErrorConfig_File_t;
    ErrorConfig_File_t.parameters.filename = ErrorConfig_n;
    ErrorConfig_File_t.parameters.length_bytes = sizeof(ErrorConfig::parameters);

    status=DataManager::add_file(ErrorConfig_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        debug("\r\nLine: %d,status: %d, %s",__LINE__, status,__PRETTY_FUNCTION__);
        return status;
    }
    ErrorConfig e_conf;
    e_conf.parameters.errCnt=0;
    status= DataManager::overwrite_file_entries(ErrorConfig_n, e_conf.data, sizeof(e_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        debug("\r\nLine: %d,status: %d, %s",__LINE__, status,__PRETTY_FUNCTION__);
        return status;    
    }

    /** SchedulerConfig */
    DataManager_FileSystem::File_t SchedulerConfig_File_t;
    SchedulerConfig_File_t.parameters.filename = SchedulerConfig_n;  
    SchedulerConfig_File_t.parameters.length_bytes = sizeof(SchedulerConfig::parameters);

    status=DataManager::add_file(SchedulerConfig_File_t, MAX_BUFFER_READING_TIMES+2); 
    if(status != NODEFLOW_OK)
    {
        debug("\r\nLine: %d,status: %d, %s",__LINE__, status,__PRETTY_FUNCTION__);
        return status;   
    }

    SchedulerConfig s_conf;
    s_conf.parameters.time_comparator=0;
    status= DataManager::overwrite_file_entries(SchedulerConfig_n, s_conf.data, sizeof(s_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        debug("\r\nLine: %d,status: %d, %s",__LINE__, status,__PRETTY_FUNCTION__);
        return status;    
    }

    /** SendSchedulerConfig 
     */
    DataManager_FileSystem::File_t SendSchedulerConfig_File_t;
    SendSchedulerConfig_File_t.parameters.filename = SendSchedulerConfig_n;  
    SendSchedulerConfig_File_t.parameters.length_bytes = sizeof( SendSchedulerConfig::parameters);
    #if(SEND_SCHEDULER)
        #if BOARD == EARHART_V1_0_0
            debug("\r\nWARNING!! SEND SCHEDULER IS ON FOR EARHART BOARD\r\nVisit https://www.loratools.nl/#/airtime to find \r\nout more. Max payload size supported 255 bytes\r\n");     
        #endif /* #if BOARD == EARHART_V1_0_0 */
        status=DataManager::add_file(SendSchedulerConfig_File_t, MAX_BUFFER_SENDING_TIMES+2); 
    #endif /* #if(SEND_SCHEDULER) */
    #if(!SEND_SCHEDULER)
        status=DataManager::add_file(SendSchedulerConfig_File_t, 2);
    #endif /* #if(!SEND_SCHEDULER) */
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

    SendSchedulerConfig ss_conf;
    ss_conf.parameters.time_comparator=0;
    status= DataManager::overwrite_file_entries(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        return status;    
    }
    
    /** ClockSynchFlag 
     */
    DataManager_FileSystem::File_t ClockSynchFlag_File_t;
    ClockSynchFlag_File_t.parameters.filename = ClockSynchFlag_n;
    ClockSynchFlag_File_t.parameters.length_bytes = sizeof( ClockSynchFlag::parameters);

    status=DataManager::add_file(ClockSynchFlag_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        return status;   
    }
    #if (CLOCK_SYNCH)
        status=overwrite_clock_synch_config(DIVIDE(CLOCK_SYNCH_TIME),CLOCK_SYNCH);
    #endif /* #if (CLOCK_SYNCH) */
    #if (!CLOCK_SYNCH)
        status=overwrite_clock_synch_config(0,CLOCK_SYNCH);
    #endif /* #if (!CLOCK_SYNCH) */
    if (status != NODEFLOW_OK)
    {
        return status;
    }   
    /** FlagSSCKConfig. Every bit is a different flag. 
     *  0:SENSE, 1:SEND, 2:CLOCK, 3:KICK && true or false for pin_wakeup
     */
    DataManager_FileSystem::File_t FlagSSCKConfig_File_t;
    FlagSSCKConfig_File_t.parameters.filename = FlagSSCKConfig_n;
    FlagSSCKConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);

    status=DataManager::add_file(FlagSSCKConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }
    status=set_flags_config(0);  

    /** MemoryFlagConfig. Memory full notation
     */
    DataManager_FileSystem::File_t MemoryFlagConfig_File_t;
    MemoryFlagConfig_File_t.parameters.filename = MemoryFlagConfig_n;
    MemoryFlagConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);

    status=DataManager::add_file(MemoryFlagConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }
      
    /** IncrementAConfig
     */
    DataManager_FileSystem::File_t IncrementAConfig_File_t;
    IncrementAConfig_File_t.parameters.filename = IncrementAConfig_n;
    IncrementAConfig_File_t.parameters.length_bytes = sizeof(IncrementAConfig::parameters);
    status=DataManager::add_file(IncrementAConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }
    IncrementAConfig i_conf;
    i_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        return status; 
    }

    /** IncrementBConfig
     */
    DataManager_FileSystem::File_t IncrementBConfig_File_t;
    IncrementBConfig_File_t.parameters.filename = IncrementBConfig_n;
    IncrementBConfig_File_t.parameters.length_bytes = sizeof(IncrementBConfig::parameters);

    status=DataManager::add_file(IncrementBConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }
    IncrementBConfig ib_conf;
    ib_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementBConfig_n, ib_conf.data, sizeof(ib_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        return status; 
    }
    /** IncrementCConfig
     */
    DataManager_FileSystem::File_t IncrementCConfig_File_t;
    IncrementCConfig_File_t.parameters.filename = IncrementCConfig_n;
    IncrementCConfig_File_t.parameters.length_bytes = sizeof(IncrementCConfig::parameters);

    status=DataManager::add_file(IncrementCConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

    IncrementCConfig ic_conf;
    ic_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementCConfig_n, ic_conf.data, sizeof(ic_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        return status; 
    }

    DataManager_FileSystem::File_t NextTimeConfig_File_t;
    NextTimeConfig_File_t.parameters.filename = NextTimeConfig_n;
    NextTimeConfig_File_t.parameters.length_bytes = sizeof(NextTimeConfig::parameters);

    status=DataManager::add_file(NextTimeConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

    DataManager_FileSystem::File_t MetricGroupConfig_File_t;
    MetricGroupConfig_File_t.parameters.filename = MetricGroupConfig_n;
    MetricGroupConfig_File_t.parameters.length_bytes = sizeof(MetricGroupConfig::parameters);

    status = DataManager::add_file(MetricGroupConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    #if (INTERRUPT_ON)
        DataManager_FileSystem::File_t InterruptConfig_File_t;
        InterruptConfig_File_t.parameters.filename = InterruptConfig_n;
        InterruptConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);
       
        #if BOARD == EARHART_V1_0_0
            status = DataManager::add_file(InterruptConfig_File_t, 300);
        #endif /* #if BOARD == EARHART_V1_0_0 */
        #if BOARD == WRIGHT_V1_0_0
             status = DataManager::add_file(InterruptConfig_File_t, 2000); 
        #endif
        if(status != NODEFLOW_OK)
        {
            debug("\r\nInterruptConfig_File_t");
            return status;
        }
    #endif

    DataManager_FileSystem::File_t MetricGroupAConfig_File_t;
    MetricGroupAConfig_File_t.parameters.filename = MetricGroupAConfig_n;
    MetricGroupAConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);
    
    #if (METRIC_GROUPS_ON > 0)
        #if BOARD == EARHART_V1_0_0
            status = DataManager::add_file(MetricGroupAConfig_File_t, 1200/METRIC_GROUPS_ON);
        #endif /* #if BOARD == EARHART_V1_0_0 */
        #if BOARD == WRIGHT_V1_0_0    
            status = DataManager::add_file(MetricGroupAConfig_File_t, 4000);
        #endif
        if(status != NODEFLOW_OK)
        {
            debug("\r\nMetricGroupAConfig_File_t");
            return status;
        }
    #endif
    #if (SCHEDULER_B || METRIC_GROUPS_ON >=2)
        DataManager_FileSystem::File_t MetricGroupBConfig_File_t;
        MetricGroupBConfig_File_t.parameters.filename = MetricGroupBConfig_n;
        MetricGroupBConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

        #if BOARD == EARHART_V1_0_0
            status = DataManager::add_file(MetricGroupBConfig_File_t, 1200/METRIC_GROUPS_ON);
        #endif /* #if BOARD == EARHART_V1_0_0 */
        #if BOARD == WRIGHT_V1_0_0 
            status = DataManager::add_file(MetricGroupBConfig_File_t, 3000);
        #endif
        if(status != NODEFLOW_OK)
        {
            debug("\r\nMetricGroupBConfig_File_t");
            return status;
        }
    #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */

    #if (SCHEDULER_C || METRIC_GROUPS_ON >=3)
        DataManager_FileSystem::File_t MetricGroupCConfig_File_t;
        MetricGroupCConfig_File_t.parameters.filename = MetricGroupCConfig_n;
        MetricGroupCConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

        #if BOARD == EARHART_V1_0_0
            status = DataManager::add_file(MetricGroupCConfig_File_t, 1200/METRIC_GROUPS_ON);
        #endif /* #if BOARD == EARHART_V1_0_0 */
        #if BOARD == WRIGHT_V1_0_0 
            status = DataManager::add_file(MetricGroupCConfig_File_t,3000);
        #endif
        if(status != NODEFLOW_OK)
        {
            debug("\r\nMetricGroupCConfig_File_t");
            return status;
        }

    #endif /* #if (SCHEDULER_C || METRIC_GROUPS_ON==3) */

    #if (SCHEDULER_D|| METRIC_GROUPS_ON==4)
        DataManager_FileSystem::File_t MetricGroupDConfig_File_t;
        MetricGroupDConfig_File_t.parameters.filename = MetricGroupDConfig_n;
        MetricGroupDConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

        #if BOARD == EARHART_V1_0_0
            status = DataManager::add_file(MetricGroupDConfig_File_t, 1200/METRIC_GROUPS_ON);
        #endif /* #if BOARD == EARHART_V1_0_0 */
        #if BOARD == WRIGHT_V1_0_0 
            status = DataManager::add_file(MetricGroupDConfig_File_t,3000);
        #endif
        if(status != NODEFLOW_OK)
        {
            debug("\r\nMetricGroupDConfig_File_t");
            return status;
        }
    #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON=42) */
    
   
    DataManager_FileSystem::File_t MetricGroupEntriesConfig_File_t;
    MetricGroupEntriesConfig_File_t.parameters.filename = MetricGroupEntriesConfig_n;
    MetricGroupEntriesConfig_File_t.parameters.length_bytes = sizeof(MetricGroupEntriesConfig::parameters);

    status = DataManager::add_file(MetricGroupEntriesConfig_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        return status;
    }

    clear_mg_counter();
    return status;
}

//todo:
int NodeFlow::create_file(uint8_t filename, int length)
{
    // filenames_len;
    // debug("\r\nFiles: %d\r\n",filenames_len);
    DataManager_FileSystem::File_t DataConfig_File_t;
    DataConfig_File_t.parameters.filename = filename; 
    DataConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    status = DataManager::add_file(DataConfig_File_t, length); 
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    DataConfig t_conf;
    t_conf.parameters.byte=length;
    status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        debug("Error.Line %d, Status: %d",__LINE__, status);
    }

    status = DataManager::read_file_entry(filename, 0, t_conf.data, sizeof(t_conf.parameters));

    debug("\r\nNew file: %d Length: %d \r\n",filename,t_conf.parameters.byte);
    return NODEFLOW_OK;
}

template <typename DataType> 
void NodeFlow::add_record(DataType data, string str1)
{   
    #if(SEND_SCHEDULER)
        //tformatter.write_string(str1);
        tformatter.write_num_type<DataType>(data);
    #endif /* #if(SEND_SCHEDULER) */

    #if(!SEND_SCHEDULER)
        tformatter.increase_entries();
        uint8_t bytes[sizeof(DataType)];
        *(DataType *)(bytes)=data;
        for (int i=sizeof(DataType); i>0; i--)
        {
            add_sensing_entry(bytes[i-1],1);
        }
    #endif /* #if(!SEND_SCHEDULER) */
}
template void NodeFlow::add_record<int8_t>(int8_t data, string str1);
template void NodeFlow::add_record<int16_t>(int16_t data, string str1);
template void NodeFlow::add_record<int32_t>(int32_t data, string str1);
template void NodeFlow::add_record<int64_t>(int64_t data, string str1);
template void NodeFlow::add_record<uint8_t>(uint8_t data, string str1);
template void NodeFlow::add_record<uint16_t>(uint16_t data, string str1);
template void NodeFlow::add_record<uint32_t>(uint32_t data, string str1);
template void NodeFlow::add_record<uint64_t>(uint64_t data, string str1);
template void NodeFlow::add_record<float>(float data, string str1);
template void NodeFlow::add_record<double>(double data, string str1);

int NodeFlow::add_payload_data(uint8_t metric_group_flag) 
{
    uint16_t c_entries;
    tformatter.get_entries(c_entries);
    if( c_entries>1)
    {
        uint8_t buffer[100];
        size_t buffer_len=0;
        tformatter.get_serialised(buffer, buffer_len);
        for (int i=0; i<buffer_len; i++)
        {
            add_sensing_entry(buffer[i],metric_group_flag);
        } 
    }
    else
    {
        debug("\r\nNo new entries from this group");
    }
    return NODEFLOW_OK;
}

int NodeFlow::add_sensing_entry(uint8_t value, uint8_t metric_group)
{
    if(metric_group==0)
    {       
        #if(INTERRUPT_ON)
            DataConfig t_conf;
            t_conf.parameters.byte=value;
            status= DataManager::append_file_entry(InterruptConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                debug("Error.Line %d, Status: %d",__LINE__, status);
            }
        #endif
    }
    
    if(metric_group==1)
    {
        DataConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(MetricGroupAConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            debug("Error.Line %d, Status: %d",__LINE__, status);
        }
    }
    
    if(metric_group==2)
    {    
        #if (SCHEDULER_B || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==2) 
            DataConfig t_conf;
            t_conf.parameters.byte=value;
            status= DataManager::append_file_entry(MetricGroupBConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                debug("Error.Line %d, Status: %d",__LINE__, status);
            }
        #endif
    }
    if(metric_group==3)
    {
        #if (SCHEDULER_C  || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
            DataConfig t_conf;
            t_conf.parameters.byte=value;
            status= DataManager::append_file_entry(MetricGroupCConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                debug("Error.Line %d, Status: %d",__LINE__, status);
            }
        #endif
    }
    if(metric_group==4)
    {   
        #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
        DataConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(MetricGroupDConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            debug("Error.Line %d, Status: %d",__LINE__, status);
        }
        #endif
    }
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupsConfig",status,__PRETTY_FUNCTION__);
    }
    return status;
    
}

/** Increment A is erased after reading the value (have to be stored immediately)
 *  Increment B is erased only if the user manually erase it
 *  Increment C is erased after the data are sent
 */
 //TODo:  erase after read 
uint16_t NodeFlow::read_inc_a()
{
    IncrementAConfig i_conf;
    status = DataManager::read_file_entry(IncrementAConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
    }
   // increment_value=i_conf.parameters.increment;
    if (status == NODEFLOW_OK)
    {
        i_conf.parameters.increment=0;
        status= DataManager::overwrite_file_entries(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
        }
    }
    
    return i_conf.parameters.increment;
}

int NodeFlow::inc_a(int i)
{
    IncrementAConfig i_conf;
    status = DataManager::read_file_entry(IncrementAConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
    }
   
    i_conf.parameters.increment=i+i_conf.parameters.increment;
    status= DataManager::overwrite_file_entries(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__); 
    }

    return status;
}

int NodeFlow::read_inc_b(uint16_t& increment_value)
{
    IncrementBConfig i_conf;
    status = DataManager::read_file_entry(IncrementBConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__);   
    }
    increment_value=i_conf.parameters.increment;

    return status;
}

int NodeFlow::clear_inc_b()
{
    IncrementBConfig i_conf;
    i_conf.parameters.increment=0;
    status= DataManager::overwrite_file_entries(IncrementBConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
    }
    return status;

}

int NodeFlow::inc_b(int i)
{
    IncrementBConfig i_conf;
    status = DataManager::read_file_entry(IncrementBConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__);   
    }

    i_conf.parameters.increment=i+i_conf.parameters.increment;
    status= DataManager::overwrite_file_entries(IncrementBConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__); 
    }
    return status;
}

int NodeFlow::read_inc_c(uint64_t& increment_value)
{
    IncrementCConfig i_conf;
    status = DataManager::read_file_entry(IncrementCConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementCConfig",status,__PRETTY_FUNCTION__);   
    }
    increment_value=i_conf.parameters.increment;

    return status;
}

int NodeFlow::inc_c(int i)
{
    uint64_t increment_value;
    status=read_inc_c(increment_value);
    if (status == NODEFLOW_OK)
    {
        IncrementCConfig i_conf;
        i_conf.parameters.increment=i+increment_value;
        status= DataManager::overwrite_file_entries(IncrementCConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementCConfig",status,__PRETTY_FUNCTION__); 
        }
    }
    return status;
}


int NodeFlow::read_mg_counter(uint16_t& mga_entries, uint16_t& mgb_entries,uint16_t& mgc_entries, uint16_t& mgd_entries, uint8_t& metric_group_active)
{   
    metric_group_active=0;
    MetricGroupEntriesConfig i_conf;
    status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    mga_entries= i_conf.parameters.MetricGroupAEntries;
    if (mga_entries!=0)
    {
        metric_group_active++;
    }
    mgb_entries= i_conf.parameters.MetricGroupBEntries;
    if (mgb_entries!=0)
    {
        metric_group_active++;
    }
    mgc_entries= i_conf.parameters.MetricGroupCEntries;
    if (mgc_entries!=0)
    {
        metric_group_active++;
    }
    mgd_entries= i_conf.parameters.MetricGroupDEntries;
    if (mgd_entries!=0)
    {
        metric_group_active++;
    }
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__); 
    }
    return NODEFLOW_OK;
}

int NodeFlow::read_interrupt_counter(uint16_t& interrupt_entries)
{
    MetricGroupEntriesConfig i_conf;
    status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__);
    }
    interrupt_entries= i_conf.parameters.InterruptEntries;
   
    return NODEFLOW_OK;
}

bool NodeFlow::is_overflow()
{
    int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes;
    uint16_t mga_entries, mgb_entries, mgc_entries, mgd_entries, interrupt_entries;
    uint8_t metric_group_active;
    read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);
    read_mg_counter(mga_entries, mgb_entries, mgc_entries, mgd_entries, metric_group_active);
    read_interrupt_counter(interrupt_entries);

    uint16_t total_bytes=mga_bytes+ mgb_bytes+ mgc_bytes+ mgd_bytes + interrupt_bytes;
    uint16_t total_entries=mga_entries + mgb_entries + mgc_entries + mgd_entries + interrupt_entries;

    debug("\r\nENTRIES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d\r\n",mga_entries, mgb_entries, mgc_entries, mgd_entries,interrupt_entries);
    debug("\r\nBYTES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d\r\n",mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);

    #if BOARD == EARHART_V1_0_0
        if((total_bytes+(total_bytes/total_entries)+53)> 800)
        {
            debug("\r\nIN THE NEXT ENTRY THE MEMORY WILL BE FULL- SENT DATA NOW");
            return true;
        }
    #endif

    if(total_bytes+(total_bytes/total_entries)+53>3800) 
    {
        debug("\r\nIN THE NEXT ENTRY THE MEMORY WILL BE FULL- SENT DATA NOW");
        return true;
    }
    return false;
}

int NodeFlow::read_mg_bytes(int& mga_bytes, int& mgb_bytes, int& mgc_bytes,int& mgd_bytes, int& interrupt_bytes)
{
    mga_bytes=0;
    mgb_bytes=0;
    mgc_bytes=0;
    mgd_bytes=0;
    interrupt_bytes=0;
    #if (INTERRUPT_ON)
        status= DataManager::get_total_written_file_entries(InterruptConfig_n, interrupt_bytes);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
            return status;
        }
    #endif
    status= DataManager::get_total_written_file_entries(MetricGroupAConfig_n, mga_bytes);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        return status;
    }
    #if (SCHEDULER_B  || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==2)
    status= DataManager::get_total_written_file_entries(MetricGroupBConfig_n, mgb_bytes);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        return status;
    }
    #endif
    #if (SCHEDULER_C || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
    status= DataManager::get_total_written_file_entries(MetricGroupCConfig_n, mgc_bytes);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        return status;
    }
    #endif
    #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    status= DataManager::get_total_written_file_entries(MetricGroupDConfig_n, mgd_bytes);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        return status;
    }
    #endif

 return NODEFLOW_OK;

} 

int NodeFlow::mg_counter(uint8_t mg_flag) 
{
    MetricGroupEntriesConfig i_conf;
    status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status == NODEFLOW_OK)
    {
        if (mg_flag == 0) //interrupt
        {
            i_conf.parameters.InterruptEntries=i_conf.parameters.InterruptEntries+1;
        }
        if (mg_flag == 1)
        {
            i_conf.parameters.MetricGroupAEntries=i_conf.parameters.MetricGroupAEntries+1;
        }
        if (mg_flag == 2)
        {
            i_conf.parameters.MetricGroupBEntries=i_conf.parameters.MetricGroupBEntries+1;
        }
        if (mg_flag == 3)
        {
            i_conf.parameters.MetricGroupCEntries=i_conf.parameters.MetricGroupCEntries+1;
        }
        if (mg_flag == 4)
        {
            i_conf.parameters.MetricGroupDEntries=i_conf.parameters.MetricGroupDEntries+1;
        }
    
        status= DataManager::overwrite_file_entries(MetricGroupEntriesConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__); 
        }
    }
    
    return status;

}

int NodeFlow::clear_mg_counter()
{
    MetricGroupEntriesConfig mge_conf;
    mge_conf.parameters.MetricGroupAEntries=0;
    mge_conf.parameters.MetricGroupBEntries=0;
    mge_conf.parameters.MetricGroupCEntries=0;
    mge_conf.parameters.MetricGroupDEntries=0;
    mge_conf.parameters.InterruptEntries=0;
    status= DataManager::overwrite_file_entries(MetricGroupEntriesConfig_n, mge_conf.data, sizeof(mge_conf.parameters));
    if (status!=NODEFLOW_OK)
    {   
        ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__);
    }
    return NODEFLOW_OK;
}

int NodeFlow::clear_increment()
{
    IncrementCConfig ic_conf;
    ic_conf.parameters.increment=0;
   
    status= DataManager::overwrite_file_entries(IncrementCConfig_n, ic_conf.data, sizeof(ic_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementCConfig",status,__PRETTY_FUNCTION__);   
    }
    return status;
}

int NodeFlow::metric_config_init(int length)
{
    DataManager_FileSystem::File_t MetricGroupTimesConfig_File_t;
    MetricGroupTimesConfig_File_t.parameters.filename = MetricGroupTimesConfig_n;
    MetricGroupTimesConfig_File_t.parameters.length_bytes = sizeof(MetricGroupTimesConfig::parameters);

    status=DataManager::add_file(MetricGroupTimesConfig_File_t, length);
    if (status != NODEFLOW_OK) 
    {
        ErrorHandler(__LINE__,"MetricGroupTimesConfig",status,__PRETTY_FUNCTION__);
        return status; 
    }
    
    DataManager_FileSystem::File_t TempMetricGroupTimesConfig_File_t;
    TempMetricGroupTimesConfig_File_t.parameters.filename = TempMetricGroupTimesConfig_n;
    TempMetricGroupTimesConfig_File_t.parameters.length_bytes = sizeof(MetricGroupTimesConfig::parameters);

    status = DataManager::add_file(TempMetricGroupTimesConfig_File_t, length);
    if(status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
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
            debug("\nWARNING!! Scheduler size too big,\nonly 1 interval time is associated with each metric group\n");
        }   
    #endif
   
    status=overwrite_sched_config(SCHEDULER,SCHEDULER_SIZE);
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"overwrite_sched_config",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint16_t schedulerOn;
    read_sched_config(0,schedulerOn);
    
    if(schedulerOn)
    {   
        #if (SCHEDULER_A)
            debug("\r\n---------------ADD SENSING TIMES GA---------------\r\n");
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
            debug("\r\n---------------ADD SENSING TIMES GB---------------\r\n");
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
            debug("\r\n---------------ADD SENSING TIMES GC---------------\r\n");       
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
            debug("\r\n--------------ADD SENSING TIMES GD---------------\r\n");
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

    time_config_init();
    #if (METRIC_GROUPS_ON != 0)
        if(!schedulerOn)
        {
            uint16_t sch_length;
            read_sched_config(1,sch_length);
            add_metric_groups();
        
            for(int i=0; i<sch_length; i++)
            { 
                status=append_sched_config(scheduler[i],(i)); 
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
                    return status;
                }
            }
        }
        #endif
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


int NodeFlow::init_send_sched_config()
{       
    #if(!SEND_SCHEDULER)
        status=overwrite_send_sched_config(SEND_SCHEDULER,0);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"overwrite_send_sched_config",status,__PRETTY_FUNCTION__);
        }
    #endif

    #if(SEND_SCHEDULER)
        debug("\r\n---------------ADD SENDING TIMES------------------\r\n");
        status=overwrite_send_sched_config(SEND_SCHEDULER,SEND_SCHEDULER_SIZE);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"overwrite_send_sched_config",status,__PRETTY_FUNCTION__);
        }
        uint16_t sendschedulerOn;
        read_send_sched_config(0,sendschedulerOn);
    
        uint16_t send_sched_length;
        uint16_t time_remainder=0;
        if(sendschedulerOn)
        {
            read_send_sched_config(1,send_sched_length);
            for(int i=0; i<send_sched_length; i++)
            {   
                #if(SEND_SCHEDULER)
                    time_remainder=DIVIDE(((int(send_scheduler[i]))*HOURINSEC)+((fmod(send_scheduler[i],1))*6000));
                #endif
                status=append_send_sched_config(time_remainder);
                if(status != NODEFLOW_OK)
                {
                    ErrorHandler(__LINE__,"append_send_sched_config",status,__PRETTY_FUNCTION__);
                    return status;
                }
                timetodate(time_remainder*2);
            }
        }

        else
        {  
            #if (METRIC_GROUPS_ON !=0)
                uint16_t sched_length;
                read_sched_config(1,sched_length);
                
                if(sched_length)
                {
                    status=append_send_sched_config(scheduler[0]);
                    if(status != NODEFLOW_OK)
                    {
                        ErrorHandler(__LINE__,"append_send_sched_config",status,__PRETTY_FUNCTION__);
                    }
                }
            #endif
        }
    #endif
return status;
}

int NodeFlow::overwrite_send_sched_config(uint16_t code,uint16_t length)
{
    SendSchedulerConfig ss_conf;
    ss_conf.parameters.time_comparator=code;

    status= DataManager::overwrite_file_entries(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SendSchedulerConfig_n",status,__PRETTY_FUNCTION__);
        return status;
    }

    ss_conf.parameters.time_comparator=length;

    status = DataManager::append_file_entry(SendSchedulerConfig_n, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SendSchedulerConfig_n",status,__PRETTY_FUNCTION__);
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
         ErrorHandler(__LINE__,"SendSchedulerConfig_n",status,__PRETTY_FUNCTION__);
    }

    return status;
}

int NodeFlow::read_send_sched_config(int i, uint16_t& time)
{
    SendSchedulerConfig ss_conf;
    status = DataManager::read_file_entry(SendSchedulerConfig_n, i, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"SendSchedulerConfig_n",status,__PRETTY_FUNCTION__);
    }

    time=ss_conf.parameters.time_comparator;

    return status;
}

//change so the user will choose
int NodeFlow::overwrite_clock_synch_config(int time_comparator, bool clockSynchOn)
{
    ClockSynchFlag c_conf;
    c_conf.parameters.clockSynchOn=clockSynchOn;
    c_conf.parameters.time_comparator=time_comparator;
    
    status = DataManager::overwrite_file_entries(ClockSynchFlag_n, c_conf.data, sizeof(c_conf.parameters));
    int count=0;
    while(status != NODEFLOW_OK && count<MAX_OVERWRITE_RETRIES) 
    {
        ErrorHandler(__LINE__,"ClockSynchFlag",status,__PRETTY_FUNCTION__);
        status = DataManager::overwrite_file_entries(ClockSynchFlag_n, c_conf.data, sizeof(c_conf.parameters));
        ++count;
    } 

    return status;
}

int NodeFlow::read_clock_synch_config(uint16_t& time, bool &clockSynchOn)
{
    ClockSynchFlag c_conf;
    status = DataManager::read_file_entry(ClockSynchFlag_n, 0, c_conf.data, sizeof(c_conf.parameters));
    if (status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"ClockSyncConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    
    clockSynchOn=c_conf.parameters.clockSynchOn;
    time=c_conf.parameters.time_comparator;

    return status;
}

int NodeFlow::read_sched_config(int i, uint16_t& time_comparator)
{
    SchedulerConfig r_conf;
    status = DataManager::read_file_entry(SchedulerConfig_n, i, r_conf.data, sizeof(r_conf.parameters));
    time_comparator=r_conf.parameters.time_comparator;
    if (status!=NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}

int NodeFlow::read_sched_group_id(int i, uint8_t& group_id)
{
    SchedulerConfig r_conf;
    status = DataManager::read_file_entry(SchedulerConfig_n, i, r_conf.data, sizeof(r_conf.parameters));
    group_id=r_conf.parameters.group_id;
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"SchedulerConfig",status,__PRETTY_FUNCTION__);
    }

    return status;
}

/**Sets the flags, for just kicking the watchdog, sensing time,clock synch time, or sending time(NOT YET) */
/** Program specific flags. Every bit is a different flag. 0:SENSE, 1:SEND, 2:CLOCK, 3:KICK
 */
int NodeFlow:: set_flags_config(uint8_t ssck_flag)
{
    FlagsConfig f_conf;
    f_conf.parameters.value=ssck_flag;
    f_conf.parameters.flag=0;

    status= DataManager::overwrite_file_entries(FlagSSCKConfig_n, f_conf.data, sizeof(f_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagSSCKConfig",status,__PRETTY_FUNCTION__); 
    }

    return status;
}

int NodeFlow::set_wakeup_pin_flag(bool wakeup_pin)
{
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagSSCKConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));

    f_conf.parameters.flag=wakeup_pin;
    status= DataManager::overwrite_file_entries(FlagSSCKConfig_n, f_conf.data, sizeof(f_conf.parameters));

    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagSSCKConfig",status,__PRETTY_FUNCTION__);  
    }
    return status;
}

/**Ignore this for now 
 */
#if (METRIC_GROUPS_ON != 0) /**Interrupt only */
void NodeFlow::add_metric_groups() 
{   
    debug("\r\n---------------ADD METRIC GROUPS------------------");
    uint16_t sch_length;
    read_sched_config(1,sch_length);
    metric_config_init(sch_length);
    MetricGroupTimesConfig sg_conf;
    for (int i=0; i<sch_length; i++)
    {
        sg_conf.parameters.time_comparator=scheduler[i];

        if(i == 0)
        {
            status= DataManager::overwrite_file_entries(MetricGroupTimesConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status!=0)
            {
                ErrorHandler(__LINE__,"MetricGroupTimesConfig",status,__PRETTY_FUNCTION__); 
            }
            status= DataManager::overwrite_file_entries(TempMetricGroupTimesConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status!=0)
            {
                ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__); 
            }
        }
        else
        {
            status=DataManager::append_file_entry(MetricGroupTimesConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status!=0)
            {
                ErrorHandler(__LINE__,"MetricGroupTimesConfig",status,__PRETTY_FUNCTION__); 
            }

            status = DataManager::append_file_entry(TempMetricGroupTimesConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status!=NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
            }
        }
        
        status = DataManager::read_file_entry(TempMetricGroupTimesConfig_n, i, sg_conf.data, sizeof(sg_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__); 
        }
        debug("\r\n%d. Metric group id: %i, wake up every: %u Seconds",i,i,sg_conf.parameters.time_comparator);
                
        }
        debug("\r\n");
}

#endif

/**Returns seconds until next reading. Sets the flags for the next event in case of timer wakeup
 */
void NodeFlow::set_scheduler(int latency, uint32_t& next_timediff)
{
    bitset<8> ssck_flag(0b0000'0000);
    uint16_t schedulerOn=0;
    #if(METRIC_GROUPS_ON != 0)
        uint16_t length;
        read_sched_config(0,schedulerOn); 
        read_sched_config(1,length);
        debug("\r\n-----------------NEXT READING TIME----------------");
    #endif
    uint32_t timediff_temp=DAYINSEC;

    uint32_t time_remainder=this->time_now();
    int32_t timediff=0;
    uint32_t next_sch_time=0; 
    uint32_t scheduled_times=0;
    uint16_t times;
    uint8_t group_id=0;
    uint8_t temp_group_id=0;
   
    #if(METRIC_GROUPS_ON != 0)
        if(schedulerOn)
        {
            for (int i=0; i<length; i++)
            {
                read_sched_config(i+2,times);
                scheduled_times=times*2;
                scheduled_times=scheduled_times%DAYINSEC;
                timediff=scheduled_times-time_remainder;
                if(timediff<0)
                {
                    if(timediff+latency>0)
                    {
                        timediff=1;
                    }
                    else
                    {
                        timediff=timediff+DAYINSEC;
                    }
                }
               
                if (timediff <= timediff_temp)
                {
                    if (timediff < timediff_temp)
                    {
                        read_sched_group_id(i+2,group_id);
                    }
                    if (timediff == timediff_temp)
                    {
                        read_sched_group_id(i+2,temp_group_id);
                        if(temp_group_id!=group_id)
                        {
                            group_id=group_id+temp_group_id;
                        }             
                    }
                    timediff_temp=timediff;
                    next_sch_time=scheduled_times; //this can be removed it's just to print the next sensing time & sending time 
                    overwrite_metric_flags(group_id);
                }
            }
            // debug("\r\nNext Sensing ");
            // timetodate(next_sch_time);
        }
        else if(!schedulerOn)
        {
            set_reading_time(timediff_temp);  
        } 
   #endif
        ssck_flag.set(0);
  
     #if(!SEND_SCHEDULER)
        ssck_flag.set(1);
    #endif

    uint16_t sendschedulerOn;
    read_send_sched_config(0,sendschedulerOn);
    
    uint32_t timediff_temp_send=DAYINSEC;
    if(sendschedulerOn)
    {
        uint16_t send_length;
        uint16_t sched_time_temp;
        read_send_sched_config(1,send_length);
        for (int i=0; i<send_length; i++)
        {   
            read_send_sched_config(i+2,sched_time_temp);
            scheduled_times=sched_time_temp*2;
            scheduled_times=scheduled_times%DAYINSEC;
            timediff=scheduled_times-time_remainder;

            if(timediff<0)
            {
                timediff=timediff+DAYINSEC;
            }
            if (timediff<timediff_temp_send)
            {
                timediff_temp_send=timediff;
                next_sch_time=scheduled_times;
            }  
        }
        
        // debug("\r\nNext Sending ");
        // timetodate(next_sch_time);
        
        if(timediff_temp_send<=timediff_temp)
        {
            ssck_flag.set(1);
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
    read_clock_synch_config(time,clockSynchOn);
    
    uint32_t cs_time=(2*time)-time_remainder;

    if(clockSynchOn)
    {
        // debug("\r\nNext ClkSync ");
        // timetodate(2*time);
        if(cs_time < 0)
        {
            cs_time=cs_time+DAYINSEC;
        }
        if(cs_time<=timediff_temp)
        {   
            ssck_flag.set(2);
            if(cs_time<timediff_temp)
            {
                timediff_temp=cs_time;
                ssck_flag.reset(0);
                ssck_flag.reset(1); 
            } 
        }
    }
    
    /**Check that its not more than 2 hours, 6600*/
    if (timediff_temp>6600)  
    {
        timediff_temp=6600;
        ssck_flag.reset(0);
        ssck_flag.reset(1);
        ssck_flag.reset(2);
        ssck_flag.set(3);  
    } 
    debug("\r\nSense: %d, Send: %d, ClockSynch: %d, KickWdg: %d\n", ssck_flag.test(0), ssck_flag.test(1), ssck_flag.test(2), ssck_flag.test(3));
    
    set_flags_config(int(ssck_flag.to_ulong()));
    overwrite_wakeup_timestamp(timediff_temp); 
    if(!schedulerOn)
    {
        latency=timediff_temp-latency-1;
      
        if(latency <= 0)
        {
            set_temp_reading_times(timediff_temp-latency);
            timediff_temp=1;
        }
        else
        {
            set_temp_reading_times(timediff_temp);
            timediff_temp=latency; 
        }
    }
    next_timediff=timediff_temp;
}


int NodeFlow::set_reading_time(uint32_t& time)
{ 
    MetricGroupTimesConfig sg_conf;
    TimeConfig t_conf;
    uint16_t sch_length;
    read_sched_config(1,sch_length);

    uint16_t temp_time[sch_length];
    bitset<8> flags(0b0000'0000);
    int time_comparator;

    status = DataManager::read_file_entry(TimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TimeConfig",status,__PRETTY_FUNCTION__);
    }

    time_comparator=t_conf.parameters.time_comparator; 
    
    status = DataManager::read_file_entry(TempMetricGroupTimesConfig_n, 0, sg_conf.data, sizeof(sg_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
    }

    int temp=sg_conf.parameters.time_comparator; 

    for (int i=0; i<sch_length; i++)
    {
        status=DataManager::read_file_entry(TempMetricGroupTimesConfig_n, i, sg_conf.data, sizeof(sg_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
        }
        int time_comparator_now= sg_conf.parameters.time_comparator;

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
    overwrite_metric_flags(int(flags.to_ulong()));
    // #if (NODEFLOW_DBG)
    //     debug("\r\nNext Reading ");
    //     timetodate(time_comparator+time_now());
    // #endif
    debug("\r\nGroupA: %d, GroupB: %d, GroupC: %d, GroupD: %d", flags.test(0), flags.test(1), flags.test(2), flags.test(3));
    time=time_comparator;

    return status;
}
int NodeFlow::set_temp_reading_times(uint32_t time)
{
    MetricGroupTimesConfig sg_conf;
    uint16_t sch_length;
    read_sched_config(1,sch_length);
    uint32_t temp_time[sch_length];
    set_time_config(time);
    for(int i=0; i<sch_length; i++)
    {
        status=DataManager::read_file_entry(TempMetricGroupTimesConfig_n, i, sg_conf.data, sizeof(sg_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"MetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
            return status;
        }
        int time_comp=sg_conf.parameters.time_comparator-time;
        temp_time[i]=time_comp;

        if(time_comp <= 0)
        {
            status=DataManager::read_file_entry(MetricGroupTimesConfig_n, i, sg_conf.data, sizeof(sg_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"MetricGroupTimesConfig",status,__PRETTY_FUNCTION__);
                return status;
            }

            if(time_comp < 0)
            { 
                time_comp=sg_conf.parameters.time_comparator- time;
                if(time_comp < 0 ) 
                {
                    time_comp=2;
                }
                temp_time[i]=time_comp;
            }
            else
            {
                temp_time[i]=sg_conf.parameters.time_comparator;
            }
        }
    }

    for(int i=0; i<sch_length; i++)
    {
        
        debug("\r\n%d. Metric group id: %i, next wakeup: %u seconds",i,i,temp_time[i]);
        sg_conf.parameters.time_comparator=temp_time[i];

        if(i == 0)
        {
            status=DataManager::overwrite_file_entries(TempMetricGroupTimesConfig_n, sg_conf.data, sizeof(sg_conf.parameters)); 
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
        else
        {
            status=DataManager::append_file_entry(TempMetricGroupTimesConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status != NODEFLOW_OK)
            {   
                ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
                return status;
            }
        }
    }
    debug("\r\n");
   return status;
}
int NodeFlow::overwrite_metric_flags(uint8_t ssck_flag)
{
    MetricGroupConfig mg_conf;
    mg_conf.parameters.metric_group_id=ssck_flag;
    status=DataManager::overwrite_file_entries(MetricGroupConfig_n, mg_conf.data, sizeof(mg_conf.parameters));
    if(status != NODEFLOW_OK)
    {   
        ErrorHandler(__LINE__,"MetricGroupConfig",status,__PRETTY_FUNCTION__);
    }
    return status;
}

int NodeFlow::get_metric_flags(uint8_t &flag)
{
    MetricGroupConfig mg_conf;
    status=DataManager::read_file_entry(MetricGroupConfig_n, 0, mg_conf.data, sizeof(mg_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    } 
    flag=mg_conf.parameters.metric_group_id;
    return status;
}
int NodeFlow::get_interrupt_latency(uint32_t &next_sch_time)
{
    NextTimeConfig t_conf;
    status=DataManager::read_file_entry(NextTimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"NextTimeConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    next_sch_time=t_conf.parameters.time_comparator-time_now();
    return status;
}

int NodeFlow::overwrite_wakeup_timestamp(uint16_t time_remainder){
    
    NextTimeConfig t_conf;
    t_conf.parameters.time_comparator=time_now()+time_remainder;
    status=DataManager::overwrite_file_entries( NextTimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK)
    {   
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
    debug("\r\n--------------------TIMESTAMP-------------------\r\n");
    uint32_t unix_time;
    //todo: remove the earhart if wright same exists
    #if BOARD == EARHART_V1_0_0
        _radio.get_unix_time(unix_time);
    #endif /* BOARD == EARHART_V1_0_0 */

    #if BOARD == WRIGHT_V1_0_0
       // _radio.get_unix_time(unix_time); 
    #endif //BOARD == WRIGHT_V1_0_0

    if (unix_time>time(NULL))
    {
        debug("\r\nReceived value: %d\n",unix_time);
        set_time(unix_time);
    }
    return NODEFLOW_OK;   
}

uint32_t NodeFlow::time_now() 
{
    return (time(NULL))%DAYINSEC;
}


void NodeFlow::_sense()
{
    uint16_t sched_length, c_entries;
    read_sched_config(1,sched_length);
   
    if (sched_length>1)
    {   
        uint8_t mg_flag;
        get_metric_flags(mg_flag);
        bitset<8> metric_flag(mg_flag);
        #if(!SEND_SCHEDULER)
            add_sensing_entry(mg_flag,1);
        #endif /* #if(!SEND_SCHEDULER) */
        debug("\r\nMGroupA: %d, MGroupB: %d, MGroupC: %d, MGroupC: %d",metric_flag.test(0),
                metric_flag.test(1),metric_flag.test(2),metric_flag.test(3));
       
        
        if(metric_flag.test(0)==1)
        {
            MetricGroupA();
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                mg_counter(1);
            }
            #if(SEND_SCHEDULER)
                add_payload_data(1);
            #endif /* #if(SEND_SCHEDULER) */
        }

        if(metric_flag.test(1)==1)
        {
            MetricGroupB();
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                mg_counter(2);
            }
            #if(SEND_SCHEDULER)
                add_payload_data(2);
            #endif /* #if(SEND_SCHEDULER) */
        }
        if(metric_flag.test(2)==1)
        {   
            MetricGroupC();
            uint16_t c_entries;
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                mg_counter(3);
            }
            #if(SEND_SCHEDULER)
                add_payload_data(3);
            #endif /* #if(SEND_SCHEDULER) */
        }
        if(metric_flag.test(3)==1)
        {
            MetricGroupD();
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                mg_counter(4);
            }
            #if(SEND_SCHEDULER)
                add_payload_data(4);
            #endif /* #if(SEND_SCHEDULER) */
        }
    }
    else
    {
        #if(!SEND_SCHEDULER)
            add_sensing_entry(1,1);
        #endif /* #if(!SEND_SCHEDULER) */
        MetricGroupA();
        tformatter.get_entries(c_entries);
        if( c_entries>1)
        {
            mg_counter(1);
        }
        #if(SEND_SCHEDULER)
            add_payload_data(1);
        #endif /* #if(SEND_SCHEDULER) */
    }
    if( is_overflow())
    {
        _send(false);
    }

}


void NodeFlow::read_write_entry(uint8_t group_tag, int len, uint8_t filename)
{
    if (len!=0)
    {
        tformatter.write(group_tag, TFormatter::GROUP_TAG); 
        tformatter.write(159, TFormatter::RAW);
        for (int i=0; i< len; i++) 
        {   
            DataConfig d_conf;
            DataManager::read_file_entry(filename, i, d_conf.data, sizeof(d_conf.parameters));
            tformatter.write(d_conf.parameters.byte, TFormatter::RAW);

        }
        uint16_t total_bentries;
        tformatter.get_entries(total_bentries);
        debug("\r\nGROUP =  %d, BYTES = %d",group_tag, total_bentries);
        tformatter.write(255, TFormatter::RAW);
    }

}

void NodeFlow::_send(bool interrupt_send)
{
    uint16_t mga_entries, mgb_entries, mgc_entries, mgd_entries, interrupt_entries;
    int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes;
    uint8_t metric_group_active=0;

    read_mg_counter(mga_entries, mgb_entries, mgc_entries, mgd_entries, metric_group_active);
    read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes); 
    read_interrupt_counter(interrupt_entries);
    uint32_t total_bytes = mga_bytes+ mgb_bytes+ mgc_bytes+ mgd_bytes+interrupt_bytes;
    size_t buffer_len=0;
    
    #if(!SEND_SCHEDULER)
        debug("\r\nBYTES = %d",total_bytes); 
        buffer= new uint8_t[total_bytes+1];
        if (interrupt_send)
        {
            buffer[0]=0;
            buffer_len++;
        }
        for (int i=0; i<total_bytes; i++) 
        { 
            DataConfig a_conf;
            status = DataManager::read_file_entry(MetricGroupAConfig_n, i, a_conf.data, sizeof(a_conf.parameters));
            buffer[buffer_len]=a_conf.parameters.byte;
            buffer_len++;
        }
    #endif /* #if(!SEND_SCHEDULER) */

    #if(SEND_SCHEDULER)
        debug("\r\nENTRIES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d\r\n",mga_entries, mgb_entries, mgc_entries, mgd_entries,interrupt_entries);
        debug("\r\nBYTES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d\r\n",mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);
        
        if (interrupt_entries!=0)
        { 
            metric_group_active++;
        }
        tformatter.serialise_main_cbor_object(metric_group_active);
       

        #if (INTERRUPT_ON)
            read_write_entry(0, interrupt_bytes, InterruptConfig_n);
        #endif
        
        read_write_entry(1, mga_bytes, MetricGroupAConfig_n);
        #if (SCHEDULER_B || METRIC_GROUPS_ON==2 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==4)
            read_write_entry(2, mgb_bytes, MetricGroupBConfig_n);
        #endif /*  #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */
        #if (SCHEDULER_C || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==4)
            read_write_entry(3, mgc_bytes, MetricGroupCConfig_n);
        #endif /*  #if (SCHEDULER_C|| METRIC_GROUPS_ON==3) */
        #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
            read_write_entry(3, mgd_bytes, MetricGroupDConfig_n);
        #endif /*  #if (SCHEDULER_C|| METRIC_GROUPS_ON==4) */
        

        uint16_t c_entries;
        tformatter.get_entries(c_entries);
        buffer= new uint8_t[c_entries];
        tformatter.return_serialised(buffer, buffer_len);
        debug("\r\nSending %d bytes..",buffer_len);
    #endif /* #if(SEND_SCHEDULER) */
    
    int response_code=-1;
    #if BOARD == WRIGHT_V1_0_0
        int rsrp=0;
        int rsrq=0;
        _radio.get_csq(rsrp,rsrq);
        debug("\r\nNBIOT last know RSRP %d and RSRQ %d",rsrp, rsrq);
        char recv_data[512];
        
        status=_radio.coap_post(buffer, buffer_len, recv_data, SaraN2::TEXT_PLAIN, response_code); 
        if(response_code == 0 || response_code == 2 ) 
        {
            debug("\r\nSuccess. Response_code %d",response_code);
            clear_after_send();
        } 

        else
        {
            clear_after_send(); //todo remove
            debug("\r\nError sending. Response_code %d, status %d",response_code,status);
            //todo:flag not_sended enabled
           // clear_after_send();
        }
        delete [] buffer;
    #endif /* BOARD == WRIGHT_V1_0_0 */

    #if BOARD == EARHART_V1_0_0
        long done = 0;
        uint8_t port=1;
        while (done < buffer_len)
        {
            long available = buffer_len - done;
            if (available>180)
            {
                available = 180;
            }
            uint8_t *buff250= new uint8_t[available];
            debug("\r\nDone: %d, Available: %d",done,available);           
            memcpy(buff250, buffer+done, available); 
            done += available;
            response_code=sendTTN(port, buff250, available);
            if(response_code < NODEFLOW_OK)
            {
                ThisThread::sleep_for(30000);
                debug("\r\nRetrying..");
                response_code=sendTTN(port, buff250, available);
            }
            ThisThread::sleep_for(30000);
            port++;
            delete [] buff250;
        }
        // response_code=sendTTN(1, buffer, buffer_len);
        if (response_code > NODEFLOW_OK) 
        {
            delete [] buffer;
            clear_after_send();
        } 
    #endif /* #if BOARD == EARHART_V1_0_0 */
}

void NodeFlow::timetodate(uint32_t remainder_time)
{
    double_t t_value=(remainder_time/float(HOURINSEC));
    double_t minutes_f=fmod(t_value,1);
    uint8_t hours= t_value- minutes_f;
    t_value=minutes_f*MINUTEINSEC; 
    minutes_f=(fmod(t_value,1))*MINUTEINSEC;
    debug("Time(HH:MM:SS):   %02d:%02d:%02d\r\n", hours, int(t_value),int(minutes_f));
}

/**LorawanTP
 */
#if BOARD == EARHART_V1_0_0
int NodeFlow::sendTTN(uint8_t port, uint8_t payload[], uint16_t length)
{
    debug("\n-------------------SEND & RECEIVE-------------------\r\n");
    status=_radio.join(CLASS_C);
    if(status < NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FAILED TO JOIN",status,__PRETTY_FUNCTION__);
        return status;
    }
    status=_radio.send_message(port, payload, length);
    if (status < NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FAILED TO SEND",status,__PRETTY_FUNCTION__);
        return status;
    }
    uint32_t rx_message;
    uint8_t rx_port;
    receiveTTN(rx_message, rx_port); //todo: The rx window closes too soon..
    
    debug("\r\n--------------------------------------------------\r\n");
    return status;
}

int NodeFlow::receiveTTN(uint32_t& rx_message, uint8_t& rx_port)
{   uint8_t port=0;
    int retcode=0;
    uint32_t rx_dec_buffer[MAX_BUFFER_READING_TIMES];
    _radio.receive_message(rx_dec_buffer,port,retcode); 
    
    if (port==SCHEDULER_PORT)
    {
        status=overwrite_sched_config(true, DIVIDE(retcode));
        if (status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"overwrite_sched_config",status,__PRETTY_FUNCTION__);
        }

        for (int i=0; i<DIVIDE(retcode); i++)
        {
            debug("\n%i.RX scheduler: %d(10)\n",i, rx_dec_buffer[i]);
            status=append_sched_config(rx_dec_buffer[i]/2,1); //TODO: CHANGE GROUP ID- depends on data
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
            }
        }
    }
    
    if(port==CLOCK_SYNCH_ACK_PORT)
    {
        bool clockSynchOn=false;
        uint16_t time;
        status=read_clock_synch_config(time,clockSynchOn);
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
        debug("\nNo Rx available\r\n"); 
    }
    else
    {
        debug("\r\nRx: %d(10)", rx_dec_buffer[0]);
        debug("\r\nPort: %d", port);
    }
    _radio.sleep();
    rx_message=rx_dec_buffer[0];
    rx_port=port;
    
    return status;
}
#endif /* #if BOARD == EARHART_V1_0_0 */

int NodeFlow::get_wakeup_flags()
{    
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagSSCKConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagSSCKConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    bitset<8> ssck_flag(f_conf.parameters.value);
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

int NodeFlow::is_delay_pin_wakeup_flag()
{    
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagSSCKConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagSSCKConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
     if (f_conf.parameters.flag)
     {
         return NodeFlow::FLAG_WAKEUP_PIN;
     }
     return NODEFLOW_OK;
}

/** Clear whatever needed i.e increments, eeprom stuff and other
 */
int NodeFlow::clear_after_send()
{
    #if (INTERRUPT_ON)
        status= DataManager::delete_file_entries(InterruptConfig_n);
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"InterruptConfig_n",status,__PRETTY_FUNCTION__); 
        }
    #endif
    status= DataManager::delete_file_entries(MetricGroupAConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupAConfig_n",status,__PRETTY_FUNCTION__); 
    }
    #if (SCHEDULER_B || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3|| METRIC_GROUPS_ON==2)
        status= DataManager::delete_file_entries(MetricGroupBConfig_n);
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"MetricGroupBConfig_n",status,__PRETTY_FUNCTION__); 
        }
    #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */
    #if (SCHEDULER_C || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
        status= DataManager::delete_file_entries(MetricGroupCConfig_n);
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"MetricGroupCConfig_n",status,__PRETTY_FUNCTION__); 
        }
    #endif /* #if (SCHEDULER_C || METRIC_GROUPS_ON==3) */
    #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    status= DataManager::delete_file_entries(MetricGroupDConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupDConfig_n",status,__PRETTY_FUNCTION__); 
    }
    #endif /* #if (SCHEDULER_D || METRIC_GROUPS_ON==4) */

   clear_increment();
   clear_mg_counter();   
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
    ThisThread::sleep_for(1);
    sleep_manager.standby(seconds, wkup_one);
}

/** Manage errors, in case of multiple consecutives  on the same line the device will restart.
 *
 * @param line that the error occured
 * @param str1 text indication of the error
 * @param status status of the error
 * @param str2 Function name
 * 
 * @return None 
 */
void NodeFlow::ErrorHandler(int line, const char* str1, int status,const char* str2) 
{
    debug("\r\nError in line No = %d, %s,Status = %d,Function name = %s\r\n",line, str1, status, str2);
    int errCnt=0;
    bool error=false;
    error_increment(errCnt, line, error); 
    debug("\r\nErrors %d",errCnt);
    if(error)
    {
       #if BOARD == EARHART_V1_0_0
            uint8_t error[3]={5,uint8_t(line),uint8_t(status)}; 
            sendTTN(219, error, 3);
        #endif /* BOARD == EARHART_V1_0_0 */
        NVIC_SystemReset();
    }
}

/** Error increment, in case of multiple consecutives  on the same line the device will restart.
 *
 * @param line that the error occured
 * 
 * @return errCnt error counter
 * @return error boolean true or false if consecutive errors are detected
 */
int NodeFlow::error_increment(int &errCnt, uint16_t line, bool &error)
{
    errCnt=0;
    error=false;
    ErrorConfig e_conf;
    status = DataManager::read_file_entry(ErrorConfig_n, 0, e_conf.data, sizeof(e_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        errCnt=0;
        error=true;
        return 0;
    }
    error=false;
    uint16_t arr[20];
    arr[e_conf.parameters.errCnt]=line;
    for (int i = 0; i < e_conf.parameters.errCnt; i++) 
    {   
        arr[i]=e_conf.parameters.line_arr[i];
        
        if (e_conf.parameters.line_arr[i] ==  e_conf.parameters.line_arr[i+1] 
            && e_conf.parameters.line_arr[i+1] == e_conf.parameters.line_arr[i+2]
            && e_conf.parameters.errCnt>=STATUS_ERROR_TOLERANCE) 
        {   
            error=true;
        } 
    }

    for (int i=0; i<e_conf.parameters.errCnt+1; i++)
    {
            e_conf.parameters.line_arr[i]=arr[i];
    }
    e_conf.parameters.errCnt=1+e_conf.parameters.errCnt;

    if(e_conf.parameters.errCnt>19)
    {
        e_conf.parameters.errCnt=0;
    }
    status= DataManager::overwrite_file_entries(ErrorConfig_n, e_conf.data, sizeof(e_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        errCnt=0;
        error=false;
        return 0;
    }
    else
    {
        errCnt=e_conf.parameters.errCnt;
    }
    
    return 0;
}


void NodeFlow::eeprom_debug()
{
    //debug("\r\nGet_max_files : %d",DataManager::get_max_files());
    //debug("\r\nGet_storage_size_bytes : %d\r\n",DataManager::get_storage_size_bytes());

    // int valid_files;
    // DataManager::total_stored_files(valid_files);
    // debug("\r\ntotal_stored_files(valid_files) : %d\r\n",valid_files);
    debug("\r\n");
    DataManager_FileSystem::GlobalStats_t g_stats;
    DataManager::get_global_stats(g_stats.data);
    DataManager::print_global_stats(g_stats);
}

void NodeFlow::tracking_memory()
{
     // allocate enough room for every thread's stack statistics
    int cnt = osThreadGetCount();
    mbed_stats_stack_t *stats = (mbed_stats_stack_t*) malloc(cnt * sizeof(mbed_stats_stack_t));
 
    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        debug("\r\nThread: 0x%lX, Stack size: %lu / %lu", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
    }
    free(stats);
 
   // mbed_mem_trace_set_callback(mbed_mem_trace_default_callback);
    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    debug("\r\nHeap size: %lu / %lu bytes", heap_stats.current_size, heap_stats.reserved_size);
}
