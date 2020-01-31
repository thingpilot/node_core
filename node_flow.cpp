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

    if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_PIN)
    {   
        debug("\r\n--------------------PIN WAKEUP--------------------\r\n");
        HandleInterrupt(); /**Pure virtual function */

        status=get_interrupt_latency(next_time);
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

            status=get_interrupt_latency(next_time);
            if (status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"get_interrupt_latency", status,__PRETTY_FUNCTION__);
            }
        }
        else
        {   
            debug("\r\n-------------------TIMER WAKEUP-------------------\r\n");
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
        debug("\r\n                      __|__       \n               --@--@--(_)--@--@--\n-------------------THING PILOT--------------------\r\n");
        debug("\nDevice Unique ID: %08X %08X %08X \r", STM32_UID[0], STM32_UID[1], STM32_UID[2]);

        status=initialise();
        if (status != NODEFLOW_OK)
        { 
            NVIC_SystemReset(); 
        }

        {
            volatile Init_State INIT_STATE = Init_State::RUN;
            volatile Test_State TEST_STATE = Test_State::WFC;
            volatile Prov_State PROV_STATE = Prov_State::WFC;

            wait_us(500000);

            UARTSerial *_serial;
            ATCmdParser *_parser;

            _serial = new UARTSerial(TP_PC_TXU, TP_PC_RXU, 9600);
            _parser = new ATCmdParser(_serial);
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
            delete _serial;

            debug("Test over yo!\r\n");
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

        status=init_send_sched_config();
        
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"init_send_sched_config", status,__PRETTY_FUNCTION__);
        }   
        
        flags=get_flags();
        status=set_scheduler(&next_time); 
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"set_scheduler", status,__PRETTY_FUNCTION__);
        }  
         
    }
    debug("\nGoing to sleep for %d ",next_time);
    
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
        //error in error
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
    SchedulerConfig_File_t.parameters.length_bytes = sizeof(SchedulerConfig::parameters);

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
        #if BOARD == EARHART_V1_0_0
            debug("\r\nWARNING!! SEND SCHEDULER IS ON FOR EARHART BOARD\r\nVisit https://www.loratools.nl/#/airtime to find \r\nout more. Max payload size supported 255 bytes\r\n");     
        #endif
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
    status=set_flags_config(0);  

    /** IncrementAConfig
     */
    DataManager_FileSystem::File_t IncrementAConfig_File_t;
    IncrementAConfig_File_t.parameters.filename = IncrementAConfig_n;
    IncrementAConfig_File_t.parameters.length_bytes = sizeof(IncrementAConfig::parameters);
    status=DataManager::add_file(IncrementAConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    IncrementAConfig i_conf;
    i_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);
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
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }
    IncrementBConfig ib_conf;
    ib_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementBConfig_n, ib_conf.data, sizeof(ib_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__);
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
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__);
        return status;   
    }

    IncrementCConfig ic_conf;
    ic_conf.parameters.increment=0;

    status= DataManager::append_file_entry(IncrementCConfig_n, ic_conf.data, sizeof(ic_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementCConfig",status,__PRETTY_FUNCTION__);
        return status; 
    }

    //TODO: make sure the buffer does not overflow
    /** CounterConfig
    */
    // DataManager_FileSystem::File_t CounterConfig_File_t;
    // CounterConfig_File_t.parameters.filename = CounterConfig_n;
    // CounterConfig_File_t.parameters.length_bytes = sizeof(CounterConfig::parameters);

    // status=DataManager::add_file(CounterConfig_File_t, 1);
    // if(status != NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"CounterConfig",status,__PRETTY_FUNCTION__);
    //     return status;   
    // }

    // CounterConfig c_conf;
    // c_conf.parameters.counter=0;

    // status= DataManager::append_file_entry(CounterConfig_n, c_conf.data, sizeof(c_conf.parameters));
    // if(status != NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"CounterConfig",status,__PRETTY_FUNCTION__);
    //     return status; 
    // }


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
  
    DataManager_FileSystem::File_t MetricGroupAConfig_File_t;
    MetricGroupAConfig_File_t.parameters.filename = MetricGroupAConfig_n;
    MetricGroupAConfig_File_t.parameters.length_bytes = sizeof(MetricGroupAConfig::parameters);

    status = DataManager::add_file(MetricGroupAConfig_File_t, 1600/METRIC_GROUPS_ON); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupAConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    #if (SCHEDULER_B || METRIC_GROUPS_ON==2)
    DataManager_FileSystem::File_t MetricGroupBConfig_File_t;
    MetricGroupBConfig_File_t.parameters.filename = MetricGroupBConfig_n;
    MetricGroupBConfig_File_t.parameters.length_bytes = sizeof(MetricGroupBConfig::parameters);

    status = DataManager::add_file(MetricGroupBConfig_File_t, 1600/METRIC_GROUPS_ON); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupBConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    #endif

    #if (SCHEDULER_C || METRIC_GROUPS_ON==3)
    DataManager_FileSystem::File_t MetricGroupCConfig_File_t;
    MetricGroupCConfig_File_t.parameters.filename = MetricGroupCConfig_n;
    MetricGroupCConfig_File_t.parameters.length_bytes = sizeof(MetricGroupCConfig::parameters);

    status = DataManager::add_file(MetricGroupCConfig_File_t,1600/METRIC_GROUPS_ON); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupCConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    #endif 
     #if (SCHEDULER_D|| METRIC_GROUPS_ON==4)
    DataManager_FileSystem::File_t MetricGroupDConfig_File_t;
    MetricGroupDConfig_File_t.parameters.filename = MetricGroupDConfig_n;
    MetricGroupDConfig_File_t.parameters.length_bytes = sizeof(MetricGroupDConfig::parameters);

    status = DataManager::add_file(MetricGroupDConfig_File_t,1600/METRIC_GROUPS_ON); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupDConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    #endif 
    
    DataManager_FileSystem::File_t MetricGroupEntriesConfig_File_t;
    MetricGroupEntriesConfig_File_t.parameters.filename =MetricGroupEntriesConfig_n;
    MetricGroupEntriesConfig_File_t.parameters.length_bytes = sizeof(MetricGroupEntriesConfig::parameters);

    status = DataManager::add_file(MetricGroupEntriesConfig_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupEntriesConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    clear_mg_counter();

    return status;
}

int NodeFlow::HandleModem()
{   
    tformatter.setup();
    if(flags==NodeFlow::FLAG_SENSING || flags==NodeFlow::FLAG_SENSE_SEND ||
        flags==NodeFlow::FLAG_SENSE_SEND_SYNCH || flags==NodeFlow::FLAG_SENSE_SYNCH)
    {
        int metric_group_flag=0;
        uint16_t sched_length;
        status=read_sched_config(1,&sched_length);
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"Read_sched_config",status,__PRETTY_FUNCTION__);
            return status;
        }
        if (sched_length>1)
        {   
            uint8_t mg_flag;
            get_metric_flags(mg_flag);
            bitset<8> flags(mg_flag);
            debug("MGroupA: %d, MGroupB: %d, MGroupC: %d, MGroupC: %d \r\n",flags.test(0),flags.test(1),flags.test(2),flags.test(3));
            #if(!SEND_SCHEDULER)
                add_sensing_entry(mg_flag,1);
            #endif
    
            if(flags.test(0)==1)
            {
                MetricGroupA();
                metric_group_flag=1;
                mg_counter(metric_group_flag);
                #if(SEND_SCHEDULER)
                    add_cbor_payload_data(metric_group_flag);
                #endif
            }

            if(flags.test(1)==1)
            {
                MetricGroupB();
                metric_group_flag=2;
                mg_counter(metric_group_flag);
                #if(SEND_SCHEDULER)
                    add_cbor_payload_data(metric_group_flag);
                #endif
            }
            if(flags.test(2)==1)
            {   
                MetricGroupC();
                metric_group_flag=3;
                mg_counter(metric_group_flag);
                #if(SEND_SCHEDULER)
                    metric_group_flag=3;
                    add_cbor_payload_data(metric_group_flag);
                #endif
            }
            if(flags.test(3)==1)
            {
                
                MetricGroupD();
                metric_group_flag=4;
                mg_counter(metric_group_flag);
                #if(SEND_SCHEDULER)
                    add_cbor_payload_data(metric_group_flag);
                #endif
            }
        }
        else
        {
            #if(!SEND_SCHEDULER)
                add_sensing_entry(1,1);
            #endif
            MetricGroupA();
            metric_group_flag=1;
            mg_counter(metric_group_flag);
            #if(SEND_SCHEDULER)
                add_cbor_payload_data(metric_group_flag);
            #endif
        }

    }

    if(flags==NodeFlow::FLAG_SENDING || flags==NodeFlow::FLAG_SENSE_SEND ||
        flags==NodeFlow::FLAG_SEND_SYNCH || flags==NodeFlow::FLAG_SENSE_SEND_SYNCH)
    { 
        uint16_t mga_entries, mgb_entries, mgc_entries, mgd_entries;
        int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes;
        uint8_t metric_group_active=0;
        #if BOARD == EARHART_V1_0_0 
            uint8_t buffer[255];
        #endif
        #if BOARD == WRIGHT_V1_0_0
            uint8_t buffer[1600];
        #endif
        size_t buffer_len=0;
    
        read_mg_counter(mga_entries, mgb_entries, mgc_entries, mgd_entries, metric_group_active);
        read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes); 
        debug("ENTRIES = MGA: %d, MGB: %d, MGC: %d, MGD: %d\r\n",mga_entries, mgb_entries, mgc_entries, mgd_entries);
        debug("BYTES = MGA: %d, MGB: %d, MGC: %d, MGD: %d\r\n",mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes);

        uint32_t total_bytes=mga_bytes+ mgb_bytes+ mgc_bytes+ mgd_bytes;
        uint8_t payload[total_bytes];
        debug("Buffer(LSB)  = %d\r\n",total_bytes);     

        #if(SEND_SCHEDULER)
            tformatter.serialise_main_cbor_object();
            tformatter.write(metric_group_active, TFormatter::ARRAY);
        #endif

        if (mga_entries!=0)
        {
            #if(SEND_SCHEDULER)
                tformatter.write(1, TFormatter::GROUP_TAG); 
                tformatter.write(mga_entries, TFormatter::ARRAY);
            #endif

            for (int i=0; i<mga_bytes; i++) 
            {   
                MetricGroupAConfig a_conf;
                status = DataManager::read_file_entry(MetricGroupAConfig_n, i, a_conf.data, sizeof(a_conf.parameters));
                debug(" %02x", a_conf.parameters.byte);   
                if( i%14 == 0 && i!=0 ) //TODO: REMOVE
                {
                    debug("\n");
                }
                #if(SEND_SCHEDULER)
                    tformatter.write(a_conf.parameters.byte, TFormatter::RAW);
                #endif
                #if(!SEND_SCHEDULER)
                    buffer[buffer_len]=a_conf.parameters.byte;
                    buffer_len++;
                #endif
            }
        }
        if (mgb_entries!=0)
        {   
            #if(SEND_SCHEDULER)
                tformatter.write(2, TFormatter::GROUP_TAG); 
                tformatter.write(mgb_entries, TFormatter::ARRAY);
            #endif
            #if (SCHEDULER_B || METRIC_GROUPS_ON==2)
                for (int i=0; i<mgb_bytes; i++) 
                {    
                    MetricGroupBConfig b_conf;
                    status = DataManager::read_file_entry(MetricGroupBConfig_n, i, b_conf.data, sizeof(b_conf.parameters));
                    #if(SEND_SCHEDULER)
                        tformatter.write(b_conf.parameters.byte, TFormatter::RAW);
                    #endif
                    #if(!SEND_SCHEDULER)
                        buffer[buffer_len]=b_conf.parameters.byte;
                        buffer_len++;
                    #endif
                }
            #endif
        }
        if (mgc_entries!=0)
        {   
            #if(SEND_SCHEDULER)
                tformatter.write(3, TFormatter::GROUP_TAG);
                tformatter.write(mgc_entries, TFormatter::ARRAY);
            #endif
            #if (SCHEDULER_C || METRIC_GROUPS_ON==3)
                for (int i=0; i<mgc_bytes; i++) 
                {    
                    MetricGroupCConfig c_conf;
                    status = DataManager::read_file_entry(MetricGroupCConfig_n, i, c_conf.data, sizeof(c_conf.parameters));
                    #if(SEND_SCHEDULER)
                        tformatter.write(c_conf.parameters.byte, TFormatter::RAW);
                    #endif
                    #if(!SEND_SCHEDULER)
                        buffer[buffer_len]=c_conf.parameters.byte;
                        buffer_len++;
                    #endif
                
                }
            #endif
        }
         if (mgd_entries!=0)
        {   
            #if(SEND_SCHEDULER)
                tformatter.write(4, TFormatter::GROUP_TAG); 
                tformatter.write(mgd_entries, TFormatter::ARRAY);
            #endif
            #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
            for (int i=0; i<mgd_bytes; i++) 
            {    
                MetricGroupDConfig d_conf;
                status = DataManager::read_file_entry(MetricGroupDConfig_n, i, d_conf.data, sizeof(d_conf.parameters));
                #if(SEND_SCHEDULER)
                    tformatter.write(d_conf.parameters.byte, TFormatter::RAW);
                #endif
                #if(!SEND_SCHEDULER)
                    buffer[buffer_len]=d_conf.parameters.byte;
                    buffer_len++;
                #endif
            }
            #endif
        }

        #if(SEND_SCHEDULER)
            tformatter.get_final_serialised(buffer, buffer_len);
        #endif
        printf("\r\n");
        int response_code=-1;
        #if BOARD == WRIGHT_V1_0_0
            int rsrp=0;
            int rsrq=0;
            _radio.get_csq(rsrp,rsrq);
            debug("NBIOT last know RSRP %d and RSRQ %d",rsrp, rsrq);
            char recv_data[512];
            char nbiot[buffer_len];
            memcpy(nbiot,buffer,buffer_len);
            status=_radio.coap_post(nbiot, recv_data, SaraN2::TEXT_PLAIN, response_code); 
            if (response_code != 0 || response_code != 2 ) 
            {
                debug("Error sending. Response_code %d",response_code);
                //TODO: HANDLE NOT SENDING
            } 
            else
            {
                _clear_after_send();
            }
        #endif

        #if BOARD == EARHART_V1_0_0
            response_code=sendTTN(1, buffer, buffer_len);
            if (response_code > NODEFLOW_OK) 
            {
                _clear_after_send();
            } 
        #endif     
    }

    return NODEFLOW_OK;
}


template <typename DataType> 
void NodeFlow::add_record(DataType data, string str1)
{   
    #if(SEND_SCHEDULER)
    tformatter.write_string(str1);
    tformatter.write_num_type<DataType>(data);
    #endif

    #if(!SEND_SCHEDULER)
    // uint8_t mg_flag;
    // get_metric_flags(mg_flag);

    uint8_t bytes[sizeof(DataType)];
    *(DataType *)(bytes)=data;

    debug("Bytes = ");
   
    for (int i=sizeof(DataType); i>0; i--)
    {
        debug("[ 0x%.2x]", bytes[i-1]);
        add_sensing_entry(bytes[i-1],1);
    }
    debug("\r\n");
    #endif
}

template void NodeFlow::add_record<float>(float data, string str1);
template void NodeFlow::add_record<int>(int data, string str1);
template void NodeFlow::add_record<int8_t>(int8_t data, string str1);
template void NodeFlow::add_record<int16_t>(int16_t data, string str1);
template void NodeFlow::add_record<int64_t>(int64_t data, string str1);
template void NodeFlow::add_record<uint8_t>(uint8_t data, string str1);
template void NodeFlow::add_record<uint16_t>(uint16_t data, string str1);
template void NodeFlow::add_record<uint32_t>(uint32_t data, string str1);
template void NodeFlow::add_record<uint64_t>(uint64_t data, string str1);



int NodeFlow::add_cbor_payload_data(uint8_t metric_group_flag) 
{
    uint8_t buffer[100];
    size_t buffer_len=0;
    tformatter.get_serialised(buffer, buffer_len);
    debug("Entries: %d\r\n",buffer_len);
    for (int i=0; i<buffer_len; i++)
    {
        add_sensing_entry(buffer[i],metric_group_flag);
    } 
    is_overflow(metric_group_flag);

    return NODEFLOW_OK;
}

int NodeFlow::add_sensing_entry(uint8_t value, uint8_t metric_group)
{
    if(metric_group==1)
    {
        MetricGroupAConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(MetricGroupAConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {   //TODO: I have many errors here verify on eeprom status 6
            status= DataManager::append_file_entry(MetricGroupAConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"MetricGroupAConfig",status,__PRETTY_FUNCTION__);
            }
        }
    }
    
    if(metric_group==2)
    {    
        #if (SCHEDULER_B || METRIC_GROUPS_ON==2)
        MetricGroupBConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(MetricGroupBConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {   
            status= DataManager::append_file_entry(MetricGroupBConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"MetricGroupBConfig",status,__PRETTY_FUNCTION__);
            }
        }
        #endif
    }
    if(metric_group==3)
    {
        #if (SCHEDULER_C || METRIC_GROUPS_ON==3)
        MetricGroupCConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(MetricGroupCConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {   //TODO: I have many errors here verify on eeprom status 6
            status= DataManager::append_file_entry(MetricGroupCConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"MetricGroupCConfig",status,__PRETTY_FUNCTION__);
            }
        }
        #endif
    }
    if(metric_group==4)
    {   
        #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
        MetricGroupDConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(MetricGroupDConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {   
            status= DataManager::append_file_entry(MetricGroupDConfig_n, t_conf.data, sizeof(t_conf.parameters));
            if(status != NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"MetricGroupDConfig",status,__PRETTY_FUNCTION__);
            }
        }
        #endif
    }
    
    return status;
    
}

/** How the user will erase the value?! daily, after sending?  TODO:ASK CAUSE THIS IS A DIFF INCREMENT
 */
int NodeFlow::read_inc_a(uint16_t& increment_value)
{
    IncrementAConfig i_conf;
    status = DataManager::read_file_entry(IncrementAConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
    }
    increment_value=i_conf.parameters.increment;

    return status;
}

int NodeFlow::inc_a(int i)
{
    uint16_t increment_value;
    status=read_inc_a(increment_value);
    if (status == NODEFLOW_OK)
    {
        IncrementAConfig i_conf;
        i_conf.parameters.increment=i+increment_value;
        status= DataManager::overwrite_file_entries(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__); 
        }
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

int NodeFlow::inc_b(int i)
{
    uint16_t increment_value;
    status=read_inc_b(increment_value);
    if (status == NODEFLOW_OK)
    {
        IncrementBConfig i_conf;
        i_conf.parameters.increment=i+increment_value;
        status= DataManager::overwrite_file_entries(IncrementBConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status!=NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__); 
        }
    }
    return status;
}


int NodeFlow::read_inc_c(uint32_t& increment_value)
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
    uint32_t increment_value;
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


// int NodeFlow::counter(uint8_t& entries_counter)
// {
//     CounterConfig i_conf;
//     status = DataManager::read_file_entry(CounterConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
//     if (status == NODEFLOW_OK)
//     {
//         i_conf.parameters.counter=i_conf.parameters.counter+1;
//         status= DataManager::overwrite_file_entries(CounterConfig_n, i_conf.data, sizeof(i_conf.parameters));
//         if (status!=NODEFLOW_OK)
//         {
//             ErrorHandler(__LINE__,"EntriesCounterConfig",status,__PRETTY_FUNCTION__); 
//         }
//     }
//     entries_counter=i_conf.parameters.counter;
//     return status;

// }
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

void NodeFlow::is_overflow(uint8_t mg_group)
{
    int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes;
    uint16_t mga_entries, mgb_entries, mgc_entries, mgd_entries;
    uint8_t metric_group_active;
    read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes);
    read_mg_counter(mga_entries, mgb_entries, mgc_entries, mgd_entries, metric_group_active);
    uint16_t total_bytes=mga_bytes+ mgb_bytes+ mgc_bytes+ mgd_bytes;
    uint16_t total_entries=mga_entries + mgb_entries + mgc_entries + mgd_entries;

    uint16_t c_entries=0;
    tformatter.get_entries(c_entries);

    #if BOARD == EARHART_V1_0_0
    debug("\nPredicted bytes %d\n",(total_bytes+(total_bytes/total_entries)+53));
    if((total_bytes+(total_bytes/total_entries)+53)>180)
    {
        debug("255 bytes is the maximum- SENT DATA NOW\r\n");
        flags=FLAG_SENDING; 
    }
    #endif
    debug("\nPredicted bytes %d\n",total_bytes+(c_entries+9)*2);
    if(total_bytes+(2*(c_entries+9))>1500)
    {
        debug("IN THE NEXT ENTRY THE MEMORY WILL BE FULL- SENT DATA NOW\r\n");
        flags=NodeFlow::FLAG_SENDING; 
    }
}

int NodeFlow::read_mg_bytes(int& mga_bytes, int& mgb_bytes, int& mgc_bytes,int& mgd_bytes)
{
    mga_bytes=0;
    mgb_bytes=0;
    mgc_bytes=0;
    mgd_bytes=0;
    status= DataManager::get_total_written_file_entries(MetricGroupAConfig_n, mga_bytes);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        return status;
    }
    #if (SCHEDULER_B || METRIC_GROUPS_ON==2)
    status= DataManager::get_total_written_file_entries(MetricGroupBConfig_n, mgb_bytes);
    if(status != NODEFLOW_OK)
    {
        debug("GET TOTAL MG");
        ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        return status;
    }
    #endif
    #if (SCHEDULER_C || METRIC_GROUPS_ON==3)
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

int NodeFlow::mg_counter(uint8_t mg_flag) //TODO: 
{
    MetricGroupEntriesConfig i_conf;
    status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    if (status == NODEFLOW_OK)
    {
        if (mg_flag==1)
        {
            i_conf.parameters.MetricGroupAEntries=i_conf.parameters.MetricGroupAEntries+1;
        }
         if (mg_flag==2)
        {
            i_conf.parameters.MetricGroupBEntries=i_conf.parameters.MetricGroupBEntries+1;
        }
         if (mg_flag==3)
        {
            i_conf.parameters.MetricGroupCEntries=i_conf.parameters.MetricGroupCEntries+1;
        }
         if (mg_flag==4)
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
    status= DataManager::overwrite_file_entries(MetricGroupEntriesConfig_n, mge_conf.data, sizeof(mge_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__); 
    }
    return NODEFLOW_OK;
}
//todo: do i need that?
// int NodeFlow::read_counter(uint8_t &entries_counter)
// {
//     CounterConfig i_conf;
//     status = DataManager::read_file_entry(CounterConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
//     if (status != NODEFLOW_OK)
//     {
//         ErrorHandler(__LINE__,"EntriesCounterConfig",status,__PRETTY_FUNCTION__); 
//     }
//     entries_counter=i_conf.parameters.counter;
//     return status;
// }
//TODO: Now is being erased every time it sends but there is a chance that this is not registered in eproom and we deleted it
//There are two ways either delete when stored- or leave it on the user when to clear it
int NodeFlow::_clear_increment()
{
    IncrementAConfig i_conf;
    i_conf.parameters.increment=0;
   
    status= DataManager::overwrite_file_entries(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
    }

    IncrementBConfig ib_conf;
    ib_conf.parameters.increment=0;
   
    status= DataManager::overwrite_file_entries(IncrementBConfig_n, ib_conf.data, sizeof(ib_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementBConfig",status,__PRETTY_FUNCTION__);   
    }

    IncrementCConfig ic_conf;
    ic_conf.parameters.increment=0;
   
    status= DataManager::overwrite_file_entries(IncrementCConfig_n, ic_conf.data, sizeof(ic_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementCConfig",status,__PRETTY_FUNCTION__);   
    }
    // todo:need?
    // CounterConfig c_conf;
    // c_conf.parameters.counter=0;
   
    // status= DataManager::overwrite_file_entries(CounterConfig_n, c_conf.data, sizeof(c_conf.parameters));
    // if (status!=NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"CounterConfig",status,__PRETTY_FUNCTION__);   
    // }

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
            debug("\nWARNING!! Scheduler size too big,\nonly 1 interval time is associated with each metric group\n");
        }   
    #endif
   
    status=overwrite_sched_config(SCHEDULER,SCHEDULER_SIZE);
    if (status != NODEFLOW_OK)
    {
        status=overwrite_sched_config(SCHEDULER,SCHEDULER_SIZE);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"overwrite_sched_config",status,__PRETTY_FUNCTION__);
            return status;
        }
    }
    uint16_t schedulerOn;
    status=read_sched_config(0,&schedulerOn);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);  
    }
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
    if(!schedulerOn)
    {
        uint16_t sch_length;
        status=read_sched_config(1,&sch_length);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);  
        }
        
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

        #if(!SEND_SCHEDULER)
            status=overwrite_send_sched_config(SEND_SCHEDULER,0);
        #endif

        #if(SEND_SCHEDULER)
        status=overwrite_send_sched_config(SEND_SCHEDULER,SEND_SCHEDULER_SIZE);
        debug("\r\n---------------ADD SENDING TIMES------------------\r\n");
        
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
                        time_remainder=DIVIDE(((int(send_scheduler[i]))*HOURINSEC)+((fmod(send_scheduler[i],1))*6000));
                    #endif
                    status=append_send_sched_config(time_remainder);
                    if(status != NODEFLOW_OK)
                    {
                        ErrorHandler(__LINE__,"append_send_sched_config",status,__PRETTY_FUNCTION__);
                        return status;
                    }
                    debug("%d. Sending ",i);
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
int NodeFlow::add_sensing_groups() 
{   
    debug("\r\n---------------ADD METRIC GROUPS------------------\r\n");
    uint16_t sch_length;
    status=read_sched_config(1,&sch_length);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"Read_sched_config",status,__PRETTY_FUNCTION__);  
    }
    sensor_config_init(sch_length);
    for (int i=0; i<sch_length; i++)
    {
        SensingGroupConfig sg_conf;
        TempSensingGroupConfig ts_conf;
        sg_conf.parameters.time_comparator=scheduler[i];
        ts_conf.parameters.time_comparator=scheduler[i];

        if(i == 0)
        {
            status= DataManager::overwrite_file_entries(SensingGroupConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status!=0)
            {
                ErrorHandler(__LINE__,"SensingGroupConfig",status,__PRETTY_FUNCTION__); 
                return status;
            }
            status= DataManager::overwrite_file_entries(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
            if(status!=0)
            {
                ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__); 
                return status;
            }
        }
        else
        {
            status=DataManager::append_file_entry(SensingGroupConfig_n, sg_conf.data, sizeof(sg_conf.parameters));
            if(status!=0)
            {
                ErrorHandler(__LINE__,"SensingGroupConfig",status,__PRETTY_FUNCTION__); 
                return status;
            }

        
            status = DataManager::append_file_entry(TempSensingGroupConfig_n, ts_conf.data, sizeof(ts_conf.parameters));
            if(status!=NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__); 
                return status;
            }
        }
        
        status = DataManager::read_file_entry(TempSensingGroupConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__); 
            return status;
        }
        debug("%d. Sensing group id: %i, wake up every: %u Seconds\r\n",i,i,ts_conf.parameters.time_comparator);
                
        }
     
    debug("--------------------------------------------------\r\n");
   
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
    debug("\r\n-----------------NEXT READING TIME----------------");
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
        debug("Group id:  %d\r\n",group_id);
        debug("Next Sensing ");
        timetodate(next_sch_time);
       
    }
   
    else if(!schedulerOn)
    {
       set_reading_time(&timediff_temp);  
    } 
    ssck_flag.set(0);

     #if(!SEND_SCHEDULER)
    //#if BOARD == EARHART_V1_0_0
        ssck_flag.set(1);
    #endif

    if(sendschedulerOn)
    {
        #if(SEND_SCHEDULER)
       // #if BOARD == WRIGHT_V1_0_0
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
            debug("Next Sending ");
            timetodate(next_sch_time);
            
            if(timediff_temp_send<=timediff_temp)
            {
            
                ssck_flag.set(1);
                if(timediff_temp_send<timediff_temp)
                {
                    timediff_temp=timediff_temp_send;
                    ssck_flag.reset(0);
                } 
            }
        #endif
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
        debug("Next ClkSync ");
        timetodate(2*time);
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
    mybit_int = int(ssck_flag.to_ulong());

    debug("\r\nSense flag:%d, Send flag:%d, Clock:%d, Kick flag:%d\n", ssck_flag.test(0), ssck_flag.test(1), ssck_flag.test(2), ssck_flag.test(3));
    ThisThread::sleep_for(100);
    set_flags_config(mybit_int);
    overwrite_wakeup_timestamp(timediff_temp); 
    if(!schedulerOn)
    {
        set_temp_reading_times(timediff_temp);
    }
    *next_timediff=timediff_temp;
    return NODEFLOW_OK;   
}


int NodeFlow::set_reading_time(uint32_t* time)
{ 
    TempSensingGroupConfig ts_conf;
    TimeConfig t_conf;
    SensingGroupConfig sg_conf;
    uint16_t sch_length;
    status=read_sched_config(1,&sch_length);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);  
    }
    uint16_t temp_time[sch_length];
    uint8_t mybit_int;
    bitset<8> flags(0b0000'0000);
    int time_comparator;

    status = DataManager::read_file_entry(TimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TimeConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    time_comparator=t_conf.parameters.time_comparator; 
    
    status = DataManager::read_file_entry(TempSensingGroupConfig_n, 0, ts_conf.data, sizeof(ts_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
        return status;
    }

    int temp=ts_conf.parameters.time_comparator; 

    //todo: change scheduler size 
    for (int i=0; i<sch_length; i++)
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
    
    debug("\r\nNext Reading ");
    timetodate(time_comparator+time_now());
    debug("GroupA: %d, GroupB: %d, GroupC: %d, GroupD: %d\n", flags.test(0), flags.test(1), flags.test(2), flags.test(3));
    *time=time_comparator;

    return status;
}
int NodeFlow::set_temp_reading_times(uint16_t time)
{
    TempSensingGroupConfig ts_conf;
    SensingGroupConfig sg_conf;
    uint16_t sch_length;
    status=read_sched_config(1,&sch_length);
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"read_sched_config",status,__PRETTY_FUNCTION__);  
    }
    uint16_t temp_time[sch_length];

    status=set_time_config(time);
    for(int i=0; i<sch_length; i++)
    {
        status=DataManager::read_file_entry(TempSensingGroupConfig_n, i, ts_conf.data, sizeof(ts_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempSensingGroupConfig",status,__PRETTY_FUNCTION__);
            return status;
        }
        int time_comp=ts_conf.parameters.time_comparator-time;
        temp_time[i]=time_comp;
       
       
        if(temp_time[i] == 0)
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

    for(int i=0; i<sch_length; i++)
    {
        debug("%d. Sensing group id: %i, next reading: %u seconds\r\n",i,i,temp_time[i]);
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

int NodeFlow:: get_metric_flags(uint8_t &flag)
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
    
    uint32_t temp=0;
    next_sch_time=t_conf.parameters.time_comparator-time_now();
    
    return status;
}

int NodeFlow::overwrite_wakeup_timestamp(uint16_t time_remainder){
    
    NextTimeConfig t_conf;
    t_conf.parameters.time_comparator=time_now()+time_remainder;
    status=DataManager::overwrite_file_entries( NextTimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK)
    {   //todo: lot of errors here status 6
        status=DataManager::overwrite_file_entries(NextTimeConfig_n, t_conf.data, sizeof(t_conf.parameters));
        if (status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"NextTimeConfig",status,__PRETTY_FUNCTION__);
        }
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
        debug("Horrayy,setting the time, bear with me\r\nRetries are set to %d\r\n",MAX_RETRY_CLOCK_SYNCH);
        retcode=_radio.send_message(223, dummy, 1);
        if(retcode<=0)
        {
            ErrorHandler(__LINE__,"FAILED TO END",retcode,__PRETTY_FUNCTION__);
            return retcode;
        }
        _radio.receive_message(rx_dec_buffer,&port,&retcode);
        port=0;
        ThisThread::sleep_for(1000);
         
        for( int i=0; ((port!=CLOCK_SYNCH_PORT) && (i<MAX_RETRY_CLOCK_SYNCH)); i++) 
        {
            debug("%i. Waiting for a server message dude \r\n",i);
            ThisThread::sleep_for(5000);
            retcode=_radio.send_message(223, dummy,1);
            if(status<0)
            {
                ErrorHandler(__LINE__,"FAILED TO SEND",retcode,__PRETTY_FUNCTION__);
            }
            int count=0;
            _radio.receive_message(rx_dec_buffer,&port,&retcode);

            if(port == CLOCK_SYNCH_PORT && retcode>0)
            {
                unix_time=rx_dec_buffer[0];
            }
        }
    
    #endif /* BOARD == EARHART_V1_0_0 */

    #if BOARD == WRIGHT_V1_0_0
       // _radio.get_unix_time(&unix_time);
    #endif

    time_t time_now=time(NULL);
    if (unix_time>time_now)
    {
        debug("Received value: %d\r\n",unix_time);
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
    debug("Time(HH:MM:SS):   %02d:%02d:%02d\r\n", hours, int(minutes),int(seconds));
    return NODEFLOW_OK;
}

/**LorawanTP
 */
#if BOARD == EARHART_V1_0_0

int NodeFlow::sendTTN(uint8_t port, uint8_t payload[], uint16_t length)
{
    status=_radio.join(CLASS_C);
    if(status < NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FAILED TO JOIN",status,__PRETTY_FUNCTION__);
        return status;
    }
    debug("\n---------------------SENDING----------------------\r\n");
    timetodate(time_now());
    
    status=_radio.send_message(port, payload, length);
    if (status < NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FAILED TO SEND",status,__PRETTY_FUNCTION__);
        return status;
    }
   
    debug("\r\nSuccesfully sending %d bytes",status);
    
    debug("\r\n--------------------------------------------------\r\n");
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
            debug("%i.RX scheduler: %d(10)\r\n",i, rx_dec_buffer[i]);
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
        debug("No Rx available\r\n"); 
    }
    else
    {
        debug("Rx: %d(10)\r\n", rx_dec_buffer[0]);
        debug("Port: %d\r\n", port);
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
    MetricGroupAConfig sd_conf;
    sd_conf.parameters.byte=0;
    status= DataManager::delete_file_entries(MetricGroupAConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupAConfig",status,__PRETTY_FUNCTION__); 
    }
    #if (SCHEDULER_B || METRIC_GROUPS_ON==2)
    status= DataManager::delete_file_entries(MetricGroupBConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupBConfig",status,__PRETTY_FUNCTION__); 
    }
    #endif
    #if (SCHEDULER_C || METRIC_GROUPS_ON==3)
    status= DataManager::delete_file_entries(MetricGroupCConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupCConfig",status,__PRETTY_FUNCTION__); 
    }
    #endif
    #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    status= DataManager::delete_file_entries(MetricGroupDConfig_n);
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupDConfig",status,__PRETTY_FUNCTION__); 
    }
    #endif

   _clear_increment();
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
      //  int retcode=_radio.sleep();
    #endif /* BOARD == EARHART_V1_0_0 */

    //Without this delay it breaks..?!
    ThisThread::sleep_for(1);
    sleep_manager.standby(seconds, wkup_one);
}
void NodeFlow::ErrorHandler(int line, const char* str1, int status,const char* str2) 
{
   debug("Error in line No = %d, %s,Status = %d,Function name = %s\r\n",line, str1, status, str2);
   //check for concecutive line and status errors error reporting
   int errCnt=0;
   bool error=false;
   error_increment(errCnt, line, error); 
   debug("Errors %d\r\n",errCnt);
   if(error)
   {
        #if BOARD == EARHART_V1_0_0
            uint8_t error[2]={uint8_t(line),uint8_t(status)};
            sendTTN(219, error, 2);
        #endif 
        NVIC_SystemReset();
   }
}

void NodeFlow::error_increment(int &errCnt, uint16_t line, bool &error) //, bool error
{
    ErrorConfig e_conf;
    status = DataManager::read_file_entry(ErrorConfig_n, 0, e_conf.data, sizeof(e_conf.parameters));
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
        ErrorHandler(__LINE__,"ErrorConfig",status,__PRETTY_FUNCTION__); 
    }

   errCnt=e_conf.parameters.errCnt;

}



