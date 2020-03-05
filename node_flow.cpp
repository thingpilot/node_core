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
                increase_mg_entries_counter(0);
                add_payload_data(0);
                #if(!SEND_SCHEDULER)
                    _send();
                #endif /* #if(!SEND_SCHEDULER) */
            }
            #if(SEND_SCHEDULER)
                if(upload_flag)
                {
                    _send();
                }
            #endif /* #if(SEND_SCHEDULER) */
            is_overflow();
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
                is_overflow();
                _sense();
            }
            if(wakeup_flag==NodeFlow::FLAG_SENDING || wakeup_flag==NodeFlow::FLAG_SENSE_SEND ||
                wakeup_flag==NodeFlow::FLAG_SEND_SYNCH || wakeup_flag==NodeFlow::FLAG_SENSE_SEND_SYNCH)
            { 
                _send();
            }

            time_t end_time=time_now();
            int latency=end_time-start_time;
            set_scheduler(latency, next_time);

            //todo: check this because the latency changes if you get a timestamp 
            if(wakeup_flag == NodeFlow::FLAG_CLOCK_SYNCH || wakeup_flag == NodeFlow::FLAG_SENSE_SYNCH 
                ||wakeup_flag == NodeFlow::FLAG_SEND_SYNCH || wakeup_flag== NodeFlow::FLAG_SENSE_SEND_SYNCH)
            {
                get_timestamp();
                set_scheduler(0, next_time);
            }
            timetodate(time_now());
           
        }
    }
    else if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_RESET || wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_SOFTWARE) 
    {
        bool initialised = false;
        #if BOARD == EARHART_V1_0_0
            DigitalIn btn(PA_8);
        #endif
        #if BOARD == WRIGHT_V1_0_0
            DigitalIn btn(PB_0);
        #endif
        status=DataManager::is_initialised(initialised);
        if(btn.read() || !initialised)
        {
            initialised=false;
            while(!initialised)
            {   
                set_time(0);
                status=initialise();
                DataManager::is_initialised(initialised);
            }
        }

        if (status != NODEFLOW_OK)
        { 
            NVIC_SystemReset(); 
        }
        status=DataManager::init_gstats();
                
        _test_provision();
    
        debug("\r\n                      __|__\n               --+--+--(_)--+--+--\n-------------------THING PILOT--------------------\r\n");
        debug("\nDevice Unique ID: %08X %08X %08X", STM32_UID[0], STM32_UID[1], STM32_UID[2]);
        #if BOARD == WRIGHT_V1_0_0
            initialise_nbiot();
        #endif /* #if BOARD == WRIGHT_V1_0_0 */
        debug("\r\n----------------------SETUP-----------------------");
        setup(); /** Pure virtual by the user */
        if(CLOCK_SYNCH) 
        {
            get_timestamp();
            timetodate(time_now());
        }
        
        start_time=time_now();
        init_sched_config();
        init_send_sched_config();
        set_scheduler(0,next_time); 
    }

    debug("\nGoing to sleep for %d s.",next_time);
    timetodate(next_time+time_now());
    
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


void NodeFlow::_test_provision()
{
    wait_us(300000);
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

/** Initialise the EEPROM
 * @return Status
 */
int NodeFlow::initialise()
{    
    DigitalOut buzzer(TP_SPI_NSS); //todo: remove specific to the app
    buzzer=1;
    ThisThread::sleep_for(50);
    buzzer=0;

    status=DataManager::init_filesystem();
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    status=DataManager::init_gstats();

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
        return status;    
    }

    /**Device specifics */
    DataManager_FileSystem::File_t DeviceConfig_File_t;
    DeviceConfig_File_t.parameters.filename = DeviceConfig_n;
    DeviceConfig_File_t.parameters.length_bytes = sizeof(DeviceConfig::parameters);
    status=DataManager::add_file(DeviceConfig_File_t, 1); 
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    #if BOARD == EARHART_V1_0_0 || BOARD == DEVELOPMENT_BOARD_V1_1_0 /* #endif at EoF */
    DeviceConfig dev_conf;
    
    #if(OVER_THE_AIR_ACTIVATION)
        dev_conf.parameters.otaa=0;
        for(int i=0; i<8; i++)
        {
            dev_conf.parameters.device_eui[i]=DevEUI[i];
            dev_conf.parameters.application_eui[i]=AppEUI[i];
        }
        for(int i=0; i<16; i++)
        {
            dev_conf.parameters.application_key[i]=AppKey[i];
        }

   #endif
    #if(!OVER_THE_AIR_ACTIVATION)
        dev_conf.parameters.otaa=1;
        dev_conf.parameters.device_address=DevAddr;
        debug("\r\n");
        for(int i=0; i<16; i++)
        {
            dev_conf.parameters.net_session_key[i]=NetSKey[i];
            dev_conf.parameters.app_session_key[i]=AppSKey[i];
        }
    #endif
    
    status= DataManager::overwrite_file_entries(DeviceConfig_n, dev_conf.data, sizeof(dev_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        return status;    
    }

    #endif
        /** SchedulerConfig */
    DataManager_FileSystem::File_t SchedulerConfig_File_t;
    SchedulerConfig_File_t.parameters.filename = SchedulerConfig_n;  
    SchedulerConfig_File_t.parameters.length_bytes = sizeof(SchedulerConfig::parameters);

    status=DataManager::add_file(SchedulerConfig_File_t, MAX_BUFFER_READING_TIMES+2); 
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

    SchedulerConfig s_conf;
    s_conf.parameters.time_comparator=0;
    status= DataManager::overwrite_file_entries(SchedulerConfig_n, s_conf.data, sizeof(s_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        return status;    
    }

    /** SendSchedulerConfig 
     */
    DataManager_FileSystem::File_t SendSchedulerConfig_File_t;
    SendSchedulerConfig_File_t.parameters.filename = SendSchedulerConfig_n;  
    SendSchedulerConfig_File_t.parameters.length_bytes = sizeof( TimeConfig::parameters);
    #if(SEND_SCHEDULER)
        #if BOARD == EARHART_V1_0_0
            debug("\r\nWARNING!! SEND SCHEDULER IS ON FOR EARHART BOARD\r\nVisit https://www.loratools.nl/#/airtime to find \r\nout more. Max payload per msg 255 bytes\r\n");     
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

    TimeConfig ss_conf;
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
    ClockSynchFlag_File_t.parameters.length_bytes = sizeof( FlagsConfig::parameters);

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

    DataManager_FileSystem::File_t NextTimeConfig_File_t;
    NextTimeConfig_File_t.parameters.filename = NextTimeConfig_n;
    NextTimeConfig_File_t.parameters.length_bytes = sizeof(TimeConfig::parameters);

    status=DataManager::add_file(NextTimeConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

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

    status = DataManager::add_file(InterruptConfig_File_t, 14000/(METRIC_GROUPS_ON+1)); 
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    #endif
  
    DataManager_FileSystem::File_t MetricGroupAConfig_File_t;
    MetricGroupAConfig_File_t.parameters.filename = MetricGroupAConfig_n;
    MetricGroupAConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);
    
    #if (METRIC_GROUPS_ON > 0)
    status = DataManager::add_file(MetricGroupAConfig_File_t, 14000/(METRIC_GROUPS_ON+1));
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    #endif

    #if (SCHEDULER_B || METRIC_GROUPS_ON >=2)
    DataManager_FileSystem::File_t MetricGroupBConfig_File_t;
    MetricGroupBConfig_File_t.parameters.filename = MetricGroupBConfig_n;
    MetricGroupBConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    status = DataManager::add_file(MetricGroupBConfig_File_t, 14000/(METRIC_GROUPS_ON+1));
    if(status != NODEFLOW_OK)
    {
        return status;
    }
    #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */

    #if (SCHEDULER_C || METRIC_GROUPS_ON >=3)
        DataManager_FileSystem::File_t MetricGroupCConfig_File_t;
        MetricGroupCConfig_File_t.parameters.filename = MetricGroupCConfig_n;
        MetricGroupCConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);
        status = DataManager::add_file(MetricGroupCConfig_File_t, 14000/(METRIC_GROUPS_ON+1));
    
        if(status != NODEFLOW_OK)
        {
            return status;
        }

    #endif /* #if (SCHEDULER_C || METRIC_GROUPS_ON==3) */

    #if (SCHEDULER_D|| METRIC_GROUPS_ON==4)
        DataManager_FileSystem::File_t MetricGroupDConfig_File_t;
        MetricGroupDConfig_File_t.parameters.filename = MetricGroupDConfig_n;
        MetricGroupDConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

        status = DataManager::add_file(MetricGroupDConfig_File_t, 14000/(METRIC_GROUPS_ON+1));
        if(status != NODEFLOW_OK)
        {
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
    return status;
}

#if BOARD == EARHART_V1_0_0 || BOARD == DEVELOPMENT_BOARD_V1_1_0 /* #endif at EoF */

void NodeFlow::getDevAddr()
{
    DeviceConfig dev_conf;
    status = DataManager::read_file_entry(DeviceConfig_n, 0, dev_conf.data, sizeof(dev_conf.parameters));

    #if(OVER_THE_AIR_ACTIVATION)
        debug("\r\n");
    for (int i=0; i<8; i++)
    {
        debug("%02x",dev_conf.parameters.device_eui[i]);
    }
    debug("\r\n");
    for (int i=0; i<8; i++)
    {
        debug("%02x",dev_conf.parameters.application_eui[i]);
    }
    debug("\r\n");
    for (int i=0; i<16; i++)
    {
        debug("%02x",dev_conf.parameters.application_key[i]);
    }
    #endif

    #if(!OVER_THE_AIR_ACTIVATION)
        debug("\r\nDevaddr %08x\r\n",dev_conf.parameters.device_address);
        for (int i=0; i<16; i++)
        {
            debug("%02x",dev_conf.parameters.net_session_key[i]);
        }
        debug("\r\n");
        for (int i=0; i<16; i++)
        {
            debug("%02x",dev_conf.parameters.app_session_key[i]);
        }
    #endif
}
#endif

#if BOARD == WRIGHT_V1_0_0
int NodeFlow::initialise_nbiot()
{
    if(_comms_stack == Comms_Radio_Stack::NBIOT)
    {
        status=_radio.ready();
        if (status != NodeFlow::NODEFLOW_OK)
        {
            return status;
        }
        char ipv4[] = "68.183.254.233";
        uint16_t port= 5683;
        char uri[] = "coap://68.183.254.233:5683/";
        uint8_t uri_length=27;
        status=_radio.configure_coap(ipv4, port, uri, uri_length);
        if(status != NodeFlow::NODEFLOW_OK)
        {
            debug("\nCoap server not configured %d\r\n",status); //todo: if not configured then??
            return status;
        }
        _radio.start(5);
    }
    return NODEFLOW_OK;      
}
#endif /* #if BOARD == WRIGHT_V1_0_0 */


// //todo:

// std::map<std::string,func_type> fm;
int NodeFlow::CreateFile(DataManager_FileSystem::File_t file, uint8_t filename, int struct_size, int length)
{

    //DataManager_FileSystem::File_t DataConfig_File_t;
    file.parameters.filename = filename; 
    file.parameters.length_bytes = struct_size;//  sizeof(DataConfig::parameters);

    status = DataManager::add_file(file, length); 
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
void NodeFlow::add_record(DataType data, string str)
{   
    if (!str.empty()) //size() == 0
    {
        tformatter.write_string(str);
    }
    tformatter.write_num_type<DataType>(data);
}
template void NodeFlow::add_record<int8_t>(int8_t data, string str);
template void NodeFlow::add_record<int16_t>(int16_t data, string str);
template void NodeFlow::add_record<int32_t>(int32_t data, string str);
template void NodeFlow::add_record<int64_t>(int64_t data, string str);
template void NodeFlow::add_record<uint8_t>(uint8_t data, string str);
template void NodeFlow::add_record<uint16_t>(uint16_t data, string str);
template void NodeFlow::add_record<uint32_t>(uint32_t data, string str);
template void NodeFlow::add_record<uint64_t>(uint64_t data, string str);
template void NodeFlow::add_record<float>(float data, string str);
template void NodeFlow::add_record<double>(double data, string str);

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
            status=add_sensing_entry(buffer[i],metric_group_flag);
            if(status!=NODEFLOW_OK)
            {
                return status;
            }
        } 
    }
    return NODEFLOW_OK;
}

int NodeFlow::add_sensing_entry(uint8_t value, uint8_t metric_group)
{
    uint8_t filename=0;
    if(metric_group==0)
    {  
        #if(INTERRUPT_ON)
        filename=InterruptConfig_n;
        #endif
    }
    
    if(metric_group==1)
    {
        filename=MetricGroupAConfig_n; 
    }
    
    if(metric_group==2)
    {    
        #if (SCHEDULER_B || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==2)
        filename=MetricGroupBConfig_n;
        #endif
    }
    if(metric_group==3)
    {
        #if (SCHEDULER_C  || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
        filename=MetricGroupCConfig_n;
        #endif
    }
    if(metric_group==4)
    {   
        #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
        filename=MetricGroupDConfig_n;
        #endif
    }
    if (filename != 0)
    {
        DataConfig t_conf;
        t_conf.parameters.byte=value;
        status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"MetricGroupsConfig",status,__PRETTY_FUNCTION__);
            return status;
        }
    }
    
    return NODEFLOW_OK;
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
    uint16_t increment_value=i_conf.parameters.increment;
i_conf.parameters.increment=0;
    status= DataManager::overwrite_file_entries(IncrementAConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"IncrementAConfig",status,__PRETTY_FUNCTION__);   
    }
    
    return increment_value;
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
    if (status != NODEFLOW_OK)
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
    increment_value = i_conf.parameters.increment;

    return status;
}

int NodeFlow::clear_inc_b()
{
    IncrementBConfig i_conf;
    i_conf.parameters.increment=0;
    status = DataManager::overwrite_file_entries(IncrementBConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
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
    status = DataManager::overwrite_file_entries(IncrementBConfig_n, i_conf.data, sizeof(i_conf.parameters));
    if (status != NODEFLOW_OK)
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
        i_conf.parameters.increment = i+increment_value;
        status = DataManager::overwrite_file_entries(IncrementCConfig_n, i_conf.data, sizeof(i_conf.parameters));
        if (status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"IncrementCConfig",status,__PRETTY_FUNCTION__); 
        }
    }
    return status;
}


int NodeFlow::read_mg_entries_counter(uint16_t& mga_entries, uint16_t& mgb_entries,uint16_t& mgc_entries, uint16_t& mgd_entries, uint16_t& interrupt_entries,uint8_t& metric_group_active)
{   
    metric_group_active = 0;
    MetricGroupEntriesConfig i_conf;
    status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    
    interrupt_entries=i_conf.parameters.InterruptEntries;
    if (interrupt_entries != 0)
    {
        metric_group_active++;
    }

    mga_entries= i_conf.parameters.MetricGroupAEntries;
    if (mga_entries != 0)
    {
        metric_group_active++;
    }
    mgb_entries = i_conf.parameters.MetricGroupBEntries;
    if (mgb_entries != 0)
    {
        metric_group_active++;
    }
    mgc_entries = i_conf.parameters.MetricGroupCEntries;
    if (mgc_entries!=0)
    {
        metric_group_active++;
    }
    mgd_entries = i_conf.parameters.MetricGroupDEntries;
    if (mgd_entries != 0)
    {
        metric_group_active++;
    }
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__); 
    }
    return NODEFLOW_OK;
}


void NodeFlow::is_overflow()
{
    
    int max_mga_bytes, max_mgb_bytes, max_mgc_bytes, max_mgd_bytes, max_interrupt_bytes;
    int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes;
    read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);
   
    MetricGroupEntriesConfig i_conf;
    status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    #if(INTERRUPT_ON)
    uint16_t b=METRIC_GROUPS_ON+1;
    #endif
    #if(!INTERRUPT_ON)
    uint16_t b=METRIC_GROUPS_ON;
    #endif
    b=14000/b;
    max_mga_bytes=(mga_bytes/i_conf.parameters.MetricGroupAEntries)*2+mga_bytes;
    max_mgb_bytes=(mgb_bytes/i_conf.parameters.MetricGroupBEntries)*2+mgb_bytes;
    max_mgc_bytes=(mgc_bytes/i_conf.parameters.MetricGroupCEntries)*2+mgc_bytes;
    max_mgd_bytes=(mgd_bytes/i_conf.parameters.MetricGroupDEntries)*2+mgd_bytes;
    max_interrupt_bytes=(interrupt_bytes/i_conf.parameters.InterruptEntries)*2+interrupt_bytes;
    
    if(max_mga_bytes > b || max_mgb_bytes > b || max_mgc_bytes > b || max_mgd_bytes > b || max_interrupt_bytes > b)
    {
        debug("\r\nMEMORYY FULL");
        status=_send();
        if(status<NODEFLOW_OK)
        {
            #if(INTERRUPT_ON)
            if(max_interrupt_bytes > b )
            {
                status=DataManager::truncate_file(InterruptConfig_n, (mga_bytes/i_conf.parameters.InterruptEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
                if(status==NodeFlow::NODEFLOW_OK)
                {
                    i_conf.parameters.InterruptEntries=i_conf.parameters.InterruptEntries-int(i_conf.parameters.MetricGroupAEntries*0.2); 
                }
            }
            #endif

            if(max_mga_bytes > b )
            {
                status=DataManager::truncate_file(MetricGroupAConfig_n, (mga_bytes/i_conf.parameters.MetricGroupAEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
                if(status==NodeFlow::NODEFLOW_OK)
                {
                    i_conf.parameters.MetricGroupAEntries=i_conf.parameters.MetricGroupAEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
                }
            }
            if(max_mgb_bytes > b)
            {
                status=DataManager::truncate_file(MetricGroupBConfig_n, (mgb_bytes/i_conf.parameters.MetricGroupBEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
                if(status==NodeFlow::NODEFLOW_OK)
                {
                    i_conf.parameters.MetricGroupBEntries=i_conf.parameters.MetricGroupBEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
                }
            }

            if(max_mgc_bytes > b )
            {
                status=DataManager::truncate_file(MetricGroupCConfig_n, (mgc_bytes/i_conf.parameters.MetricGroupCEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
                if(status==NodeFlow::NODEFLOW_OK)
                {
                    i_conf.parameters.MetricGroupCEntries=i_conf.parameters.MetricGroupCEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
                }
            }

            if(max_mgd_bytes > b )
            {
                status=DataManager::truncate_file(MetricGroupDConfig_n, (mgd_bytes/i_conf.parameters.MetricGroupDEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
                if(status==NodeFlow::NODEFLOW_OK)
                {
                    i_conf.parameters.MetricGroupDEntries=i_conf.parameters.MetricGroupDEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
                }
            }

            status= DataManager::overwrite_file_entries(MetricGroupEntriesConfig_n, i_conf.data, sizeof(i_conf.parameters));
            if (status!=NODEFLOW_OK)
            {
                ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__); 
            }
       }
    }
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

int NodeFlow::increase_mg_entries_counter(uint8_t mg_flag) 
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
    MetricGroupTimesConfig_File_t.parameters.length_bytes = sizeof(TimeConfig::parameters);

    status=DataManager::add_file(MetricGroupTimesConfig_File_t, length);
    if (status != NODEFLOW_OK) 
    {
        ErrorHandler(__LINE__,"MetricGroupTimesConfig",status,__PRETTY_FUNCTION__);
        return status; 
    }
    
    DataManager_FileSystem::File_t TempMetricGroupTimesConfig_File_t;
    TempMetricGroupTimesConfig_File_t.parameters.filename = TempMetricGroupTimesConfig_n;
    TempMetricGroupTimesConfig_File_t.parameters.length_bytes = sizeof(TimeConfig::parameters);

    status = DataManager::add_file(TempMetricGroupTimesConfig_File_t, length);
    if(status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
        return status;
    }

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
    TimeConfig ss_conf;
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
    TimeConfig ss_conf;
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
    TimeConfig ss_conf;
    status = DataManager::read_file_entry(SendSchedulerConfig_n, i, ss_conf.data, sizeof(ss_conf.parameters));
    if(status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"SendSchedulerConfig_n",status,__PRETTY_FUNCTION__);
    }

    time=ss_conf.parameters.time_comparator;

    return status;
}


int NodeFlow::overwrite_clock_synch_config(int time_comparator, bool clockSynchOn)
{
    FlagsConfig c_conf;
    c_conf.parameters.flag=clockSynchOn;
    c_conf.parameters.value=time_comparator;
    
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
    FlagsConfig c_conf;
    status = DataManager::read_file_entry(ClockSynchFlag_n, 0, c_conf.data, sizeof(c_conf.parameters));
    if (status != NODEFLOW_OK)
    {
         ErrorHandler(__LINE__,"ClockSyncConfig",status,__PRETTY_FUNCTION__);
        return status;
    }
    
    clockSynchOn=c_conf.parameters.flag;
    time=c_conf.parameters.value;

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
    TimeConfig sg_conf;
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
    //TimeConfig sg_conf;
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
    
    status = DataManager::read_file_entry(TempMetricGroupTimesConfig_n, 0, t_conf.data, sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
    }

    int temp=t_conf.parameters.time_comparator; 

    for (int i=0; i<sch_length; i++)
    {
        status=DataManager::read_file_entry(TempMetricGroupTimesConfig_n, i, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"TempMetricGroupTimesConfig_n",status,__PRETTY_FUNCTION__);
        }
        int time_comparator_now= t_conf.parameters.time_comparator;

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
    debug("\r\nGroupA: %d, GroupB: %d, GroupC: %d, GroupD: %d", flags.test(0), flags.test(1), flags.test(2), flags.test(3));
    time=time_comparator;

    return status;
}
int NodeFlow::set_temp_reading_times(uint32_t time)
{
    TimeConfig sg_conf;
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
   return status;
}
int NodeFlow::overwrite_metric_flags(uint8_t ssck_flag)
{
    MetricGroupConfig mg_conf;
    mg_conf.parameters.metric_group_id=ssck_flag;
    status=DataManager::overwrite_file_entries(MetricGroupConfig_n, mg_conf.data, sizeof(mg_conf.parameters));
    if(status != NODEFLOW_OK)
    {   
        ErrorHandler(__LINE__,"MetricGroupConfig_n",status,__PRETTY_FUNCTION__);
    }
    return status;
}

int NodeFlow::get_metric_flags(uint8_t &flag)
{
    MetricGroupConfig mg_conf;
    status=DataManager::read_file_entry(MetricGroupConfig_n, 0, mg_conf.data, sizeof(mg_conf.parameters));
    if (status!=NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"MetricGroupConfig_n",status,__PRETTY_FUNCTION__);
        return status;
    } 
    flag=mg_conf.parameters.metric_group_id;
    return status;
}



int NodeFlow::get_interrupt_latency(uint32_t &next_sch_time)
{
    TimeConfig t_conf;
    status=DataManager::read_file_entry(NextTimeConfig_n, 0, t_conf.data,sizeof(t_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"NextTimeConfig_n",status,__PRETTY_FUNCTION__);
        return status;
    }
    next_sch_time=t_conf.parameters.time_comparator-time_now();
    return status;
}

int NodeFlow::overwrite_wakeup_timestamp(uint16_t time_remainder){
    
    TimeConfig t_conf;
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
        //debug("\r\nReceived value: %d\n",unix_time);
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
    
        debug("\r\nMGroupA: %d, MGroupB: %d, MGroupC: %d, MGroupC: %d",metric_flag.test(0),
                metric_flag.test(1),metric_flag.test(2),metric_flag.test(3));
       
        
        if(metric_flag.test(0)==1)
        {
            MetricGroupA();
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                increase_mg_entries_counter(1);
            }
            add_payload_data(1);
        }

        if(metric_flag.test(1)==1)
        {
            MetricGroupB();
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                increase_mg_entries_counter(2);
            }
            add_payload_data(2);
        }
        if(metric_flag.test(2)==1)
        {   
            MetricGroupC();
            uint16_t c_entries;
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                increase_mg_entries_counter(3);
            }
            add_payload_data(3);
        }
        if(metric_flag.test(3)==1)
        {
            MetricGroupD();
            tformatter.get_entries(c_entries);
            if( c_entries>1)
            {
                increase_mg_entries_counter(4);
            }
            add_payload_data(4);
        }
    }
    else
    {

        MetricGroupA();
        tformatter.get_entries(c_entries);
        if( c_entries>1)
        {
            increase_mg_entries_counter(1);
        }
        add_payload_data(1);
    }
 
    is_overflow();

}


void NodeFlow::read_write_entry(uint8_t group_tag, int start_len, int end_len, uint8_t filename)
{
    if (end_len!=0)
    {
        if(start_len==0)
        {
            tformatter.write(group_tag, TFormatter::GROUP_TAG); 
            tformatter.write(159, TFormatter::RAW);
        }
        else
        {
            tformatter.decrease_entries();
        }
        int i=start_len;
        for (i=start_len; i<end_len; i++) 
        {   
            DataConfig d_conf;
            DataManager::read_file_entry(filename, i, d_conf.data, sizeof(d_conf.parameters));
            tformatter.write(d_conf.parameters.byte, TFormatter::RAW);

        }
        int total_bytes=0;
        status= DataManager::get_total_written_file_entries(filename, total_bytes);
        if(status != NODEFLOW_OK)
        {
            ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
        }
        if (total_bytes==i)
        {
             tformatter.write(255, TFormatter::RAW);
        }
       
    }
}

void NodeFlow::UploadNow()
{
   
    upload_flag=true;
}

int NodeFlow::_send()
{
    uint16_t mga_entries, mgb_entries, mgc_entries, mgd_entries, interrupt_entries;
    int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes;
    uint8_t metric_group_active=0;

    read_mg_entries_counter(mga_entries, mgb_entries, mgc_entries, mgd_entries, interrupt_entries, metric_group_active);
    read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes); 
   
    uint32_t total_bytes = mga_bytes+ mgb_bytes+ mgc_bytes+ mgd_bytes+interrupt_bytes;
    size_t buffer_len=0;
    send_block_number=0;
    debug("\r\nENTRIES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d",mga_entries, mgb_entries, mgc_entries, mgd_entries,interrupt_entries);
    debug("\r\nBYTES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d",mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);
    if(total_bytes != 0)
    {
        tformatter.serialise_main_cbor_object(metric_group_active);
    
        uint16_t available=0;

        tformatter.get_entries(available);

        total_blocks=ceil((available+total_bytes)/TP_TX_BUFFER);
        
        available=TP_TX_BUFFER-available;
        
        #if (INTERRUPT_ON)
            status=_divide_to_blocks(5, InterruptConfig_n, interrupt_bytes, available);
            if (status < NODEFLOW_OK)
            {
                return status;
            }
        #endif
        
        status=_divide_to_blocks(1, MetricGroupAConfig_n, mga_bytes, available);
        if (status < NODEFLOW_OK)
        {
            return status;
        }
        
        #if (SCHEDULER_B || METRIC_GROUPS_ON==2 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==4)
            status=_divide_to_blocks(2, MetricGroupBConfig_n, mgb_bytes, available);
            if (status < NODEFLOW_OK)
            {
                return status;
            }
        #endif /*  #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */

        #if (SCHEDULER_C || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==4)
            status=_divide_to_blocks(3, MetricGroupCConfig_n, mgc_bytes, available);
            if (status < NODEFLOW_OK)
            {
                return status;
            }
        #endif /*  #if (SCHEDULER_C|| METRIC_GROUPS_ON==3) */
        
        #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
            status=_divide_to_blocks(4, MetricGroupDConfig_n, mgd_bytes, available);
            if (status < NODEFLOW_OK)
            {
                return status;
            }
        #endif /*  #if (SCHEDULER_C|| METRIC_GROUPS_ON==4) */
        
        status=_send_blocks();
        if (status < NODEFLOW_OK)
        {
            debug("\r\nLine %d",__LINE__);
            return status;
        }
    }
    
   return NodeFlow::NODEFLOW_OK;
}

int NodeFlow::_divide_to_blocks(uint8_t group, uint8_t filename, uint16_t buffer_len, uint16_t&available)
{
    uint16_t entries=0;
    uint16_t done=0;
    tformatter.get_entries(entries);
    available=TP_TX_BUFFER-entries;
    
    if(buffer_len!=0)
    {
        done=0;
        while(done<buffer_len)
        {   
            
            if (available>TP_TX_BUFFER)
            {
                available = TP_TX_BUFFER;
            }
            if(buffer_len<available)
            {
                available=buffer_len;
            }
            read_write_entry(group, done ,done+available, filename);
            done += available;
            tformatter.get_entries(entries);
            if (entries >= TP_TX_BUFFER )
            {
                status=_send_blocks();
                if (status < NODEFLOW_OK)
                {
                    debug("\r\nLine %d",__LINE__);
                    return status;
                }
            }
            available= buffer_len - done;
        }
    }
    return NodeFlow::NODEFLOW_OK;
}
int NodeFlow::_send_blocks()
{
    uint8_t send_more_block=0;
    if(total_blocks> send_block_number)
    {
        send_more_block=1;
    }
    size_t buffer_len=0;
    uint16_t c_entries;
    tformatter.get_entries(c_entries);
    buffer= new uint8_t[c_entries];
    buffer=tformatter.return_serialised(buffer_len);

    debug("\r\nSending %d bytes, msg: %d, more_block: %d",buffer_len, send_block_number,send_more_block);
    int response_code=-1;
    
    #if BOARD == WRIGHT_V1_0_0
        recv_data= new char[50];
        status=_radio.coap_post(buffer, buffer_len, recv_data, SaraN2::TEXT_PLAIN, send_block_number,
                                send_more_block, response_code);
    
        if((response_code == 0 || response_code == 2) && (send_more_block == false) ) 
        {
            debug("\r\nHorrayy, you just sended a message!");
            clear_after_send();
        } 
        if(status!=NODEFLOW_OK)
        {
            debug("\r\nUnsuccess..");
            return SEND_FAILED;
        }
        
    #endif /* BOARD == WRIGHT_V1_0_0 */

    #if BOARD == EARHART_V1_0_0

        status=_radio.send_message(total_blocks+1, buffer, buffer_len);
        if (status < NODEFLOW_OK)
        {
            return SEND_FAILED;
        }
        handle_receive(); //todo: The rx window closes too soon..
        if(send_more_block)
        {
            ThisThread::sleep_for(10000);
        }
        if (status >= NODEFLOW_OK && send_more_block == false) 
        {
            clear_after_send();
        }
       
    #endif /* #if BOARD      */

    delete [] buffer;
    delete [] recv_data;
    send_block_number++;

    return NODEFLOW_OK;
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


int NodeFlow::handle_receive()
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
        debug("\nNo Rx available"); 
    }
    else
    {
        debug("\r\nRx: %d(10), Port: %d", rx_dec_buffer[0], port);
    }
    _radio.sleep();
    // rx_message=rx_dec_buffer[0];
    // rx_port=port;
    
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
    int errCnt=0;
    bool error=false;
    error_increment(errCnt, line, error); 
    debug("\r\nError in line No = %d, %s,Status = %d,Function name = %s, Errors %d\r\n",line, str1, status, str2, errCnt);
    if(error)
    {
       #if BOARD == EARHART_V1_0_0
            uint8_t error[3]={5,uint8_t(line),uint8_t(status)};
            _radio.send_message(219, error, 3);
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
    DataManager_FileSystem::GlobalStats_t g_stats;
    DataManager::get_global_stats(g_stats.data);
    DataManager::print_global_stats(g_stats);
}

void NodeFlow::tracking_memory()
{
    int cnt = osThreadGetCount();
    mbed_stats_stack_t *stats = (mbed_stats_stack_t*) malloc(cnt * sizeof(mbed_stats_stack_t));
 
    cnt = mbed_stats_stack_get_each(stats, cnt);
    for (int i = 0; i < cnt; i++) {
        debug("\r\nThread: 0x%lX, Stack size: %lu / %lu", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
    }
    free(stats);
 
    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    debug("\r\nHeap size: %lu / %lu bytes", heap_stats.current_size, heap_stats.reserved_size);
}
