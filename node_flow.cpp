/**
 ******************************************************************************
 * @file    NodeFlow.cpp
 * @version 2.0.0
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
 * @param frequency_hz The bus frequency in hertz. 
 
 */
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


/** Start the device. kick the watchdog, initialise files, 
 *  Find the Wakeup type. 
 */
void NodeFlow::start()
{
    debug("\r\n                      __|__\n               --+--+--(_)--+--+--\n-------------------THING PILOT--------------------\r\n");
    debug("\nDevice Unique ID: %08X %08X %08X", STM32_UID[0], STM32_UID[1], STM32_UID[2]);
    starttime=time_now(); //
    starttime= fmod(starttime,86400);
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
            debug("\r\n----------------------SETUP-----------------------\r");
            if(CLOCK_SYNCH) 
            {
               // get_timestamp();
                set_time(1591276800);
                starttime=time_now(); 
                starttime=fmod(starttime,86400); 
                timetodate(time_now());
            }
            setup(); /** Pure virtual by the user */
        }
    }
    if (status != NODEFLOW_OK)
    { 
        NVIC_SystemReset(); 
    }
    while(true)
    {
        int status=0;
        uint8_t send_block_number=0;
        uint8_t total_blocks=0;

        Serial a(TP_PC_TXU,TP_PC_RXU);
        DataManager dm(TP_EEPROM_WC, TP_I2C_SDA, TP_I2C_SCL, 100000);
        
        TP_Sleep_Manager::WakeupType_t wkp = sleep_manager.get_wakeup_type();
       // debug("\r\nStart Wkaeup: %d\r\n",wkp);
    
        uint32_t next_time=0;
        watchdog.kick();
        time_t start_time=time_now();
        if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_PIN)
        { 
            debug("\r\n--------------------PIN WAKEUP--------------------\r\n");
            UserDefinedInterrupts u_conf;
            int entries=0;
            status= DataManager::get_total_written_file_entries(UserDefinedInterrupts_n, entries);
            for(int i=0; i<entries; i++)
            {
                status = DataManager::read_file_entry(UserDefinedInterrupts_n, i, u_conf.data, sizeof(u_conf.parameters));
                DigitalIn interrupt(u_conf.parameters.pin);
                if(interrupt.read())
                {
                    debug("\r\nInterrupt Detected, priority %d \r\n",i); 
                    u_conf.parameters.f();
                }
            }
            time_t time_after_false_wakeup=hold_time-time_now();
            next_time=time_after_false_wakeup;
        }
        if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_TIMER) 
        {
            if (kick==true) //remove the pin wakeup?? //pin_wakeup==true ||
            {
                debug("\r\n-------------------KICK WDG-------------------\r\n");
                time_t time_after_false_wakeup=hold_time-time_now();
                next_time=time_after_false_wakeup;
            }
            else 
            {
                debug("\r\n-------------------TIMER WAKEUP-------------------\r\n");
                if (clock_synch==true)
                {
                    get_timestamp();
                }
                if (user_function==true)
                {
                    tformatter.setup();
                    int entries=0;
                    UserDefinedScheduler u_conf;
                    status= DataManager::get_total_written_file_entries(TempSchedulerConfig_n, entries);
                    for(int i=0; i<entries; i++)
                    {
                        status = DataManager::read_file_entry(TempSchedulerConfig_n, i, u_conf.data, sizeof(u_conf.parameters));
                        u_conf.parameters.f();
                    }
                }
                set_scheduler(0, next_time);
            }
        }
        if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_RESET || wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_SOFTWARE) 
        {
            status=DataManager::init_gstats();
            _test_provision();
            debug("\r\n----------------------START----------------------\r\n");
           
            #if BOARD == WRIGHT_V1_0_0
                initialise_nbiot();
            #endif /* #if BOARD == WRIGHT_V1_0_0 */
            
            start_time=time_now();
            set_scheduler(0, next_time);

        }

        if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_UNKNOWN)
        {
            debug("\r\nWAKEUP_UNKNOWN\r\n");
            time_t time_after_false_wakeup=hold_time-time_now();
            next_time=time_after_false_wakeup;
            debug("Next time, time_after_false_wakeup %d", next_time);
        }
        debug("\nGoing to sleep for %d s.\nWakeup at ", next_time);
        timetodate(next_time+time_now());

        #if (INTERRUPT_ON) //todo: bug
            if (next_time<15) //to prevent more delays
            {
                sleep_manager.stop(next_time, false);
            }
            else
            {
                sleep_manager.stop(next_time, true);
            }
        #endif
        #if (!INTERRUPT_ON)
                sleep_manager.stop(next_time, false);
        #endif    
    }
}

void NodeFlow::attachInterval(int (*user_def_function)(), uint32_t interval)
{
    UserDefinedScheduler u_conf;
    u_conf.parameters.f=user_def_function;
    u_conf.parameters.trigger_time=fmod(starttime+interval,86400);
    u_conf.parameters.interval_time=interval;
    status= DataManager::append_file_entry(UserDefinedScheduler_n, u_conf.data, sizeof(u_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        debug("\r\nstatus %d", status); //errorhandler
    }

}

void NodeFlow::attachSchedule(int (*user_def_function)(), float schedule_array[], int arr_size)
{
    UserDefinedScheduler u_conf;
    u_conf.parameters.f=user_def_function;
    u_conf.parameters.interval_time=0;
    for(int i=0; i<arr_size; i++)
    {
        time_t time_remainder=((int(schedule_array[i]))*HOURINSEC)+((fmod(schedule_array[i],1))*6000);
        u_conf.parameters.trigger_time=time_remainder; 
        status= DataManager::append_file_entry(UserDefinedScheduler_n, u_conf.data, sizeof(u_conf.parameters));
        if (status != NODEFLOW_OK)
        {
            debug("\r\nstatus %d", status);
        }
    }
   
    ThisThread::sleep_for(1000);

}

void NodeFlow::attachInterrupt(int (*user_def_function)(), PinName pin)
{
    //userdefinterrupts
    UserDefinedInterrupts u_conf;
    u_conf.parameters.f=user_def_function;
    u_conf.parameters.pin=pin;
    status= DataManager::append_file_entry(UserDefinedInterrupts_n, u_conf.data, sizeof(u_conf.parameters));
    if (status != NODEFLOW_OK)
    {
        debug("\r\nstatus %d", status);
    }

}

int NodeFlow::test_function()
{
    debug("\r\n TEST \r\n ");
    return 0;
}

void NodeFlow::printSchedule()
{
    debug("\r\n");
    UserDefinedScheduler u_conf;
    int entries=0;
    status= DataManager::get_total_written_file_entries(UserDefinedScheduler_n, entries);
    for (int i=0; i<entries; i++)
    {
        status = DataManager::read_file_entry(UserDefinedScheduler_n, i, u_conf.data, sizeof(u_conf.parameters));
        debug("%d.",i);
        timetodate(u_conf.parameters.trigger_time);

    }
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
    DigitalOut buzzer(TP_SPI_NSS); //todo: remove? specific to the app?
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

    DataManager_FileSystem::File_t TotalFiles_File_t;
    TotalFiles_File_t.parameters.filename = TotalFiles_n;  
    TotalFiles_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    status=DataManager::add_file(TotalFiles_File_t, 50);  //50 scheduled times
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

    /**User defined SchedulerConfig */
    DataManager_FileSystem::File_t TempSchedulerConfig_File_t;
    TempSchedulerConfig_File_t.parameters.filename = TempSchedulerConfig_n;  
    TempSchedulerConfig_File_t.parameters.length_bytes = sizeof(UserDefinedScheduler::parameters);

    status=DataManager::add_file(TempSchedulerConfig_File_t, 50);  //50 scheduled times
    if(status != NODEFLOW_OK)
    {
        return status;   
    }

     /**User defined SchedulerConfig */
    DataManager_FileSystem::File_t UserDefinedScheduler_File_t;
    UserDefinedScheduler_File_t.parameters.filename = UserDefinedScheduler_n;  
    UserDefinedScheduler_File_t.parameters.length_bytes = sizeof(UserDefinedScheduler::parameters);

    status=DataManager::add_file(UserDefinedScheduler_File_t, 50);  //50 scheduled times
    if(status != NODEFLOW_OK)
    {
        return status;   
    }


    /**User Defined InterruptConfig */
    DataManager_FileSystem::File_t UserDefinedInterrupts_File_t;
    UserDefinedInterrupts_File_t.parameters.filename = UserDefinedInterrupts_n;  
    UserDefinedInterrupts_File_t.parameters.length_bytes = sizeof(UserDefinedInterrupts::parameters);

    status=DataManager::add_file(UserDefinedInterrupts_File_t, 50);  //50 scheduled times
    if(status != NODEFLOW_OK)
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

    /** FlagSSCKConfig. Every bit is a different flag. 
     *  0:SENSE, 1:SEND, 2:CLOCK, 3:KICK && true or false for pin_wakeup
     */
    DataManager_FileSystem::File_t FlagUCKConfig_File_t;
    FlagUCKConfig_File_t.parameters.filename = FlagSSCKConfig_n;
    FlagUCKConfig_File_t.parameters.length_bytes = sizeof(FlagsConfig::parameters);

    status=DataManager::add_file(FlagUCKConfig_File_t, 1);
    if(status != NODEFLOW_OK)
    {
        return status;   
    }
    //status=set_flags_config(0);  

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

    // #if (INTERRUPT_ON)
    // DataManager_FileSystem::File_t InterruptConfig_File_t;
    // InterruptConfig_File_t.parameters.filename = InterruptConfig_n;
    // InterruptConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    // status = DataManager::add_file(InterruptConfig_File_t, 800/(METRIC_GROUPS_ON+1)); 
    // if(status != NODEFLOW_OK)
    // {
    //     return status;
    // }
    // #endif
  
    // DataManager_FileSystem::File_t MetricGroupAConfig_File_t;
    // MetricGroupAConfig_File_t.parameters.filename = MetricGroupAConfig_n;
    // MetricGroupAConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);
    
    // #if (METRIC_GROUPS_ON > 0)
    // status = DataManager::add_file(MetricGroupAConfig_File_t, 800/(METRIC_GROUPS_ON+1));
    // if(status != NODEFLOW_OK)
    // {
    //     return status;
    // }
    // #endif

    // #if (SCHEDULER_B || METRIC_GROUPS_ON >=2)
    // DataManager_FileSystem::File_t MetricGroupBConfig_File_t;
    // MetricGroupBConfig_File_t.parameters.filename = MetricGroupBConfig_n;
    // MetricGroupBConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    // status = DataManager::add_file(MetricGroupBConfig_File_t, 800/(METRIC_GROUPS_ON+1));
    // if(status != NODEFLOW_OK)
    // {
    //     return status;
    // }
    // #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */

    // #if (SCHEDULER_C || METRIC_GROUPS_ON >=3)
    //     DataManager_FileSystem::File_t MetricGroupCConfig_File_t;
    //     MetricGroupCConfig_File_t.parameters.filename = MetricGroupCConfig_n;
    //     MetricGroupCConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);
    //     status = DataManager::add_file(MetricGroupCConfig_File_t, 800/(METRIC_GROUPS_ON+1));
    
    //     if(status != NODEFLOW_OK)
    //     {
    //         return status;
    //     }

    // #endif /* #if (SCHEDULER_C || METRIC_GROUPS_ON==3) */

    // #if (SCHEDULER_D|| METRIC_GROUPS_ON==4)
    //     DataManager_FileSystem::File_t MetricGroupDConfig_File_t;
    //     MetricGroupDConfig_File_t.parameters.filename = MetricGroupDConfig_n;
    //     MetricGroupDConfig_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    //     status = DataManager::add_file(MetricGroupDConfig_File_t, 800/(METRIC_GROUPS_ON+1));
    //     if(status != NODEFLOW_OK)
    //     {
    //         return status;
    //     }
    // #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON=42) */
    
   
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


int NodeFlow::CreateFile(uint8_t filename, int struct_size, int length)
{

    debug("\r\nNew Filename (no) %d, %d bytes, max_entries %d", filename, struct_size, length);
    DataManager_FileSystem::File_t filename_File_t;
    filename_File_t.parameters.filename = filename; 
    filename_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    status = DataManager::add_file(filename_File_t, length*struct_size); 
    if(status != NODEFLOW_OK)
    {
        debug("\r\nError.Line %d, Status: %d",__LINE__, status);
        return status;
    }
    else
    {
        DataConfig u_conf;
        u_conf.parameters.byte=filename;
        status = DataManager::append_file_entry(TotalFiles_n, u_conf.data, sizeof(u_conf.parameters));
    }
    return NODEFLOW_OK;
}

void NodeFlow::save(char* buffer, uint8_t b_size, uint8_t filename)
{
    DataConfig t_conf;
    t_conf.parameters.byte=buffer[0];
    status= DataManager::overwrite_file_entries(filename, t_conf.data, sizeof(t_conf.parameters));
    for(int i=1; i<b_size; i++)
    {
        t_conf.parameters.byte=buffer[i];
        status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            debug("\r\nError.Line %d, Status: %d",__LINE__, status);
        }
    }
    for(int i=0; i<b_size; i++)
    {
        status = DataManager::read_file_entry(filename, i, t_conf.data, sizeof(t_conf.parameters));
    }
}

void NodeFlow::add(char* buffer, uint8_t b_size, uint8_t filename)
{
    DataConfig t_conf;
    for(int i=1; i<b_size; i++)
    {
        t_conf.parameters.byte=buffer[i];
        status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            debug("\r\nError.Line %d, Status: %d",__LINE__, status);
        }
    }
    int remaining_files=0;
    status=total_remaining_file_table_entries(remaining_files);
    debug("\r\ntotal_remaining_file_table_entries %d\r\n",remaining_files/b_size);
    if((remaining_files/b_size)<=1)
    {
        debug("Memory full\r\n");
        is_overflow_v2(filename, b_size);
    }

}

uint8_t* NodeFlow::read(uint8_t filename, uint16_t& file_entries, uint16_t bytes_to_read)
{
    DataConfig t_conf;
    int entries=0;
    status= DataManager::get_total_written_file_entries(filename, entries);
    file_entries=entries;
    uint8_t* buffer= new uint8_t[entries];
    if (bytes_to_read!=0)
    {
        entries=bytes_to_read;
    }
    for (int i=0; i<entries; i++)
    {
        status = DataManager::read_file_entry(filename, i, t_conf.data, sizeof(t_conf.parameters));
       // debug("%02X ", t_conf.parameters.byte);
        buffer[i]=t_conf.parameters.byte;
    }
    return buffer;
}

void NodeFlow::set_scheduler(int latency, uint32_t& next_timediff)
{
    bitset<8> uck_flag(0b0000'0000);
    int entries=0;
    UserDefinedScheduler u_conf;
    status= DataManager::get_total_written_file_entries(UserDefinedScheduler_n, entries);

    uint32_t timediff_temp=DAYINSEC;
    uint32_t time_remainder=this->time_now();
    time_remainder=fmod(time_remainder,86400);
    int32_t timediff=0;
    uint32_t next_sch_time=0; 
    uint32_t scheduled_times=0;
    uint16_t times;
    uint8_t group_id=0;
    uint8_t temp_group_id=0;
    user_function=false;
    clock_synch= false;
    kick= false;

    int * foo;
    foo = new int [entries];
    int no_of_functions=0;
   
    for (int i=0; i<entries; i++)
    {
        status = DataManager::read_file_entry(UserDefinedScheduler_n, i, u_conf.data, sizeof(u_conf.parameters));
        scheduled_times=u_conf.parameters.trigger_time;
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
                foo[0]=i;
                no_of_functions=1;
            }
            if (timediff == timediff_temp)
            {    
                foo[no_of_functions]=i;
                no_of_functions++;
            }
            timediff_temp=timediff;
            next_sch_time=scheduled_times; //this can be removed it's just to print the next sensing time & sending time 
            user_function=true;
        }
    }
    uck_flag.set(0);
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
            uck_flag.set(1);
            clock_synch=true;
            if(cs_time<timediff_temp)
            {
                timediff_temp=cs_time;
                uck_flag.reset(0); 
                user_function=false;
            } 
        }
    }
    /**Check that its not more than 2 hours, 6600*/
    if (timediff_temp>6600)  
    {
        timediff_temp=6600;
        uck_flag.reset(0);
        uck_flag.reset(1);
        uck_flag.set(2);
        kick=true;
    } 
    if (uck_flag.test(0))
    {
        int x=0;
        for (int i=0; i<entries; i++)
        {
            status = DataManager::read_file_entry(UserDefinedScheduler_n, i, u_conf.data, sizeof(u_conf.parameters));
            if (foo[x]==i && x<no_of_functions)
            {
                if(x==0)
                {
                    status = DataManager::overwrite_file_entries(TempSchedulerConfig_n, u_conf.data, sizeof(u_conf.parameters));
                }
                else
                {
                    status = DataManager::append_file_entry(TempSchedulerConfig_n, u_conf.data, sizeof(u_conf.parameters));
                }
                u_conf.parameters.trigger_time=fmod(u_conf.parameters.trigger_time+u_conf.parameters.interval_time, 86400);
                u_conf.parameters.interval_time=u_conf.parameters.interval_time;
                u_conf.parameters.f=u_conf.parameters.f;
                x++;
            }
            else
            {
                u_conf.parameters.trigger_time=u_conf.parameters.trigger_time;
                u_conf.parameters.interval_time=u_conf.parameters.interval_time;
                u_conf.parameters.f=u_conf.parameters.f;
            }
            status = DataManager::append_file_entry(UserDefinedScheduler_n, u_conf.data, sizeof(u_conf.parameters));
        }
        status = DataManager::truncate_file(UserDefinedScheduler_n, entries);
    }

    debug("\r\nUser defined fun %d, ClockSynch: %d, KickWdg: %d\n", uck_flag.test(0), uck_flag.test(1), uck_flag.test(2));
    next_timediff=timediff_temp;
    hold_time=timediff_temp+time_now();
    delete foo;
}


void NodeFlow::UploadNow(uint8_t filename)
{
    if(filename == 0)
    {
        int entries=0;
        DeviceConfig u_conf;
        status= DataManager::get_total_written_file_entries(TotalFiles_n, entries);
        debug("\r\nUpload %d file(s):");
        for(int i=0; i<entries; i++)
        {
            DataConfig u_conf;
            status = DataManager::read_file_entry(TotalFiles_n, i, u_conf.data, sizeof(u_conf.parameters));
            debug("%d ",u_conf.parameters.byte);
        }
    }

  //  upload_flag=true;
}

template <typename DataType> 
void NodeFlow::add_record(DataType data, string str)
{   
    if (!str.empty()) 
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
    // uint8_t filename=0;
    // if(metric_group==0)
    // {  
    //     #if(INTERRUPT_ON)
    //     filename=InterruptConfig_n;
    //     #endif
    // }
    
    // if(metric_group==1)
    // {
    //     filename=MetricGroupAConfig_n; 
    // }
    
    // if(metric_group==2)
    // {    
    //     #if (SCHEDULER_B || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==2)
    //     filename=MetricGroupBConfig_n;
    //     #endif
    // }
    // if(metric_group==3)
    // {
    //     #if (SCHEDULER_C  || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
    //     filename=MetricGroupCConfig_n;
    //     #endif
    // }
    // if(metric_group==4)
    // {   
    //     #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    //     filename=MetricGroupDConfig_n;
    //     #endif
    // }
    // if (filename != 0)
    // {
    //     DataConfig t_conf;
    //     t_conf.parameters.byte=value;
    //     status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
    //     if(status != NODEFLOW_OK)
    //     {
    //         ErrorHandler(__LINE__,"MetricGroupsConfig",status,__PRETTY_FUNCTION__);
    //         return status;
    //     }
    // }
    
    return NODEFLOW_OK;
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


void NodeFlow::is_overflow_v2(uint8_t filename, uint16_t entries_to_remove)
{

     status=DataManager::truncate_file(filename, entries_to_remove);
     if (status != 0)
     {
        ErrorHandler(__LINE__,"truncate_file",status,__PRETTY_FUNCTION__); 
     }

}

void NodeFlow::is_overflow()
{
    // int max_mga_bytes, max_mgb_bytes, max_mgc_bytes, max_mgd_bytes, max_interrupt_bytes;
    // int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes;
    // read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);
   
    // MetricGroupEntriesConfig i_conf;
    // status = DataManager::read_file_entry(MetricGroupEntriesConfig_n, 0, i_conf.data, sizeof(i_conf.parameters));
    // #if(INTERRUPT_ON)
    // uint16_t b=METRIC_GROUPS_ON+1;
    // #endif
    // #if(!INTERRUPT_ON)
    // uint16_t b=METRIC_GROUPS_ON;
    // #endif
    // b=12000/b;
    // max_mga_bytes=(mga_bytes/i_conf.parameters.MetricGroupAEntries)*2+mga_bytes;
    // max_mgb_bytes=(mgb_bytes/i_conf.parameters.MetricGroupBEntries)*2+mgb_bytes;
    // max_mgc_bytes=(mgc_bytes/i_conf.parameters.MetricGroupCEntries)*2+mgc_bytes;
    // max_mgd_bytes=(mgd_bytes/i_conf.parameters.MetricGroupDEntries)*2+mgd_bytes;
    // max_interrupt_bytes=(interrupt_bytes/i_conf.parameters.InterruptEntries)*2+interrupt_bytes;
    
    // if(max_mga_bytes > b || max_mgb_bytes > b || max_mgc_bytes > b || max_mgd_bytes > b || max_interrupt_bytes > b)
    // {
    //     debug("\r\nMEMORYY FULL");
    //     status=_send();
    //     if(status<NODEFLOW_OK)
    //     {
    //         #if(INTERRUPT_ON)
    //         if(max_interrupt_bytes > b )
    //         {
    //             debug("\ntruncate\r\n");
               
    //             if(status==NodeFlow::NODEFLOW_OK)
    //             {
    //                 i_conf.parameters.InterruptEntries=i_conf.parameters.InterruptEntries-int(i_conf.parameters.MetricGroupAEntries*0.2); 
    //             }
    //         }
    //         #endif

    //         if(max_mga_bytes > b )
    //         {
    //             status=DataManager::truncate_file(MetricGroupAConfig_n, (mga_bytes/i_conf.parameters.MetricGroupAEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
    //             if(status==NodeFlow::NODEFLOW_OK)
    //             {
    //                 i_conf.parameters.MetricGroupAEntries=i_conf.parameters.MetricGroupAEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
    //             }
    //         }
    //         if(max_mgb_bytes > b)
    //         {
    //             status=DataManager::truncate_file(MetricGroupBConfig_n, (mgb_bytes/i_conf.parameters.MetricGroupBEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
    //             if(status==NodeFlow::NODEFLOW_OK)
    //             {
    //                 i_conf.parameters.MetricGroupBEntries=i_conf.parameters.MetricGroupBEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
    //             }
    //         }

    //         if(max_mgc_bytes > b )
    //         {
    //             status=DataManager::truncate_file(MetricGroupCConfig_n, (mgc_bytes/i_conf.parameters.MetricGroupCEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
    //             if(status==NodeFlow::NODEFLOW_OK)
    //             {
    //                 i_conf.parameters.MetricGroupCEntries=i_conf.parameters.MetricGroupCEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
    //             }
    //         }

    //         if(max_mgd_bytes > b )
    //         {
    //             status=DataManager::truncate_file(MetricGroupDConfig_n, (mgd_bytes/i_conf.parameters.MetricGroupDEntries)*(int(i_conf.parameters.MetricGroupAEntries*0.2)));
    //             if(status==NodeFlow::NODEFLOW_OK)
    //             {
    //                 i_conf.parameters.MetricGroupDEntries=i_conf.parameters.MetricGroupDEntries-(int(i_conf.parameters.MetricGroupAEntries*0.2));
    //             }
    //         }

    //         status= DataManager::overwrite_file_entries(MetricGroupEntriesConfig_n, i_conf.data, sizeof(i_conf.parameters));
    //         if (status!=NODEFLOW_OK)
    //         {
    //             ErrorHandler(__LINE__,"MetricGroupEntriesConfig_n",status,__PRETTY_FUNCTION__); 
    //         }
    //    }
    // }
}

int NodeFlow::read_mg_bytes(int& mga_bytes, int& mgb_bytes, int& mgc_bytes,int& mgd_bytes, int& interrupt_bytes)
{
    // mga_bytes=0;
    // mgb_bytes=0;
    // mgc_bytes=0;
    // mgd_bytes=0;
    // interrupt_bytes=0;
    // #if (INTERRUPT_ON)
    //     status= DataManager::get_total_written_file_entries(InterruptConfig_n, interrupt_bytes);
    //     if(status != NODEFLOW_OK)
    //     {
    //         status= DataManager::get_total_written_file_entries(InterruptConfig_n, interrupt_bytes);
    //         if(status != NODEFLOW_OK)
    //         {
    //             ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
    //             return status;
    //         }
    //     }
    // #endif
    // status= DataManager::get_total_written_file_entries(MetricGroupAConfig_n, mga_bytes);
    // if(status != NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
    //     return status;
    // }
    // #if (SCHEDULER_B  || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==2)
    // status= DataManager::get_total_written_file_entries(MetricGroupBConfig_n, mgb_bytes);
    // if(status != NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
    //     return status;
    // }
    // #endif
    // #if (SCHEDULER_C || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
    // status= DataManager::get_total_written_file_entries(MetricGroupCConfig_n, mgc_bytes);
    // if(status != NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
    //     return status;
    // }
    // #endif
    // #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    // status= DataManager::get_total_written_file_entries(MetricGroupDConfig_n, mgd_bytes);
    // if(status != NODEFLOW_OK)
    // {
    //     ErrorHandler(__LINE__,"get_total_written_file_entries",status,__PRETTY_FUNCTION__);
    //     return status;
    // }
    // #endif

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



int NodeFlow::timetoseconds(float scheduler_time, uint8_t group_id)
{
    uint16_t time_remainder=DIVIDE(((int(scheduler_time))*HOURINSEC)+((fmod(scheduler_time,1))*6000));
    timetodate(time_remainder*2);
    
    return status;
}


int NodeFlow::overwrite_clock_synch_config(int time_comparator, bool clockSynchOn)
{
    FlagsConfig c_conf;
    c_conf.parameters.flag=clockSynchOn;
    c_conf.parameters.value=time_comparator;
    
    status = DataManager::overwrite_file_entries(ClockSynchFlag_n, c_conf.data, sizeof(c_conf.parameters));
    if(status != NODEFLOW_OK) 
    {
        ErrorHandler(__LINE__,"ClockSynchFlag",status,__PRETTY_FUNCTION__);
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




int NodeFlow:: set_flags_configV2(uint8_t uck_flag)
{
    FlagsConfig f_conf;
    f_conf.parameters.value=uck_flag;
    f_conf.parameters.flag=0; //i dont remember why is that

    status= DataManager::overwrite_file_entries(FlagUCKConfig_n, f_conf.data, sizeof(f_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagUCKConfig",status,__PRETTY_FUNCTION__); 
    }

    return status;
}


int NodeFlow::set_wakeup_pin_flag(bool wakeup_pin)
{
    FlagsConfig f_conf;
    status=DataManager::read_file_entry(FlagSSCKConfig_n, 0, f_conf.data,sizeof(f_conf.parameters));

    if (wakeup_pin)
    {
        f_conf.parameters.flag=1;
    }
    else
    {
         f_conf.parameters.flag=0;
    }
    //f_conf.parameters.flag=wakeup_pin;
    status= DataManager::overwrite_file_entries(FlagSSCKConfig_n, f_conf.data, sizeof(f_conf.parameters));

    if(status != NODEFLOW_OK)
    {
        ErrorHandler(__LINE__,"FlagSSCKConfig",status,__PRETTY_FUNCTION__);  
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


uint32_t NodeFlow::time_now() 
{
    return (time(NULL))%DAYINSEC;
}


// void NodeFlow::_sense()
// {
//     uint16_t sched_length, c_entries;
//     read_sched_config(1,sched_length);
   
//     if (sched_length>1)
//     {   
//         uint8_t mg_flag;
//         get_metric_flags(mg_flag);
//         bitset<8> metric_flag(mg_flag);
    
//         debug("\r\nMGroupA: %d, MGroupB: %d, MGroupC: %d, MGroupC: %d",metric_flag.test(0),
//                 metric_flag.test(1),metric_flag.test(2),metric_flag.test(3));
       
        
//         if(metric_flag.test(0)==1)
//         {
//             MetricGroupA();
//             tformatter.get_entries(c_entries);
//             if( c_entries>1)
//             {
//                 increase_mg_entries_counter(1);
//             }
//             add_payload_data(1);
//         }

//         if(metric_flag.test(1)==1)
//         {
//             MetricGroupB();
//             tformatter.get_entries(c_entries);
//             if( c_entries>1)
//             {
//                 increase_mg_entries_counter(2);
//             }
//             add_payload_data(2);
//         }
//         if(metric_flag.test(2)==1)
//         {   
//             MetricGroupC();
//             uint16_t c_entries;
//             tformatter.get_entries(c_entries);
//             if( c_entries>1)
//             {
//                 increase_mg_entries_counter(3);
//             }
//             add_payload_data(3);
//         }
//         if(metric_flag.test(3)==1)
//         {
//             MetricGroupD();
//             tformatter.get_entries(c_entries);
//             if( c_entries>1)
//             {
//                 increase_mg_entries_counter(4);
//             }
//             add_payload_data(4);
//         }
//     }
//     else
//     {

//         MetricGroupA();
//         tformatter.get_entries(c_entries);
//         if( c_entries>1)
//         {
//             increase_mg_entries_counter(1);
//         }
//         add_payload_data(1);
//     }
 
//     is_overflow();

// }


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



int NodeFlow::_send()
{
    // uint16_t mga_entries, mgb_entries, mgc_entries, mgd_entries, interrupt_entries;
    // int mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes;
    // uint8_t metric_group_active=0;

    // read_mg_entries_counter(mga_entries, mgb_entries, mgc_entries, mgd_entries, interrupt_entries, metric_group_active);
    // read_mg_bytes(mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes); 
   
    // uint32_t total_bytes = mga_bytes+ mgb_bytes+ mgc_bytes+ mgd_bytes+interrupt_bytes;
    // size_t buffer_len=0;
    // send_block_number=0;
    // debug("\r\nENTRIES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d",mga_entries, mgb_entries, mgc_entries, mgd_entries,interrupt_entries);
    // debug("\r\nBYTES = MGA: %d, MGB: %d, MGC: %d, MGD: %d, MGI: %d",mga_bytes, mgb_bytes, mgc_bytes, mgd_bytes, interrupt_bytes);
    // if(total_bytes != 0)
    // {
        
    //     tformatter.serialise_main_cbor_object(metric_group_active);
    //     debug("\r\metric_group_active %d",metric_group_active);
    //     uint16_t available=0;
    //     tformatter.get_entries(available);
    //     debug("\r\navailable %d",available);
    //     total_blocks=ceil((available+total_bytes)/TP_TX_BUFFER);
        
    //     available=TP_TX_BUFFER-available;
        
    //     #if (INTERRUPT_ON)
    //         status=_divide_to_blocks(5, InterruptConfig_n, interrupt_bytes, available);
    //         if (status < NODEFLOW_OK)
    //         {
    //             return status;
    //         }
    //     #endif
        
    //     status=_divide_to_blocks(1, MetricGroupAConfig_n, mga_bytes, available);
    //     if (status < NODEFLOW_OK)
    //     {
    //         return status;
    //     }
        
    //     #if (SCHEDULER_B || METRIC_GROUPS_ON==2 || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==4)
    //         status=_divide_to_blocks(2, MetricGroupBConfig_n, mgb_bytes, available);
    //         if (status < NODEFLOW_OK)
    //         {
    //             return status;
    //         }
    //     #endif /*  #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */

    //     #if (SCHEDULER_C || METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==4)
    //         status=_divide_to_blocks(3, MetricGroupCConfig_n, mgc_bytes, available);
    //         if (status < NODEFLOW_OK)
    //         {
    //             return status;
    //         }
    //     #endif /*  #if (SCHEDULER_C|| METRIC_GROUPS_ON==3) */
        
    //     #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    //         status=_divide_to_blocks(4, MetricGroupDConfig_n, mgd_bytes, available);
    //         if (status < NODEFLOW_OK)
    //         {
    //             return status;
    //         }
    //     #endif /*  #if (SCHEDULER_C|| METRIC_GROUPS_ON==4) */
        
    //     status=_send_blocks();
    //     if (status < NODEFLOW_OK)
    //     {
    //         debug("\r\nLine %d",__LINE__);
    //         return status;
    //     }
    // }
    
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
    // time(&remainder_time);
    // debug("%d, %s",remainder_time, ctime (&remainder_time));
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
{   
    uint8_t port=0;
    int retcode=0;
    uint32_t rx_dec_buffer[MAX_BUFFER_READING_TIMES];
    _radio.receive_message(rx_dec_buffer,port,retcode); 
    
    if (port==SCHEDULER_PORT)
    {
        //status=overwrite_sched_config(true, DIVIDE(retcode));
        // if (status != NODEFLOW_OK)
        // {
        //     ErrorHandler(__LINE__,"overwrite_sched_config",status,__PRETTY_FUNCTION__);
        // }

        for (int i=0; i<DIVIDE(retcode); i++)
        {
            debug("\n%i.RX scheduler: %d(10)\n",i, rx_dec_buffer[i]);
            // status=append_sched_config(rx_dec_buffer[i]/2,1); //TODO: CHANGE GROUP ID- depends on data
            // if (status != NODEFLOW_OK)
            // {
            //     ErrorHandler(__LINE__,"append_sched_config",status,__PRETTY_FUNCTION__);
            // }
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
       // return status;
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
//     #if (INTERRUPT_ON)
//         status= DataManager::delete_file_entries(InterruptConfig_n);
//         if (status!=NODEFLOW_OK)
//         {
//             ErrorHandler(__LINE__,"InterruptConfig_n",status,__PRETTY_FUNCTION__); 
//         }
//     #endif
//     status= DataManager::delete_file_entries(MetricGroupAConfig_n);
//     if (status!=NODEFLOW_OK)
//     {
//         ErrorHandler(__LINE__,"MetricGroupAConfig_n",status,__PRETTY_FUNCTION__); 
//     }
//     #if (SCHEDULER_B || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3|| METRIC_GROUPS_ON==2)
//         status= DataManager::delete_file_entries(MetricGroupBConfig_n);
//         if (status!=NODEFLOW_OK)
//         {
//             ErrorHandler(__LINE__,"MetricGroupBConfig_n",status,__PRETTY_FUNCTION__); 
//         }
//     #endif /* #if (SCHEDULER_B || METRIC_GROUPS_ON==2) */
//     #if (SCHEDULER_C || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
//         status= DataManager::delete_file_entries(MetricGroupCConfig_n);
//         if (status!=NODEFLOW_OK)
//         {
//             ErrorHandler(__LINE__,"MetricGroupCConfig_n",status,__PRETTY_FUNCTION__); 
//         }
//     #endif /* #if (SCHEDULER_C || METRIC_GROUPS_ON==3) */
//     #if (SCHEDULER_D || METRIC_GROUPS_ON==4)
//     status= DataManager::delete_file_entries(MetricGroupDConfig_n);
//     if (status!=NODEFLOW_OK)
//     {
//         ErrorHandler(__LINE__,"MetricGroupDConfig_n",status,__PRETTY_FUNCTION__); 
//     }
//     #endif /* #if (SCHEDULER_D || METRIC_GROUPS_ON==4) */

//    clear_mg_counter();   
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
