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

        Serial a(TP_PC_TXU,TP_PC_RXU);
        DataManager dm(TP_EEPROM_WC, TP_I2C_SDA, TP_I2C_SCL, 100000);
        TP_Sleep_Manager::WakeupType_t wkp = sleep_manager.get_wakeup_type();

        int latency=0;
        int status=0;
        uint32_t next_time=0;
        uint8_t send_block_number=0;
        uint8_t total_blocks=0;

        watchdog.kick();
        time_t start_time=time_now();
        if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_PIN)
        { 
            debug("\r\n--------------------PIN WAKEUP--------------------\r\n");
            tformatter.setup();
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
                kick=false;
            }
            else 
            {
                debug("\r\n-------------------TIMER WAKEUP-------------------\r\n");
                timetodate(time_now());
                if (clock_synch==true)
                {
                    get_timestamp();
                    clock_synch==false;
                }
                if (user_function==true)
                {
                    tformatter.setup();
                    int entries=0;
                    UserDefinedScheduler u_conf;
                    status= DataManager::get_total_written_file_entries(TempSchedulerConfig_n, entries);
                    debug("\r\nFunctions %d\r\n",entries);
                    for(int i=0; i<entries; i++)
                    {
                        status = DataManager::read_file_entry(TempSchedulerConfig_n, i, u_conf.data, sizeof(u_conf.parameters));
                        u_conf.parameters.f();
                    }
                }
                latency=time_now()-start_time;
                set_scheduler(latency, next_time);
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
           
            latency=time_now()-start_time;
            set_scheduler(latency, next_time);
        }

        if(wkp==TP_Sleep_Manager::WakeupType_t::WAKEUP_UNKNOWN)
        {
            debug("\r\nWAKEUP_UNKNOWN\r\n");
            time_t time_after_false_wakeup=hold_time-time_now();
            next_time=time_after_false_wakeup;
            debug("Next time, time_after_false_wakeup %d", next_time);
        }
        int neg_next_time=next_time;
        if(neg_next_time<0)
        {
            next_time=1;
        }
        debug("\nGoing to sleep for %d s.\nWakeup at ", next_time);
        timetodate(next_time+time_now());
        bool wkup=false;
        #if (INTERRUPT_ON) //todo: bug
            if (next_time<15) //to prevent more delays
            {
               wkup=false;
            }
            else
            {
                wkup=true;
            }
        #endif
        #if (!INTERRUPT_ON)
            wkup=false;
        #endif 
           sleep_manager.stop(next_time, false);
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
    TotalFiles_File_t.parameters.length_bytes = sizeof(FilesConfig::parameters);

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


int NodeFlow::CreateFile(uint8_t filename, int struct_size, int max_entries)
{

    debug("\r\nNew Filename (no) %d, %d bytes, max_entries %d", filename, struct_size, max_entries);
    if(struct_size>23)
    {
        debug("\r\nOH OH CBOR PROBLEMS, the struct should be less than 23 bytes "); 
    }
    DataManager_FileSystem::File_t filename_File_t;
    filename_File_t.parameters.filename = filename; 
    filename_File_t.parameters.length_bytes = sizeof(DataConfig::parameters);

    status = DataManager::add_file(filename_File_t, max_entries*struct_size); 
    if(status != NODEFLOW_OK)
    {
        debug("\r\nError.Line %d, Status: %d",__LINE__, status);
        return status;
    }
    else
    {
        FilesConfig u_conf;
        u_conf.parameters.byte=filename;
        u_conf.parameters.size=struct_size;
        status = DataManager::append_file_entry(TotalFiles_n, u_conf.data, sizeof(u_conf.parameters));
    }
    return NODEFLOW_OK;
}

void NodeFlow::save(char* buffer, uint8_t b_size, uint8_t filename)
{
    DataConfig t_conf;
    t_conf.parameters.byte=buffer[0];
    status= DataManager::overwrite_file_entries(filename, t_conf.data, sizeof(t_conf.parameters));
    if(status != NODEFLOW_OK)
    {
        debug("\r\nError.Line %d, Status: %d",__LINE__, status);
    }
    for(int i=1; i<b_size; i++)
    {
        t_conf.parameters.byte=buffer[i];
        status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            debug("\r\nError.Line %d, Status: %d",__LINE__, status);
        }
    }
}

void NodeFlow::add(char* buffer, uint8_t b_size, uint8_t filename)
{
    DataConfig t_conf;
    for(int i=0; i<b_size; i++)
    {
        t_conf.parameters.byte=buffer[i];
        status= DataManager::append_file_entry(filename, t_conf.data, sizeof(t_conf.parameters));
        if(status != NODEFLOW_OK)
        {
            debug("\r\nError.Line %d, Status: %d",__LINE__, status);
        }
    }
    int remaining_entries=0;
    status=get_remaining_file_entries(filename, remaining_entries);
    //debug("\r\nget_remaining_file_entries %d\r\n",remaining_entries/b_size);
    if((remaining_entries/b_size)<=1)
    {
        debug("Memory full\r\n");
        is_overflow(filename, b_size);
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
    int latency_here=time_now();
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
                if(u_conf.parameters.trigger_time!=0)
                {
                    while(fmod(latency_here,86400)>u_conf.parameters.trigger_time)
                    {
                        u_conf.parameters.trigger_time=u_conf.parameters.trigger_time+u_conf.parameters.interval_time;
                    }
                }
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
    latency_here=time_now()-latency_here;
    next_timediff=next_timediff-latency_here;
    hold_time=timediff_temp+time_now();
    delete foo;
}


void NodeFlow::UploadNow(uint8_t filename)
{
    //_send(
    if(filename == 0)
    {
        uint8_t file=0;
        int entries=0;
        int file_entries=0;
        int struct_size=0;
        uint16_t total_bytes=0;
        FilesConfig f_conf;
        status= DataManager::get_total_written_file_entries(TotalFiles_n, entries);
        debug("\r\nUpload %d file(s)",entries);
        int mg_active=0;
        for(int i=0; i<entries; i++)
        {
            status = DataManager::read_file_entry(TotalFiles_n, i, f_conf.data, sizeof(f_conf.parameters));
            file=f_conf.parameters.byte;
            struct_size=f_conf.parameters.size;

            DataConfig u_conf;
            status= DataManager::get_total_written_file_entries(file, file_entries);
            total_bytes=total_bytes+file_entries; 
            if(file_entries!=0)
            {
               mg_active++; 
            }
        }
        debug("\r\nTotal bytes: %d",total_bytes);

        tformatter.serialise_main_cbor_object(mg_active);
        debug("\r\nMG: %d",file-FILENAME_START);
        uint16_t available=0;
        tformatter.get_entries(available);
        total_blocks=ceil((available+total_bytes)/TP_TX_BUFFER);
        available=TP_TX_BUFFER-available;
        debug("\r\nAvailable %d\r\n",available);

        for(int i=0; i<entries; i++)
        {
            status = DataManager::read_file_entry(TotalFiles_n, i, f_conf.data, sizeof(f_conf.parameters));
            debug("\r\nFile: %d",f_conf.parameters.byte);
            file=f_conf.parameters.byte;
            struct_size=f_conf.parameters.size;

            DataConfig u_conf;
            status= DataManager::get_total_written_file_entries(file, file_entries);
            if(file_entries != 0)
            {
                status=_divide_to_blocks(file-FILENAME_START, file, file_entries, struct_size, available);
                if (status < NODEFLOW_OK)
                {
                    debug("\r\nLine %d",__LINE__);
                }
                else
                {
                    status=DataManager::delete_file_entries(file);
                }
            }
        }
        status=_send_blocks();
        if (status < NODEFLOW_OK)
        {
            debug("\r\nLine %d",__LINE__);
        }
        debug("\r\n");
    }

}



void NodeFlow::is_overflow(uint8_t filename, uint16_t entries_to_remove)
{

     status=DataManager::truncate_file(filename, entries_to_remove);
     if (status != 0)
     {
        ErrorHandler(__LINE__,"truncate_file",status,__PRETTY_FUNCTION__); 
     }

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


uint32_t NodeFlow::time_now() 
{
    return (time(NULL))%DAYINSEC;
}


void NodeFlow::read_write_entry(uint8_t group_tag, int start_len, int end_len, uint8_t filename, uint16_t struct_size)
{
    if (end_len!=0)
    {
        if(start_len==0)
        {
            tformatter.write(group_tag, TFormatter::GROUP_TAG); 
            tformatter.write(159, TFormatter::RAW);
            tformatter.write(struct_size+64, TFormatter::RAW);
        }
        else
        {
            tformatter.decrease_entries();
        }
        int i=start_len;
        int mod=0;
        for (i=start_len; i<end_len; i++) 
        {   
            if(fmod(mod,struct_size)==0 && i!=0)
            {
                tformatter.write(struct_size+64, TFormatter::RAW);
            }
            DataConfig d_conf;
            DataManager::read_file_entry(filename, i, d_conf.data, sizeof(d_conf.parameters));
            tformatter.write(d_conf.parameters.byte, TFormatter::RAW);
            mod++;

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




int NodeFlow::_divide_to_blocks(uint8_t group, uint8_t filename, uint16_t buffer_len, uint16_t struct_size, uint16_t&available)
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
            read_write_entry(group, done ,done+available, filename, struct_size);
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
            //clear_after_send();
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
            //clear_after_send();
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




/** Manage device sleep times before calling sleep_manager.standby().
 *  Ensure that the maximum time the device can sleep for is 6600 seconds,
 *  this is due to the watchdog timer timeout, set at 7200 seconds
 *
 * @param seconds Number of seconds to sleep for 
 * @param wkup_one If true the device will respond to rising edge interrupts
 *                 on WKUP_PIN1
 * @return None 
 */
// void NodeFlow::enter_standby(int seconds, bool wkup_one) 
// { 
//     if(seconds < 2)
//     {
//         seconds = 2;
//     } 
//     #if BOARD == EARHART_V1_0_0
//         int retcode=_radio.sleep();
//     #endif /* BOARD == EARHART_V1_0_0 */

//     //Without this delay it breaks..?!
//     ThisThread::sleep_for(1);
//     sleep_manager.standby(seconds, wkup_one);
// }

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
