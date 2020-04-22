/**
 ******************************************************************************
 * @file    NodeFLow.h
 * @version 2.0.0
 * @author  Rafaella Nofytou,  Adam Mitchell
 * @brief   Header file of the Wright || Earheart node from Think Pilot. 
 * Handles sleeping times/ eeprom driver/ lorawan/ nb-iot communication
 ******************************************************************************
 */

#ifndef NODEFLOW_H
#define NODEFLOW_H

#pragma once
/** Includes
 */

#include "mbed.h"
#include "config_device.h"
#include "DataManager.h"
#include "TPL5010.h"
#include "tp_sleep_manager.h"
#include "tformatter.h"
//#include <cmath>
#include <bitset>
//#include <algorithm>    
#include "mbed_mem_trace.h"

#define NODEFLOW_DBG true

#if BOARD == EARHART_V1_0_0
    #include "LorawanTP.h"
    #define MODULATION 0
#elif BOARD == WRIGHT_V1_0_0
    #include "tp_nbiot_interface.h"
    #define MODULATION 1
#else
     #define MODULATION 2
#endif

#define size(x)  (sizeof(x) / sizeof((x)[0]))
#define DIVIDE(x) (x)/2

#define FILENAME_START 7 
/** Time related defines 
 */
#define DAYINSEC    86400
#define HOURINSEC   3600
#define MINUTEINSEC 60

#if !defined(INTERRUPT_DELAY)
   #define INTERRUPT_ON false
   #define INTERRUPT_DELAY 0
#endif

/** Define retries for sending
 */
#define MAX_SEND_RETRIES 3
#define MAX_OVERWRITE_RETRIES 3


/** Nodeflow Class
 */
class NodeFlow: public DataManager
{
    public:
       
        /** Enumerated list of possible comms radio stacks
         */
        enum class Comms_Radio_Stack
        {
            NBIOT     = 0,
            LORA      = 1,
            UNDEFINED = 10
        };

        enum class Init_State
        {
            TEST = 1,
            PROV = 2,
            RUN  = 3
        };

        enum class Test_State
        {
            GPIO_TEST = 0,
            WFC       = 1,
            END       = 2
        };

        enum class Prov_State
        {
            PROVISION = 0,
            WFC       = 1,
            END       = 2
        };

        volatile Init_State INIT_STATE = Init_State::RUN;
        volatile Test_State TEST_STATE = Test_State::WFC;
        volatile Prov_State PROV_STATE = Prov_State::WFC;
        
        /** CONSTRUCTORS *********************************************************************************************/
         /** Constructor. Create a NodeFlow interface, connected to the pins specified 
         *  operating at the specified frequency
         * 
         * @param write_control GPIO to enable or disable write functionality
         * @param sda I2C data line pin
         * @param scl I2C clock line pin
         * @param frequency_hz The bus frequency in hertz. 

          PinName mosi=TP_LORA_SPI_MOSI, PinName miso=TP_LORA_SPI_MISO, PinName sclk=TP_LORA_SPI_SCK, PinName nss=TP_LORA_SPI_NSS, PinName reset=TP_LORA_RESET,
                PinName dio0=PB_4, PinName dio1=PB_1, PinName dio2=PB_0, PinName dio3=PC_13, PinName dio4=NC, PinName dio5=NC, PinName rf_switch_ctl1=NC, 
                PinName rf_switch_ctl2=NC, PinName txctl=NC, PinName rxctl=NC, PinName ant_switch=NC, PinName pwr_amp_ctl=NC, PinName tcxo=TP_VDD_TCXO,
         */
        #if BOARD == EARHART_V1_0_0
        NodeFlow(PinName write_control=TP_EEPROM_WC, PinName sda=TP_I2C_SDA, PinName scl=TP_I2C_SCL, int frequency_hz=100000, 
                PinName mosi=TP_LORA_SPI_MOSI, PinName miso=TP_LORA_SPI_MISO, PinName sclk=TP_LORA_SPI_SCK, PinName nss=TP_LORA_SPI_NSS, PinName reset=TP_LORA_RESET,
                PinName dio0=PB_4, PinName dio1=PB_1, PinName dio2=PB_0, PinName dio3=PC_13, PinName dio4=NC, PinName dio5=NC, PinName rf_switch_ctl1=NC, 
                PinName rf_switch_ctl2=NC, PinName txctl=NC, PinName rxctl=NC, PinName ant_switch=NC, PinName pwr_amp_ctl=NC, PinName tcxo=TP_VDD_TCXO,PinName done=TP_DONE);
        #endif /* #if BOARD == EARHART_V1_0_0 */

        #if BOARD == WRIGHT_V1_0_0
        NodeFlow(PinName write_control=TP_EEPROM_WC, PinName sda=TP_I2C_SDA, PinName scl=TP_I2C_SCL, int frequency_hz=100000,
                PinName txu=TP_NBIOT_TXU, PinName rxu=TP_NBIOT_RXU, PinName cts=TP_NBIOT_CTS, PinName rst=TP_NBIOT_RST, 
                PinName vint=TP_NBIOT_VINT, PinName gpio=TP_NBIOT_GPIO, int baud=TP_NBIOT_BAUD, PinName done=TP_DONE);
        #endif /* #if BOARD == WRIGHT_V1_0_0 */
        /** CONSTRUCTORS END *****************************************************************************************/

        /** NodeFlow destructor 
         */
        ~NodeFlow();

        /** start() drives all the application. It handles the different modem and configuration.
         */
        void start();

        /** VIRTUAL FUNCTIONS *****************************************************************************************
         *  Virtual functions MUST be overridden by the application developer.
         */
    
        /** setup() allows the user to write code that will only be executed once when the device is initialising.
         *  This is akin to Arduino's setup function and can be used to, for example, configure a sensor
         */
        virtual void setup() = 0;   
        /** Virtual functions END ************************************************************************************/
        
        /** PUBLIC FUNCTIONS *****************************************************************************************
         */
        /** User can create a new file with a specific struct size and entries */
        int CreateFile(uint8_t filename, int struct_size, int max_entries);
        
        /** User can attach intervals connected to user defined functions */
        void attachInterval(int (*user_def_function)(), uint32_t interval);

        /** User can attach Interrupts connected to user defined functions,
         *  connected with or gate to PA0?
         */
        void attachInterrupt(int (*user_def_function)(void), PinName pin);
        /** User can attach scheduled times of HH.MM connected to user defined functions
         */
        void attachSchedule(int (*user_def_function)(), float schedule_array[], int arr_size);
        
        void attachClockSynch(time_t clock_synch_time_interval);
       
        void printSchedule();

        void save(char* buffer, uint8_t b_size, uint8_t filename);
        void add(char* buffer, uint8_t b_size, uint8_t filename);

        //TODO:ti
        uint8_t* read(uint8_t filename, uint16_t& file_entries, uint16_t bytes_to_read=NULL); 

        void is_overflow(uint8_t filename, uint16_t entries_to_remove);

        void UploadNow(uint8_t filename=NULL);
        
        #if BOARD == EARHART_V1_0_0 || BOARD == DEVELOPMENT_BOARD_V1_1_0 /* #endif at EoF */
        void getDevAddr();
        #endif

        
    private:

        void _test_provision();
        void _oob_enter_test();

        void _oob_gpio_test_handler();

        void _oob_enter_prov();

        void _oob_end_handler();

        void _run();

        /** Initialise files after reset, set flags
         * 
         * @return          It could be one of these:
         */
        int initialise(); 
        
        /**Time Related functions ********************************************************************************/
        
        /** Get current timestamp in UNIX time
         *                 
         * @return          It could be one of these:
         *                  
         */ 
        int get_timestamp();

        /** Get current remainder of seconds for a day 0-86400
         *                 
         * @return          The time remainder
         *                  
         */ 
        uint32_t time_now();

        /** Return the time in seconds to HH:MMM:SS format
         *                 
         * @return          Time
         *                  
         */ 
        void timetodate(uint32_t remainder_time);


        /** Converts the time given from the user in HH.MM fromat to seconds
         *                 
         * @return          Time
         *                  
         */ 
        int timetoseconds(float scheduler_time, uint8_t group_id);

         /** Format entry with CBOR and add to the queque to sent
         *                  
         */ 
        
        void read_write_entry(uint8_t group_tag, int start_len, int end_len, uint8_t filename, uint16_t struct_size);
        
        /** Scheduler for reading metric groups at specific times each day. 
         *                  
         * @Setting/updating the saram time   ntil next reading/sensing e or intervaltc sensors measurement in seconds.
         * @return          It could be one of these:
         *                                       
         */     
        void set_scheduler(int latency, uint32_t& next_timediff);

     
        /** Overwrite the clock_synch_config. 
         *                                      
         */
        int overwrite_clock_synch_config(int time_comparator, bool clockSynchOn);
        
        /** Read the Send Scheduler holds the length and group id for each specific time. 
         *                  
         * @param time          The sleeping time until clock synch etc in seconds.
         *
         * @param clockSynchOn  True if the user want's to synch the time from server 
         *
         * @return              It could be one of these:
         *                                       
         */
        int read_clock_synch_config(uint16_t& time,bool &clockSynchOn);
        
        /* Divide the data to blocks to sent them over radio
         */
        int _divide_to_blocks(uint8_t group, uint8_t filename, uint16_t buffer_len, uint16_t struct_size, uint16_t&available);
        
        /* Divide the data to blocks to sent them over radio
         */
        int _send_blocks();
        
    
        /** SLEEP MANAGER*********************************************************************************************/
        /** Manage device sleep times before calling sleep_manager.standby().
         *  Ensure that the maximum time the device can sleep for is 6600 seconds,
         *  this is due to the watchdog timer timeout, set at 7200 seconds
         *
         * @param seconds Number of seconds to sleep for 
         * @param wkup_one If true the device will respond to rising edge interrupts
         *                 on WKUP_PIN1
         * @return None 
         */
        // void enter_standby(int seconds, bool wkup_one);

        /** Instance of TPL5010 watchdog timer 
         */
        TPL5010 watchdog;

        /** Instance of TP_Sleep_Manager to handle LL deepsleep setup 
         */
        TP_Sleep_Manager sleep_manager;

        /** TFORMATTER************************************************************************************************/
        /** Instance of TFormatter to handle serialisation of the data. Currently supports CBOR
         */
        TFormatter tformatter;
       
        /** LORAWAN **************************************************************************************************/
        #if BOARD == EARHART_V1_0_0
    
        /** Handles received messages from the Network Server .
         *
         * @return              It could be one of these:
         *                       i)  Number of bytes send on sucess.
         *                       ii) A negative error code on failure
         *                      LORAWAN_STATUS_NOT_INITIALIZED   if system is not initialized with initialize(),
         *                      LORAWAN_STATUS_NO_ACTIVE_SESSIONS if connection is not open,
         *                      LORAWAN_STATUS_WOULD_BLOCK       if another TX is ongoing,
         *                      LORAWAN_STATUS_PORT_INVALID      if trying to send to an invalid port (e.g. to 0)
         *                      LORAWAN_STATUS_PARAMETER_INVALID if NULL data pointer is given or flags are invalid
         */
        int handle_receive();

        #endif /* #if BOARD == EARHART_V1_0_0 */
        /** LORAWAN END **********************************************************************************************/

        /** NB-IoT ***************************************************************************************************/
        #if BOARD == WRIGHT_V1_0_0
        /** Attempt to connect to NB-IoT network with default parameters
         *  described in tp_nbiot_interface.h. The function blocks and will
         *  time out after 5 minutes at which point the NB-IoT modem will 
         *  regress to minimum functionality in order to conserve power whilst
         *  the application decides what to do
         */
        int initialise_nbiot();

        #endif /* #if BOARD == WRIGHT_V1_0_0 */
        /** NB-IoT ***************************************************************************************************/

        /** Conditionally instantiate _radio object and _comms_stack
         *  dependent on target board
         */
        #if BOARD == WRIGHT_V1_0_0
            TP_NBIoT_Interface _radio;
            Comms_Radio_Stack _comms_stack = Comms_Radio_Stack::NBIOT;
        #elif BOARD == EARHART_V1_0_0
            LorawanTP _radio;
            Comms_Radio_Stack _comms_stack = Comms_Radio_Stack::LORA;
        #else 
            Comms_Radio_Stack _comms_stack = Comms_Radio_Stack::UNDEFINED;
        #endif /* #if BOARD == ... */

        /**Handle the errors @todo: Critical errors that the device will need to reset if happens
         * 
         *@param line stores the line of the error in order to measure consecutive errors
         */
        void ErrorHandler(int line, const char* str1, int status, const char* str2);

        /**Read current error increment value. @todo: Critical errors that the device will need to reset if happens
         * 
         *@param increment_value increment_value
         */
        int error_increment(int &errCnt, uint16_t line, bool &error); 

        /** Eeprom details
         */
        void eeprom_debug();
        void tracking_memory();
        
        uint8_t* buffer;
        char* recv_data;
        int status;
        uint8_t send_block_number=0;
        uint8_t total_blocks=0;
        time_t starttime=0;

        //v2 new stuff
        bool clock_synch= false;
        bool kick= false;
        bool user_function= false;
        time_t hold_time=0;
       
        /**
         */
        enum
        {   
            NODEFLOW_OK                 =  0,
            DATA_MANAGER_FAIL           = -1,
            LORAWAN_TP_FAILED           = -2,
            NBIOT_TP_FAILED             = -3,
            EEPROM_DRIVER_FAILED        = -4,
            SEND_FAILED                 = -5

        };
};


/** Eeprom configuration. 
 *
 * @param DeviceConfig. Device specifics- send with the message payload.
 * @param SensorDataConfig. Each sensor will be able to store a specific amount of values (to be specified).
 * @param SchedulerConfig. Holds the scheduled times by the user.
 * @param SensingGroupConfig. Each sensor is registered in the Sensor config file.
 */
#if(OVER_THE_AIR_ACTIVATION)
    union DeviceConfig
    {
        struct 
        {
            uint16_t otaa; //true (0)
            uint8_t device_eui[8];
            uint8_t application_eui[8];
            uint8_t application_key[16];
            uint16_t hey;
        } parameters;    
        char data[sizeof(DeviceConfig::parameters)];
    };
#endif

#if(!OVER_THE_AIR_ACTIVATION)
    union DeviceConfig
    {
        struct 
        {
            uint8_t otaa; //true (0)
            uint32_t device_address;
            uint8_t net_session_key[16];
            uint8_t app_session_key[16];
        } parameters;    
        char data[sizeof(DeviceConfig::parameters)];
    };
#endif

/** */
union DataConfig
{
    struct 
    {
        uint16_t byte;
        
    } parameters;

    char data[sizeof(DataConfig::parameters)];
};

union FilesConfig
{
    struct 
    {
        uint8_t byte;
        uint8_t size;
        
    } parameters;

    char data[sizeof(FilesConfig::parameters)];
};

typedef   int (NodeFlow::*fpointer)();
union MainScheduler //clock, kick watchdog //any other time based function within nodeflow
{
    struct 
    {
        fpointer f;
        time_t trigger_time;
        time_t interval_time;
        
    } parameters;

    char data[sizeof(MainScheduler::parameters)];
};
typedef   int (*fp)();
union UserDefinedScheduler
{
    struct 
    {
        fp f;
        time_t trigger_time;
        time_t interval_time;
        
    } parameters;

    char data[sizeof(UserDefinedScheduler::parameters)];
};

union UserDefinedInterrupts
{
    struct 
    {
        fp f;
        PinName pin;
        
    } parameters;

    char data[sizeof(UserDefinedInterrupts::parameters)];
};

/** Program specific flags. Every bit is a different flag. 0:SENSE, 1:SEND, 2:CLOCK, 3:KICK
 */
union FlagsConfig
{
    struct 
    {    
        uint16_t value;
        bool  flag;
         
    } parameters;

    char data[sizeof(FlagsConfig::parameters)];
};

union ErrorConfig
{
    struct 
    {
        uint16_t errCnt;
        uint16_t line_arr[20];
    } parameters;

    char data[sizeof(ErrorConfig::parameters)];
};

/** Each filename in the eeprom hold a unique number
 */
enum Filenames
{
    ErrorConfig_n                   = 0, /**Holds an increment of concecutives errors */
    DeviceConfig_n                  = 1, 
    TempSchedulerConfig_n           = 2,
    TotalFiles_n                    = 3,
    ClockSynchFlag_n                = 4,
    MainScheduler_n                 = 5,
    UserDefinedScheduler_n          = 6,
    UserDefinedInterrupts_n         = 7,

 };

#endif // NODEFLOW_H
