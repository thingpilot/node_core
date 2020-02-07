/**
 ******************************************************************************
 * @file    NodeFLow.h
 * @version 0.4.0
 * @author  Rafaella Nofytou,  Adam Mitchell
 * @brief   Header file of the Wright || Earheart node from Think Pilot. 
 * Handles sleeping times/ eeprom driver/ lorawan/ nb-iot communication
 ******************************************************************************
 */
#pragma once
/** Includes
 */

#include "mbed.h"
#include "config_device.h"
#include "DataManager.h"
#include "TPL5010.h"
#include "tp_sleep_manager.h"
#include "tformatter.h"
#include <cmath>
#include <bitset>
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

/** Time related defines 
 */
#define DAYINSEC    86400
#define HOURINSEC   3600
#define MINUTEINSEC 60

/** Define retries for sending
 */
#define MAX_SEND_RETRIES 3
#define MAX_OVERWRITE_RETRIES 3

#if (!SCHEDULER_B)
    #define SCHEDULER_B_SIZE 0
#endif
#if (!SCHEDULER_C)
    #define SCHEDULER_C_SIZE 0
#endif
#if (!SCHEDULER_D)
    #define SCHEDULER_D_SIZE 0
#endif
#if (SCHEDULER)
    #define SCHEDULER_SIZE (SCHEDULER_A_SIZE + SCHEDULER_B_SIZE +  SCHEDULER_C_SIZE + SCHEDULER_D_SIZE)
    extern float scheduler[]; 
    extern float schedulerA[];
    extern float schedulerB[];
    extern float schedulerC[];
    extern float schedulerD[];
#endif

#if(!SCHEDULER)
    #define SCHEDULER_SIZE METRIC_GROUPS_ON
    extern float scheduler[]; 
#endif

#if(SEND_SCHEDULER)
     extern float send_scheduler[];
#endif

#define MAX_BUFFER_SENDING_TIMES 10

/** Eeprom configuration. 
 *
 * @param DeviceConfig. Device specifics- send with the message payload.
 * @param SensorDataConfig. Each sensor will be able to store a specific amount of values (to be specified).
 * @param SchedulerConfig. Holds the scheduled times by the user.
 * @param SensingGroupConfig. Each sensor is registered in the Sensor config file.
 */
union DeviceConfig
{
    struct 
    {
        uint32_t device_sn; //Device unique id?! our unique id?
        uint8_t modulation; //defined 0 or 1 for lora, nbiot respectively,
        uint8_t APPEUI[8];
        uint8_t DEVEUI[8];
        uint8_t APPKEY[16];
        
    } parameters;    char data[sizeof(DeviceConfig::parameters)];
};

/** */
union MetricGroupAConfig
{
    struct 
    {
        uint16_t byte;
        
    } parameters;

    char data[sizeof(MetricGroupAConfig::parameters)];
};
#if (SCHEDULER_B || METRIC_GROUPS_ON==4 ||METRIC_GROUPS_ON==3 || METRIC_GROUPS_ON==2)
    union MetricGroupBConfig
    {
        struct 
        {
            uint16_t byte;
        } parameters;

        char data[sizeof(MetricGroupBConfig::parameters)];
    };
#endif
#if (SCHEDULER_C || METRIC_GROUPS_ON==4 || METRIC_GROUPS_ON==3)
    union MetricGroupCConfig
    {
        struct 
        {
            uint16_t byte;
        } parameters;

        char data[sizeof(MetricGroupCConfig::parameters)];
    };
#endif
#if (SCHEDULER_D || METRIC_GROUPS_ON==4)
    union MetricGroupDConfig
    {
        struct 
        {
            uint16_t byte;
        } parameters;

        char data[sizeof(MetricGroupDConfig::parameters)];
    };
#endif

union InterruptConfig
{
    struct 
    {
        uint16_t byte;
    } parameters;

    char data[sizeof(InterruptConfig::parameters)];
};

union MetricGroupEntriesConfig
{
    struct 
    {
        uint8_t MetricGroupAEntries;
        uint8_t MetricGroupBEntries;
        uint8_t MetricGroupCEntries;
        uint8_t MetricGroupDEntries;
        uint16_t InterruptEntries;

    } parameters;

    char data[sizeof(MetricGroupEntriesConfig::parameters)];
};

/** The User can define MAX_BUFFER_READING_TIMES 
 */
union SchedulerConfig
{
    struct 
    {   
        uint16_t time_comparator; 
        uint8_t group_id;    
    } parameters;

    char data[sizeof(SchedulerConfig::parameters)];
};

union MultiSchedulerConfig
{
    struct 
    {   
        uint16_t time_comparator; 
        uint8_t group_id;
    } parameters;

    char data[sizeof(MultiSchedulerConfig::parameters)];

};

/** The User can define MAX_BUFFER_READING_TIMES 
 */
union SendSchedulerConfig
{
    struct 
    {   
        uint16_t time_comparator;
    } parameters;

    char data[sizeof(SendSchedulerConfig::parameters)];
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

/** Program specific flags. Every bit is a different flag. 0:SENSE, 1:SEND, 2:CLOCK, 3:KICK
 */
union FlagsConfig
{
    struct 
    {    
        uint8_t ssck_flag;
        bool  pin_wakeup;
         
    } parameters;

    char data[sizeof(FlagsConfig::parameters)];
};

union NextTimeConfig
{
    struct 
    {
        uint32_t time_comparator;
        
    } parameters;

    char data[sizeof(NextTimeConfig::parameters)];
};

union IncrementAConfig
{
    struct 
    {    
        uint16_t  increment; 
    } parameters;

    char data[sizeof(IncrementAConfig::parameters)];
};

union IncrementBConfig
{
    struct 
    {    
        uint32_t  increment; 
    } parameters;

    char data[sizeof(IncrementBConfig::parameters)];
};
union IncrementCConfig
{
    struct 
    {    
        uint64_t  increment; 
    } parameters;

    char data[sizeof(IncrementCConfig::parameters)];
};

/** MetricGroupTimes Config for storing the values of ,MetricGroupTimes Config, Time Config
 */
union MetricGroupTimesConfig
{
    struct 
    {   
        uint32_t time_comparator; 
    } parameters;

    char data[sizeof(MetricGroupTimesConfig::parameters)];
};

union TempMetricGroupTimesConfig
{
    struct 
    {  
        uint32_t time_comparator; 
    } parameters;

    char data[sizeof(TempMetricGroupTimesConfig::parameters)];
};

/**TODO: Merge with ssck_flags group,Flags for each group */
union MetricGroupConfig
{
    struct 
    {
        uint16_t metric_group_id;        
        
    } parameters;

    char data[sizeof(MetricGroupConfig::parameters)];

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
    MetricGroupAConfig_n            = 1, 
    SchedulerConfig_n               = 2,
    ClockSynchConfig_n              = 3,
    FlagsConfig_n                   = 4, 
    IncrementAConfig_n              = 5,
    MetricGroupTimesConfig_n        = 6, 
    TimeConfig_n                    = 7,
    TempMetricGroupTimesConfig_n    = 8,
    NextTimeConfig_n                = 9,
    SendSchedulerConfig_n           = 10,
    MetricGroupConfig_n             = 11,
    IncrementBConfig_n              = 12,
    IncrementCConfig_n              = 13,
    MetricGroupBConfig_n            = 14,
    MetricGroupCConfig_n            = 15,
    MetricGroupDConfig_n            = 16,
    MetricGroupEntriesConfig_n      = 17,
    InterruptConfig_n               = 18
    
 };

/** Nodeflow Class
 */
class NodeFlow: public DataManager
{
    public:

        /** Enumerated list of flags 
         */
        enum Flags 
        {
            FLAG_WDG                = 0,
            FLAG_SENSING            = 1,
            FLAG_CLOCK_SYNCH        = 2,
            FLAG_SENDING            = 3,
            FLAG_WAKEUP_PIN         = 4,
            FLAG_SENSE_SYNCH        = 5,
            FLAG_SENSE_SEND         = 6,     
            FLAG_SEND_SYNCH         = 7,
            FLAG_SENSE_SEND_SYNCH   = 8,
            FLAG_UNKNOWN            = 9 /**This should not happen*/          
        };

        /** Enumerated list of possible comms radio stacks
         */
        enum class Comms_Radio_Stack
        {
            NBIOT     = 0,
            LORA      = 1,
            UNDEFINED = 10
        };
        
        /** CONSTRUCTORS *********************************************************************************************/
        #if BOARD == EARHART_V1_0_0
        /** Constructor. Create a NodeFlow interface, connected to the pins specified 
         *  operating at the specified frequency
         * 
         * @param write_control GPIO to enable or disable write functionality
         * @param sda I2C data line pin
         * @param scl I2C clock line pin
         * @param frequency_hz The bus frequency in hertz. 
         */
        NodeFlow(PinName write_control=TP_EEPROM_WC, PinName sda=TP_I2C_SDA, PinName scl=TP_I2C_SCL, int frequency_hz=TP_I2C_FREQ, 
                PinName mosi=TP_LORA_SPI_MOSI, PinName miso=TP_LORA_SPI_MISO, PinName sclk=TP_LORA_SPI_SCK, PinName nss=TP_LORA_SPI_NSS, PinName reset=TP_LORA_RESET,
                PinName dio0=PB_4, PinName dio1=PB_1, PinName dio2=PB_0, PinName dio3=PC_13, PinName dio4=NC, PinName dio5=NC, PinName rf_switch_ctl1=NC, 
                PinName rf_switch_ctl2=NC, PinName txctl=NC, PinName rxctl=NC, PinName ant_switch=NC, PinName pwr_amp_ctl=NC, PinName tcxo=TP_VDD_TCXO,PinName done=TP_DONE);
        #endif /* #if BOARD == EARHART_V1_0_0 */

        #if BOARD == WRIGHT_V1_0_0
        NodeFlow(PinName write_control=TP_EEPROM_WC, PinName sda=TP_I2C_SDA, PinName scl=TP_I2C_SCL, int frequency_hz=TP_I2C_FREQ,
                PinName txu=TP_NBIOT_TXU, PinName rxu=TP_NBIOT_RXU, PinName cts=TP_NBIOT_CTS, PinName rst=TP_NBIOT_RST, 
                PinName vint=TP_NBIOT_VINT, PinName gpio=TP_NBIOT_GPIO, int baud=TP_NBIOT_BAUD, PinName done=TP_DONE);
        #endif /* #if BOARD == WRIGHT_V1_0_0 */
        /** CONSTRUCTORS END *****************************************************************************************/

        /** NodeFlow destructor 
         */
        ~NodeFlow();

        /** VIRTUAL FUNCTIONS *****************************************************************************************
         *  Virtual functions MUST be overridden by the application developer. The description of these functions 
         *  is given above each virtual definition
         */
        
        /** setup() allows the user to write code that will only be executed once when the device is initialising.
         *  This is akin to Arduino's setup function and can be used to, for example, configure a sensor
         */
        virtual void setup() = 0;

        /** HandleInterrupt() allows the user to define what should happen if an interrupt wakes the processor
         *  from sleep. For example, an accelerometer could be configured to detect when the device is moving 
         *  and alert the processor to this to trigger an upload of the devices current location
         */
        virtual void HandleInterrupt() = 0;

        /** MetricGroupA-D() allows the user to periodically read any sensors that are on the board. Every variant 
         *  of a board is different, uses different sensors, and thus requires application-specific code in order
         *  to interact with the sensors.
         */  
        virtual void MetricGroupA() = 0; 
       
        virtual void MetricGroupB() = 0;
      
        virtual void MetricGroupC() = 0;
    
        virtual void MetricGroupD() = 0;
    
        /** Virtual functions END ************************************************************************************/

        
        /** start() drives all the application. It handles the different modem and configuration.
         */
        void start();
        /**Template function for handling the different data types
         */
        template <typename DataType>
        /**add_record(DataType data) for adding variable type records to eeprom
         *
         *@param data Actual data to be written to the eeprom
         */
        void add_record(DataType data, string str1=NULL);

        
        /**Increment with a value.
         * 
         *@param i increment value
         */
        int inc_a(int i);

        /**Read current increment value.
         * 
         *@param increment_value increment_value
         */
        int read_inc_a(uint16_t& increment_value);

        /**Increment with a value.
         * 
         *@param i increment value
         */
        int inc_b(int i);

        /**Read current increment value.
         * 
         *@param increment_value increment_value
         */
        int read_inc_b(uint16_t& increment_value);

        /**Clears the increment value.
         * 
         *@param increment_value increment_value
         */
        int clear_inc_b();
        /**Increment with a value.
         * 
         *@param i increment value
         */
        int inc_c(int i);

        /**Read current increment value.
         * 
         *@param increment_value increment_value
         */
        int read_inc_c(uint64_t& increment_value);
        
        // int counter(uint8_t& entries_counter); //todo remove?
        // int read_counter(uint8_t &entries_counter);


    private:

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
        
        /** Initialise the time config file. Holds the time until the next wakeup
         *                 
         * @return          time_comparator
         *                  
         */ 
        int time_config_init();
        /** Holds the time until the next wakeup
         *                 
         * @return          time_comparator
         *                  
         */ 
        int set_time_config(int time_comparator);

        /**Interval/ periodic sensing of metric groups *******************************************************
         */

        /** Initialise the metric groups file.
         *                 
         * @return          It could be one of these:
         *                  
         */ 
        int metric_config_init(int length);
        /** Adds the metric groups for interval sensing times, handles each group differently
         *                 
         * @return          It could be one of these:
         *                  
         */    
        void add_metric_groups(); 

        /** Sets the next reading time for interval periodic sensing, handles each group differently
         *                 
         * @param time      The sleeping time until next reading/sensing etc sensors measurement in seconds.
         * @return          It could be one of these:
         *                  
         */        
        int set_reading_time(uint32_t& time); 

        /** Fixes the next reading times after interrupt/clock etc interrupt for each group differently
         *                 
         * @param time      The sleeping time until next reading/sensing etc sensors measurement in seconds.
         * @return          It could be one of these:
         *                  
         */
         int set_temp_reading_times(uint32_t time);
        
        /** Specific times for each sensing of metric groups *******************************************************
         */

        /** Initialise the Scheduler for reading metric groups at specific times each day. 
         *                  
         * @return          It could be one of these:                         
         */
        int init_sched_config();

        /** Scheduler for reading metric groups at specific times each day. 
         *                  
         * @param time      The sleeping time until next reading/sensing etc sensors measurement in seconds.
         * @return          It could be one of these:
         *                                       
         */     
        void set_scheduler(int latency, uint32_t& next_timediff);

        /** Scheduler holds the length and group id for each specific times 
         *                  
         * @param time      The sleeping time until next reading/sensing etc sensors measurement in seconds.
         * @return          It could be one of these:
         *                                       
         */ 
        int read_sched_config(int i,uint16_t& time_comparator);

        int read_sched_group_id(int i,uint8_t& time_comparator);
        /** Overwrite the Scheduler holds the length and group id for each specific time. 
         *                                      
         */ 
        int overwrite_sched_config(uint16_t code,uint16_t length);
         /** Append entries to the scheduler. 
         *                                      
         */ 
        int append_sched_config(uint16_t time_comparator, uint8_t group_id);

        /** Initialise the Send Scheduler for reading metric groups at specific times each day. 
         *                  
         * @return          It could be one of these:                         
         */
        int init_send_sched_config();

        /** Read the Send Scheduler holds the length and group id for each specific time. 
         *                  
         * @param time      The sleeping time until next reading/sensing/sending etc in seconds.
         * @return          It could be one of these:
         *                                       
         */ 
        int read_send_sched_config(int i, uint16_t& time);
        /** Overwrite the send scheduler. 
         *                                      
         */
        int overwrite_send_sched_config(uint16_t code,uint16_t length);

         /** Append entries to the send scheduler. 
         *                                      
         */ 
        int append_send_sched_config(uint16_t time_comparator);
        
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

        /** Get wakeup flag.
         *
         *@return               It could be one of these:
         *                      FLAG_WDG
         *                      FLAG_SENSING
         *                      FLAG_CLOCK_SYNCH
         *                      FLAG_SENDING
         *                      FLAG_WAKEUP_PIN
         *                      FLAG_SENSE_SYNCH
         *                      FLAG_SENSE_SEND   
         *                      FLAG_SEND_SYNCH
         *                      FLAG_SENSE_SEND_SYNCH
         *                      FLAG_UNKNOWN
         */
        int get_wakeup_flags();


        /** Set flags. 4 flags for sensing, sending, clock synchronisation and kick the watchdog
         *
         *@return               It could be one of these:
         *                      FLAG_SENSING = dec(1)
         *                      FLAG_SENDING = dec(2) 
         *                      FLAG_SENSE_SEND = dec(3)
         *                      FLAG_CLOCK_SYNCH = dec(4)
         *                      FLAG_SENSE_SYNCH = dec(5) 
         *                      FLAG_SEND_SYNCH = dec(6)
         *                      FLAG_SENSE_SEND_SYNCH = dec(7)
         *                      FLAG_WDG = dec(8) 
         */
        int set_flags_config(uint8_t ssck_flag);

        /** Set wakeup pin flag to true or false.
         *
         */
        int set_wakeup_pin_flag(bool wakeup_pin);

        /** Metric flags are used for each of the groups A to D. 
         *
         *@return               A decimal represantation of:
         *                      MetricGroupA = dec(1)
         *                      MetricGroupB = dec(2) 
         *                      MetricGroupA&&B = dec(3)
         *                      MetricGroupC = dec(4)
         *                      MetricGroupA&&C = dec(5) 
         *                      MetricGroupB&&C = dec(6)
         *                      MetricGroupA&&B&&C  = dec(7)
         *                      MetricGroupD = dec(8)
         *                      ..
         *                      MetricGroupA&&B&&C&&D = dec(15)
         */
        int overwrite_metric_flags(uint8_t ssck_flag);

        /** Get metric flags are used for each of the groups A to D in order to handle each group after a wakeup timer. 
         *
         *@return               A decimal represantation of:
         *                      MetricGroupA = dec(1)
         *                      MetricGroupB = dec(2) 
         *                      MetricGroupA&&B = dec(3)
         *                      MetricGroupC = dec(4)
         *                      MetricGroupA&&C = dec(5) 
         *                      MetricGroupB&&C = dec(6)
         *                      MetricGroupA&&B&&C  = dec(7)
         *                      MetricGroupD = dec(8)
         *                      ..
         *                      MetricGroupA&&B&&C&&D = dec(15)
         */
        int get_metric_flags(uint8_t &flag);

        int cbor_object_string(const string& object_str, const string& input_str);
        int add_cbor_payload_data(uint8_t metric_group_flag);
        
        void _sense();
        void _send(bool interrupt_send);
        /**Adds a bytes of sensing entries added as record by the user.
         */
        int add_sensing_entry(uint8_t value, uint8_t metric_group);

        bool is_overflow();

        /**Counter for each metric group entry
         * 
         *@param mg_flag which metric group to increment
         */
        int mg_counter( uint8_t mg_flag);

        /**Read current counter for each metric group entry
         * 
         *@return mg_entries for each group
         */
        int read_mg_counter(uint16_t& mga_entries, uint16_t& mgc_entries,uint16_t& mgd_entries, uint16_t& mgb_entries,uint8_t & metric_group_active);
        
        /**Read current bytes written for each metric group
         * 
         *@return mg_bytes for each mgroup
         */
        int read_mg_bytes(int& mga_bytes, int& mgb_bytes, int& mgc_bytes, int& mgd_byte, int& interrupt_bytes); 

        /** INTERRUPT**************************************************************************************************/
        /** Handle Interrupt 
         */
        int read_interrupt_counter(uint16_t& interrupt_entries);

        int is_delay_pin_wakeup_flag();

        int correct_latency(int latency);

        /** Re-measures the sleeping time after an interrupt 
         */
        int get_interrupt_latency(uint32_t &next_sch_time);

        /** Holds the wakeup (full timestamp). 
            TODO: this will be used to check that we didn't missed a measurement while on program not implemented
         */
        int overwrite_wakeup_timestamp(uint16_t time_remainder);

        /** CLEARS**************************************************************************************************/
        /**Clear the counter entries
         */
        int clear_mg_counter();

        /**Clears the increment/s.
         */
        int clear_increment();

        /**Clears the increment/s && the eeprom after sending
         */
        int clear_after_send();

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
        void enter_standby(int seconds, bool wkup_one);

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

        /** Send a message from the Network Server on a specific port.
         *
         * @param port          The application port number. Port numbers 0 and 224 are reserved,
         *                      whereas port numbers from 1 to 223 (0x01 to 0xDF) are valid port numbers.
         *                      Anything out of this range is illegal.
         * @param payload       The buffer to be sent.
         * @param length        The size of data in bytes.
         * @return              It could be one of these:
         *                       i)  Number of bytes send on sucess.
         *                       ii) A negative error code on failure
         *                      LORAWAN_STATUS_NOT_INITIALIZED   if system is not initialized with initialize(),
         *                      LORAWAN_STATUS_NO_ACTIVE_SESSIONS if connection is not open,
         *                      LORAWAN_STATUS_WOULD_BLOCK       if another TX is ongoing,
         *                      LORAWAN_STATUS_PORT_INVALID      if trying to send to an invalid port (e.g. to 0)
         *                      LORAWAN_STATUS_PARAMETER_INVALID if NULL data pointer is given or flags are invalid
         */        
        int sendTTN(uint8_t port, uint8_t payload[], uint16_t length);
        
        /** Receive a message from the Network Server on a specific port.
         *
         * @param port          The application port number. Port numbers 0 and 224 are reserved,
         *                      whereas port numbers from 1 to 223 (0x01 to 0xDF) are valid port numbers.
         *                      
         * @param rx_message    Received message.
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
        int receiveTTN(uint32_t& rx_message, uint8_t& rx_port);

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
         *@param increment_value increment_value
         */
        int error_increment(int &errCnt, uint16_t line, bool &error); 

        /** Eeprom details
         */
        void eeprom_debug();
        void tracking_memory();
        
        #if(SCHEDULER)
            float* scheduler;
        #endif
        uint8_t* buffer;
        int status;
        /**
         */
        enum
        {   
            NODEFLOW_OK                 =  0,
            DATA_MANAGER_FAIL           = -1,
            LORAWAN_TP_FAILED           = -2,
            NBIOT_TP_FAILED             = -3,
            EEPROM_DRIVER_FAILED        = -4,

        };
};



