/**
 ******************************************************************************
 * @file    NodeFLow.h
 * @version 1.1.0
 * @author  Rafaella Nofytou,  Adam Mitchell
 * @brief   Header file of the Wright || Earheart node from Think Pilot. 
 * Handles sleeping times/ eeprom driver/ lorawan/ nb-iot communication
 ******************************************************************************
 */

/** Includes
 */
#include "config_device.h"
#include "DataManager.h"
#include "rtc_api_hal.h"
#include "TPL5010.h"
#include "rtos.h"
#include <cmath>
#include "tp_sleep_manager.h"

#if BOARD == EARHART_V1_0_0
    #include "LorawanTP.h"
    #define MODULATION 0
#elif BOARD == WRIGHT_V1_0_0
    #include "tp_nbiot_interface.h"
    #define MODULATION 1
#else
    #pragma message "TARGET UNKNOWN"
     #define MODULATION 2
#endif

#define size(x)  (sizeof(x) / sizeof((x)[0]))
#define DIVIDE(x) (x)/2

/**Time related defines 
 */
#define DAYINSEC    86400
#define HOURINSEC   3600
#define MINUTEINSEC 60

/** Nodeflow Class
 */
class NodeFlow: public DataManager
{
    public:

        /** Enumerated list of NodeFlow error codes
         */
        // enum 
        // {
        //     NODEFLOW_OK = 0
        // };

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
            FLAG_UNKNOWN            = 9, /**This flag should not happen*/          
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
        virtual int setup() = 0;

        /** HandleInterrupt() allows the user to define what should happen if an interrupt wakes the processor
         *  from sleep. For example, an accelerometer could be configured to detect when the device is moving 
         *  and alert the processor to this to trigger an upload of the devices current location
         */
        virtual int HandleInterrupt() = 0;

        /** HandlePeriodic() allows the user to periodically read any sensors that are on the board. Every variant 
         *  of a board is different, users different sensors, and thus requires application-specific code in order
         *  to interact with the sensors. This is where that code should be
         */  
        virtual uint8_t* HandlePeriodic(uint16_t &length) = 0; //uint8_t payload[], uint16_t &length
        /** Virtual functions END ************************************************************************************/ 

        int getPlatform();
        int HandleModem();

        /** Initialise device.
         */
        void start();

        /** Add sensors ids
         *  @param device_sn Seconds until next wakeup
         */
        int add_sensors(); //uint8_t device_sn[],uint16_t reading_time[],size_t number_of_sensors

        /** Reading sensors periodically, handles each sensor differently?!
         * @param arr    Specific times for reading the sensors,                 
         * @param length The size of the array.
         * @return       It could be one of these:
         *               i) The sleeping time until next reading sensors measurement.
         *               ii) Watchdog wakeup if the time until next reading is more than two hours
         */        
        int set_reading_time(); //uint16_t arr[],size_t n
    
        /** Scheduler for reading sensors
         * @param reading_specific_time_h_m      Specific times for reading the sensors,                   
         * @param length                         The size of the array.
         * @return                               It could be one of these:
         *                                       i) The sleeping time until next reading sensors measurement.
         *                                       ii) Watchdog wakeup if the time until next reading is more than two hours
         */        
        int set_scheduler();

        /** LORAWAN **************************************************************************************************/
        #if BOARD == EARHART_V1_0_0

        /** Join the LPWAN- TTN network either with ABP or OTAA. 
         *  You must call this before using the send/receive.
         *
         * @return        i) LORAWAN_STATUS_OK on success
         *               ii) A negative error code on failure.            
         */
        int joinTTN();

        /** Send a message from the Network Server on a specific port.
         *
         * @param port          The application port number. Port numbers 0 and 224 are reserved,
         *                      whereas port numbers from 1 to 223 (0x01 to 0xDF) are valid port numbers.
         *                      Anything out of this range is illegal.
         * @param payload       A buffer with data from the user or stored in eeproom.
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
        
        uint64_t receiveTTN();

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

    //private:
        
        /** Wakeup/time 
         */
        int initialise(); 
        uint8_t get_timestamp();
        uint32_t time_now();
        uint8_t timetodate(uint32_t remainder_time);


        /** Let's see all of our newly created file's parameters
         */
        int get_file_parameters(uint8_t filename, DataManager_FileSystem::File_t &file);
        
        /** Add a new file to EEPROM that only accepts a single entry.
         *  When adding a new file, despite the File_t type having many parameters,
         *   we only need to define the filename and length_bytes as shown
         */
        int device_config_init(uint8_t entries_to_store,uint16_t device_id,uint8_t timestamp);
    
    /**EEPROM
     */
     /** Read global stats
     */
    void get_global_stats();

        int set_time_config(int time_comparator);

        //int sched_config_init(int length);
        //int set_sched_config(int time_comparator);
        int overwrite_sched_config(uint16_t code,uint16_t length);
        int append_sched_config(uint16_t time_comparator);
        int init_sched_config();
        uint16_t read_sched_config(int i);

        int overwrite_send_sched_config(uint16_t code,uint16_t length);
        int append_send_sched_config(uint16_t time_comparator);
        int init_send_sched_config();
        uint16_t read_send_sched_config(int i);

        int overwrite_clock_synch_config(int time_comparator, bool clockSynchOn);
        
        uint16_t read_clock_synch_config(bool &clockSynchOn);
        
        /** Initialisation for all config files 
         */
        int config_init();
        int time_config_init();
        
        int sensor_config_init(int length);
        /** Set flags for the next wake up
         */
        int set_flags_config(bool kick_wdg, bool sense_time, bool clock_synch, bool send_time);

        int get_flags();
        int delay_pin_wakeup();
        int set_wakeup_pin_flag(bool wakeup_pin);
        int increment(int i);
        int read_increment();
        void _clear_increment();
        int _clear_after_send();

        /** Handle Interrupt 
         */
        uint16_t get_interrupt_latency();
        int ovewrite_wakeup_timestamp(uint16_t time_remainder);

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

        /** Conditionally instantiate _radio object and _comms_stack
         *  dependent on target board
         */
        #if BOARD == WRIGHT_V1_0_0
            TP_NBIoT_Interface _radio;
            Comms_Radio_Stack _comms_stack = Comms_Radio_Stack::NBIOT;
        #elif BOARD == EARHART_V1_0_0
            LorawanTP lpwan;
            /* Instantiate LoRa _radio object here */
            Comms_Radio_Stack _comms_stack = Comms_Radio_Stack::LORA;
        #else 
            Comms_Radio_Stack _comms_stack = Comms_Radio_Stack::UNDEFINED;
        #endif /* #if BOARD == ... */

/**Critical errors that the device will need to reset if happens */
    enum
    {   NODEFLOW_OK             =  0,
        DATA_MANAGER_FAIL       = -1,
        LORAWAN_TP_FAILED       = -2,
        NBIOT_TP_FAILED         = -3,
        EEPROM_DRIVER_FAILED    = -4,

    };
};



