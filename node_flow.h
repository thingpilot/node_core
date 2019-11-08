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
#include "mbed.h"
#include "DataManager.h"
#include "rtc_api_hal.h"
#include "TPL5010.h"
#include "LorawanTP.h"
#include <string>
#include "rtos.h"
#include <cmath>

//#include "board.h"
 

/** WakeupType enum, possible ways to wake up from sleep.
 */
enum WakeupType {
                    WAKEUP_RESET    ,
                    WAKEUP_TIMER    ,
                    WAKEUP_PIN      ,
                    WAKEUP_SOFTWARE ,
                    WAKEUP_LOWPOWER ,
                    WAKEUP_UNKNOWN  
};

enum Flags {
                    FLAG_WDG,
                    FLAG_SENSING,
                    FLAG_CLOCK_SYNCH,
                    FLAG_UNKNOWN
                    
};

/** Nodeflow Class
 */

class NodeFlow: public DataManager, public LorawanTP{ 

public:

    /** Constructor. Create a NodeFlow interface, connected to the pins specified 
     *  operating at the specified frequency
     * 
     * @param write_control GPIO to enable or disable write functionality
     * @param sda I2C data line pin
     * @param scl I2C clock line pin
     * @param frequency_hz The bus frequency in hertz. */
    NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz);

    ~NodeFlow();

    /** Initialise device.
     */
    int start(string device_id);

    /** Add sensors ids
     *  
     *  @param device_sn                     Seconds until next wakeup
     *
     *  @param 
     *  
     *
     */
    int add_sensors(uint8_t device_sn[],uint16_t reading_time[],
                    size_t number_of_sensors);
    

    /**Sets the device in standby mode.
     *  
     *  @param seconds                     Seconds until next wakeup
     *
     *  @param wkup_one
     *  
     *
     */
     void standby(int seconds, bool wkup_one, bool wkup_two);

    /** Reading sensors periodically, handles each sensor differently?!
    *
    * @param arr                            Specific times for reading the sensors,
    *                      
    * @param length                         The size of the array.
    *
    * @return                               It could be one of these:
    *                                       i) The sleeping time until next reading sensors measurement.
    *                                       ii) Watchdog wakeup if the time until next reading is more than two hours
    */        
    int set_reading_time(uint16_t arr[],size_t n);
   
   /** Scheduler for reading sensors
    *
    * @param reading_specific_time_h_m      Specific times for reading the sensors,
    *                      
    * @param length                         The size of the array.
    *
    * @return                               It could be one of these:
    *                                       i) The sleeping time until next reading sensors measurement.
    *                                       ii) Watchdog wakeup if the time until next reading is more than two hours
    */        
     int set_scheduler(float reading_specific_time_h_m[],size_t length);

     
     

   /**LORAWAN
    */
   /** Join the LPWAN- TTN network either with ABP or OTAA. 
    *
    * You must call this before using the send/receive.
    *
    * @return        i) LORAWAN_STATUS_OK on success
    *               ii) A negative error code on failure.
    *                 
    */
    int joinTTN();

   /** Send a message from the Network Server on a specific port.
    *
    * @param port          The application port number. Port numbers 0 and 224 are reserved,
    *                      whereas port numbers from 1 to 223 (0x01 to 0xDF) are valid port numbers.
    *                      Anything out of this range is illegal.
    *
    * @param payload       A buffer with data from the user or stored in eeproom.
    *
    * @param length        The size of data in bytes.
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
    int sendTTN(uint8_t port, uint8_t payload[], uint16_t length);
    
    uint64_t receiveTTN();
    
    //private:
    
    /** Wakeup/time 
     */
    int initialise(); 
    void _init_rtc();
    void SystemPower_Config();
    static void rtc_set_wake_up_timer_s(uint32_t delta);
    void clear_uc_wakeup_flags();
    static int get_wakeup_type();
    uint8_t get_timestamp();
    uint32_t time_now();
   
    /**EEPROM
     */
     /** Read global stats
     */
    int get_global_stats();

    /** Let's see all of our newly created file's parameters
     */
    int get_file_parameters(uint8_t filename, DataManager_FileSystem::File_t &file);
    
    /** Add a new file to EEPROM that only accepts a single entry.
     *  When adding a new file, despite the File_t type having many parameters,
     *   we only need to define the filename and length_bytes as shown
     */
    int add_data_config_file(uint16_t entries_to_store, uint16_t device_id,int timestamp,
                            uint16_t mode, uint16_t property, uint8_t flag,uint8_t cool);
    
    int time_config_init();
    int set_time_config(int time_comparator);
    int flags_config_init();
    
    /** Set flags for the next wake up
     */
    int set_flags_config(bool kick_wdg, bool sense_time, bool clock_synch);

    int get_flags();
   
    
};

