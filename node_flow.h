
 /** Includes
 */
#include "mbed.h"
#include "DataManager.h"
#include "board.h"
#include "rtc_api_hal.h"
#include "TPL5010.h"
#include "SX1276_LoRaRadio.h"
#include "lorawan/LoRaWANInterface.h"
#include <string>

  /** Get wakeup type
     */ 
    enum WakeupType {
    WAKEUP_RESET    ,
    WAKEUP_TIMER    ,
    WAKEUP_PIN      ,
    WAKEUP_SOFTWARE ,
    WAKEUP_LOWPOWER ,
    WAKEUP_UNKNOWN  

    };

class NodeFlow: public DataManager{ //, public TPL5010{

    public: 
    NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz);

    

    ~NodeFlow();


    /** Check to see if filesystem is initialised
     */
    int start(string device_id);

    int initialise(); 

    /** Read global stats
     */
    int get_global_stats();

    /** Let's see all of our newly created file's parameters
     */
    int get_file_parameters(uint8_t filename, DataManager_FileSystem::File_t &file);

    int add_sensors(uint8_t device_sn[],uint8_t device_type[],uint16_t reading_time[],
                    size_t number_of_sensors);


    /**
     *  Sets the device in standby mode. Handles the times, 
     *  @param periodic_intervals   Sets periodic time intervals for device to wakeup
     *  periodic times must equal with the highest common factor
     *  @param interrupt_sleep_time Sets time that the interrupt will sleep after first wakeup
     *  ex. enter_standby(periodic_intervals, interrupt_sleep_time);
     */
    
     int enter_standby(int intervals=NULL);
     void standby(int seconds, bool wkup_one, bool wkup_two);
    

     int set_reading_time(uint16_t arr[],int n);

     bool isReadingTime(int device_id);

    //private:
    
    /** Wakeup/time 
     */
    
    void _init_rtc();
    void SystemPower_Config();
    static void rtc_set_wake_up_timer_s(uint32_t delta);
    void clear_uc_wakeup_flags();
    static int get_wakeup_type();
    int get_timestamp();

    void SystemClock_Config();
    /**
     
     */
    int wakeup();

    
    /** Add a new file to EEPROM that only accepts a single entry.
     *  When adding a new file, despite the File_t type having many parameters,
     *   we only need to define the filename and length_bytes as shown
     */
    int add_data_config_file(uint16_t entries_to_store, uint16_t device_id,int timestamp,
                            uint16_t mode, uint16_t property, uint8_t flag,uint8_t cool);
    
    int add_time_config_file();
   
   // int return_status(wchar_t filename);
    
    
};

// class Sensor: public I2C

// {
// public:
//  /*Status*/
//     typedef int Sensor_Status_t;

//         enum 
//         {
//             SENSOR_OK                            = 0, /**< enum value 1 */
//             SENSOR_READ_FAIL                     = 1,
//             SENSOR_WRITE_FAIL                    = 2,
//             SENSOR_VERIFY_FAIL                   = 3,
//             SENSOR_CHIP_ERROR                    = -255
//         };
// Sensor(int device_id);
// //~Sensor(int device_id);

// virtual int ReadSignature();
// virtual float ReadValue()=0;
// virtual unsigned long ReadDeviceId();

// uint16_t read2Bytes(int chip_addr, int offset);

// private: 
// virtual bool isReadingTime()=0; //derived class defines if is reading time

// };