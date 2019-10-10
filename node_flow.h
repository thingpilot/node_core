#include "mbed.h"
#include "DataManager.h"
#include "board.h"

class NodeFlow: public DataManager{

    public: 
    NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz);

    ~NodeFlow();


    /** Check to see if filesystem is initialised
     */
    int start();

    int initialise(); 

    /** Read global stats
     */
    int get_global_stats();

    /** Let's see all of our newly created file's parameters
     */
    int get_file_parameters(uint8_t filename, DataManager_FileSystem::File_t &file);

    int add_sensors(uint16_t device_id[],uint16_t device_type[],uint16_t reading_time[],
                    uint16_t number_of_sensors);



    private:

    /** Add a new file to EEPROM that only accepts a single entry.
     *  When adding a new file, despite the File_t type having many parameters,
     *   we only need to define the filename and length_bytes as shown
     */
    int add_data_config_file(uint16_t entries_to_store, uint16_t device_id,int timestamp,
                            uint16_t mode, uint16_t property, uint8_t flag,uint8_t cool);


    

    
    
    
};