#include "node_flow.h"


int status = -1;
Serial pc(PC_TXD, PC_RXD);


NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz): 
 DataManager(write_control, sda, scl, frequency_hz) {

 }


 NodeFlow::~NodeFlow() {

 }



/** Eeprom configuration- how the eeprom will look like which files initialised
 *  
 */
    union DeviceConfig
{
    struct 
    {
        uint16_t device_id;
        int timestamp;
        uint16_t mode;
        uint16_t property;
        uint8_t flag;
        uint8_t cool;
    } parameters;

    char data[sizeof(DeviceConfig::parameters)];
};

    union SensorConfig
{
    struct 
    {
        uint16_t device_id;
        uint16_t device_type;
        uint16_t time_comparator; 
    
    } parameters;

    char data[sizeof(SensorConfig::parameters)];
};

union SensorA
{
    struct 
    {
        float value;
        uint16_t device_type;
        int timestamp;
    } parameters;

    char data[sizeof(SensorA::parameters)];
};

union SensorB
{
    struct 
    {
        uint16_t value;
        uint16_t device_type;
        int timestamp;
    } parameters;

    char data[sizeof(SensorB::parameters)];
};
union BatteryVoltage
{
    struct 
    {
        uint16_t device_type;
        int timestamp;


    } parameters;

    char data[sizeof(BatteryVoltage::parameters)];
};

enum Filenames
{
    DeviceConfig_n   = 0,
    SensorA_n        = 1,
    SensorB_n        = 2,
    SensorConfig_n   = 3,
    BatteryVoltage_n = 9   
};

int NodeFlow::start(){

    initialise();
    return 0;
}

int NodeFlow::initialise(){

 
 status=DataManager::init_filesystem();

 bool initialised = false;

 status=DataManager::is_initialised(initialised);
 if(status!=true){
 pc.printf("Filesystem initialisation faileds_initialised status: %i, is initialised: %i\r\n", status, initialised);   
    return 255;
 }
 //
   else {
       pc.printf("init_filesystem status: %i\r\n", status);   
   }
    return status;
}

int NodeFlow::get_global_stats() {
    DataManager_FileSystem::GlobalStats_t g_stats;
    status = DataManager::get_global_stats(g_stats.data);
    DataManager::print_global_stats(pc, g_stats);
 
 return status;
}

int NodeFlow::add_data_config_file(uint16_t entries_to_store,uint16_t device_id,int timestamp,
                            uint16_t mode, uint16_t property, uint8_t flag,uint8_t cool){

    DataManager_FileSystem::File_t DeviceConfig_File_t;
    DeviceConfig_File_t.parameters.filename = DeviceConfig_n;
    DeviceConfig_File_t.parameters.length_bytes = sizeof(DeviceConfig::parameters);

     if(DataManager::add_file(DeviceConfig_File_t, entries_to_store)!=0){
        
         pc.printf("Unsuccess! status: %i\r\n", status);
     }

     else{
        pc.printf("add_file status: %i\r\n", status);
        
        DeviceConfig w_conf;
        w_conf.parameters.device_id = device_id;
        w_conf.parameters.timestamp = timestamp;
        w_conf.parameters.flag = mode;
        w_conf.parameters.mode = property;
        w_conf.parameters.property = flag;
        w_conf.parameters.cool = cool;
        for(int i = 0; i < 2; i++)
    {
        w_conf.parameters.device_id = i;
        status = DataManager::append_file_entry(DeviceConfig_n, w_conf.data, sizeof(w_conf.parameters));
        pc.printf("append_file_entry attempt %i status: %i\r\n", i, status);
    }
     }
    
return status;
}


/*Device config, Sensor_1-8*/
 int NodeFlow::get_file_parameters(uint8_t filename, DataManager_FileSystem::File_t &file){
    
    // status = DataManager::get_file_by_name(filename, file);
    // DataManager::print_file(pc, file);
    return status;
 }

 int NodeFlow::add_sensors( uint16_t device_id[],uint16_t device_type[],uint16_t reading_time[],
                             uint16_t number_of_sensors) {
  
    if (number_of_sensors>8){
        pc.printf("Error more than 8 sensors %i\r\n", status);   
        
    }

    else {
        for (int i=0; i<=number_of_sensors; i++){
            DataManager_FileSystem::File_t SensorConfig_File_t;
            SensorConfig_File_t.parameters.filename = SensorConfig_n;
            SensorConfig_File_t.parameters.length_bytes = sizeof(SensorConfig::parameters);

            if(DataManager::add_file(SensorConfig_File_t, 1)!=0){
                pc.printf("Unsuccess! status: %i\r\n", status);
            }

            else{
                pc.printf("add_file status: %i\r\n", status);
                
                SensorConfig s_conf;
                s_conf.parameters.device_id = device_id[i];
                s_conf.parameters.device_type = device_type[i];
                s_conf.parameters.time_comparator=reading_time[i];
            
                for(int i = 0; i < 2; i++){
                s_conf.parameters.device_id = i;
                status = DataManager::append_file_entry(SensorConfig_n, s_conf.data, sizeof(s_conf.parameters));
                pc.printf("append_file_entry attempt %i status: %i\r\n", i, status);
                }    
            }
        }
    
    }
     return status;
}






