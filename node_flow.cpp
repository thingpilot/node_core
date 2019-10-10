#include "node_flow.h"


int status = -1;
Serial pc(PC_TXD, PC_RXD);

NodeFlow::NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz): 
 DataManager(write_control, sda, scl, frequency_hz) {

 }


 NodeFlow::~NodeFlow() {


 }


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

union SensorA
{
    struct 
    {
        float temp;
        int timestamp;
    } parameters;

    char data[sizeof(SensorA::parameters)];
};

union SensorB
{
    struct 
    {
        uint16_t hum;
        int timestamp;
    } parameters;

    char data[sizeof(SensorB::parameters)];
};

enum Filenames
{
    DeviceConfig_n = 0,
    SensorA_n      = 1,
    SensorB_n      = 2
};

int NodeFlow::initialise(){

 
 status=DataManager::init_filesystem();

 bool initialised = false;
 if(DataManager::is_initialised(initialised)!=true){
    int status = -1;
    status=DataManager::init_filesystem();
    return 255;
 }
 //pc.printf("is_initialised status: %i, is initialised: %i\r\n", status, initialised);
   else {
       return 0;
   }
}

int NodeFlow::get_global_stats() {
    DataManager_FileSystem::GlobalStats_t g_stats;
    status = DataManager::get_global_stats(g_stats.data);
    DataManager::print_global_stats(pc, g_stats);

}