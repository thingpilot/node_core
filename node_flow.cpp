#include "node_flow.h"

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
 int status = -1;
 status=DataManager::init_filesystem();
 return 0;
}