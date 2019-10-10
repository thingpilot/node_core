#include "mbed.h"
#include "DataManager.h"
#include "board.h"

class NodeFlow: public DataManager{

    public: 
    NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz);

    ~NodeFlow();

    int initialise(); 

    int get_global_stats();



    
};