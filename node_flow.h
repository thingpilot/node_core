#include "DataManager.h"
#include "board.h"

class NodeFlow {

    public: 
    NodeFlow(PinName write_control, PinName sda, PinName scl, int frequency_hz);

    ~NodeFlow();

    int initialise(); 

    
    DataManager _dm;


    
};