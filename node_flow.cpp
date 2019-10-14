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
    {   uint8_t device_id;
        uint8_t device_sn;
        uint8_t device_type;
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

 wdg.kick();
// TPL5010::kick(); 
 _init_rtc();

 //WakeupType wkp;
 int wkp=get_wakeup_type();
 
 if (wkp==WAKEUP_PIN) {
    pc.printf("Already initialised, wakeup from pin %d\r\n", wkp);

    
 }
 if (wkp==WAKEUP_TIMER) {
    pc.printf("Already initialised");
    
 }

 else{
    pc.printf("Initialising");
    initialise();
    
 }
    return wkp;
}

int NodeFlow::initialise(){
 
 status=DataManager::init_filesystem();

 bool initialised = false;

 status=DataManager::is_initialised(initialised);
 if(status!=0){
 pc.printf("Filesystem initialisation failed. status: %i, is initialised: %i\r\n", status, initialised);   
    
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
        pc.printf("\r\nadd_file status: %i\r\n", status);
        
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
        pc.printf("append_file_entry No: %i status: %i\r\n", i, status);
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

 int NodeFlow::add_sensors( uint8_t device_sn[],uint8_t device_type[],uint16_t reading_time[],
                             size_t number_of_sensors) {
  
    get_global_stats();
    if (number_of_sensors>8){
        status=-1; //change 
        pc.printf("Error more than 8 sensors. Status %d \r\n", status);   
        
    }

    else {
        DataManager_FileSystem::File_t SensorConfig_File_t;
        SensorConfig_File_t.parameters.filename = SensorConfig_n;
        SensorConfig_File_t.parameters.length_bytes = sizeof(SensorConfig::parameters);
        status=DataManager::add_file(SensorConfig_File_t, number_of_sensors);
        for (int i=0; i<number_of_sensors; i++){

            if(status!=0){
                pc.printf("Unsuccess! status: %i\r\n", status);
            }

            else{
                pc.printf("\r\nAdd_file status: %i\r\n", status);
                
                SensorConfig s_conf;
                s_conf.parameters.device_id=i;
                s_conf.parameters.device_sn = device_sn[i];
                s_conf.parameters.device_type =device_type[i];
                s_conf.parameters.time_comparator=reading_time[i];
               
            
               
                status = DataManager::append_file_entry(SensorConfig_n, s_conf.data, sizeof(s_conf.parameters));
                if(status!=0){
                pc.printf("Error append_file_entry No: %i status: %i\r\n", i, status);
                
                }
                else{
                status = DataManager::read_file_entry(SensorConfig_n, i, s_conf.data, sizeof(s_conf.parameters));
                pc.printf("read_file_entry attempt %i status: %i\r\n", i, status);
                pc.printf("device_id: %u\r\n", s_conf.parameters.device_id);
                pc.printf("device_type: %i\r\n", s_conf.parameters.device_type);
                pc.printf("time_comparator: %u\r\n", s_conf.parameters.time_comparator);
                }
            }
        }
    
    }
     get_global_stats();
     return status;
}

int NodeFlow::set_reading_time(uint16_t arr[], int n){
      //each time the user creates a new sensor  
    
    int time_comparator=0; 
    int temp=0;

    
   // int sizeofarray=sizeof(arr)/sizeof(arr[0]);
    for (int i=0; i<=n; i++){
        //int temp=arr[i];
        if (temp>=arr[i] && arr[i]!=0){
            temp=arr[i];   
        }
    time_comparator=temp;
    printf("\r\nMin: %d , %d",time_comparator, n);
    printf("\r\n %d",time_comparator);
    } 
    return time_comparator;

}

RTC_HandleTypeDef RtcHandle;

void NodeFlow::_init_rtc() {
   PlatformMutex *mtx = new PlatformMutex;
   mtx->lock();
   rtc_init();
   mtx->unlock();
   delete(mtx);
}
void NodeFlow::SystemPower_Config() {
   HAL_Init();
   GPIO_InitTypeDef GPIO_InitStructure;
   __HAL_RCC_PWR_CLK_ENABLE();
   HAL_PWREx_EnableUltraLowPower();
   HAL_PWREx_EnableFastWakeUp();
   __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOH_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();
   GPIO_InitStructure.Pin = GPIO_PIN_All;
   GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStructure.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
   HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
   __HAL_RCC_GPIOA_CLK_DISABLE();
   __HAL_RCC_GPIOB_CLK_DISABLE();
   __HAL_RCC_GPIOC_CLK_DISABLE();
   __HAL_RCC_GPIOD_CLK_DISABLE();
   __HAL_RCC_GPIOH_CLK_DISABLE();
   __HAL_RCC_GPIOE_CLK_DISABLE();
}


 void NodeFlow::rtc_set_wake_up_timer_s(uint32_t delta) {
   uint32_t clock = RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
   // HAL_RTCEx_SetWakeUpTimer_IT will assert that delta is 0xFFFF at max
   if (delta > 0xFFFF) {
       delta -= 0x10000;
       clock = RTC_WAKEUPCLOCK_CK_SPRE_17BITS;
   }
   RtcHandle.Instance = RTC;
   HAL_StatusTypeDef status = HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, delta, clock);
   if (status != HAL_OK) {
       NVIC_SystemReset();
    }
}
void NodeFlow::clear_uc_wakeup_flags() {
   __HAL_RCC_CLEAR_RESET_FLAGS();
   SET_BIT(PWR->CR, PWR_CR_CWUF);
}

int NodeFlow::get_wakeup_type(){
 if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
       return WAKEUP_RESET;
   }
   if(READ_BIT(RTC->ISR, RTC_ISR_WUTF)) {
       return WAKEUP_TIMER;
   }
   if(READ_BIT(PWR->CSR, PWR_CSR_WUF)) {
       return WAKEUP_PIN;
   }
   return WAKEUP_RESET;
    
}


int NodeFlow:: enter_standby(int intervals) {
return 0;
}
int NodeFlow::standby(int seconds, bool wkup_one, bool wkup_two) {
   SystemPower_Config();
   core_util_critical_section_enter();
   clear_uc_wakeup_flags();
   // Enable wakeup timer.
   rtc_set_wake_up_timer_s(seconds);
   if(wkup_one) {
       HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
   }
   else {
       HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
   }
   if(wkup_two) {
       HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
   }
   else {
       HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
   }
   HAL_PWR_EnterSTANDBYMode();
   // this should not happen...
   //rtc_deactivate_wake_up_timer();
   core_util_critical_section_exit();
   // something went wrong, let's reset
   NVIC_SystemReset();
}








