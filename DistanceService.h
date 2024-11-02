#include <ble/att_server.h>

#define DIST_GATT_SUFFIX "-4b72-696c-6f6e-446973740000"

#define DIST_GATT_CAT(a) a DIST_GATT_SUFFIX
#define DIST_GATT_SERVICE DIST_GATT_CAT("53727601")
#define DIST_GATT_CHAR DIST_GATT_CAT("4d737201")  

class DistanceService
{
  public:

  DistanceService();

  static uint16_t SetupBtStack();

  void Init(uint16_t serviceHandle);
  void DistanceServiceNotify(unsigned long currentMillis = -1);
  void DistanceServiceUpdateValue(uint16_t distanceInMm);

  void Disconnect();

  private:
    static void DistanceServiceCanSendNow(void * context);
    static uint16_t GattDistanceServiceReadCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
    static int GattDistanceServiceWriteCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
   
    hci_con_handle_t con_handle;
    att_service_handler_t distance_service;

    // characteristic: Distance Mesurement 
    uint16_t measurement_value_handle;

    // characteristic descriptor: Client Characteristic Configuration
    uint16_t measurement_client_configuration_descriptor_handle;
    uint16_t measurement_client_configuration_descriptor_notify;
    btstack_context_callback_registration_t measurement_callback;

    uint16_t distanceValue;
    bool valueUpdated;
    bool reportWithCmResolution;
    unsigned long prevNotifyMillis;
};

extern DistanceService distanceService;