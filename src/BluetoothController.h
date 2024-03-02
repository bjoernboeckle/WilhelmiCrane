
#include "BluetoothSerial.h"

#define BLUETOOTH_VISIBLE_NAME "WilhelmiCrane"
#define BT_DISCOVER_TIME  10000


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
 

class BluetoothControl
{
public:
    BluetoothControl();
    bool begin();
    bool end();


    bool IsBTClientConnected;
    int BTredValue;
    int BTgreenValue;
    int BTblueValue;
    int BTDrehenValue;
    int BTHochRunterValue;
    int BTLaufkatzeValue;

    String inputBuffer;

    void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

private:
    BluetoothSerial SerialBT;   // Bluetooth Serial Object (Handle)
    bool btScanSync = false;
    void InitBluetooth(const char * btName, esp_spp_cb_t handler);
};

