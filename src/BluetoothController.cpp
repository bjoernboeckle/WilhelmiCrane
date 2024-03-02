
#include "BluetoothController.h"


static BluetoothControl* g_BluetoothApp = NULL;
void g_BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) { g_BluetoothApp->BT_EventHandler(event, param); }


BluetoothControl::BluetoothControl()
{
    IsBTClientConnected = false;
    BTredValue = 0;
    BTgreenValue = 0;
    BTblueValue = 0;
    BTDrehenValue = 0;
    BTHochRunterValue = 0;
    BTLaufkatzeValue = 0;
}

bool BluetoothControl::begin()
{
    if (g_BluetoothApp != NULL)
      return false;

    g_BluetoothApp = this;
    InitBluetooth(BLUETOOTH_VISIBLE_NAME, g_BT_EventHandler);
    return true;
}


bool BluetoothControl::end()
{
  if (g_BluetoothApp == NULL)
    return false;

  SerialBT.end();
  g_BluetoothApp = NULL;
}

void BluetoothControl::InitBluetooth(const char * btName, esp_spp_cb_t handler) 
{
    // Serial.println(btName);
    if(SerialBT.begin(btName)) // Bluetooth device name
    {
        int i = 0;
        while(!SerialBT.isReady())
        {
            vTaskDelay(100);
            if(i++ > 15)
            {
                Serial.printf("INIT FAILED: Bluetooth device '%s' failed to initialize after %d ms.\n\r", btName, i*100);
                return;
            }
        }

        Serial.printf("Bluetooth device '%s' initialized after %d ms.\n\r", btName, i*100);
        
        if (btScanSync) {
            Serial.println("Starting discover...");
            BTScanResults *pResults = SerialBT.discover(BT_DISCOVER_TIME);
            if (pResults)
                pResults->dump(&Serial);
            else
                Serial.println("Error on BT Scan, no result!");
        }
        // Attach The CallBack Function Definition To SerialBlutooth Events
        //SerialBT.register_callback(&handler); // Attach The CallBack Function Definition To SerialBlutooth Events

        // If above line is in error, try the line below.
        // For Earlier versions of BluetoothSerial.h (SerialBT)
        SerialBT.register_callback(handler); 
    }
}



// Handle Bluetooth events
void BluetoothControl::BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_START_EVT) 
  {
    Serial.println("Bluetooth Initialized...");
  }
  else if (event == ESP_SPP_SRV_OPEN_EVT ) 
  {
    IsBTClientConnected = true;
    inputBuffer.clear();
    Serial.println("BT Client connected");
  }
  else if (event == ESP_SPP_CLOSE_EVT  ) 
  {
    IsBTClientConnected = false;
    inputBuffer.clear();
    Serial.println("BT Client disconnected");
  }
  else if (event == ESP_SPP_DATA_IND_EVT ) 
  {
    // INCOMING DATA HANDLED HERE...
    bool eof = false;

    // Save any input bytes to buffer
    while (SerialBT.available()) 
    {
      // Read a character from the input (may be many characters to read)
      char incoming = SerialBT.read();

      // is the current character NOT a CR ('\r')? 
      if(!(eof = incoming == '\r'))
      {
        // a return character ('\r') defines the end of a packet
        // if no return is received, just continue to get data
        inputBuffer += incoming;
        continue;
      }
    }

    // packet is not full (no CR was rcvd), so just return
    if(!eof) return;

    // Process a full packet, CR was found
    inputBuffer.toUpperCase();

    // Take a look at what's in the buffer
    Serial.println(inputBuffer);

    if (inputBuffer.startsWith("RED="))
      sscanf(&inputBuffer[4], "%d", &BTredValue);
    else if (inputBuffer.startsWith("GREEN="))
      sscanf(&inputBuffer[6], "%d", &BTgreenValue);
    else if (inputBuffer.startsWith("BLUE="))
      sscanf(&inputBuffer[5], "%d", &BTblueValue);

    else if (inputBuffer.startsWith("DREHEN="))
      sscanf(&inputBuffer[7], "%d", &BTDrehenValue);
    else if (inputBuffer.startsWith("HOCH="))
      sscanf(&inputBuffer[5], "%d", &BTHochRunterValue);
    else if (inputBuffer.startsWith("LAUFKATZE="))
      sscanf(&inputBuffer[10], "%d", &BTLaufkatzeValue);


    // Clear the input to be ready for another command
    inputBuffer.clear();
  }
}