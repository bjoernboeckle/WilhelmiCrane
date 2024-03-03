#include <Arduino.h>
#include <map>
#include <L293D.h>
#include <PS4Controller.h>
#include "esp_bt_device.h"

#include "BluetoothController.h"


#define HOST_PS4_ADDRESS     "2c:9e:00:e9:c0:3e"


// Bluetooth LED
#define LED_BLUE                 12

// Buzzer
//#define BUZZER                   33

// RGB LED
#define PWM_FREQUENCY           150
#define PWM_RESOLUTION            8
#define PWM_CHANNEL_RED           3
#define PWM_CHANNEL_GREEN         4
#define PWM_CHANNEL_BLUE          5
#define RGB_LED_BLUE             25
#define RGB_LED_GREEN            26
#define RGB_LED_RED              27  


// common Motor data
#define PWM_MOTOR_FREQUENCY     150
#define PWM_MOTOR_RESOLUTION      8

// Motor Drehen
#define PWM_CHANNEL_DREHEN        6
#define DREHEN_ENABLE            23
#define DREHEN_1                 22
#define DREHEN_2                 21

// Motor Hoch/Runter
#define PWM_CHANNEL_HOCHRUNTER    1
#define HOCHRUNTER_ENABLE         5
#define HOCHRUNTER_1             18
#define HOCHRUNTER_2             19

// Motor Laufkatze
#define PWM_CHANNEL_LAUFKATZE     2
#define LAUFKATZE_ENABLE         15
#define LAUFKATZE_1               2
#define LAUFKATZE_2               4

// Joystick
#define JOYSTICK_X               39
#define JOYSTICK_Y               34
#define JOYSTICK_SWITCH          32

#define BLUETOOTH_SELECT         35


unsigned long lastTimeStampPWM = 0;
#define EVENTS 0
#define BUTTONS 0
#define JOYSTICKS 1
#define SENSORS 0



BluetoothControl BluetoothApp;


L293D motorDrehen(DREHEN_1, DREHEN_2, DREHEN_ENABLE, PWM_CHANNEL_DREHEN);
L293D motorHochRunter(HOCHRUNTER_1, HOCHRUNTER_2, HOCHRUNTER_ENABLE, PWM_CHANNEL_HOCHRUNTER);
L293D motorLaufkatze(LAUFKATZE_1, LAUFKATZE_2, LAUFKATZE_ENABLE, PWM_CHANNEL_LAUFKATZE);


// debounce joystick button, revert pwm afer each press
bool IsJoyStickEnabled = false;
unsigned long lastJoyButtonChanged_ms = 0;
bool joyStickButtonState = false;
int pwmJoyStickButton = 80;


// put function declarations here:
bool IsPs4Mode = false;
void removePairedDevices();
void printDeviceAddress();
void onConnect();
void onDisConnect();
void HandleLED();
void InitBluetoothMode(bool exitCurMode = false);
bool CheckJoystickHelath();
double GetPwmJoystickValue(int16_t joystickValue);


void setup() 
{
  // put your setup code here, to run once:
  pinMode(LED_BLUE, OUTPUT);                // Bluetooth state
  pinMode(JOYSTICK_SWITCH, INPUT_PULLUP);   // Joystick button
  pinMode(BLUETOOTH_SELECT, INPUT);         // Bluetooth switch
  

  //ledcSetup(PWM_CHANNEL_SERVO, PWM_FREQUENCY, PWM_RESOLUTION);
  //ledcAttachPin(PWM_SERVO_PIN, PWM_CHANNEL_SERVO);
  //ledcWrite(PWM_CHANNEL_SERVO, 0);

  // Motoren
  motorDrehen.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  motorHochRunter.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  motorLaufkatze.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);

  
  pinMode(RGB_LED_RED, OUTPUT);                  // Bluetooth state red
  pinMode(RGB_LED_GREEN, OUTPUT);                // Bluetooth state green
  pinMode(RGB_LED_BLUE, OUTPUT);                 // Bluetooth state blue
  ledcSetup(PWM_CHANNEL_RED, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_GREEN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BLUE, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(RGB_LED_RED, PWM_CHANNEL_RED);
  ledcAttachPin(RGB_LED_GREEN, PWM_CHANNEL_GREEN);
  ledcAttachPin(RGB_LED_BLUE, PWM_CHANNEL_BLUE);

  Serial.begin(115200);
  
  IsJoyStickEnabled = CheckJoystickHelath();
  
  InitBluetoothMode(false);
}


bool CheckJoystickHelath()
{
    bool curJoyButtonStart = (digitalRead(JOYSTICK_SWITCH) == 0);
    double pwmJoyXStart =  GetPwmJoystickValue(analogRead(JOYSTICK_X));
    double pwmJoyYStart =  GetPwmJoystickValue(analogRead(JOYSTICK_Y));
    
    for(int i=0; i<20; i++)
    {
      delay(10);
      if ( curJoyButtonStart != (digitalRead(JOYSTICK_SWITCH) == 0) ||
           pwmJoyXStart != GetPwmJoystickValue(analogRead(JOYSTICK_X)) ||
           pwmJoyYStart != GetPwmJoystickValue(analogRead(JOYSTICK_Y)) )
      {
        Serial.println("Joystick health failed - disabled!");
        return false;
      }
    }

    Serial.println("Joystick health ok - enabled!");
    return true;
}


void InitBluetoothMode(bool exitCurMode)
{
  if (exitCurMode)
  {
    if (IsPs4Mode)
    {
      ESP.restart();  // PS4 exit doesn't work
      removePairedDevices();
      PS4.end();
      Serial.println("PS4 Mode stopped");
    }
    else
    {
      BluetoothApp.end();
      Serial.println("Bluetooth App stopped");
    }
  }

  IsPs4Mode = digitalRead(BLUETOOTH_SELECT);
  if (IsPs4Mode)
  {
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);      
    PS4.begin(HOST_PS4_ADDRESS);
    removePairedDevices(); // This helps to solve connection issues
    Serial.print("PS4 Mode: This device MAC is: ");
    printDeviceAddress();
    Serial.println("");
  }
  else
  {
    Serial.print("APP Mode: ");
    BluetoothApp.begin();
  }
}



double GetPwmPercentValue(int8_t stickValue)
{
  double pwmValue = 0;
  int StickValue = stickValue;
  
  if ( abs(StickValue) > 27)
    pwmValue = StickValue  -  (stickValue < 0 ? -27 : 27);  // 0..127, minus 27 --> 0..100%

  return pwmValue;
}



double GetPwmJoystickValue(int16_t joystickValue)
{
  double pwmValue = 0;
  int StickValue = joystickValue - 1900;   // 0...4095 --> -2000...2095 (ignoring 95)
  
  if ( abs(StickValue) > 200)
  {
    if (StickValue > 0 )
      pwmValue = (StickValue - 200) / 19.95;  // 0....1995
    else
    pwmValue = (StickValue + 200) / 17.0;  // 0....1700
  }

  return pwmValue;
}


void handleConnectedController()
{
  if (PS4.PSButton())
  {
    removePairedDevices();
    return;
  }

  double pwmPercentDrehen = GetPwmPercentValue(PS4.LStickX());
  double pwmPercentHochRunter = GetPwmPercentValue(PS4.RStickY());
  double pwmPercentLaufkatze = GetPwmPercentValue(PS4.LStickY());
  
  // motoren
  motorDrehen.SetMotorSpeed(pwmPercentDrehen);
  motorHochRunter.SetMotorSpeed(pwmPercentHochRunter);
  motorLaufkatze.SetMotorSpeed(pwmPercentLaufkatze);
}



void HandleLedColor(double pwmValue, int pwmChannel, bool buttonPressed, int btValue)
{
  if (pwmValue == 0)
    ledcWrite(pwmChannel, (BluetoothApp.IsBTClientConnected ? btValue : (buttonPressed ? 255 : 0) ) );
  else
    ledcWrite(pwmChannel, abs(pwmValue) * 255 / 100);
}

void HandleLED()
{
  HandleLedColor(motorDrehen.GetCurrentMotorSpeed(), PWM_CHANNEL_RED, PS4.Circle(), BluetoothApp.BTredValue);
  HandleLedColor(motorHochRunter.GetCurrentMotorSpeed(), PWM_CHANNEL_GREEN, PS4.Triangle(), BluetoothApp.BTgreenValue);
  HandleLedColor(motorLaufkatze.GetCurrentMotorSpeed(), PWM_CHANNEL_BLUE, PS4.Square(), BluetoothApp.BTblueValue);
}



void handleDisConnectedController()
{
    if (IsJoyStickEnabled)
    {
      uint16_t joyX = analogRead(JOYSTICK_X);
      uint16_t joyY = analogRead(JOYSTICK_Y);

      double pwmJoyX =  GetPwmJoystickValue(joyX);
      double pwmJoyY =  GetPwmJoystickValue(joyY);

      // motoren
      motorDrehen.SetMotorSpeed(pwmJoyX);
      motorLaufkatze.SetMotorSpeed(pwmJoyY);
      if (joyStickButtonState)
        motorHochRunter.SetMotorSpeed(pwmJoyStickButton);
      else
        motorHochRunter.SetMotorSpeed(0);
    
      //Serial.print("PWM X: ");       Serial.print(joyX);
      //Serial.print(": ");            Serial.print(pwmJoyX);
      //Serial.print("   PWM Y: ");    Serial.print(joyY);
      //Serial.print(": ");            Serial.print(pwmJoyY);
      //Serial.print("   Switch: ");   Serial.println(digitalRead(JOYSTICK_SWITCH));
    }
}

void handleBTConnected()
{
    // motoren
    motorDrehen.SetMotorSpeed(BluetoothApp.BTDrehenValue);
    motorLaufkatze.SetMotorSpeed(BluetoothApp.BTLaufkatzeValue);
    motorHochRunter.SetMotorSpeed(BluetoothApp.BTHochRunterValue);
}


void HandleJoyStickButton()
{
  // handle joystickbutton
  bool curJoyButton = digitalRead(JOYSTICK_SWITCH) == 0;
  
  if (lastJoyButtonChanged_ms == 0 && curJoyButton != joyStickButtonState) // state has changed
  {
    Serial.println("Button changed");
    lastJoyButtonChanged_ms = millis();
  }
  else if (lastJoyButtonChanged_ms != 0 && curJoyButton != joyStickButtonState && (millis() - lastJoyButtonChanged_ms) > 200 ) 
  {
    // state has changed for more than 200ms
    joyStickButtonState = curJoyButton;
    lastJoyButtonChanged_ms = 0;

    if (!curJoyButton)
      pwmJoyStickButton = pwmJoyStickButton * -1;
  }
}



void loop() 
{
  HandleJoyStickButton();
  if (millis() - lastTimeStampPWM > 80)
  {
    lastTimeStampPWM = millis();

    if(PS4.isConnected()) 
    {
      handleConnectedController();
      //digitalWrite(LED_BLUE, HIGH);  OnConnect
    }
    else if(BluetoothApp.IsBTClientConnected)
    {
      digitalWrite(LED_BLUE, HIGH);
      handleBTConnected();
    }
    else
    {
      handleDisConnectedController();
      digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    }
    HandleLED();

    if (IsPs4Mode != digitalRead(BLUETOOTH_SELECT) )
    {
      InitBluetoothMode(true);
    }
  }
}


void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}

void onConnect() {
  Serial.println("Connected!");
  digitalWrite(LED_BLUE, HIGH);
}



void onDisConnect() {
  Serial.println("Disconnected!");
}