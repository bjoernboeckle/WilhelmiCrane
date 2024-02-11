#include <Arduino.h>

#include <L293D.h>

#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"


#define HOST_PS4_ADDRESS     "2c:9e:00:e9:c0:3e"

// Unused
//#define LED_RED                23
//#define PWM_SERVO_PIN          22
//#define PWM_FREQUENCY          50
//#define PWM_RESOLUTION          8
//#define PWM_CHANNEL_LED         3
//#define PWM_CHANNEL_SERVO       4

// Bluetooth LED
#define LED_BLUE                 12

// UNUSED_LEFT_TORIGHT           33
// RGB LED
#define RGB_LED_BLUE             25
#define RGB_LED_GREEN            26
#define RGB_LED_RED              27  


// common Motor data
#define PWM_MOTOR_FREQUENCY     150
#define PWM_MOTOR_RESOLUTION      8

// Motor Drehen
#define PWM_CHANNEL_DREHEN        0
#define DREHEN_ENABLE            23
#define DREHEN_1                 22
#define DREHEN_2                 21

// Motor Hoch/Runter
#define PWM_CHANNEL_HOCHRUNTER    3
#define HOCHRUNTER_ENABLE        05
#define HOCHRUNTER_1             18
#define HOCHRUNTER_2             19

// Motor Laufkatze
#define PWM_CHANNEL_LAUFKATZE     4
#define LAUFKATZE_ENABLE          2
#define LAUFKATZE_1               4
#define LAUFKATZE_2               0

// Joystick
#define JOYSTICK_X               39
#define JOYSTICK_Y               34
#define JOYSTICK_SWITCH          32
// UNUSED_LEFT                   35


unsigned long lastTimeStamp = 0;
unsigned long lastTimeStampPWM = 0;
#define EVENTS 0
#define BUTTONS 0
#define JOYSTICKS 1
#define SENSORS 0



L293D motorDrehen(DREHEN_1, DREHEN_2, DREHEN_ENABLE, PWM_CHANNEL_DREHEN);
L293D motorHochRunter(HOCHRUNTER_1, HOCHRUNTER_2, HOCHRUNTER_ENABLE, PWM_CHANNEL_HOCHRUNTER);
L293D motorLaufkatze(LAUFKATZE_1, LAUFKATZE_2, LAUFKATZE_ENABLE, PWM_CHANNEL_LAUFKATZE);


// debounce joystick button, revert pwm afer each press
unsigned long lastJoyButtonChanged_ms = 0;
bool joyStickButtonState = false;
int pwmJoyStickButton = 80;


// put function declarations here:
void removePairedDevices();
void printDeviceAddress();
void notify();
void onConnect();
void onDisConnect();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BLUE, OUTPUT);                // Bluetooth state
  pinMode(JOYSTICK_SWITCH, INPUT_PULLUP);   // Joystick button

  //ledcSetup(PWM_CHANNEL_SERVO, PWM_FREQUENCY, PWM_RESOLUTION);
  //ledcSetup(PWM_CHANNEL_LED, PWM_FREQUENCY, PWM_RESOLUTION);
  //ledcAttachPin(PWM_SERVO_PIN, PWM_CHANNEL_SERVO);
  //ledcAttachPin(LED_RED, PWM_CHANNEL_LED);
  //ledcWrite(PWM_CHANNEL_SERVO, 0);
  //ledcWrite(PWM_CHANNEL_LED, 0);

  // Motoren
  motorDrehen.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  motorHochRunter.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
  motorLaufkatze.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);

  //ledcAttachPin(RGB_LED_RED, PWM_CHANNEL_DREHEN);
  pinMode(RGB_LED_GREEN, OUTPUT);                // Bluetooth state green
  pinMode(RGB_LED_BLUE, OUTPUT);                 // Bluetooth state blue
  pinMode(RGB_LED_RED, OUTPUT);                  // Bluetooth state red

  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin(HOST_PS4_ADDRESS);
  removePairedDevices(); // This helps to solve connection issues
  Serial.print("This device MAC is: ");
  printDeviceAddress();
  Serial.println("");  
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
  double pwmPercentDrehen = GetPwmPercentValue(PS4.LStickX());
  double pwmPercentHochRunter = GetPwmPercentValue(PS4.LStickY());
  double pwmPercentLaufkatze = GetPwmPercentValue(PS4.RStickY());
  
  // motoren
  motorDrehen.SetMotorSpeed(pwmPercentDrehen);
  motorHochRunter.SetMotorSpeed(pwmPercentHochRunter);
  motorLaufkatze.SetMotorSpeed(pwmPercentLaufkatze);

  if (PS4.PSButton())
    removePairedDevices();


  digitalWrite(RGB_LED_BLUE, PS4.Circle() ? HIGH : LOW);
  digitalWrite(RGB_LED_GREEN, PS4.Triangle() ? HIGH : LOW);
  digitalWrite(RGB_LED_RED, PS4.Square() ? HIGH : LOW);

  if (PS4.R1())
    PS4.setRumble(10, 50);


  // Servo
  //double pwm = pwmPercent < 0 ? 2.5 : (2.5 + (pwmPercent / 10.0));
  //ledcWrite(PWM_CHANNEL_SERVO, (int)(pwm * 255.0 / 100.0));

  // LED
  //ledcWrite(PWM_CHANNEL_LED, abs((int)(pwmPercent * 255.0 / 100.0)));
}


void handleDisConnectedController()
{
    motorDrehen.SetMotorSpeed(0);
    motorHochRunter.SetMotorSpeed(0);
    motorLaufkatze.SetMotorSpeed(0);
    //ledcWrite(PWM_CHANNEL_LED, 0);
    //ledcWrite(PWM_CHANNEL_SERVO, 2.5*255.0 / 100.0);

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
    

    //Serial.print("PWM X: ");
    //Serial.print(joyX);
    //Serial.print(": ");
    //Serial.print(pwmJoyX);
    //Serial.print("   PWM Y: ");
    //Serial.print(joyY);
    //Serial.print(": ");
    //Serial.print(pwmJoyY);
    //Serial.print("   Switch: ");
    //Serial.println(digitalRead(JOYSTICK_SWITCH));
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

  if (millis() - lastTimeStampPWM > 100)
  {
    lastTimeStampPWM = millis();

    if(PS4.isConnected()) 
      handleConnectedController();
    else
    {
      handleDisConnectedController();
      digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
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

void notify() {
#if EVENTS
  boolean sqd = PS4.event.button_down.square,
          squ = PS4.event.button_up.square,
          trd = PS4.event.button_down.triangle,
          tru = PS4.event.button_up.triangle;
  if (sqd)
    Serial.println("SQUARE down");
  else if (squ)
    Serial.println("SQUARE up");
  else if (trd)
    Serial.println("TRIANGLE down");
  else if (tru)
    Serial.println("TRIANGLE up");
#endif

#if BUTTONS
  boolean sq = PS4.Square(),
          tr = PS4.Triangle();
  if (sq)
    Serial.print(" SQUARE pressed");
  if (tr)
    Serial.print(" TRIANGLE pressed");
  if (sq | tr)
    Serial.println();
#endif

  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp > 1500) {
#if JOYSTICKS
    Serial.printf("lx:%4d,ly:%4d,rx:%4d,ry:%4d\n",
                  PS4.LStickX(),
                  PS4.LStickY(),
                  PS4.RStickX(),
                  PS4.RStickY());
#endif
#if SENSORS
    Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d\n",
                  PS4.GyrX(),
                  PS4.GyrY(),
                  PS4.GyrZ(),
                  PS4.AccX(),
                  PS4.AccY(),
                  PS4.AccZ());
#endif
    lastTimeStamp = millis();
  }
}

void onDisConnect() {
  Serial.println("Disconnected!");
}