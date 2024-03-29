# WilhelmiCrane-esp32

An automated crane controlled by a PS4 remote controller or android app.

<p float="left">
<img src="Crane.png" alt="logo" style="width:100px;margin: 5px"/>
<img src="psControllerColored.png" alt="logo" style="width:100px;margin: 5px"/>
<img src="smartphone-mobile-icon.png" alt="logo" style="width:100px;margin: 5px"/>
</p>


The project was created for an 9th degree student project to automate a crane.

The crane can be controlled by a wired connected arduino joystick, or after connecting a PS4 controller using a bluetooth Playstation controller.
In the last update also a playstion app was added which can be used as an alternatove to the PS4 controller, also allowing to chnange the RGB Color values.

The motors support the analog controll by using PWM motor controll.

## ESP32 code


https://github.com/bjoernboeckle/WilhelmiCrane




# Setup
The crane is controlled using an ESP32 ESP32 NodeMCU, which controlls three DC motors by using two L293D motor drivers.



| **ESP32 - Pin** | **Komponente**          | **Funktion**            |
|-----------------|-------------------------|-------------------------|
| 12              | Blaue LED               | Verbindungsstatus       |
| 35              | Schalter                | Mode (PS4 oder App)     |
| **Drehen**                                                          |
| 23              | L293D Oben Enable 1,2   | PWM                     |
| 22              | L293D Oben Input 1      | Links                   |
| 21              | L293D Oben Input 2      | Rechts                  |
| **Hoch/Runter**                                                     |
| 5               | L293D Oben Enable 3,4   | PWM                     |
| 18              | L293D Oben Input 3      | Runter                  |
| 19              | L293D Oben Input 4      | Hoch                    |
| **Laufkatze**                                                       |
| 15              | L293D Unten Enable 1,2  | PWM                     |
| 2               | L293D Unten Input 1     | Vor                     |
| 4               | L293D Unten Input 2     | Zurück                  |
| **Joystick**                                                        |
| 39              | Joystick x              | Drehen                  |
| 34              | Joystick y              | Laufkatze               |
| 32              | Joystick switch         | Hoch/Runter             |
| **RGB LED**                                                         |
| 25              | Rot                     |                         |
| 26              | Grün                    |                         |
| 27              | Blau                    |                         |