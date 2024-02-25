
​
Ritesh Persaud
​
// Import the Libraries into the file

#include <Servo.h>

#include <Ultrasonic.h>

#include <Wire.h>

#include "SSD1306.h"

#include <ESP8266WiFi.h>

#include "23F_PubSubClient.h"

#include "23F_WiFiManager.h"

#include <math.h>

 

// Setting up the MQTT Server Variables

// Communication

char payload_global[130];

bool flag_payload;

 

// Server Address/Arena Number, Username, Password, Unique ID, Port Number

const char *mqtt_server = "192.168.0.140"; // north

const char *MQusername = "user";

const char *MQpassword = "Stevens1870";

const char *MQtopic = "louis_lidar_new";

const int mqtt_port = 1883;

 

// const char* mqtt_server = "arena2";//south

// const char* MQusername = "user";

// const char* MQpassword = "Stevens1870";

// const char* MQtopic = "louis_lidar_new";

// const int mqtt_port = 1883;

 

// const char* mqtt_server = "192.168.0.218";//competition

// const char* MQusername = "user";

// const char* MQpassword = "Stevens1870";

// const char* MQtopic = "louis_lidar_comp";

// const int mqtt_port = 1883;

 

// WiFi SSID and Password

const char *ssid = "TP-Link_4DB1";

const char *password = "01747331";

 

// const char *ssid = "TP-Link_8FFD"; //Competition

// const char *password = "68287078";

 

// const char* ssid = "TP-Link_9402"; // South Wifi

// const char* password = "77556578";

 

// Define the WiFi Network

WiFiClient espClient;

PubSubClient client(espClient);

 

// Define the pins of the OLED Display

SSD1306 display(0x3c, D14, D15);

 

// Define the pins the motors will use

#define motorRpin D0

#define motorLpin D2

 

// Create objects for the motors

Servo motorR;

Servo motorL;

 

// Variable that will read and contain the data from each sensor

int distanceTop;

int distanceRight;

int distanceLeft;

 

// Robot Position Variables

int xPoss = 0;

int yPoss = 0;

int xTarg[6] = {1600, 1600, 2000, 110, 300, 700};

int yTarg[6] = {130, 600, 700, 150, 550, 130};

int targ = 0;

int xPrev = 300;

int yPrev = 900;

int turnConstant = 750; // 850

int VP = 0;

float alpha = 0;

int distToTarget = 0;

int targetDelay = 0;

int dx = 0;

int dy = 0;

double turnDelay = 0;

double driftDelay = 0;

int counterPath = 5;

int counterPoss = 0;

 

// Create objects of the Ultrasonic Sensors with pins

// Echos are D5-D7 (Purple)

// Trigs are D8-D10 (Blue)

Ultrasonic ultrasonic_top(D8, D5);

Ultrasonic ultrasonic_right(D10, D6);

Ultrasonic ultrasonic_left(D9, D7);

 

// Connect to WiFi

void setup_wifi()

{

    delay(10);

    // We start by connecting to a Stevens WiFi network

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)

    {

        delay(500);

        Serial.print(".");

    }

    randomSeed(micros());

}

// Reconnect to WiFi

void reconnect()

{

    // Loop until we're reconnected

    while (!client.connected())

    {

        // Create a random client ID

        String clientId = "ESP8266Client-";

        clientId += String(random(0xffff), HEX);

        // Attempt to connect

        if (client.connect(clientId.c_str(), MQusername, MQpassword))

        {

            client.subscribe(MQtopic);

        }

        else

        {

            // Wait 5 seconds before retrying

            delay(5000);

        }

    }

}

// Connect to Server

void callback(char *topic, byte *payload, unsigned int length)

{

    for (int i = 0; i < length; i++)

    {

        payload_global[i] = (char)payload[i];

    }

    payload_global[length] = '\0';

    flag_payload = true;

}

// Robot Movement Functions

void forward()

{

    motorR.write(111);

    motorL.write(115);

}

void stopMotors()

{

    motorR.write(90);

    motorL.write(90);

}

void slower()

{

    motorR.write(103);

    motorL.write(103);

}

void reverseLeft()

{

    motorR.write(180);

    motorL.write(0);

}

void reverseRight()

{

    motorR.write(0);

    motorL.write(180);

}

void left()

{

    motorR.write(106); // 110

    motorL.write(73);  // 70

}

void left_drift()

{

    motorR.write(106);

    motorL.write(81);

}

void right()

{

    motorR.write(70);  // 70

    motorL.write(107); // 110

}

void reverse()

{

    motorR.write(0);

    motorL.write(0);

}

void right_drift()

{

    motorR.write(81);

    motorL.write(106);

}

// Pathfinder & Angel Functions

// The path finder is not calibrated, in the begining

void pathFind()

{

    int line1X = xPoss - xPrev;

    int line1Y = yPoss - yPrev;

    int line2X = xTarg[targ] - xPoss;

    int line2Y = yTarg[targ] - yPoss;

 

    double magL1 = sqrt(pow(line1X, 2) + pow(line1Y, 2));

    double magL2 = sqrt(pow(line2X, 2) + pow(line2Y, 2));

    double DP = (line1X * line2X) + (line1Y * line2Y);

 

    alpha = acos(DP / (magL1 * magL2)) * 180 / PI;

    VP = ((xPoss - xPrev) * (yTarg[targ] - yPoss)) - (yPoss - yPrev) * (xTarg[targ] - xPoss);

 

    if (VP < 0)

    {

        turnDelay = turnConstant * alpha / 90;

        right();

        delay(turnDelay);

    }

    else if (VP > 0)

    {

        turnDelay = turnConstant * alpha / 90;

        left();

        delay(turnDelay);

    }

}

void setup()

{

    // put your setup code here, to run once:

    Serial.begin(115200);

 

    // Connecting to the Server and WiFi

    setup_wifi();

    delay(3000);

    Serial.println("Wemos POWERING UP ......... ");

    client.setServer(mqtt_server, mqtt_port);

    client.setCallback(callback);

 

    // Setting up Motors

    motorR.attach(motorRpin);

    motorL.attach(motorLpin);

 

    // Setting up Display

    display.init();

    display.setFont(ArialMT_Plain_16);

    display.drawString(0, 0, "ROBOTBOY");

    display.drawString(0, 15, "POWERING");

    display.drawString(0, 30, "UP....");

    display.display();

    delay(2000);

    display.clear();

}

 

void loop()

{

    // Right Motor is D0, R1, M1

    // Left Motor is D2, R2, M2

 

    // Get data from MQTT server testCollector[0-1] have x and y positions

    if (!client.connected())

    {

        Serial.print("...");

        reconnect();

    }

    client.loop();

 

    String payload(payload_global);

    int testCollector[10];

    int count = 0;

    int prevIndex, delimIndex;

 

    prevIndex = payload.indexOf('[');

    while ((delimIndex = payload.indexOf(',', prevIndex + 1)) != -1)

    {

        testCollector[count++] = payload.substring(prevIndex + 1, delimIndex).toInt();

        prevIndex = delimIndex;

    }

    delimIndex = payload.indexOf(']');

    testCollector[count++] = payload.substring(prevIndex + 1, delimIndex).toInt();

 

    // Update the Robot's X,Y Location

    xPoss = testCollector[0];

    yPoss = testCollector[1];

 

    // Sensors update

    distanceTop = ultrasonic_top.read(CM);

    distanceLeft = ultrasonic_left.read(CM);

    distanceRight = ultrasonic_right.read(CM);

 

    // At Target 1st check

    dx = xTarg[targ] - xPoss;

    dy = yTarg[targ] - yPoss;

    distToTarget = sqrt(pow(dx, 2) + pow(dy, 2));

 

    if (distToTarget < 300) {

      if (targ == 1 || targ == 4)

        {

            slower();

            delay(100);

            xPoss = xTarg[targ];

            yPoss = yTarg[targ];

            targ++;

            pathFind();

            counterPath = 0;

        }        

    }

 

    if (distToTarget < 150) // 150

    {  

        pathFind();

        slower();

        delay(400); // 600

        stopMotors();

        delay(1000);

        xPoss = xTarg[targ];

        yPoss = yTarg[targ];

        targ++;

        reverse();

        delay(100);

        pathFind();

        counterPath = 0;

        // pathFind();

        // slower();

        // delay(600); //600

        // stopMotors();

        // delay(1000);

        // xPoss = xTarg[targ];

        // yPoss = yTarg[targ];

        // targ++;

        // pathFind();

        // counterPath = 0;

        // //counterPoss = 0;

    }

 

    /////////////////       PATHFINDER          ///////////////

 

    if ((counterPath >= 20) && ((xPoss != xPrev || yPoss != yPrev) && (yPoss < 5000)))

    {

        pathFind();

        counterPath = 0;

    }

 

    //////////////           DISPLAY          /////////////////

 

    // Orange-SDA D14 Yellow-SCL D15

    // Create arrays containing 50 empty characters

    char displayRight[50];

    char displayLeft[50];

    char displayTop[50];

    char displayXPos[50];

    char displayYPos[50];

    char displayTarget[50];

    char displayAlpha[50];

    char displayCounter[50];

 

    // Convert the data from the ultrasonic sensors into elements in array

    sprintf(displayRight, "Right (CM): %d", distanceRight);

    sprintf(displayLeft, "Left (CM): %d", distanceLeft);

    sprintf(displayTop, "Top (CM): %d", distanceTop);

    sprintf(displayTarget, "Target: %d", targ + 1);

    sprintf(displayAlpha, "Alpha: %d", counterPoss);

    sprintf(displayXPos, "X Position: %d", xPoss);

    sprintf(displayYPos, "Y Position: %d", yPoss);

    sprintf(displayCounter, "CounterP: %d", counterPath);

 

    // Reassign the arrays to a new variable

    const char *r = displayRight;

    const char *l = displayLeft;

    const char *t = displayTop;

    const char *x = displayXPos;

    const char *y = displayYPos;

    const char *ta = displayTarget;

    const char *a = displayAlpha;

    const char *c = displayCounter;

 

    // Display the distances from the sides and top of the robot

    // display.drawString(0, 15, r);

    // display.drawString(0, 30, l);

    // display.drawString(0, 45, t);

    // display.drawString(0, 15, ta);

    display.drawString(0, 15, a);

    display.drawString(0, 30, ta);

    display.drawString(0, 45, c);

 

    display.display();

 

    /////////////       Object Avoidance     ////////////////

    if (distanceTop < 10)

    {

        slower();

        if (distanceRight < 10 && distanceLeft < 10)

        {

            if (distanceRight < distanceLeft)

            {

                reverse();

                delay(100);

                reverseLeft();

                delay(500);

            }

            else

            {

                reverse();

                delay(100);

                reverseRight();

                delay(500);

            }

        }

        else if (distanceRight < distanceLeft)

        {

            while (distanceTop < 10)

            {

                left();

                break;

            }

            // counterPath = 0;

        }

        else if (distanceRight >= distanceLeft)

        {

            while (distanceTop < 10)

            {

                right();

                break;

            }

            // counterPath = 0;

        }

    }

    else if (distanceRight < 10)

    {

        while (distanceRight < 10)

        {

            left_drift();

            break;

        }

        // counterPath = 0;

    }

    else if (distanceLeft < 10)

    {

        while (distanceLeft < 10)

        {

            right_drift();

            break;

        }

        // counterPath = 0;

    }

    else

    {

        forward();

    }

 

    ////////////        POSITION UPDATE?    /////////////

 

    xPrev = xPoss;

    yPrev = yPoss;

 

    if (targ > 5)

    {

        stopMotors();

        delay(1000000000);

    }

 

    counterPath++;

 

    display.clear();

}