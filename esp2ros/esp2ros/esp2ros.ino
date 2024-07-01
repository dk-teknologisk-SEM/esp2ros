
#include <SPI.h>
#include <Ethernet.h>

// Network settings
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress server(192, 168, 1, 123);                                            // IP address of the PC running the ROS2 node
const int pinCount = 5;                                                        // Number of pins to monitor
const int pins[pinCount] = {PIN_I0_0, PIN_I0_1, PIN_I0_2, PIN_I0_3, PIN_I0_4}; // Array of input pins

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 1, 100);

// Store the previous state of each pin
int previousStates[pinCount];

// Ethernet client
EthernetClient client;

const bool USE_DHCP = false;

void setup()
{
    // Start serial communication
    Serial.begin(115200);

    // start the Ethernet connection:
    if (USE_DHCP)
    {
        Serial.println("Initialize Ethernet with DHCP:");
        if (Ethernet.begin(mac) == 0)
        {
            Serial.println("Failed to configure Ethernet using DHCP");
            while (true)
            {
                delay(1); // do nothing, no point running without Ethernet hardware
            }
        }
        else
        {
            Serial.print("  DHCP assigned IP ");
            Serial.println(Ethernet.localIP());
        }
    }
    else
    {
        Serial.println("Initialize Ethernet with static IP:");

        // configure using IP address
        Ethernet.begin(mac, ip);
    }

    // Allow the hardware to initialize
    delay(1500);

    // Check for Ethernet hardware present
    EthernetHardwareStatus status = Ethernet.hardwareStatus();
    if (status == EthernetNoHardware)
    {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true)
        {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
    }
    else
    {
        Serial.println("Hardware present");
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
        Serial.println("Ethernet cable is not connected.");
    }
    else
    {
        Serial.println("Ethernet cable is connected.");
    }

    // Attempt to connect to the server
    bool connected = false;
    for (int i = 0; i < 5; i++)
    {
        if (client.connect(server, 1883))
        {
            Serial.println("Connected to server");
            connected = true;
            break;
        }
        else
        {
            Serial.println("Connection failed");
            delay(500);
        }
    }

    if (!connected)
    {
        Serial.println("Connection could not be established, check your hardware and try again");
        while (true)
        {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
    }

    // Set the input pin modes and initialize previous states
    Serial.println("Reading initial pin states");
    for (int i = 0; i < pinCount; i++)
    {
        pinMode(pins[i], INPUT);
        previousStates[i] = digitalRead(pins[i]);
    }

    Serial.println("Starting main loop");
}

void loop()
{
    for (int i = 0; i < pinCount; i++)
    {
        int currentState = digitalRead(pins[i]);
        if (currentState != previousStates[i])
        {
            // If the state has changed, send a message
            sendMessage(pins[i], currentState);
            previousStates[i] = currentState;
        }
    }

    delay(100); // Adjust as needed
}

void sendMessage(int pin, int state)
{
    Serial.println("In sendMessage");
    if (client.connected())
    {
        String message = "Pin " + String(pin) + " changed to " + String(state);
        client.println(message);
        Serial.println("Message sent: " + message);
    }
    else
    {
        Serial.println("Disconnected from server");
    }
}
