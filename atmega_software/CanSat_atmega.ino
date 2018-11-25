//Raspberry Pi
#include <AltSoftSerial.h>
AltSoftSerial altSerial;
#define PI_READ_BUFFER 120
#define PI_SERIAL altSerial
#define PI_RAIL_PIN 6
#define PI_BOOT_TIMEOUT 25
bool pi_rail_on = false;
char received_from_pi[PI_READ_BUFFER] = {0};
uint8_t received_from_pi_ptr = 0;

//macros
#define DEBUG_SERIAL Serial
#define debug(x) DEBUG_SERIAL.print("$D" + String(x));

/**
 * Controls the state of 5v rail
 */
void switch_pi_rail(bool state)
{
    digitalWrite(PI_RAIL_PIN, state);
}

/**
 * Sets up the pins for PI communication
 */
void setupPiComms()
{
    pinMode(PI_RAIL_PIN, OUTPUT);
    //digitalWrite(PI_RAIL_PIN, 0);
    PI_SERIAL.begin(38400);
    PI_SERIAL.println("");
}

//LoRa
#include <SPI.h>
#include <LoRa_mod.h>
#define LORA_CS 10
#define LORA_RST 7
#define LORA_INT 2
#define LORA_BANDWIDTH 125000
#define LORA_CODING_RATE 5
uint8_t spreading_factor = 9;
uint8_t tx_buffer[255] = {0};

/**
 * This function sets up the LoRa module with settings from the top of the file
 * It returns 0 on success and 1 on failure.
 */
bool initLora()
{
    LoRa.setPins(LORA_CS, LORA_RST, LORA_INT);

    if (!LoRa.begin(433E6))
    {
        debug(" LoRa init failed");
        return 1;
    }
    LoRa.setSpreadingFactor(spreading_factor);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setupInterrupts(onTxDone, onRxDone);
    LoRa.receive();
    return 0;
}

/**
 * This function sends the first <<bytes>> of the tx_buffer.
 */
void sendBuffer(uint8_t bytes)
{
    LoRa.beginPacket();
    LoRa.write(tx_buffer, bytes);
    LoRa.endPacket();
}

/**
 * This function is called when the transmission of a packet has ended.
 */
void onTxDone()
{
}

/**
 * This function is executed when a full packet is received, number of bytes
 * is parsed to packet_size, and if there was a crc error the boolean is set 
 * to true. DO NOT PERFORM LONG TASKS HERE
 */
void onRxDone(int packet_size, bool crc_error)
{
    if (LoRa.read() == 0x08 && LoRa.read() == 0x07)
    {
        //the message is for us
        uint8_t action = LoRa.read();
        switch (action)
        {
        case 0x01: //ACKNOWLEDGEMENT
            break;
        case 0x02: //ACTIVATE FLIGHT MODE
            break;
        default:
            break;
        }
    }
}

//GPS
#define GPS_SERIAL //gps serial
/**
 * Initializes the gps receiver serial port
 */
void initGPS()
{
    //TODO
}
/**
 * Switches gps to low power mode
 */
void setGPSLowPower()
{
    //TODO
}

/**
 * Switches GPS to maximum efficiency
 */
void setGPSMaxPower()
{
    //TODO
}



uint8_t pi_status = 0;
#define PI_OK_STATUS 0x03 //two working devices (00000011)

/**
 * does nothing basically
 */
void getStatusFromPi()
{
}

void setup()
{
    DEBUG_SERIAL.begin(38400);
    debug(" AT online");
    setupPiComms();
    switch_pi_rail(LOW);
    //digitalWrite(PI_RAIL_PIN, LOW);
    delay(5000);
    switch_pi_rail(HIGH);
    long task_started = millis();
    initGPS();
    initLora();
    while (!PI_SERIAL.available() && millis() - task_started < long(PI_BOOT_TIMEOUT) * 1000);
    if (millis() - task_started >= long(PI_BOOT_TIMEOUT) * 1000)
    {
        debug(" PI failed to boot");
    }
    else
    {
        debug(" PI booted");
        read_from_pi();
    }
}

/**
 * Reads all currently available bytes from PI_SERIAL, and handles the messsage if newline is detected
 * */
void read_from_pi()
{
    while (PI_SERIAL.available() && received_from_pi_ptr < PI_READ_BUFFER)
    {
        received_from_pi[received_from_pi_ptr] = PI_SERIAL.read();
        if (received_from_pi[received_from_pi_ptr] == '\n')
        {

            if (received_from_pi[0] == '$')
            {
                switch (received_from_pi[1])
                {
                case 'O':
                    debug("got O");
                case 'S':
                    debug("got status");
                    break;
                default:
                    break;
                }
            }
            received_from_pi_ptr = 0;
        }
        else
        {
            received_from_pi_ptr += 1;
        }
    }
}

void loop()
{
  /*
    while (Serial.available())
    {
        PI_SERIAL.write(D.read());
    }
    read_from_pi();
    */
}
