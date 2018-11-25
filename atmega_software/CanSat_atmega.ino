//macros
#define DEBUG_SERIAL Serial
#define debug(x) DEBUG_SERIAL.print("$D" + String(x));

//globals
#define PRE_LAUNCH_PHASE 0
#define FLIGHT_PHASE 1
#define POST_FLIGHT_PHASE 2
uint8_t mission_phase = PRE_LAUNCH_PHASE;
bool status_acknowledged = false;

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

/**
 * Controls the state of the 5v rail
 */
void switch_pi_rail(bool state)
{
    digitalWrite(PI_RAIL_PIN, state);
}

/**
 * Sets up the pins and serial for PI communication
 */
void setupPiComms()
{
    pinMode(PI_RAIL_PIN, OUTPUT);
    //digitalWrite(PI_RAIL_PIN, 0);
    PI_SERIAL.begin(38400);
    PI_SERIAL.println("");
}

//LoRa config, vars and functions
#include <SPI.h>
#include <LoRa_mod.h>
#define LORA_CS 10
#define LORA_RST 7
#define LORA_INT 2
#define LORA_BANDWIDTH 125000L
#define LORA_CODING_RATE 5
uint8_t spreading_factor = 9;
uint8_t lora_buffer[255] = {0};
uint8_t lora_buffer_ptr = 0;
bool lora_ok = false;

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
        return 0;
    }
    LoRa.setSpreadingFactor(spreading_factor);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setupInterrupts(onTxDone, onRxDone);
    LoRa.receive();
    return 1;
}

/**
 * This function sends the first <<bytes>> of the tx_buffer.
 */
void sendBuffer(uint8_t bytes)
{
    LoRa.beginPacket();
    LoRa.write(lora_buffer, bytes);
    LoRa.endPacket();
}

/**
 * This function is called when the transmission of a packet has ended.
 */
void onTxDone()
{
}

#define ACTION_ACK 0x01
#define ACTION_FLY 0x02
#define ACTION_CMD 0x03

/**
 * This function is executed when a full packet is received, number of bytes
 * is parsed to packet_size, and if there was a crc error the boolean is set 
 * to true. DO NOT PERFORM LONG TASKS HERE
 */
void onRxDone(int packet_size, bool crc_error)
{
    //check if the packet is for us
    if (packet_size >= 3 && LoRa.read() == 0x08 && LoRa.read() == 0x07) 
    {
        uint8_t action = LoRa.read();
        switch (action)
        {
        case ACTION_ACK: //ACKNOWLEDGEMENT
            status_acknowledged = true;
            break;
        case ACTION_FLY: //ACTIVATE FLIGHT MODE
            switch_to_flight_phase();
            break;
        case ACTION_CMD: //send command to pi, return the response
            parse_lora_packet();
            send_command_to_pi(lora_buffer, lora_buffer_ptr);
            lora_buffer_ptr = 0;
            break;
        default:
            break;
        }
    }
}

void parse_lora_packet(){
    while(LoRa.available()){
        lora_buffer[lora_buffer_ptr] = LoRa.read();
        lora_buffer_ptr+=1;
    }
}

void send_command_to_pi(uint8_t* command, uint8_t length)
{
    uint8_t ptr = 0;
    PI_SERIAL.write("$C");
    for (ptr; ptr < length; ++ptr)
    {
        PI_SERIAL.write(command[ptr]);
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
#define PI_OK_STATUS 0b00000111 //two working devices and pi booted (00000111)
#define PI_FAILED_BOOT 0b00000000

/**
 * does nothing basically
 */
uint8_t get_pi_status()
{
    long task_started = millis();
    while (!PI_SERIAL.available() && millis() - task_started < long(PI_BOOT_TIMEOUT) * 1000)
        ;
    if (millis() - task_started >= long(PI_BOOT_TIMEOUT) * 1000)
    {
        debug(" PI boot timeout");
        return PI_FAILED_BOOT;
    }
    else
    {
        debug(" PI booted ok");
        //return read_from_pi();
    }
}

void setup()
{
    DEBUG_SERIAL.begin(38400);
    debug("AT online");
    setupPiComms();
    switch_pi_rail(HIGH);
    initGPS();
    setGPSLowPower();
    lora_ok = initLora();
    pi_status = get_pi_status();
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
    switch (mission_phase)
    {
    case PRE_LAUNCH_PHASE:
        pre_launch_loop();
        break;
    case FLIGHT_PHASE:
        flight_loop();
        break;
    case POST_FLIGHT_PHASE:
        post_flight_loop();
        break;
    }
}

/**
 * Executed untill launch command is received or a big altitude change is detected,
 * switches to mission_phase 1 afterwards.
 */
void pre_launch_loop()
{
    //TODO

    //If ACK hasn’t been received & 10s have passed from last time: AT sends status
    //(handled by the lora interrupt function)

    //If commands are received ($C) they are passed to and executed on PI, and the output is returned via LoRa
    //((handled by the lora interrupt function)

    //get gps altitude, compare with initial, if the signal is good and the altitude is over
    //200m greater initiate flight phase
}

void switch_to_flight_phase()
{
    //AT enables 5V rail, PI is booted
    switch_pi_rail(HIGH);

    //AT enables full efficiency on GPS and BME
    setGPSMaxPower();

    //AT sends “launch confirmation” to ground
    memcpy(lora_buffer, "$L", 2);
    //lora_buffer[2] = pi status
    sendBuffer(3);
    
    //AT waits for PI to boot
    pi_status = get_pi_status();

    //PI performs boot routine[1]
    
    //AT sends “flight mode ($F)” to PI
    send_command_to_pi((uint8_t*)"$F", 2);

}

void flight_loop()
{
    //TODO
}

/**
 * Executed after while waiting for recovery after the mission, battery power needs to
 * be used very considerately here, as it will last up to 6 hours
 */
void post_flight_loop()
{
    //go to sleep for 10 seconds

    //send a packet with location at SF12
}