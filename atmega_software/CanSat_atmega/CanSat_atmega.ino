#include <AltSoftSerial.h>
AltSoftSerial altSerial;

//macros
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 38400
#define debug(x) DEBUG_SERIAL.print("$D" + String(x));

//globals
#define PRE_LAUNCH_PHASE 0
#define FLIGHT_PHASE 1
#define POST_FLIGHT_PHASE 2
uint8_t mission_phase = PRE_LAUNCH_PHASE;

#include "compressed_number.h"
compressed_num temperature(-15, 30, 11);
compressed_num pressure(60000, 110000, 16);
compressed_num humidity(0, 100, 7);
compressed_num latitude(52.164, 52.165, 16);
compressed_num longitude(21.050, 21.060, 16);
compressed_num altitude(0, 3500, 15);
compressed_num roll(0, 360, 12);
compressed_num pitch(0, 360, 12);
compressed_num heading(0, 360, 12);

bool rpi_works = 0, mpu_works = 0, camera_works = 0, movidius_works = 0; //devices on raspberry
bool lora_works = 0, gps_works = 0, bme_works = 0;  //devices on atmega
bool status_acknowledged = false;

/**
 * returns the status of all devices as a byte
 */
uint8_t get_concentrated_status()
{
    uint8_t status = rpi_works | (mpu_works << 1) | (camera_works << 2) |
                     (lora_works << 3) | (gps_works << 4) | (bme_works << 5);
    return status;
}

//GPS
#include "gps_module.h"
#define GPS_SERIAL
#define GPS_ALTITUDE_LAUNCH_TRESHOLD 500.0 //gps altitude change before launch is confirmed
gps_module gps;
double initial_gps_altitude = 1000.0 ; //over sea level, in meters

//bme280
#include "bme.h"
bme_sensor bme;


//battery
#include "battery.h"


//Raspberry Pi
#define PI_READ_BUFFER 120
#define PI_SERIAL altSerial
#define PI_RAIL_PIN 6
#define PI_BOOT_TIMEOUT 25
#define PI_SERIAL_BAUDRATE 38400L

char received_from_pi[PI_READ_BUFFER] = {0};
uint8_t received_from_pi_ptr = 0;

//messages from pi
#define DEBUG_FROM_PI 'D'
#define LANDING_FROM_PI 'L'
#define STATUS_FROM_PI 'S'

//messages to pi
#define COMMAND_TO_PI 'C'
#define ENABLE_FLIGHT_MODE 'F'

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
    PI_SERIAL.begin(PI_SERIAL_BAUDRATE);
    PI_SERIAL.println("");
}
//LoRa config, vars and functions

#include "radio.h"
radio lora;
//constants for communication with ground
#define SELF_ID 'C'
#define GROUND_ID 'G'
#define LAUNCH_CONFIRMATION 'L'
#define STATUS_MESSAGE 'S'
#define DATA_PACKET 'D'
#define PI_OUTPUT 'O'

//possible commands from ground station (via radio)
#define ACTION_ACK 'A'        //acknowledge status
#define ACTION_FLY 'L'        //enable flight mode
#define ACTION_CMD 'C'        //a command to raspberry pi
#define ACTION_REQ 'R' //request to send current status

#define MAX_PACKET_SEPARATION 950  //maximum time between packets in milliseconds
#define STATUS_REPEAT_PERIOD 10000 //time between status retransmissions in phase 1

/**
 * This function is called when the transmission of a packet has ended.
 */
void onTxDone()
{
    lora_works=true;
}

/**
 * This function is executed when a full packet is received, number of bytesl
 * is parsed to packet_size, and if there was a crc error the boolean is set 
 * to true. DO NOT PERFORM LONG TASKS HERE
 */
void onRxDone(int packet_size, bool crc_error)
{
    //check if the packet is for us
    if (packet_size >= 3 && LoRa.read() == GROUND_ID && LoRa.read() == SELF_ID)
    {
        handle_gs_message(LoRa.read());
    }

    if (mission_phase == POST_FLIGHT_PHASE){
        send_beacon_packet();
    }
}

/**
 * Performs the action requested by the ground station
 */
void handle_gs_message(uint8_t action)
{
    switch (action)
    {
    case ACTION_ACK: //ACKNOWLEDGEMENT
        status_acknowledged = true;
        break;
    case ACTION_FLY: //ACTIVATE FLIGHT MODE
        switch_to_flight_phase();
        break;
    case ACTION_CMD: //send command to pi, return the response
        lora.parse_packet();
        send_command_to_pi(lora.buffer, lora.buffer_ptr);
        lora.buffer_ptr = 0;
        break;
    case ACTION_REQ:
        send_status_to_gs();
        break;
    default:
        break;
    }
}


/**
 * Sends the status byte
 */
void send_status_to_gs()
{
    uint8_t message[4] = {SELF_ID, GROUND_ID, STATUS_MESSAGE, 0x00};
    message[3] = get_concentrated_status();
    LoRa.beginPacket();
    LoRa.write(message, 4);
    LoRa.endPacket();
}

/**
 * Sends the supplied command with $C prefix
 */
void send_command_to_pi(uint8_t *command, uint8_t length)
{
    PI_SERIAL.write('$');
    PI_SERIAL.write(COMMAND_TO_PI);
    PI_SERIAL.write(command, length);
}

/**
 * waits for activity on PI_SERIAL, returns error on timeout
 */
void boot_pi()
{
    debug(" waiting for pi to boot... ") long boot_initiated = millis();
    while (!PI_SERIAL.available() && millis() - boot_initiated < long(PI_BOOT_TIMEOUT) * 1000)
    {
        yield;
    }
    if (millis() - boot_initiated >= long(PI_BOOT_TIMEOUT) * 1000)
    {
        debug(" PI boot timeout\n");
        //set status of all pi based modules to 0
        rpi_works = false;
        camera_works = false;
        mpu_works = false;
    }
    else
    {
        debug(" PI booted ok\n");
        delay(20);
        while (PI_SERIAL.available())
        {
            received_from_pi[received_from_pi_ptr] = PI_SERIAL.read();
            received_from_pi_ptr += 1;
            delay(5);
        }
        if (received_from_pi[0] == '$' && received_from_pi[1] == STATUS_FROM_PI)
        {
            rpi_works = true;
            parse_status_from_pi();
        }
        else
        {
            debug("message from pi is broken, retransfering status request\n");
            debug(String(received_from_pi[0]) + (received_from_pi[1]));
            rpi_works = false;
            camera_works = false;
            mpu_works = false;
            //TODO: retransfer status request
        }
    }
}

/**
 * Gets called when status is received from pi, parses it to bools
 */
bool parse_status_from_pi(){
    if (received_from_pi[0] != '$' || received_from_pi[1] != STATUS_FROM_PI) return 0;
    camera_works = (received_from_pi[2]=='1');
    mpu_works = (received_from_pi[3]=='1');
    return 1;
}


/**
 * Can be treated as "enter pre launch phase" function
 */
void setup()
{
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    debug(" AT online\n");
    setupPiComms();
    switch_pi_rail(HIGH);
    gps_works = gps.begin();
    lora_works = lora.begin();
    boot_pi();
}

/**
 * Reads all currently available bytes from PI_SERIAL, 
 * and handles the messsage if newline is detected
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
                handle_pi_message(received_from_pi[1]);
            }
            received_from_pi_ptr = 0;
        }
        else
        {
            received_from_pi_ptr += 1;
        }
    }
}

/**
 * Performs the action requested by raspberry pi
 */
void handle_pi_message(uint8_t message)
{
    switch (message)
    {
    case DEBUG_FROM_PI:
        LoRa.beginPacket();
        uint8_t header[3] = {SELF_ID, GROUND_ID, PI_OUTPUT};
        LoRa.write(received_from_pi, received_from_pi_ptr);
        LoRa.endPacket();
        break;
    case LANDING_FROM_PI:
        switch_to_post_flight_phase();
        break;
    case STATUS_FROM_PI:
        parse_status_from_pi();
        break;
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
 * Sends the status untill acknowledgement is received, checks for launch symptoms
 */
void pre_launch_loop()
{
    //TODO

    //If ACK hasn’t been received & 10s have passed from last time: AT sends status
    //(handled by the lora interrupt function)
    if (!status_acknowledged && millis() - lora.last_packet_transmission > STATUS_REPEAT_PERIOD)
    {
        send_status_to_gs();
    }

    //If commands are received ($C) they are passed to and executed on PI,
    //and the output is returned via LoRa
    //((handled by the lora interrupt function)

    //get gps altitude, compare with initial, if the signal is good and the altitude is over
    //500m greater initiate flight phase
    if (gps.altitude - initial_gps_altitude > GPS_ALTITUDE_LAUNCH_TRESHOLD)  //add signal quality condition
    { 
        //this shouldn't happen, as launch command is supposed to be
        //sent by radio
        debug(" gps altitude jump, switching to flight phase\n")
            switch_to_flight_phase();
    }
}

/**
 * Enables the pi and sets up everything at max efficiency for the flight,
 * switches mission_phase to POST_FLIGHT_PHASE
 */
void switch_to_flight_phase()
{
    //AT enables 5V rail, PI is booted
    switch_pi_rail(HIGH);

    //AT enables full efficiency on GPS and BME
    gps.set_max_power();
    //setBMEMaxPower(); - to be implemented

    uint8_t message[3] = {SELF_ID, GROUND_ID, LAUNCH_CONFIRMATION};
    LoRa.beginPacket();
    LoRa.write(message, 3);
    LoRa.endPacket();

    //AT waits for PI to boot
    boot_pi();

    send_status_to_gs();

    //AT sends “launch confirmed” to PI
    send_command_to_pi(ENABLE_FLIGHT_MODE, 1);

    mission_phase = FLIGHT_PHASE;
}

/**
 * Sends gps data to pi, gets back imu data and status
 */
void exchange_data_with_pi()
{
    //TODO
    //first send gps data
    //then receive imu and status
}

/**
 * Sends all of available data to the ground station
 */
void send_data_to_gs()
{
    //TODO
}

/**
 * Executed during the flight phase
 */
void flight_loop()
{
    //read bme data
    bme.parse_data();

    //Read GPS data
    gps.process_input();

    //Send converted gps data to PI, get data in return
    exchange_data_with_pi();

    //if time from last send is > 0.9s, send new packet
    if (millis() - lora.last_packet_transmission > MAX_PACKET_SEPARATION)
    {
        send_data_to_gs();
    }

    //get_battery_voltage();

    //If “landing” recieved from PI: break loop
    //(handled by on_rx_done)
}

/**
 * Switches everything to low power consumption, and changes the mission
 * phase to POST_FLIGHT_PHASE
 */
void switch_to_post_flight_phase()
{
    mission_phase = POST_FLIGHT_PHASE;
    lora.setSpreadingFactor(12);
    gps.set_low_power();
    //TODO
}

/**
 * Executed after while waiting for recovery after the mission, battery power needs to
 * be used very considerately here, as it will last up to 6 hours
 */
void post_flight_loop()
{
    //go to sleep for 10 seconds
    delay(10000);

    //send a packet with location at SF12
    send_beacon_packet();
}

void send_beacon_packet(){
    LoRa.beginPacket();
    //TODO: send location in plaintext
    LoRa.endPacket();
}