//macros
#define DEBUG_SERIAL Serial
#define debug(x) DEBUG_SERIAL.print("$D" + String(x));

//globals
#define PRE_LAUNCH_PHASE 0
#define FLIGHT_PHASE 1
#define POST_FLIGHT_PHASE 2

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

/*
bit | device
0   raspberry
1   mpu9250
2   camera
3   -------
4   bme280
5   LoRa
6   gps
7   --------
*/
uint8_t devices_status = 0b00000000; //initially no devices are assumed to work
bool rpi_works = 0, mpu_works = 0, camera_works = 0;
bool lora_works = 0, gps_works = 0, bme_works = 0;
uint8_t mission_phase = PRE_LAUNCH_PHASE;
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
#define GPS_SERIAL //gps serial
#define GPS_ALTITUDE_LAUNCH_TRESHOLD 500.0 //gps altitude change before launch is confirmed
gps_module gps;

#include "battery.h"

//Raspberry Pi
#include <AltSoftSerial.h>
AltSoftSerial altSerial;
#define PI_READ_BUFFER 120
#define PI_SERIAL altSerial
#define PI_RAIL_PIN 6
#define PI_BOOT_TIMEOUT 25
uint8_t received_from_pi[PI_READ_BUFFER] = {0};
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

#include "radio.h"
radio lora;
bool lora_ok = false;

//constants for communication with ground
#define SELF_ID 'A'
#define GROUND_ID 'G'
#define LAUNCH_CONFIRMATION 'L'
#define STATUS_MESSAGE 'S'
#define DATA_PACKET 'D'

#define MAX_PACKET_SEPARATION 950  //maximum time between packets in milliseconds
#define STATUS_REPEAT_PERIOD 10000 //time between status retransmissions in phase 1

//possible commands from ground station (via radio)
#define ACTION_ACK 0x01        //acknowledge status
#define ACTION_FLY 0x02        //enable flight mode
#define ACTION_CMD 0x03        //a command to raspberry pi
#define ACTION_REQ_STATUS 0x04 //request to send current status

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
    //check if the packet is for us
    if (packet_size >= 3 && LoRa.read() == GROUND_ID && LoRa.read() == SELF_ID)
    {
        handle_gs_message(LoRa.read());
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
    case ACTION_REQ_STATUS:
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
    lora.send_buffer(message, 4);
}

/**
 * Sends the supplied command with $C prefix
 */
void send_command_to_pi(uint8_t *command, uint8_t length)
{
    PI_SERIAL.write("$C");
    PI_SERIAL.write(command, length);
}



#define PI_OK_STATUS 0b00000111 //two working devices and pi booted (00000111)
#define PI_FAILED_BOOT 0b00000000

/**
 * waits for activity on PI_SERIAL, returns error on timeout
 */
void boot_pi()
{
    debug(" waiting for pi to boot") long boot_initiated = millis();
    while (!PI_SERIAL.available() && millis() - boot_initiated < long(PI_BOOT_TIMEOUT) * 1000)
    {
        yield;
    }
    if (millis() - boot_initiated >= long(PI_BOOT_TIMEOUT) * 1000)
    {
        debug(" PI boot timeout\n");
        devices_status &= 0b00011111; //set status of all pi based modules to 0
    }
    else
    {
        debug(" PI booted ok\n");
        delay(20);
        while (PI_SERIAL.available())
        {
            received_from_pi[received_from_pi_ptr] = PI_SERIAL.read();
            debug(received_from_pi[received_from_pi_ptr]); //for debug purposes
            received_from_pi_ptr += 1;
            delay(5);
        }
        if (received_from_pi[0] == '$' && received_from_pi[0] == 'S')
        {
            uint8_t status = PI_SERIAL.read();
            //change first 4 bits of devices_status without changing the last 4
            devices_status |= status & 0b11110000;
            devices_status &= status | 0b00001111;
        }
        else
        {
            debug("message from pi is broken, retransfering status request");
            devices_status &= 0b00011111;
            //retransfer status request
        }
    }
}

double initial_gps_altitude = 1000;
/**
 * Can be treated as "enter pre launch phase" function
 */
void setup()
{
    DEBUG_SERIAL.begin(38400);
    debug(" AT online");
    setupPiComms();
    switch_pi_rail(HIGH);
    gps.begin();
    gps.set_low_power();
    lora.begin();
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

#define PI_DEBUG 'D'
#define PI_LANDING 'L'

void handle_pi_message(uint8_t message)
{
    switch (message)
    {
    case PI_DEBUG:
        lora.send_buffer(received_from_pi, received_from_pi_ptr);
        break;
    case PI_LANDING:
        switch_to_post_flight_phase();
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
        debug("gps altitude jump, switching to flight phase")
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
    lora.send_buffer(message, 3);

        //AT waits for PI to boot
        boot_pi();

    send_status_to_gs();

    //AT sends “flight mode ($F)” to PI
    send_command_to_pi((uint8_t *)"$F", 2);

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
    //Try to get data from BME (change status if failed) - as often as possible
    //parse_bme_data();

    //Read GPS data
    //process_gps();

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
}