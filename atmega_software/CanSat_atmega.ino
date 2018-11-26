//macros
#define DEBUG_SERIAL Serial
#define debug(x) DEBUG_SERIAL.print("$D" + String(x));

//globals
#define PRE_LAUNCH_PHASE 0
#define FLIGHT_PHASE 1
#define POST_FLIGHT_PHASE 2
uint8_t mission_phase = PRE_LAUNCH_PHASE;
bool status_acknowledged = false;

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
uint8_t devices_status = 0b00000000; //initially no devices working is assumed

void set_status_bit(uint8_t bit, bool value){
    devices_status |= 0b11111111 & value << bit;
}


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

long last_packet_transmission = 0; //the value of millis() of last transmission
#define MAX_PACKET_SEPARATION 950 //maximum time between packets in milliseconds
#define STATUS_REPEAT_PERIOD 10000 //time between status retransmissions in phase 1

//possible commands from ground station (via radio)
#define ACTION_ACK 0x01 //acknowledge status
#define ACTION_FLY 0x02 //enable flight mode
#define ACTION_CMD 0x03 //a command to raspberry pi

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
    debug(" LoRa init ok");
    return 1;
}

/**
 * This function sends the first <<bytes>> bytes of <<buffer>> via lora
 */
void sendBuffer(uint8_t* buffer, uint8_t bytes)
{
    last_packet_transmission = millis();
    LoRa.beginPacket();
    LoRa.write(buffer, bytes);
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

/**
 * Copies all data from lora rx buffer to local lora_buffer
 */
void parse_lora_packet()
{
    while (LoRa.available())
    {
        lora_buffer[lora_buffer_ptr] = LoRa.read();
        lora_buffer_ptr += 1;
    }
}

/**
 * Sends the supplied command with $C prefix
 */
void send_command_to_pi(uint8_t *command, uint8_t length)
{
    PI_SERIAL.write("$C");
    PI_SERIAL.write(command, length);
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

#define PI_OK_STATUS 0b00000111 //two working devices and pi booted (00000111)
#define PI_FAILED_BOOT 0b00000000

/**
 * waits for activity on PI_SERIAL, returns error on timeout
 */
void boot_pi()
{
    long task_started = millis();
    while (!PI_SERIAL.available() && millis() - task_started < long(PI_BOOT_TIMEOUT) * 1000)
        ;
    if (millis() - task_started >= long(PI_BOOT_TIMEOUT) * 1000)
    {
        debug(" PI boot timeout");
        devices_status &= 0b00011111;
    }
    else
    {
        debug(" PI booted ok");
        delay(20); //give pi time to write whole status
        if (PI_SERIAL.read() == '$' && PI_SERIAL.read() == 'S'){
            uint8_t status = PI_SERIAL.read();
            //change first 4 bits of devices_status without changing the last 4
            devices_status |= status & 0b11110000;
            devices_status &= status | 0b00001111;
        }else{
            debug("message from pi is broke");
            devices_status &= 0b00011111; 
            //get status again
        }
        

    }
}

/**
 * Can be treated as "enter pre launch phase" function
 */
void setup()
{
    DEBUG_SERIAL.begin(38400);
    debug("AT online");
    setupPiComms();
    switch_pi_rail(HIGH);
    initGPS();
    setGPSLowPower();
    set_status_bit(6, initLora());
    boot_pi();
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
 * Sends the status untill acknowledgement is received, checks for launch symptoms
 */
void pre_launch_loop()
{
    //TODO

    //If ACK hasn’t been received & 10s have passed from last time: AT sends status
    //(handled by the lora interrupt function)
    if (!status_acknowledged && millis()-last_packet_transmission > STATUS_REPEAT_PERIOD){
        uint8_t packet[3] = {0x07, 0x08, devices_status};
        sendBuffer(packet, 3);
    }

    //If commands are received ($C) they are passed to and executed on PI, and the output is returned via LoRa
    //((handled by the lora interrupt function)

    //get gps altitude, compare with initial, if the signal is good and the altitude is over
    //200m greater initiate flight phase

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
    setGPSMaxPower();
    //setBMEMaxPower(); - to be implemented
    

    //AT waits for PI to boot
    boot_pi();

    uint8_t message[4] = {0x07, 0x08, 'L'};
    message[4] = devices_status;
    //AT sends “launch confirmation” to ground
    sendBuffer(message, 4);

    //AT sends “flight mode ($F)” to PI
    send_command_to_pi((uint8_t *)"$F", 2);

    mission_phase = FLIGHT_PHASE;
}


/**
 * Sends gps data to pi, gets back imu data and status
 */
void exchange_data_with_pi(){
    //TODO
    //first send gps data
    //then receive imu and status
}

/**
 * Sends all of available data to the ground station
 */
void send_data_to_gs(){
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
    if (millis()-last_packet_transmission > MAX_PACKET_SEPARATION){
        send_data_to_gs();
    }
    
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
    LoRa.setSpreadingFactor(12);
    setGPSLowPower();
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