#ifndef RADIO_H
#define RADIO_H

#include <SPI.h>
#include <LoRa_mod.h>
#define LORA_CS 10
#define LORA_RST 7
#define LORA_INT 2
#define LORA_BANDWIDTH 125000L
#define LORA_CODING_RATE 5
uint8_t spreading_factor = 9;

class radio
{
  public:
    radio()
    {
    }
    void setSpreadingFactor(uint8_t sf)
    {
        LoRa.setSpreadingFactor(sf);
    }
    void parse_packet();
    bool begin();
    void send_buffer(uint8_t *buffer, uint8_t bytes);

    uint8_t buffer_ptr = 0;
    uint8_t buffer[255] = {0};
    long last_packet_transmission = 0;

  private:
};

/**
* This function sets up the LoRa module with settings from the top of the file
* It returns 0 on success and 1 on failure.
*/
bool radio::begin()
{
    LoRa.setPins(LORA_CS, LORA_RST, LORA_INT);

    if (!LoRa.begin(433E6))
    {
        return 0;
    }
    LoRa.setSpreadingFactor(spreading_factor);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setupInterrupts(onTxDone, onRxDone);
    return 1;
}

/**
 * Copies all data from lora rx buffer to local lora_buffer
 */
void radio::parse_packet()
{
    buffer_ptr = 0;
    while (LoRa.available())
    {

        buffer[buffer_ptr] = LoRa.read();
        buffer_ptr += 1;
    }
}

#endif //RADIO_H