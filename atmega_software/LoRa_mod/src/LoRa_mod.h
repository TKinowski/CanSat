// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_MOD_H
#define LORA_MOD_H

#include <Arduino.h>
#include <SPI.h>

#ifdef ARDUINO_SAMD_MKRWAN1300
#define LORA_DEFAULT_SPI           SPI1
#define LORA_DEFAULT_SPI_FREQUENCY 250000
#define LORA_DEFAULT_SS_PIN        LORA_IRQ_DUMB
#define LORA_DEFAULT_RESET_PIN     -1
#define LORA_DEFAULT_DIO0_PIN      -1
#else
#define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6 
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2
#endif

//dio0 configs
#define RX_INTERRUPT 0
#define TX_INTERRUPT 1

#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

class LoRaClass : public Stream
{
  public:
    LoRaClass();

    int begin(long frequency);
    void end();

    int beginPacket(int implicitHeader = false);
    void endPacket();

    int parsePacket(bool &crcError, int size = 0);
    int packetRssi();
    float packetSnr();

    // from Print
    virtual size_t write(uint8_t byte);
    virtual size_t write(const uint8_t *buffer, size_t size);

    // from Stream
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    void dio0Interrupt(uint8_t dio0_setting);
    void setupInterrupts(void (*tx_callback)(void), void (*rx_callback)(int, bool));

    void receive(int size = 0);
    void idle();
    void sleep();

    void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
    void setFrequency(long frequency);
    void setSpreadingFactor(int sf);
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int denominator);
    void setPreambleLength(long length);
    void setSyncWord(int sw);
    void enableCrc();
    void disableCrc();

    // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
    // Only with PA_BOOST pin connected
    void set20dBm_sx127x(bool turn_on);
    // Over Current Protection control (Semtech SX1276/77/78/79 5.4.4.)
    void setOCP_sx127x(uint8_t mA);

    // deprecated
    void crc() { enableCrc(); }
    void noCrc() { disableCrc(); }

    uint8_t random();

    void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
    void setSPIFrequency(uint32_t frequency);

    void dumpRegisters(Stream &out);
    uint8_t getMode();

  private:
    void explicitHeaderMode();
    void implicitHeaderMode();

    void handleDio0Rise();
    uint8_t readRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t singleTransfer(uint8_t address, uint8_t value);

    static void onDio0Rise();

  private:
    SPISettings _spiSettings;
    int _ss;
    int _reset;
    int _dio0;
    uint8_t _dio0Setting;
    uint8_t _mode;
    int _frequency;
    int _packetIndex;
    int _implicitHeaderMode;
    void (*_onRxDone)(int, bool);
    void (*_onTxDone)(void);
};

extern LoRaClass LoRa;

#endif
