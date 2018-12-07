#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#ifndef GPS_SERIAL
    #include "SoftwareSerial.h"
    SoftwareSerial ss(4, 3);
    #define GPS_SERIAL ss
#endif //GPS_SERIAL

class gps_module{
    public:

    //settings
    unsigned long baudrate = 9600;

    gps_module(){} //constructor
    //~gps_module(){} //destructor
    bool begin(){
        GPS_SERIAL.begin(baudrate);
        set_low_power();
    }
    bool set_max_power(){
        //TODO
    }


    bool set_low_power(){
        //TODO
    }
    void process_input(){
        //TODO
    }
    
    //all data received from gps goes here
    double altitude = 0;
    double latitude = 0;
    double longitude = 0;
    //to be added: velocity, quality, etc 
};

#endif //GPS_H