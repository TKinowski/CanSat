#ifndef GPS_MODULE_H
#define GPS_MODULE_H

class gps_module{
    public:
    gps_module(){} //constructor
    //~gps_module(){} //destructor
    bool begin(){
    }
    bool set_max_power(){
        //TODO
    }


    bool set_low_power(){
        //TODO
    }
    
    //all data received from gps goes here
    double altitude = 0;
    double latitude = 0;
    double longitude = 0;
    //to be added: velocity, quality, etc 
};

#endif //GPS_H