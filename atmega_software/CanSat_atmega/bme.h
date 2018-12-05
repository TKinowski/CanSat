#ifndef BME_H
#define BME_H

/**
 * basic class for bme280 interfacing
 */
class bme_sensor{
    public:
    double temperature;
    double humidity;
    long pressure;
    double altitude;

    bme_sensor(){}
    ~bme_sensor(){}
    void parse_data(){
        //TODO
    }
    void set_low_power(){
        //TODO
    }

    void begin(){
        set_low_power();
        //sth else?
    }
};

#endif