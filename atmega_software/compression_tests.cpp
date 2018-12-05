#include "compressed_number.h"
#include<iostream>
#include<inttypes.h>
#include<string>

using namespace std;

string to_bits(uint8_t b){
    string ret = "00000000";
    
    for(size_t i = 0; i < 8; i++)
    {
        ret[7-i] = ((b%2) == 0) ? '0' : '1';
        b /= 2;
    }
    
    return ret;
}


int main(){
    int a = 0;
    int data[14] = {0xFF, 0x00};
    compressed_num altitude(0, 3500, 14);
    compressed_num temperature(-15, 30, 12);
    temperature.set_value(11.31);
    altitude.set_value(1000);
    altitude.insert_to_array(data, a);
    temperature.insert_to_array(data, a);

    for(int i=0; i<5; i++){
        cout<<" "<<to_bits((data[i]));
    }
}

