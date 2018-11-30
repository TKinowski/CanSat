#ifndef COMPRESSED_NUMBER_H
#define COMPRESSED_NUMBER_H

#include <inttypes.h>
#include <math.h>
#include <string>
#include <iostream>

using namespace std;

class compressed_num
{
  private:
    double value;
    double max_value;
    double min_value;
    uint8_t bits;

  public:
    compressed_num(double minimum, double maximum, uint8_t _bits)
    {
        min_value = minimum;
        max_value = maximum;
        bits = _bits;
        value = min_value;
    }
    ~compressed_num() {}

    /**
     * Sets the new value, wraps and returns 1 on overflow
     */
    bool set_value(double new_value)
    {
        value = new_value;
        if (value > max_value)
        {
            value -= max_value - min_value;
            return 1;
        }
        return 0;
    }
    //[11111111 11****** *******0]
    void insert_to_array(int *arr, int &ptr)
    {
        int mapped = min_value + (value - min_value) * 1.0 / (max_value - min_value) * (pow(2, bits) - 1);
        bool bs[bits] = {0};

        for (int i = bits - 1; i >= 0; i--)
        {
            bs[i] = mapped % 2 == 1;
            mapped -= mapped % 2;
            mapped /= 2;
        }

        for (int i = ptr; i < ptr + bits; ++i)
        {
            if (bs[i - ptr] == 0)
            {
                arr[i / 8] &= ~(1 << (7 - (i % 8)));
            }
            else
            {
                arr[i / 8] |= (bs[i - ptr] << 7 - (i % 8));
            }
        }
        ptr += bits;
    }
};

#endif //COMPRESSED_NUMBER_H