#pragma once
#include "SPI.h"
#include "AD7193.h"

extern AD7193 Adc0;
extern AD7193 Adc1; 

class AdcChannel {
    public:
        AdcChannel(uint8_t channel, AD7193 &adc) :
            adc(adc),
            channel(channel)
        {};
        
        bool init();
        bool update();
        int32_t get_data();
    private:
        uint8_t channel;
        double data; //volts
        AD7193 &adc;
};