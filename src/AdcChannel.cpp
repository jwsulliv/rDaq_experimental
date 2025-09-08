#include "AdcChannel.h"

bool AdcChannel::init() {
    return adc.add_channel(channel);
}

bool AdcChannel::update() {
    if (channel == 8){
        data = adc.get_temperature()-273;
    }
    else {
        // actual sensor poll handled by ADC hardware class.
        data = (int32_t)(adc.read_channel(channel) * 1e6);
    }
    return true;
}

int32_t AdcChannel::get_data() {
    return data;
}