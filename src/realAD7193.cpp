#include "realAD7193.h"

bool AD7193::init() {
    spi.setDataMode(SPI_MODE3);
    spi.setClockDivider(SPI_CLOCK_DIV16);
    spi.begin();
    return true;
    pinMode(cs_pin,OUTPUT);
}


bool AD7193::configure_channels(const std::array<ChannelConfig, 9>& channelConfigs){
        AD7193_driver_arg_t driver_args;

	uint16_t gain_bits;
    switch(pga_gain){
        case 1:     gain_bits = 0x0;    break;
        case 8:     gain_bits = 0x03;   break;
        case 16:    gain_bits = 0x04;   break;
        case 32:    gain_bits = 0x05;   break;
        case 64:    gain_bits = 0x06;   break;
        case 128:   gain_bits = 0x07;   break;
        default:    gain_bits = 0x0;
    }
    uint32_t config_write_payload = 0x0 | (!differential << AD7193_DIFFERENTIAL_BIT) | channel_bits | (gain_bits);
    uint32_t mode_reg_payload_calib = 0x080060 | (1 << 20) | (0b001 << 21); // calibrates adc, should be done on startup
    uint32_t mode_reg_payload_cont = 0x180001; // continuous read mode
    uint32_t mode_reg_payload_single = 0x200001; // single read mode
    uint32_t mode_reg_payload_idle = 0x580001; // idle
    uint32_t mode_reg_payload_pwrdwn = 0x780001; // pwrdwn

    pinMode(cs_pin,OUTPUT);
  
    digitalWrite(cs_pin, LOW);

    for(int i = 0; i < 9; i++){
        switch(channelConfigs[i].pgaGain){
            case 1:     gain_bits = 0x0;    break;
            case 8:     gain_bits = 0x03;   break;
            case 16:    gain_bits = 0x04;   break;
            case 32:    gain_bits = 0x05;   break;
            case 64:    gain_bits = 0x06;   break;
            case 128:   gain_bits = 0x07;   break;
            default:    gain_bits = 0x0;
        }
        uint32_t config_write_payload = 0x0 | (!differential << AD7193_DIFFERENTIAL_BIT) | channel_bits | (gain_bits);
        driver_args.comm_bits = AD7193_REG_WRITE | AD7193_CONFIG_REG;
        driver_args.write_payload = config_write_payload;
        AD7193_driver(driver_args);
    }
    

    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_MODE_REG;
    driver_args.write_payload = mode_reg_payload_calib;
    AD7193_driver(driver_args);

    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_MODE_REG;
    driver_args.write_payload = mode_reg_payload_cont;
    AD7193_driver(driver_args); 
    
    
    
    
    
    
    return true;
}


bool AD7193::update(uint8_t channel){
    AD7193_driver_arg_t driver_args;
    if (channel > 8) return false;  // Invalid channel

	uint16_t gain_bits;
    switch(pga_gain){
        case 1:     gain_bits = 0x0;    break;
        case 8:     gain_bits = 0x03;   break;
        case 16:    gain_bits = 0x04;   break;
        case 32:    gain_bits = 0x05;   break;
        case 64:    gain_bits = 0x06;   break;
        case 128:   gain_bits = 0x07;   break;
        default:    gain_bits = 0x0;
    }
    uint32_t config_write_payload = 0x0 | (!differential << AD7193_DIFFERENTIAL_BIT) | channel_bits | (gain_bits);
    uint32_t config_write_payload_differential = 0x04FF00; // all channels
    uint32_t config_write_payload_pt = 0x041F00; // for pressure transducer
    uint32_t config_write_payload_tc = 0x000107; // for thermocouple
    uint32_t mode_reg_payload_calib = 0x080060 | (1 << 20) | (0b001 << 21); // calibrates adc, should be done on startup
    uint32_t mode_reg_payload_cont = 0x180001; // continuous read mode
    uint32_t mode_reg_payload_idle = 0x580001; // idle
    uint32_t mode_reg_payload_pwrdwn = 0x780001; // pwrdwn

    pinMode(cs_pin,OUTPUT);
  
    digitalWrite(cs_pin, LOW);

    
    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_CONFIG_REG;
    driver_args.write_payload = config_write_payload;
    AD7193_driver(driver_args);

    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_MODE_REG;
    driver_args.write_payload = mode_reg_payload_calib;
    AD7193_driver(driver_args);

    driver_args.comm_bits = AD7193_REG_WRITE | AD7193_MODE_REG;
    driver_args.write_payload = mode_reg_payload_cont;
    AD7193_driver(driver_args); 

    driver_args.comm_bits = AD7193_REG_READ | AD7193_DATA_REG | 0b01011000;

    int length = channel_readings.size();

    uint32_t results [length];
    uint32_t timeb [length];
    uint8_t statusv [length];

    
    for(int i = 0; i < length; i++){
        uint32_t result = AD7193_driver(driver_args);
        uint8_t status = (uint8_t)(result & 0x0000007F);
        statusv[i] = status;
        results[i] = result;
    }

    for(int i = 0; i < length; i++){
        if(statusv[i]!=8){
            double volts = AD7193_codeToVolts(results[i] >> 8, gain_bits, false);
            channel_readings[statusv[i]] = volts;
            // Serial.print("statusv[");
            // Serial.print(i);
            // Serial.print("] = ");
            // Serial.print(statusv[i]);
            // Serial.print("\n");
        }
        temperature = results[i] >> 8;
    }

    digitalWrite(cs_pin, HIGH);
	return true;
}

const std::array<double,9>& AD7193::read_channels() const {
    return channel_readings;
}


uint32_t AD7193::AD7193_driver(AD7193_driver_arg_t args){
    //Basic init stuff -- figure out which SPI bus we're using, then get and set the right pins accordingly...
    //pinMode(cs_pin, OUTPUT);
     // digitalWrite(cs_pin, LOW);
    //ADC enters a low power mode after making a conversion, so we have to wait for it to fully power up before communicating with it to avoid problems...
    
    uint8_t reg_id = args.comm_bits & 0b00111000;
    if((args.comm_bits & AD7193_REG_READ) && reg_id == AD7193_DATA_REG){
        while(1){
            if(digitalRead(drdy_pin) == LOW){
                //Serial.print("DRDY went low!\n");
                break;
            }
            else{
                //Serial.print("DRDY was still high\n");
            }
        }
    }

    //TODO WARNING this statement is not thread safe. Make it so when integrating with MINOS
    spi.transfer(args.comm_bits);
    
    if(args.comm_bits & AD7193_REG_READ){
        //Reading data from one of the ADC's registers...
        //Readable registers in the AD7193 have one of two sizes: 8-bit, and 24-bit. The 8-bit registers are STATUS, ID, and GPOCON, with all others being 24-bit...
        if(reg_id == AD7193_STATUS_REG || reg_id == AD7193_ID_REG || reg_id == AD7193_GPOCON_REG)
            return spi.transfer(0);
        else{
            //Read operations from the data register trigger a fresh conversion in the ADC, so we have to wait for the result to settle before reading...
            //The ADC signals that it's ready to output a reading by bringing its D_OUT pin low (without being triggered by the clock), so we wait for this to happen before proceeding...

            uint32_t sensor_return = 0;
            //Bits are delivered to us going from MSB to LSB...
            //Include logic here to account for whether or not the status register bits are being appended!!!!
            sensor_return |= spi.transfer(0) << 24;
            sensor_return |= spi.transfer(0) << 16;
            sensor_return |= spi.transfer(0) << 8;
            sensor_return |= spi.transfer(0);
            return sensor_return;
        }
        //End of READ if-block...
    }
    else{
        //Writing data to one of the ADC's registers...
        //Only one of the AD7193's writeable registers has 8-bit size -- the rest are 24-bit...
        if(reg_id == AD7193_GPOCON_REG)
            spi.transfer((uint8_t)args.write_payload);
        else{
            //We transfer the bits going from MSB to LSB...
            spi.transfer((uint8_t)(args.write_payload >> 16));
            spi.transfer((uint8_t)(args.write_payload >> 8));
            spi.transfer((uint8_t)args.write_payload);
        }
        return 0;
        //End of WRITE else-block
    }
    //Deselect device to avoid spurious writes...
    //digitalWrite(cs_pin, HIGH);
}