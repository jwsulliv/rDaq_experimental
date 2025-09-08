#pragma once
#include "SPI.h"
#include <map>

//Macros relevant to the Communications register... (the first byte transferred to the AD7193 in a transaction will always be written to the Comm register, so it has no unique address)
//Bits 0, 6, and 7 must always be empty (0). Bit 1 = r/w ; bits 2-4 = register address ; bit 5 = enable continuous read (has an effect for data reg reads only)
#define AD7193_REG_READ 0b01000000            //Signals a read operation
#define AD7193_REG_WRITE 0b00000000           //Signals a write operation
#define AD7193_STATUS_REG 0b00000000        //8-bit reg (r)
#define AD7193_MODE_REG 0b00001000          //24-bit reg (r/w)  
#define AD7193_CONFIG_REG 0b00010000        //24-bit reg (r/w)
#define AD7193_DATA_REG 0b00011000          //24-bit reg (r)
#define AD7193_ID_REG 0b00100000            //8-bit reg (r)
#define AD7193_GPOCON_REG 0b00101000        //8-bit reg (r/w)
#define AD7193_OFFSET_REG 0b00110000        //24-bit reg (r/w)
#define AD7193_FULLSCALE_REG 0b00111000     //24-bit reg (r/w)

#define AD7193_CHANNEL_BIT 8
#define AD7193_DIFFERENTIAL_BIT 18
#define AD7193_TEMP_BIT 0x010000
#define AD7193_PGA_BIT 0
#define AD7183 POLARITY_BIT 3
#define AD7193_CHANNEL_SHIFT 0x00000100

//Have to include some global variables here that affect the conversion from output code to volts for the AD7193 --
//change these in the AD7193 functions or hardcode them to something if they're never going to change!!!

//This info appears on sheet 3 of the MoteV5 schematic -- it appears as REFIN1(+)!!
#define VREF 2.5
#define AD7193_CODE_EXP_TERM 16777216   //This is 2^24

class AD7193 {
    public:
        AD7193(uint32_t cs_pin, SPIClass &adc_spi, uint32_t drdy) : 
            cs_pin(cs_pin),
            spi(adc_spi),
            drdy_pin(drdy),
            differential(false), //maybe make a parameter
            channel_readings(),
            pga_gain(1) //ditto
        {};

        bool add_channel(uint8_t channel);
        double read_channel(uint8_t channel);
        void set_differential(bool val);
        void set_pga_gain(uint16_t gain);


        uint32_t get_temperature();
        bool init();
        bool update();
        uint32_t get_cspin();

    private:
        struct AD7193_driver_arg_t {
            uint8_t comm_bits;
            uint32_t write_payload;
        };
        
        struct AD7193_reading_t
        {
            uint8_t channel;
            double result;
        };

        std::map<uint8_t, double> channel_readings;
        uint32_t temperature = 0;
        
        uint32_t AD7193_driver(AD7193_driver_arg_t args);
        double AD7193_codeToVolts(uint32_t code, uint16_t currentGainSetting, bool unipolar);

        uint32_t cs_pin;
        uint32_t drdy_pin;
        SPIClass &spi;
        bool differential;
        uint16_t pga_gain;
        uint32_t channel_bits = 0x0;
};