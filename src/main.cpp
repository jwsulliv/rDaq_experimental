#include "AD7193.h"
#include <array>

#define ADC1_CS 0
#define ADC1_SPI SPI1
#define ADC1_DRDY 1

#define ADC0_CS 10
#define ADC0_SPI SPI
#define ADC0_DRDY 12

AD7193 Adc0(ADC0_CS, ADC0_SPI, ADC0_DRDY);
std::array<double, 9> channel_volts; 

void setup()
{
  Serial.print("Started setup\n");
  Serial.begin(9600);
  Adc0.init();
  //Adc0.update();
  delayMicroseconds(1);
  
  std::array<ChannelConfig, 9> channelGains{{
            {1}, {1}, {1}, {1}, {1}, {1}, {1}, {1}, {1}
        }};

  for (int i = 0; i < 9; i++) {
    Adc0.configure_channels(channelGains);
  }  
  Serial.print("Finished Setup\n");
}

void loop()
{
  for (int i = 0; i<9; i++){
    Adc0.update(i);
  }
  channel_volts = Adc0.read_channels();
  Serial.print("---\n");
  
  for (int i = 0; i<9; i++){
    Serial.print(channel_volts[i]);
    Serial.print("\n");
  }
delay(10);
}