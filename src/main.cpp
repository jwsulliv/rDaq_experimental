#include "AD7193.h"
#include "AdcChannel.h"
#include <array>

#define ADC1_CS 0
#define ADC1_SPI SPI1
#define ADC1_DRDY 1

#define ADC0_CS 10
#define ADC0_SPI SPI
#define ADC0_DRDY 12

AD7193 Adc0(ADC0_CS, ADC0_SPI, ADC0_DRDY);

// std::array<AdcChannel*, 9> adcChannels; // 8 channels plus temp sense
 std::array<AdcChannel*, 9> adcChannels; // 8 channels plus temp sense

void setup()
{
  Serial.print("Started setup\n");
  
  
  Serial.begin(9600);
  Adc0.init();
  Adc0.update();
  delayMicroseconds(1);
  
  for (int i = 0; i < 9; i++) {
    Adc0.update();
    delayMicroseconds(1);
    adcChannels[i] = new AdcChannel(i, Adc0); 
    adcChannels[i]->init();
  }  
    //  adcChannels[0] = new AdcChannel(3, Adc0); 
    //  adcChannels[0]->init();


  Serial.print("Finished Setup\n");
}

void loop()
{
  Adc0.update();
  delayMicroseconds(1);

  Serial.print("Printing all channels:\n");
  int i = 0;
  for (auto &channel : adcChannels) {
    channel->update();  // Update the reading
    int32_t data = channel->get_data();  // Get the data
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(data);
    Serial.print("\n");
    i++;
}
  Serial.print("\n");
  delay(100);
}