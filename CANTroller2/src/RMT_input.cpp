#include <driver/rmt.h>
#include "RMT_Input.h"
#include <Arduino.h>

RMTInput::RMTInput(rmt_channel_t channel, gpio_num_t gpio)
{
  channel_ = channel;
  gpio_ = gpio;
}

void RMTInput::init()
{

  rmt_config_t config;
  config.channel = channel_;
  config.gpio_num = gpio_;
  config.clk_div = 50; // slowed from 80 because the buffer was getting full
  config.mem_block_num = 1;
  config.rmt_mode = RMT_MODE_RX;
  config.flags = 0;
  config.rx_config.filter_en = true;          // Enable the filter
  config.rx_config.filter_ticks_thresh = 100; // Set the filter threshold
  config.rx_config.idle_threshold = 12000;    // Set the idle threshold

  esp_err_t config_result = rmt_config(&config);
  if (config_result != ESP_OK)
  {
    Serial.printf("Failed to configure RMT: %d\n", config_result);
    while (1)
      ; // halt execution
  }

  esp_err_t install_result = rmt_driver_install(channel_, 2000, 0);
  if (install_result != ESP_OK)
  {
    Serial.printf("Failed to install RMT driver: %d\n", install_result);
    while (1)
      ; // halt execution
  }

  rmt_get_ringbuf_handle(channel_, &rb_);
  if (rb_ == NULL)
  {
    Serial.println("Failed to initialize ring buffer");
    while (1)
      ; // halt execution
  }

  esp_err_t start_result = rmt_rx_start(channel_, 1);
  if (start_result != ESP_OK)
  {
    Serial.printf("Failed to start RMT receiver: %d\n", start_result);
    while (1)
      ; // halt execution
  }
}

float RMTInput::readPulseWidth(bool reject_zeroes=false)
{
  size_t rx_size = 0;
  rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb_, &rx_size, 0);
  if (item != NULL && rx_size == sizeof(rmt_item32_t))
  {
    uint32_t pulse_width = item->duration0 + item->duration1;
    vRingbufferReturnItem(rb_, (void *)item);
    if (!reject_zeroes || pulse_width > 0) pulse_width_last = (float)pulse_width * scale_factor;
  }
  return pulse_width_last; // No data
}