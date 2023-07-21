#include <driver/rmt.h>

class RMTInput
{

public:
  RMTInput(rmt_channel_t channel, gpio_num_t gpio);

  void init();

  int32_t readPulseWidth(bool persistence=true);

private:
  rmt_channel_t channel_;
  gpio_num_t gpio_;
  int32_t pulse_width, pulse_width_last;
  float scale_factor = 0.625;
  RingbufHandle_t rb_;
};
