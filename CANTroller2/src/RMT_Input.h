#include <driver/rmt.h>

class RMTInput
{

public:
  RMTInput(rmt_channel_t channel, gpio_num_t gpio);

  void init();

  float readPulseWidth(bool reject_zeroes);

private:
  rmt_channel_t channel_;
  gpio_num_t gpio_;
  float pulse_width_last;
  float scale_factor = 0.625;
  RingbufHandle_t rb_;
};
