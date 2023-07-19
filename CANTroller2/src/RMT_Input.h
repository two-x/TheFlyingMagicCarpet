#include <driver/rmt.h>

class RMTInput
{

public:
  RMTInput(rmt_channel_t channel, gpio_num_t gpio);

  void init();

  uint32_t readPulseWidth();

private:
  rmt_channel_t channel_;
  gpio_num_t gpio_;

  RingbufHandle_t rb_;
};
