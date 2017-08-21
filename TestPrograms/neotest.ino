/* Clockless controllers.  These controllers have 3 control points in their cycle for each bit.  The first point
  is raised hi.  The second pointsnt is where the line is dropped low for a zero.  The third point is where the
#dw for a one.  T1, T2, and T3 correspond to the timings for those three in clock cycles.
*/
#define T1 ?
#define T2 ?
#define T3 ?
#define TADJUST 0
#define TOTAL ( (T1+TADJUST) + (T2+TADJUST) + (T3+TADJUST) )
#define T1_MARK (TOTAL - (T1+TADJUST))
#define T2_MARK (T1_MARK - (T2+TADJUST))

static uint32_t showRGBInternal(PixelController<RGB_ORDER, LANES, PORT_MASK> &allpixels) {
  // get num leds
  // int nLeds = allpixels.mLen;
   int numLeds = 10;

  // Setup the pixel controller and load/scale the first byte
	// Lines b0,b1,b2;

  // allpixels.preStepFirstByteDithering();
	// for(uint8_t i = 0; i < LANES; i++) {
	// 	b0.bytes[i] = allpixels.loadAndScale0(i);
	// }

	// Setup and start the clock
  TC_Configure(DUE_TIMER,DUE_TIMER_CHANNEL,TC_CMR_TCCLKS_TIMER_CLOCK1);
  pmc_enable_periph_clk(DUE_TIMER_ID);
  TC_Start(DUE_TIMER,DUE_TIMER_CHANNEL);

  #if (MY_ALLOW_INTERRUPTS == 1)
  cli();
  #endif
	uint32_t next_mark = (DUE_TIMER_VAL + (TOTAL));
	while(nLeds--) {
    allpixels.stepDithering();
    #if (MY_ALLOW_INTERRUPTS == 1)
    cli();
    if(DUE_TIMER_VAL > next_mark) {
      if((DUE_TIMER_VAL - next_mark) > ((WAIT_TIME-INTERRUPT_THRESHOLD)*CLKS_PER_US)) {
        sei(); TC_Stop(DUE_TIMER,DUE_TIMER_CHANNEL); return DUE_TIMER_VAL;
      }
    }
    #endif

		// Write first byte, read next byte
		writeBits<8+XTRA0,1>(next_mark, b0, b1, allpixels);

		// Write second byte, read 3rd byte
		writeBits<8+XTRA0,2>(next_mark, b1, b2, allpixels);

		// Write third byte
		writeBits<8+XTRA0,0>(next_mark, b2, b3, allpixels);

		// Write fourth byte
		writeBits<8+XTRA0,0>(next_mark, b3, b0, allpixels);

    #if (MY_ALLOW_INTERRUPTS == 1)
    sei();
    #endif
	}

	return DUE_TIMER_VAL;
}

template<int BITS,int PX> __attribute__ ((always_inline)) inline static void writeBits(register uint32_t & next_mark, register Lines & b, Lines & b3, PixelController<RGB_ORDER,LANES, PORT_MASK> &pixels) { // , register uint32_t & b2)  {
  Lines b2;
  transpose8x1(b.bytes,b2.bytes);

   uint8_t * bytes;

  // register uint8_t d = pixels.template getd<PX>(pixels);
  // register uint8_t scale = pixels.template getscale<PX>(pixels);

  for(uint32_t i = 0; (i < LANES) && (i<8); i++) {
    while(DUE_TIMER_VAL < next_mark);
    next_mark = (DUE_TIMER_VAL+TOTAL);

    //*FastPin<FIRST_PIN>::sport() = PORT_MASK;
    *mysport = PORT_MASK;

    // this selects the i-th strip and then steps index bytes into it.
    uint32_t offset = ( numLeds - ( i * ledsPerStrip ) ) * bytesPerLed ) + index;

    while((next_mark - DUE_TIMER_VAL) > (T2+T3+6));
    // *FastPin<FIRST_PIN>::cport() = (~b2.bytes[7-i]) & PORT_MASK;
    *mycport = ~bytes[] & PORT_MASK;

    while((next_mark - (DUE_TIMER_VAL)) > T3);
    //*FastPin<FIRST_PIN>::cport() = PORT_MASK;
    *mycport = PORT_MASK;

    // b3.bytes[i] = pixels.template loadAndScale<PX>(pixels,i,d,scale);
  }

  // this just writes the unused lanes, we use them all so ignore
  // for(uint32_t i = LANES; i < 8; i++) {
  //   while(DUE_TIMER_VAL > next_mark);

  //   next_mark = DUE_TIMER_VAL - (TOTAL-3);
  //   *FastPin<FIRST_PIN>::sport() = PORT_MASK;

  //   while((next_mark - DUE_TIMER_VAL) > (T2+T3+6));
  //   *FastPin<FIRST_PIN>::cport() = (~b2.bytes[7-i]) & PORT_MASK;

  //   while((next_mark - DUE_TIMER_VAL) > T3);
  //   *FastPin<FIRST_PIN>::cport() = PORT_MASK;
  // }
}
