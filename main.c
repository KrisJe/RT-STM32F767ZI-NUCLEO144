/*

*/

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"

#include "chprintf.h"
#include "usbcfg.h"


/* Virtual serial port over USB.*/
//SerialUSBDriver SDU1;
//BaseSequentialStream *chp = (BaseSequentialStream *)&SDU1;

//registers for STM32F722ZE
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF0F44E)) //110deg
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF0F44C)) //30deg
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (300))

#if 0
#include "adc_multi.h"
#endif

/* TRUE means that DMA-accessible buffers are placed in a non-cached RAM
   area and that no cache management is required.*/
#define DMA_BUFFERS_COHERENCE TRUE

/*
 * GPT4 configuration.
 */
static const GPTConfig gpt4cfg1 = {
  .frequency    = 1000000U,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,    /* MMS = 010 = TRGO on Update Event.    */
  .dier         = 0U
};


/*
 * GPT6 configuration.
 */
static const GPTConfig gpt6cfg1 = {
  .frequency    = 1000000U,
  .callback     = NULL,
  .cr2          = TIM_CR2_MMS_1,    /* MMS = 010 = TRGO on Update Event.    */
  .dier         = 0U
};


/*===========================================================================*/
/* ADC driver related.                                                       */
/*===========================================================================*/

#define ADC_GRP1_NUM_CHANNELS   3
#define ADC_GRP1_BUF_DEPTH      8

#if !DMA_BUFFERS_COHERENCE
/* Note, the buffer is aligned to a 32 bytes boundary because limitations
   imposed by the data cache. Note, this is GNU specific, it must be
   handled differently for other compilers.
   Only required if the ADC buffer is placed in a cache-able area.*/
#if defined(__GNUC__)
__attribute__((aligned (32)))
#endif
#endif
static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0, nz = 0;
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

#if !DMA_BUFFERS_COHERENCE
  /* DMA buffer invalidation because data cache, only invalidating the
     half buffer just filled.
     Only required if the ADC buffer is placed in a cache-able area.*/
  dmaBufferInvalidate(buffer,
                      n * adcp->grpp->num_channels * sizeof (adcsample_t));
#else
  (void)adcp;
#endif

  nz++;

  /* Updating counters.*/
  if (samples1 == buffer) {
    nx += n;
  }
  else {
    ny += n;
  }

  if ((nz % 10000) == 0) {
      	  palToggleLine(LINE_LED2);
        }

}

/*
 * ADC errors callback, should never happen.
 */
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Continuous, 16 samples of 2 channels, HS triggered by
 *              GPT4-TRGO.
 * Channels:    Sensor, VRef.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  true,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0,                                                    /* CR1   */
  ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12),        /* CR2   */
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144) |
  ADC_SMPR1_SMP_VREF(ADC_SAMPLE_144)  |
  ADC_SMPR1_SMP_VBAT(ADC_SAMPLE_144),                   /* SMPR1 */
  0,                                                    /* SMPR2 */
  0,                                                    /* SQR1  */
  0,  													/* SQR2  */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_SENSOR) |					 /* SQR3  */
  ADC_SQR3_SQ2_N(ADC_CHANNEL_VREFINT)|
  ADC_SQR3_SQ3_N(ADC_CHANNEL_VBAT)

};

/*===========================================================================*/
/* Application code.                                                         */
/*===========================================================================*/

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED attached to TP1.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
	  	palToggleLine(LINE_LED1);
	    chThdSleepMilliseconds(200);

  }
}



int32_t Temp_calc (uint32_t raw_data){
  int32_t temperature;
  temperature = ((raw_data * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR ) ;
  temperature = temperature * (int32_t)(110 - 30);
  temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
  temperature = temperature + 30;
  return(temperature);
}

void SWV_puts(const char *s )
{
    while (*s) ITM_SendChar(*s++);

}




/*
 * Application entry point.
 */
int main(void) {
	int n,ch;



  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);


    /*
      * Starting GPT4 driver, it is used for triggering the ADC.
      */
     gptStart(&GPTD4, &gpt4cfg1);

     /*
      * Fixed an errata on the STM32F7xx, the DAC clock is required for ADC
      * triggering.
      */
     rccEnableDAC1(false);

     /*
      * Activates the ADC1 driver and the temperature sensor.
      */
     adcStart(&ADCD1, NULL);
     adcSTM32EnableTSVREFE();

     /*
      * Starts an ADC continuous conversion triggered with a period of
      * 1/10000 second.
      */
     adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
     gptStartContinuous(&GPTD4, 100);

     /*
      * Creates the example thread.
      */
     chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */

  //int u=0;
  while (true) {
	  chThdSleepMilliseconds(250);
	    //chprintf(chp, "\r\n(%8u,%8u): [%4u,%4u,%4u,%4u]\r\n", nx, ny, samples[0],samples[3],samples[6],samples[9]);
	  	chprintf(chp, "\r\n(%8u): ", nx);

	  	 for (ch = 0; ch < ADC_GRP1_NUM_CHANNELS; ch++)
	  	 {
	  		 chprintf(chp, "\r\nchannel %2u: ",ch);
	  		 for (n = 0; n < ADC_GRP1_BUF_DEPTH; n++) {
	  			 chprintf(chp, "%4u,", samples1[ch + n * ADC_GRP1_NUM_CHANNELS]);
	  		 }
	  	 }

	    chprintf(chp, "\r\n%4u degC", Temp_calc(samples1[0]));


	    SWV_puts("hello\n");

	    //if ((ADC1->ISR & ADC_ISR_EOC)){
	    //Temperature_show(Temerature_calc(ADC1->DR));

	    //chprintf(chp, "\r\nTEMP = %4u" ,Temp_calc(ADC1->DR));

	    //ADC1->ISR |= ADC_ISR_EOC;
	    //}




  }
}

