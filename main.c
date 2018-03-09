/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
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

#include "adc_multi.h"

/* Number of ADCs used in multi ADC mode (2 or 3) */
#define ADC_N_ADCS 3

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS_PER_ADC   2

/* Depth of the conversion buffer, channels are sampled one time each.*/
#define ADC_GRP1_BUF_DEPTH      (4*ADC_N_ADCS) // must be 1 or even

static adcsample_t samples[ADC_GRP1_NUM_CHANNELS_PER_ADC * ADC_N_ADCS * ADC_GRP1_BUF_DEPTH];


adcsample_t avg_PC1, avg_PC2, avg_PC3, avg_PC4;
adcsample_t PC1_mV, PC2_mV, PC3_mV, PC4_mV;

/*
 * ADC streaming callback.
 */
size_t mysample = 0;
size_t nx = 0, ny = 0 , nz = 0;
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	/*ADC notification callback type.
	Parameters:
    [in]	adcp	pointer to the ADCDriver object triggering the callback
    [in]	buffer	pointer to the most recent samples data
    [in]	n	number of buffer rows available starting from buffer

    Giovanni:
    The callback is called twice for circular conversions! One at half of the buffer and one at the end of the conversion.
    */


  (void)adcp;
  //ny = n;
  //nx = buffer-samples; //6 integer values in buffer

  nz++;

  if (samples == buffer) { //compare pointers
    nx += n; //n = ADC_GRP1_BUF_DEPTH / 2
    //avg_PC1 = (samples[0] + samples[3] + samples[6]  + samples[9]) / 4;

    avg_PC1 = 0;
    for (n = 0; n < ADC_GRP1_BUF_DEPTH; n++) avg_PC1 += samples[n];

    avg_PC1 /= ADC_GRP1_BUF_DEPTH;


    palToggleLine(LINE_LED3);

  }
  else {
    ny += n;
  }

/*

	  palClearLine(LINE_LED3);

      // Calculates the average values from the ADC samples
	  avg_PC1 = (samples[0] + samples[4] + samples[8]  + samples[12]) / 4;      // temp sensor
	  avg_PC2 = (samples[1] + samples[5] + samples[9]  + samples[13]) / 4;
	  avg_PC3 = (samples[2] + samples[6] + samples[10] + samples[14]) / 4;
	  avg_PC4 = (samples[3] + samples[7] + samples[11] + samples[15]) / 4;

	  // Conversion in volt
	  PC1_mV = (avg_PC1/4095) * 3300;
	  PC2_mV = (avg_PC2/4095) * 3300;
	  PC3_mV = (avg_PC3/4095) * 3300;
	  PC4_mV = (avg_PC4/4095) * 3300;
*/

}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

#define ADC_SAMPLE_DEF ADC_SAMPLE_3
//#define ADC_SAMPLE_DEF ADC_SAMPLE_15
//#define ADC_SAMPLE_DEF ADC_SAMPLE_28
//#define ADC_SAMPLE_DEF ADC_SAMPLE_56
//#define ADC_SAMPLE_DEF ADC_SAMPLE_84
//#define ADC_SAMPLE_DEF ADC_SAMPLE_112
//#define ADC_SAMPLE_DEF ADC_SAMPLE_144
//#define ADC_SAMPLE_DEF ADC_SAMPLE_480
/*
 * ADC conversion group for ADC0 as multi ADC mode master.
 * Mode:        Circular buffer, triple ADC mode master, SW triggered.
 * Channels:    PA0, PA3
 */
static const ADCConversionGroup adcgrpcfg1 = {
  TRUE, // Circular conversion
  ADC_GRP1_NUM_CHANNELS_PER_ADC,
  adccallback, /* end of conversion callback */
  adcerrorcallback, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  ADC_CR2_SWSTART, // CR2
  0, // SMPR1
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_DEF)
   | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_DEF), // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN3) // SQR3
};

/*
 * ADC conversion group for ADC2.
 * Mode:        triple ADC mode slave.
 * Channels:    PA1, PC0
 */
static const ADCConversionGroup adcgrpcfg2 = {
  TRUE,
  0,
  NULL, /* end of conversion callback */
  NULL, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  0, // CR2
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_DEF), // SMPR1
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_DEF), // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10) // SQR3
};

/*
 * ADC conversion group for ADC3.
 * Mode:        triple ADC mode slave.
 * Channels:    PA2, PC1
 */
static const ADCConversionGroup adcgrpcfg3 = {
  TRUE,
  0,
  NULL, /* end of conversion callback */
  NULL, /* error callback */
  /* HW dependent part.*/
  0, // CR1
  0, // CR2
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_DEF), // SMPR1
  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_DEF), // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_PER_ADC), // SQR1
  0, // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2)
   | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) // SQR3
};






/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
	    palSetLine(LINE_LED1);
	    chThdSleepMilliseconds(50);
	    palSetLine(LINE_LED2);
	    chThdSleepMilliseconds(50);
	    palSetLine(LINE_LED3);
	    chThdSleepMilliseconds(200);
	    palClearLine(LINE_LED1);
	    chThdSleepMilliseconds(50);
	    palClearLine(LINE_LED2);
	    chThdSleepMilliseconds(50);
	    //palClearLine(LINE_LED3);
	    chThdSleepMilliseconds(200);
  }
}

/*
 * Application entry point.
 */
int main(void) {
	int n;



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
      * Setting up analog inputs used by the demo.
      */
     //settings from F4 ipv F7!!!!   ->					memory layout
     palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); //samples[0+i*3]
     palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); //samples[1+i*3]
     palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); //samples[2+i*3]


     //palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); // samples[1+i*3] !!!!

     //palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); //samples[1+i*3] IN11
     //palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); //samples[2+i*3] IN12





     /*
      * Creates the blinker thread.
      */
     chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);


  /*
     * Activates the ADC drivers.
     */
    adcMultiStart();


    /*
     * Starts an ADC continuous conversion.
     */
    adcMultiStartConversion(&adcgrpcfg1, &adcgrpcfg2, &adcgrpcfg3, samples, ADC_GRP1_BUF_DEPTH);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */

  //int u=0;
  while (true) {
	  chThdSleepMilliseconds(10);
	    //chprintf(chp, "\r\n(%8u,%8u): [%4u,%4u,%4u,%4u]\r\n", nx, ny, samples[0],samples[3],samples[6],samples[9]);
	  	chprintf(chp, "\r\n(%8u,%4u): ", nx, avg_PC1);

	    for (n = 0; n < ADC_GRP1_NUM_CHANNELS_PER_ADC * ADC_N_ADCS * ADC_GRP1_BUF_DEPTH; n++) {
	       chprintf(chp, "%4u,", samples[n]);
	    }


  }
}

