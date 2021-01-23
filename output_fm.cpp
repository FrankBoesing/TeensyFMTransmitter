/*
   Legal note

  TeensyFMTransmitter is an experimental program, designed only for experimentation.
  It is in no way intended to become a personal media center or a tool to operate a radio station,
  or even broadcast sound to one's own stereo system.

  In most countries, transmitting radio waves without a state-issued licence specific
  to the transmission modalities (frequency, power, bandwidth, etc.) is illegal.

  Therefore, always connect a shielded transmission line from the Teensy directly to a radio receiver,
  so as not to emit radio waves. Never use an antenna.

  Even if you are a licensed amateur radio operator, using TeensyFMTransmitter to transmit
  radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

  I could not be held liable for any misuse of your own Teensy.
  Any experiment is made under your own responsibility.

*/

//  (c) Frank B

#if defined(__IMXRT1062__)

#include "output_fm.h"
#include <utility/dspinst.h>


#define NUM_SAMPLES (AUDIO_BLOCK_SAMPLES / 2)

DMAMEM __attribute__((aligned(32))) static int fm_tx_buffer[NUM_SAMPLES * 2];
audio_block_t * AudioOutputFM::block_left = NULL;
bool AudioOutputFM::update_responsibility = false;
DMAChannel AudioOutputFM::dma(false);

static double FM_MHz;
static const float FM_Hub = 75000.0f; //deviation

static const int ndiv = 10000;
static double FS; //PLL Frequency

//preemphasis
static float pre_a0;
static float pre_a1;
static float pre_b;

extern "C" void xbar_connect(unsigned int input, unsigned int output);
inline void calc(audio_block_t *block, const unsigned offset);

// https://www.nxp.com/docs/en/application-note/AN5078.pdf
#define PADCONFIG ((0 << 0) | (1 << 3) | (1 << 6)) //Pg 622 Reference Manual
/*
        (1 << 0) | // Slew rate (1 = fast)
        (1 << 3) | // 3 Bit Driver Strength;
        (1 << 6);  // 2 Speed
*/

FLASHMEM
void AudioOutputFM::begin(uint8_t mclk_pin, unsigned MHz, int preemphasis)
{
  memset(fm_tx_buffer, 0, sizeof(fm_tx_buffer));

  dma.begin(true); // Allocate the DMA channel first

  FM_MHz = MHz;
  FS = FM_MHz * 1000000 * 4;

    // pre-emphasis has been based on the filter designed by Jonti
    // use more sophisticated pre-emphasis filter: https://jontio.zapto.org/download/preempiir.pdf
    // https://github.com/jontio/JMPX/blob/master/libJMPX/JDSP.cpp
  // TODO: uncomment this in case that a sample rate of 192ksps is used !
  // sample rate 192ksps
/*  if(preemphasis == PREEMPHASIS_50)
  {
    pre_a0 = 5.309858008;
    pre_a1 = -4.794606188;
    pre_b = 0.4847481783;
  }
  else
  {
    pre_a0 = 7.681633687l;
    pre_a1 = -7.170926091l;
    pre_b = 0.4892924010l;
  }
*/

    // this is a more sophisticated pre-emphasis filter: https://jontio.zapto.org/download/preempiir.pdf
  // filter coefficients calculated for new sample rate of 44.1ksps by DD4WH, 2021-01-23
  // sample rate 44.1ksps
  if(preemphasis == PREEMPHASIS_50)
  {
    pre_a0 = 4.655206052723760;
    pre_a1 = -2.911399421812300;
    pre_b = -0.743806630911458;
  }
  else
  { // PREAMPHASIS_75
    pre_a0 = 6.597864306804010;
    pre_a1 = -4.854202108640480;
    pre_b = -0.743662198163528;
  }
  
  //PLL:

  int n1 = 2; //SAI prescaler
  int n2 = 1 + (24000000 * 27) / (FS * n1);

  double C = (FS * n1 * n2) / 24000000;

  int nfact = C;
  int nmult = C * ndiv - (nfact * ndiv);

  CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_ENABLE
                         | CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(0) // 2: 1/4; 1: 1/2; 0: 1/1
                         | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(nfact);

  CCM_ANALOG_PLL_VIDEO_NUM   = nmult;
  CCM_ANALOG_PLL_VIDEO_DENOM = ndiv;

  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_POWERDOWN;//Switch on PLL
  while (!(CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK)) {}; //Wait for pll-lock

  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_BYPASS;//Disable Bypass

  //ENABLE I2S
  if (mclk_pin == 33) {
    CCM_CCGR5 |= CCM_CCGR5_SAI2(CCM_CCGR_ON);
    CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI2_CLK_SEL_MASK))
                 | CCM_CSCMR1_SAI2_CLK_SEL(1); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4,
    CCM_CS2CDR = (CCM_CS2CDR & ~(CCM_CS2CDR_SAI2_CLK_PRED_MASK | CCM_CS2CDR_SAI2_CLK_PODF_MASK))
                 | CCM_CS2CDR_SAI2_CLK_PRED(n1 - 1)
                 | CCM_CS2CDR_SAI2_CLK_PODF(n2 - 1);
    IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI2_MCLK3_SEL_MASK))
                      | (IOMUXC_GPR_GPR1_SAI2_MCLK_DIR | IOMUXC_GPR_GPR1_SAI2_MCLK3_SEL(0));  //Select MCLK

    I2S2_TMR = 0;
    I2S2_TCR2 = I2S_TCR2_MSEL(1);
    CORE_PIN33_CONFIG = 2;  // EMC_07, 2=SAI2_MCLK
    CORE_PIN33_PADCONFIG = PADCONFIG;
  }

  else if (mclk_pin == 30) {
    CCM_CCGR5 |= CCM_CCGR5_SAI3(CCM_CCGR_ON);
    CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI3_CLK_SEL_MASK))
                 | CCM_CSCMR1_SAI3_CLK_SEL(1); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4,
    CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI3_CLK_PRED_MASK | CCM_CS1CDR_SAI3_CLK_PODF_MASK))
                 | CCM_CS1CDR_SAI3_CLK_PRED(n1 - 1)
                 | CCM_CS1CDR_SAI3_CLK_PODF(n2 - 1);
    IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI3_MCLK3_SEL_MASK))
                      | (IOMUXC_GPR_GPR1_SAI3_MCLK_DIR | IOMUXC_GPR_GPR1_SAI3_MCLK3_SEL(0));  //Select MCLK

    I2S3_TMR = 0;
    I2S3_TCR2 = I2S_TCR2_MSEL(1);
    CORE_PIN30_CONFIG = 3; // EMC_37, 3=SAI3_MCLK
    CORE_PIN30_PADCONFIG = PADCONFIG;
  }

  else if (mclk_pin == 23) {
    CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);
    CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
                 | CCM_CSCMR1_SAI1_CLK_SEL(1); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4
    CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
                 | CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1) // &0x07
                 | CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1); // &0x3f

    // Select MCLK
    IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
                      | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));

    I2S1_TMR = 0;
    I2S1_TCR2 = I2S_TCR2_MSEL(1);

    CORE_PIN23_CONFIG = 3;
    CORE_PIN23_PADCONFIG = PADCONFIG;

  }

  //QTimer
  const int comp1 = ((float)F_BUS_ACTUAL) / (AUDIO_SAMPLE_RATE_EXACT / 2.0f) / 2.0f + 0.5f;
  TMR4_ENBL &= ~(1 << 3); //Disable
  TMR4_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_FORCE;
  TMR4_CSCTRL3 = TMR_CSCTRL_CL1(1) /*| TMR_CSCTRL_TCF1EN*/;
  TMR4_CNTR3 = 0;
  TMR4_LOAD3 = 0;
  TMR4_COMP13 = comp1;
  TMR4_CMPLD13 = comp1;
  TMR4_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(3);
  /*Comparator Preload Register 1 DMA Enable
    Setting this bit enables DMA write requests for CMPLD1 whenever data is transferred out of the CMPLD1
    register into the COMP1 register.
  */
  TMR4_DMA3 = TMR_DMA_CMPLD1DE;
  TMR4_CNTR3 = 0;
  TMR4_ENBL |= (1 << 3); //Enable
 
  //route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);  //enable XBAR
  xbar_connect(XBARA1_IN_QTIMER4_TIMER3, XBARA1_OUT_DMA_CH_MUX_REQ30);
  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(3) | XBARA_CTRL_DEN0;
  
  //setup DMA  
  dma.TCD->SADDR = fm_tx_buffer;
  dma.TCD->SOFF = 4;
  dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
  dma.TCD->NBYTES_MLNO = 4;
  dma.TCD->SLAST = -sizeof(fm_tx_buffer);
  dma.TCD->DOFF = 0;
  dma.TCD->CITER_ELINKNO = sizeof(fm_tx_buffer) / 4;
  dma.TCD->DLASTSGA = 0;
  dma.TCD->BITER_ELINKNO = sizeof(fm_tx_buffer) / 4;
  dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
  dma.TCD->DADDR = (void *)((uint32_t)&CCM_ANALOG_PLL_VIDEO_NUM);
  
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  dma.attachInterrupt(dmaISR); 
  
  dma.enable();
 
  update_responsibility = update_setup();
}

void AudioOutputFM::update(void)
{
  if (AudioOutputFM::block_left) release(AudioOutputFM::block_left);
  AudioOutputFM::block_left = receiveReadOnly(0); // input 0
}

void AudioOutputFM::dmaISR()
{
  dma.clearInterrupt();

  if (dma.TCD->SADDR < (uint32_t)fm_tx_buffer + NUM_SAMPLES * 4) {
    // DMA is transmitting the first half of the buffer
    // so we must fill the second half
    if (AudioOutputFM::block_left) calc(AudioOutputFM::block_left, 0);
  } else {
    //fill the first half
    if (AudioOutputFM::block_left) calc(AudioOutputFM::block_left, NUM_SAMPLES);
    if (AudioOutputFM::update_responsibility) AudioStream::update_all();
  }

  asm("DSB");
}

inline void calc(audio_block_t *block, const unsigned offset)
{

  static float lastSample = 0;
  static float lastInputSample = 0;
  float fsample;

  for (unsigned idx = offset; idx < NUM_SAMPLES+offset; idx++)
  {
    fsample = block->data[idx];

    // pre-emphasis filter: https://jontio.zapto.org/download/preempiir.pdf
    // https://github.com/jontio/JMPX/blob/master/libJMPX/JDSP.cpp
    fsample = pre_a0 * fsample + pre_a1 * lastInputSample + pre_b * fsample;
    lastInputSample = block->data[idx];

    //TBD: add pilot-tone, process stereo
    //TBD: RDS(?)

    //Calc PLL:
    float fs = FS + fsample * 4.0f  /* volume correction: */ * 2.0f * (FM_Hub / 4.0f / 32767.0f);

    const int n1 = 2; //SAI prescaler
    int n2 = 1 + (24000000 * 27) / (fs * n1);

    double C = (fs * n1 * n2) / 24000000;
    int nfact = C;
    int nmult = C * ndiv - (nfact * ndiv);
    fm_tx_buffer[idx] = nmult;

  }
  arm_dcache_flush_delete(&fm_tx_buffer[offset], sizeof(fm_tx_buffer) / 2 );
}


#endif
