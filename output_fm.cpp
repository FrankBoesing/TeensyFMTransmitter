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


IntervalTimer myTimer;

static double FM_MHz;
static const float FM_Hub = 75000.0f; //deviation

static const int ndiv = 10000;
static double FS; //PLL Frequency

//preemphasis:  EU: 50 us -> tau = 50e-6, USA: 75 us -> tau = 75e-6
static float FM_preemphasis, preemp_alpha, onem_preemp_alpha;


inline void calc(audio_block_t *block, const unsigned offset);

// https://www.nxp.com/docs/en/application-note/AN5078.pdf
#define PADCONFIG ((0 << 0) | (1 << 3) | (1 << 6)) //Pg 622 Reference Manual
/*
        (1 << 0) | // Slew rate (1 = fast)
        (1 << 3) | // 3 Bit Driver Strength;
        (1 << 6);  // 2 Speed
*/

FLASHMEM
void AudioOutputFM::begin(uint8_t mclk_pin, unsigned MHz, float preemphasis)
{
  memset(fm_tx_buffer, 0, sizeof(fm_tx_buffer));

  dma.begin(true); // Allocate the DMA channel first

  FM_MHz = MHz;
  FS = FM_MHz * 1000000 * 4;

  FM_preemphasis    = preemphasis; //EU: 50 us -> tau = 50e-6, USA: 75 us -> tau = 75e-6
  preemp_alpha      = (1.0 - exp(- 1.0 / (AUDIO_SAMPLE_RATE_EXACT * (double)FM_preemphasis)));
  onem_preemp_alpha = (1.0 - preemp_alpha);

  //PLL:

  // PLL between 27*24 = 648MHz und 54*24=1296MHz
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

  dma.attachInterrupt(dmaISR);
  //  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX);
  dma.enable();

  update_responsibility = update_setup();



  //TBD: Don't use this. Use DMA !important!
  myTimer.begin(isr, 1000000.0 / AUDIO_SAMPLE_RATE_EXACT);
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

void AudioOutputFM::isr(void)
{
  static int idx = 0;
  audio_block_t *block = AudioOutputFM::block_left;

  // CCM_ANALOG_PLL_VIDEO_NUM   = fm_tx_buffer[idx];
  AudioOutputFM::dma.triggerManual();
#if 0   
  idx++;
  if (idx >= AUDIO_BLOCK_SAMPLES) {
    idx = 0;
    if (AudioOutputFM::update_responsibility) AudioStream::update_all();
    if (AudioOutputFM::block_left) calc(AudioOutputFM::block_left);
  }
#endif
  asm("DSB");
}

inline void calc(audio_block_t *block, const unsigned offset)
{

  static float lastSample = 0;
  float fsample;

  for (unsigned idx = offset; idx < NUM_SAMPLES+offset; idx++)
  {
    fsample = block->data[idx];

    //pre-emphasis
    //   fsample = preemp_alpha * fsample + onem_preemp_alpha * lastSample; //de-emphasis
    fsample = onem_preemp_alpha * fsample + preemp_alpha * lastSample; //pre-emphasis
    lastSample = fsample;

    //TBD: add pilot-tone, process stereo
    //TBD: RDS(?)

    //Calc PLL:
    float fs = FS + fsample * 4.0f * (FM_Hub / 4.0f / 32767.0f);

    // PLL between 27*24 = 648MHz und 54*24=1296MHz
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
