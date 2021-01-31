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
  radio waves on ham frequencies without any filtering between the Teensy and an antenna is most probably illegal
  because the square-wave carrier is very rich in harmonics, so the bandwidth requirements are likely not met.

  I could not be held liable for any misuse of your own Teensy.
  Any experiment is made under your own responsibility.

*/

//  (c) Frank B

#if defined(__IMXRT1062__)

#include "output_fm.h"

// Attention: numTaps in the interpolation filter must be a multiple of the interpolation factor L


#define NUM_SAMPLES (AUDIO_BLOCK_SAMPLES / 2) //Input samples
#define I_NUM_SAMPLES (NUM_SAMPLES * INTERPOLATION)


DMAMEM static  __attribute__((aligned(32))) int fm_tx_buffer[I_NUM_SAMPLES * 2];
audio_block_t * AudioOutputFM::block_left = NULL;
audio_block_t * AudioOutputFM::block_right = NULL;
unsigned long AudioOutputFM::us = 0;
bool AudioOutputFM::update_responsibility = false;
DMAChannel AudioOutputFM::dma(false);

static struct {
  float FS; //PLL Frequency
  int i2s_clk_pred;
  int i2s_clk_podf;
  float i2s_clk_divprod;
} carrier;


#define FM_DEVIATION        75000.0;
#define PLL_FREF            24000000.0
#define PLL_FREF_MULT       27.0
#define PLL_DENOMINATOR     (1<<20)
#define PLL_POST_DIV_SELECT 0    // 0: 1/4; 1: 1/2; 2: 1/1

static const float FM_deviation = 4.0 * FM_DEVIATION;

// Attention: interpolation_taps for the interpolation filter must be a multiple of the interpolation factor L (constant INTERPOLATION)
static const unsigned interpolation_taps = 64;
static const float n_att = 90.0; // desired stopband attenuation
static const float interpolation_cutoff = 15000.0f;  // AUDIO_SAMPLE_RATE_EXACT / 2.0f; //
static float interpolation_coeffs[interpolation_taps];
static float interpolation_L_state[NUM_SAMPLES + interpolation_taps / INTERPOLATION];
static float interpolation_R_state[NUM_SAMPLES + interpolation_taps / INTERPOLATION];
static arm_fir_interpolate_instance_f32 interpolationL;
static arm_fir_interpolate_instance_f32 interpolationR;

//preemphasis filter coefficients
static float pre_a0;
static float pre_a1;
static float pre_b1;

//extern and forward declarations:
extern "C" void xbar_connect(unsigned int input, unsigned int output);
extern void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float fc, float Astop, int type, float dfc, float Fsamprate);
extern double cotan(double i);
extern void rds_begin();
extern void rds_update();
extern float rds_sample();
static void process(const audio_block_t *blockL, const audio_block_t *blockR, unsigned offset);
inline static void processStereo(const float blockL[I_NUM_SAMPLES], const float blockR[I_NUM_SAMPLES], const unsigned offset);


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
  pin = mclk_pin;

  carrier.FS = MHz * 1000000 * 4;


  // pre-emphasis has been based on the more sophisticated Pre-emphasis filter designed by Jonti
  // https://jontio.zapto.org/download/preempiir.pdf
  // https://github.com/jontio/JMPX/blob/master/libJMPX/JDSP.cpp
  // I put the formulas from his paper into the code here to calculate filter coeffs dynamically
  // based on sample rate, interpolation factor and pre emphasis time constant tau
  double delta = 1.0 / (2.0 * PI * 15000.0);
  double tau = 50e-6;
  if (preemphasis == PREEMPHASIS_75)
  {
    tau = 75e-6;
  }

  double pre_T = 1.0 / I_SAMPLERATE;
  double tauP = pre_T / 2.0 * cotan(pre_T / 2.0 / tau);
  double deltaP = pre_T / 2.0 * cotan(pre_T / 2.0 / delta);
  double pre_bP = sqrt(-tauP * tauP + sqrt(tauP * tauP * tauP * tauP + 8 * tauP * tauP * deltaP * deltaP)) * 0.5;
  double pre_aP = sqrt(2.0 * pre_bP * pre_bP + tauP * tauP);

  pre_a0 = (2.0 * pre_aP + pre_T) / (2.0 * pre_bP + pre_T);
  pre_a1 = (pre_T - 2.0 * pre_aP) / (2.0 * pre_bP + pre_T);
  pre_b1 = (2.0 * pre_bP - pre_T) / (2.0 * pre_bP + pre_T);

  //calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float fc, float Astop, int type, float dfc, float Fsamprate)
  calc_FIR_coeffs(interpolation_coeffs, interpolation_taps, interpolation_cutoff, n_att, 0, 0.0, I_SAMPLERATE);

  // we initiate the number of input samples, NOT the number of interpolated samples
  if (arm_fir_interpolate_init_f32(&interpolationL, INTERPOLATION, interpolation_taps, interpolation_coeffs, interpolation_L_state, NUM_SAMPLES) ||
      arm_fir_interpolate_init_f32(&interpolationR, INTERPOLATION, interpolation_taps, interpolation_coeffs, interpolation_R_state, NUM_SAMPLES))
  {
    Serial.println("Init of interpolation failed");
    while (1);
  }

  rds_begin();

  carrier.i2s_clk_pred = 1; //SAI prescaler
  carrier.i2s_clk_podf = 1 + (PLL_FREF * PLL_FREF_MULT) / (carrier.FS * carrier.i2s_clk_pred);
  carrier.i2s_clk_divprod = carrier.i2s_clk_pred * carrier.i2s_clk_podf / PLL_FREF;
  double C = carrier.FS * carrier.i2s_clk_divprod;
  int nfact = C;
  int mult = C * PLL_DENOMINATOR - (nfact * PLL_DENOMINATOR);

  CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_ENABLE
                         | CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(PLL_POST_DIV_SELECT) // 0: 1/4; 1: 1/2; 2: 1/1
                         | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(nfact);

  CCM_ANALOG_PLL_VIDEO_NUM   = mult;
  CCM_ANALOG_PLL_VIDEO_DENOM = PLL_DENOMINATOR;

  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_POWERDOWN;//Switch on PLL
  while (!(CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK)) {}; //Wait for pll-lock

  CCM_ANALOG_PLL_VIDEO &= ~CCM_ANALOG_PLL_VIDEO_BYPASS;//Disable Bypass

  //QTimer
  const int comp1 = I_TIMERVAL - 1;
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

  //route the timer output through XBAR to edge trigger DMA request
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
  enable( true ); //Enable I2s
}

void AudioOutputFM::enable(bool enabled)
{
  switch (pin) {

    case 33:
      this->isenabled = enabled;
      if (enabled) {
        CCM_CCGR5 |= CCM_CCGR5_SAI2(CCM_CCGR_ON);
        CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI2_CLK_SEL_MASK))
                     | CCM_CSCMR1_SAI2_CLK_SEL(1); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4,
        CCM_CS2CDR = (CCM_CS2CDR & ~(CCM_CS2CDR_SAI2_CLK_PRED_MASK | CCM_CS2CDR_SAI2_CLK_PODF_MASK))
                     | CCM_CS2CDR_SAI2_CLK_PRED(carrier.i2s_clk_pred - 1)
                     | CCM_CS2CDR_SAI2_CLK_PODF(carrier.i2s_clk_podf - 1);
        IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI2_MCLK3_SEL_MASK))
                          | (IOMUXC_GPR_GPR1_SAI2_MCLK_DIR | IOMUXC_GPR_GPR1_SAI2_MCLK3_SEL(0));  //Select MCLK

        I2S2_TMR = 0;
        I2S2_TCR2 = I2S_TCR2_MSEL(1);
        CORE_PIN33_CONFIG = 2;  // EMC_07, 2=SAI2_MCLK
        CORE_PIN33_PADCONFIG = PADCONFIG;
      } else {
        CORE_PIN33_CONFIG = 0;
      }
      break;

    case 30:
      this->isenabled = enabled;
      if (enabled) {
        CCM_CCGR5 |= CCM_CCGR5_SAI3(CCM_CCGR_ON);
        CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI3_CLK_SEL_MASK))
                     | CCM_CSCMR1_SAI3_CLK_SEL(1); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4,
        CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI3_CLK_PRED_MASK | CCM_CS1CDR_SAI3_CLK_PODF_MASK))
                     | CCM_CS1CDR_SAI3_CLK_PRED(carrier.i2s_clk_pred - 1)
                     | CCM_CS1CDR_SAI3_CLK_PODF(carrier.i2s_clk_podf - 1);
        IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI3_MCLK3_SEL_MASK))
                          | (IOMUXC_GPR_GPR1_SAI3_MCLK_DIR | IOMUXC_GPR_GPR1_SAI3_MCLK3_SEL(0));  //Select MCLK

        I2S3_TMR = 0;
        I2S3_TCR2 = I2S_TCR2_MSEL(1);
        CORE_PIN30_CONFIG = 3; // EMC_37, 3=SAI3_MCLK
        CORE_PIN30_PADCONFIG = PADCONFIG;
      } else {
        CORE_PIN30_CONFIG = 0;
      }
      break;

    case 23:
      this->isenabled = enabled;
      if (enabled) {
        CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);
        CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
                     | CCM_CSCMR1_SAI1_CLK_SEL(1); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4
        CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
                     | CCM_CS1CDR_SAI1_CLK_PRED(carrier.i2s_clk_pred - 1) // &0x07
                     | CCM_CS1CDR_SAI1_CLK_PODF(carrier.i2s_clk_podf - 1); // &0x3f

        // Select MCLK
        IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
                          | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));
        I2S1_TMR = 0;
        I2S1_TCR2 = I2S_TCR2_MSEL(1);
        CORE_PIN23_CONFIG = 3;
        CORE_PIN23_PADCONFIG = PADCONFIG;
      } else {
        CORE_PIN23_CONFIG = 0;
      }
      break;
    default:
      this->isenabled = false;
  }
}

//Update Audio Library Data:
void AudioOutputFM::update(void)
{
  if (AudioOutputFM::block_left) release(AudioOutputFM::block_left);
  if (AudioOutputFM::block_right) release(AudioOutputFM::block_right);
  AudioOutputFM::block_left = receiveReadOnly(0); // input 0
  AudioOutputFM::block_right = receiveReadOnly(1); // input 1
}

//DMA interrupt:
void AudioOutputFM::dmaISR()
{
  static unsigned long t1;
  dma.clearInterrupt();

  if ((uintptr_t)dma.TCD->SADDR < (uintptr_t)&fm_tx_buffer[I_NUM_SAMPLES]) {

    t1 = micros();
    // DMA is transmitting the first half of the buffer
    // so we must fill the second half
    process(AudioOutputFM::block_left, AudioOutputFM::block_right,  NUM_SAMPLES);
    t1 = micros() - t1;
    if (AudioOutputFM::update_responsibility) AudioStream::update_all();

  } else {

    //fill the first half
    unsigned long t2 = micros();
    process(AudioOutputFM::block_left, AudioOutputFM::block_right, 0);
    //update RDS Data
    rds_update();
    t2 = micros() - t2;
    unsigned long us = t1 + t2;
    if (us > AudioOutputFM::us) AudioOutputFM::us = us;
  }

}

//Translates a sample to PLL multiplicator:
static inline __attribute__ ((pure))
uint32_t calcPLLnmult(float fsample)
{
  float C = (carrier.FS + FM_deviation * fsample) * carrier.i2s_clk_divprod;
  unsigned nfact = C;
  uint32_t mult = C * PLL_DENOMINATOR - (nfact * PLL_DENOMINATOR);
  return mult;
}

//Called for every (half-)block of samples:
static inline
void process(const audio_block_t *blockL, const audio_block_t *blockR, unsigned offset)
{
  if (blockL == nullptr) return;

  float bL[NUM_SAMPLES];
  float bR[NUM_SAMPLES];

  //convert 16 bit samples to float & normalize
  if (blockR != nullptr) {
    for (unsigned idx = 0; idx < NUM_SAMPLES; idx++) {
      bL[idx] = blockL->data[idx + offset] / 32768.0f;
      bR[idx] = blockR->data[idx + offset] / 32768.0f;
    }
  } else {
    //blockR not filled: copy left channel.
    for (unsigned idx = 0; idx < NUM_SAMPLES; idx++) {
      bR[idx] = bL[idx] = blockL->data[idx + offset] / 32768.0f;
    }
  }


  //interpolation:

  float iL[I_NUM_SAMPLES];
  float iR[I_NUM_SAMPLES];

  arm_fir_interpolate_f32(&interpolationL, bL, iL, NUM_SAMPLES);
  arm_fir_interpolate_f32(&interpolationR, bR, iR, NUM_SAMPLES);
  //Scaling after interpolation
  //This is done later, so we can save some multiplications
  //arm_scale_f32(iL, (float)INTERPOLATION, iL, I_NUM_SAMPLES);
  //arm_scale_f32(iR, (float)INTERPOLATION, iR, I_NUM_SAMPLES);

  offset *= INTERPOLATION;
  processStereo(iL, iR, offset);

  //Flush cache to memory, so that DMA sees the new data
  arm_dcache_flush_delete(&fm_tx_buffer[offset], sizeof(fm_tx_buffer) / 2 );
}

inline
static void processStereo(const float blockL[I_NUM_SAMPLES], const float blockR[I_NUM_SAMPLES], const unsigned offset)
{
  static float pilot_acc = 0;
  const float  pilot_inc = 19000.0 * TWO_PI / I_SAMPLERATE  ; // increment per sample for 19kHz pilot tone & AUDIO_SAMPLE_RATE_EXACT*INTERPOLATION

  float sample, tmp;
  float sample_L;
  float sample_R;
  float LminusR;
  float LplusR;

  static float lastInputSampleL = 0;
  static float lastInputSampleR = 0;

  for (unsigned idx = 0; idx < I_NUM_SAMPLES; idx++)
  {
    // 1.) Pre-emphasis filter
    //     https://jontio.zapto.org/download/preempiir.pdf
    tmp = blockL[idx];
    sample_L = pre_a0 * tmp + pre_a1 * lastInputSampleL + pre_b1 * tmp;
    lastInputSampleL = tmp;
    tmp = blockR[idx];
    sample_R = pre_a0 * tmp + pre_a1 * lastInputSampleR + pre_b1 * tmp;
    lastInputSampleR = tmp;

    // 2.) Create MPX signal
    pilot_acc = pilot_acc + pilot_inc;

    LminusR  = INTERPOLATION * 0.5f * (sample_L - sample_R);      // generate LEFT minus RIGHT signal
    LplusR   = INTERPOLATION * 0.5f * (sample_L + sample_R);      // generate LEFT plus RIGHT signal
    sample = LplusR + (LminusR * arm_sin_f32(2.0f * pilot_acc));  // generate MPX signal with LEFT minus right DSB signal around 2*19kHz = 38kHz
    sample = sample * 0.85f;                                      // 85% signal
    sample = sample + 0.1f * arm_sin_f32(pilot_acc);              // 10% pilot tone at 19kHz

    // 3.) Add RDS sample:
    sample = sample + 0.5f * rds_sample() * arm_sin_f32(3.0f * pilot_acc); // 5% RDS (rds_sample() maxvalue 0.1)

    // wrap around pilot tone phase accumulator
    if (pilot_acc > (float)TWO_PI) pilot_acc -= (float)TWO_PI;

    fm_tx_buffer[idx + offset] = calcPLLnmult(sample);
  }

}

#endif
