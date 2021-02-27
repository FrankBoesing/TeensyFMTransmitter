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

#define NUM_SAMPLES (AUDIO_BLOCK_SAMPLES / 2) //Input samples
#define I_NUM_SAMPLES (NUM_SAMPLES * INTERPOLATION)
#define I_TIMERVAL ((int)(((double)F_BUS_ACTUAL) / (AUDIO_SAMPLERATE * INTERPOLATION) + 0.5))

DMAMEM static  __attribute__((aligned(32))) int fm_tx_buffer[I_NUM_SAMPLES * 2];
audio_block_t * AudioOutputFM::block_left = NULL;
audio_block_t * AudioOutputFM::block_right = NULL;
unsigned long AudioOutputFM::us = 0;
bool AudioOutputFM::update_responsibility = false;
DMAChannel AudioOutputFM::dma(false);

static struct {
  uint32_t FS; //PLL Frequency
  int i2s_clk_pred;
  int i2s_clk_podf;
  float ma, mb;
} carrier;

#define AUDIO_BANDWIDTH     15000.0
#define FM_DEVIATION        75000.0
#define FM_PILOT            19000.0
#define PLL_FREF            24000000.0
#define PLL_DENOMINATOR     960000 // MCUexpresso says 960000 max (thx DM5SG)

//preemphasis filter coefficients
static float pre_a0;
static float pre_a1;
static float pre_b1;

#if INTERPOLATION > 1
// Attention: interpolation_taps for the interpolation filter must be a multiple of the interpolation factor L (constant INTERPOLATION)
static const unsigned interpolation_taps = 16 * INTERPOLATION;
static const float n_att = 90.0; // desired stopband attenuation
DMAMEM static __attribute__((aligned(32))) float interpolation_coeffs[interpolation_taps];
DMAMEM static __attribute__((aligned(32))) float interpolation_L_state[(NUM_SAMPLES-1) + interpolation_taps / INTERPOLATION];
DMAMEM static __attribute__((aligned(32))) float interpolation_R_state[(NUM_SAMPLES-1) + interpolation_taps / INTERPOLATION];
static arm_fir_interpolate_instance_f32 interpolationL;
static arm_fir_interpolate_instance_f32 interpolationR;
#endif

//extern and forward declarations:
extern "C" void xbar_connect(unsigned int input, unsigned int output);
extern void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, double fc, double Astop, int type, double dfc, double Fsamprate);
extern double cotan(double i);
static void process(const audio_block_t *blockL, const audio_block_t *blockR, unsigned offset);
#if (INTERPOLATION > 1)
inline static void processStereo(const float blockL[I_NUM_SAMPLES], const float blockR[I_NUM_SAMPLES], const unsigned offset);
#if FM_STEREO_RDS
extern void rds_begin();
extern void rds_update();
extern float rds_sample();
#endif
#endif

// https://www.nxp.com/docs/en/application-note/AN5078.pdf
#define PADCONFIG ((0 << 0) | (1 << 3) | (1 << 6)) //Pg 622 Reference Manual
/*
        (1 << 0) | // Slew rate (1 = fast)
        (1 << 3) | // 3 Bit Driver Strength;
        (1 << 6);  // 2 Speed
*/

struct sdivisors {
  uint8_t div_select;
  uint8_t post_div_select;
  uint8_t video_div;
  uint8_t sai_clk_pred;
  uint8_t sai_clk_podf;
};


/*
POST_DIV_SELECT:
00 — Divide by 4.
01 — Divide by 2.
10 — Divide by 1.
11 — Reserved

video div:
00 divide by 1 (Default)
01 divide by 2
10 divide by 1
11 divide by 4
*/

FLASHMEM
sdivisors searchDivSettings(double f)
{
  sdivisors divisors;
  uint32_t best_nom = 0;

  for (unsigned n1 = 4; n1 >= 1 ; n1 /= 2) {

    for (unsigned n2 = 4; n2 >= 1; n2 /= 2) {
      if (n2 > n1) continue;
      if (2 == n2 && 2 == n1) continue;
      
      for (unsigned saidiv = 8; saidiv >= 1; saidiv--) {
        for (unsigned saidiv2 = 1; saidiv2 <= 64; saidiv2++) {         
          
          double fPLL = (n1 * n2 * saidiv * saidiv2) * f;
          if (fPLL <  648000000) continue;
          if (fPLL > 1300000000) break;
          if (fPLL / n1 / n2 / saidiv > 300000000) continue;
  
          unsigned divselect = floor(fPLL / PLL_FREF);
          if (divselect < 27) continue;
          if (divselect > 54) break;
  
          uint32_t nom = (fPLL - ((double)divselect * PLL_FREF)) * PLL_DENOMINATOR / PLL_FREF;
  
          if (nom < 1) continue;
          if (nom >= PLL_DENOMINATOR) continue;
  
          if ( (abs(PLL_DENOMINATOR / 2.0 - nom) < abs(PLL_DENOMINATOR / 2.0 - best_nom)) ) {
            best_nom = nom;
            divisors.div_select = divselect;
            divisors.post_div_select = n1;
            divisors.video_div = n2;
            divisors.sai_clk_pred = saidiv;
            divisors.sai_clk_podf = saidiv2;
#if 0
            Serial.print("**");
#endif
          }
#if 0
          Serial.printf("PLL post div:%d ", n1);
          Serial.printf("PLL post div2:%d ", n2);
          Serial.printf("SAI pred:%d ", saidiv);
          Serial.printf("SAI podf:%d ", saidiv2);
          Serial.printf("fpll:%d ",(unsigned) fPLL);
          Serial.printf("divselect:%d ", divselect);
          Serial.printf("Nominator:%d ", nom);
  
          nom = ((n1 * n2 * saidiv * saidiv2) * (f - 75000) - (divselect * fref)) / mx;
          Serial.printf("-75khZ: %d", nom);
  
          nom = ((n1 * n2 * saidiv * saidiv2) * (f + 75000) - (divselect * fref)) / mx;
          Serial.printf("+75khZ: %d", nom);
          Serial.printf("\n");
#endif
        }
      }
    }
  }
  return divisors;
}

FLASHMEM
void AudioOutputFM::begin(uint8_t mclk_pin, float MHz, int preemphasis)
{
  const uint8_t translate_pll_post_div[] =  { 2, 1, 0, 0 };
  const uint8_t translate_pll_video_div[] = { 0, 1, 0, 3 };
  
  memset(fm_tx_buffer, 0, sizeof(fm_tx_buffer));

  dma.begin(true); // Allocate the DMA channel first
  pin = mclk_pin;

  // pre-emphasis has been based on the more sophisticated Pre-emphasis filter designed by Jonti
  // https://jontio.zapto.org/download/preempiir.pdf
  // https://github.com/jontio/JMPX/blob/master/libJMPX/JDSP.cpp
  // I put the formulas from his paper into the code here to calculate filter coeffs dynamically
  // based on sample rate, interpolation factor and pre emphasis time constant tau
  double delta = 1.0 / (2.0 * PI * AUDIO_BANDWIDTH);
  double tau = 50e-6;
  if (preemphasis == PREEMPHASIS_75)
  {
    tau = 75e-6;
  }

  //double pre_T = 1.0 / I_SAMPLERATE;
  double pre_T = 1.0 / AUDIO_SAMPLERATE;
  double tauP = pre_T / 2.0 * cotan(pre_T / 2.0 / tau);
  double deltaP = pre_T / 2.0 * cotan(pre_T / 2.0 / delta);
  double pre_bP = sqrt(-tauP * tauP + sqrt(tauP * tauP * tauP * tauP + 8.0 * tauP * tauP * deltaP * deltaP)) * 0.5;
  double pre_aP = sqrt(2.0 * pre_bP * pre_bP + tauP * tauP);

  pre_a0 = (2.0 * pre_aP + pre_T) / (2.0 * pre_bP + pre_T);
  pre_a1 = (pre_T - 2.0 * pre_aP) / (2.0 * pre_bP + pre_T);
  pre_b1 = (2.0 * pre_bP - pre_T) / (2.0 * pre_bP + pre_T);

#if INTERPOLATION > 1
  //calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float fc, float Astop, int type, float dfc, float Fsamprate)
  calc_FIR_coeffs(interpolation_coeffs, interpolation_taps, AUDIO_BANDWIDTH, n_att, 0, 0.0, I_SAMPLERATE);

  // we initialize the number of input samples, NOT the number of interpolated samples
  if (arm_fir_interpolate_init_f32(&interpolationL, INTERPOLATION, interpolation_taps, interpolation_coeffs, interpolation_L_state, NUM_SAMPLES) ||
      arm_fir_interpolate_init_f32(&interpolationR, INTERPOLATION, interpolation_taps, interpolation_coeffs, interpolation_R_state, NUM_SAMPLES))
  {
    Serial.println("Init of interpolation failed");
    while (1);
  }

#if FM_STEREO_RDS
  rds_begin();
#endif
#endif

  //while(!Serial);
  carrier.FS = (double)MHz * 1000000.0;
  sdivisors divisors = searchDivSettings(carrier.FS);
  carrier.i2s_clk_pred = divisors.sai_clk_pred;
  carrier.i2s_clk_podf = divisors.sai_clk_podf;
  uint32_t mult = (uint32_t)divisors.post_div_select * divisors.video_div * divisors.sai_clk_pred *  divisors.sai_clk_podf;
  carrier.ma = FM_DEVIATION * (PLL_DENOMINATOR / PLL_FREF) * mult;
  carrier.mb = (PLL_DENOMINATOR / PLL_FREF) * ((double)MHz * 1000000.0 * mult - PLL_FREF * divisors.div_select);

  CCM_ANALOG_PLL_VIDEO = CCM_ANALOG_PLL_VIDEO_BYPASS | CCM_ANALOG_PLL_VIDEO_ENABLE
                         | CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(translate_pll_post_div[divisors.post_div_select - 1]) // 0: 1/4; 1: 1/2; 2: 1/1
                         | CCM_ANALOG_PLL_VIDEO_DIV_SELECT(divisors.div_select);

  CCM_ANALOG_MISC2 = (CCM_ANALOG_MISC2 & ~(CCM_ANALOG_MISC2_VIDEO_DIV(3)))
                         | CCM_ANALOG_MISC2_VIDEO_DIV(translate_pll_video_div[divisors.video_div - 1]);

  CCM_ANALOG_PLL_VIDEO_NUM   = carrier.mb;
  CCM_ANALOG_PLL_VIDEO_DENOM = PLL_DENOMINATOR;

  CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_VIDEO_POWERDOWN;//Switch on PLL
  while (!(CCM_ANALOG_PLL_VIDEO & CCM_ANALOG_PLL_VIDEO_LOCK)) {}; //Wait for pll-lock
  CCM_ANALOG_PLL_VIDEO_CLR = CCM_ANALOG_PLL_VIDEO_BYPASS;//Disable Bypass

  //QTimer
  const int comp1 = I_TIMERVAL - 1;
  TMR4_ENBL &= ~(1 << 3); //Disable
  TMR4_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_FORCE;
  TMR4_CSCTRL3 = 0;
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

  enable( true ); //Enable I2s
  update_responsibility = update_setup();

  //pinMode(13, OUTPUT);
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
    //digitalWrite(13, HIGH);
    t1 = micros();
    // DMA is transmitting the first half of the buffer
    // so we must fill the second half
    process(AudioOutputFM::block_left, AudioOutputFM::block_right,  NUM_SAMPLES);
    t1 = micros() - t1;
    if (AudioOutputFM::update_responsibility) AudioStream::update_all();

  } else {
   //digitalWrite(13, LOW);
    //fill the first half
    unsigned long t2 = micros();
    process(AudioOutputFM::block_left, AudioOutputFM::block_right, 0);
#if FM_STEREO_RDS
    //update RDS Data
    rds_update();
#endif
    t2 = micros() - t2;
    unsigned long us = t1 + t2;
    if (us > AudioOutputFM::us) AudioOutputFM::us = us;
  }

}

//Translates a sample to PLL multiplicator:
static inline
uint32_t calcPLLnmult(float fsample)
{
  return carrier.ma * fsample + carrier.mb;
}

//Called for every (half-)block of samples:
static inline
void process(const audio_block_t *blockL, const audio_block_t *blockR, unsigned offset)
{
  if (blockL == nullptr) return;

  float bL[NUM_SAMPLES];
#if (INTERPOLATION > 1)  
  float bR[NUM_SAMPLES];

  //convert 16 bit samples to float, normalize, take interpolation into account 
  const float scale = 32768 * 2 / INTERPOLATION;
  if (blockR != nullptr) {
    for (unsigned idx = 0; idx < NUM_SAMPLES; idx+=4) {
      bL[idx + 0] = blockL->data[idx + 0 + offset] / scale;
      bR[idx + 0] = blockR->data[idx + 0 + offset] / scale;
      bL[idx + 1] = blockL->data[idx + 1 + offset] / scale;
      bR[idx + 1] = blockR->data[idx + 1 + offset] / scale;
      bL[idx + 2] = blockL->data[idx + 2 + offset] / scale;
      bR[idx + 2] = blockR->data[idx + 2 + offset] / scale;
      bL[idx + 3] = blockL->data[idx + 3 + offset] / scale;
      bR[idx + 3] = blockR->data[idx + 3 + offset] / scale;
    }
  } else {
    //blockR not filled: copy left channel.
    for (unsigned idx = 0; idx < NUM_SAMPLES; idx+=4) {
      bR[idx + 0] = bL[idx + 0] = blockL->data[idx + 0 + offset] / scale;
      bR[idx + 1] = bL[idx + 1] = blockL->data[idx + 1 + offset] / scale;      
      bR[idx + 2] = bL[idx + 2] = blockL->data[idx + 2 + offset] / scale;
      bR[idx + 3] = bL[idx + 3] = blockL->data[idx + 3 + offset] / scale;
    }
  }

  // 1.) Pre-emphasis filter
  //     https://jontio.zapto.org/download/preempiir.pdf  
  
  static float lastInputSampleL, lastInputSampleR;
  float tmp;
  
  for (unsigned idx = 0; idx < NUM_SAMPLES; idx++)
  {
    tmp = bL[idx];    
    bL[idx] = pre_a0 * tmp + pre_a1 * lastInputSampleL + pre_b1 * tmp;
    lastInputSampleL = tmp;

    tmp = bR[idx];
    bR[idx] = pre_a0 * tmp + pre_a1 * lastInputSampleR + pre_b1 * tmp;
    lastInputSampleR = tmp;
  }

  //2.) Interpolation
  float iL[I_NUM_SAMPLES];
  float iR[I_NUM_SAMPLES];

  //Scaling is done above before interpolation
  arm_fir_interpolate_f32(&interpolationL, bL, iL, NUM_SAMPLES);
  arm_fir_interpolate_f32(&interpolationR, bR, iR, NUM_SAMPLES);

  offset *= INTERPOLATION;
  processStereo(iL, iR, offset);
    
#else   
  if (blockR != nullptr) {
    const float scale = 32768 * 2;
    for (unsigned idx = 0; idx < NUM_SAMPLES; idx+=4) {
       bL[idx + 0] = ((int32_t)blockL->data[idx + 0 + offset] + blockR->data[idx + 0 + offset]) / scale;
       bL[idx + 1] = ((int32_t)blockL->data[idx + 1 + offset] + blockR->data[idx + 1 + offset]) / scale;
       bL[idx + 2] = ((int32_t)blockL->data[idx + 2 + offset] + blockR->data[idx + 2 + offset]) / scale;
       bL[idx + 3] = ((int32_t)blockL->data[idx + 3 + offset] + blockR->data[idx + 3 + offset]) / scale;
    }    
  }
  else {
    const float scale = 32768;
    for (unsigned idx = 0; idx < NUM_SAMPLES; idx+=4) {
       bL[idx + 0] = blockL->data[idx + 0 + offset] / scale;
       bL[idx + 1] = blockL->data[idx + 1 + offset] / scale;
       bL[idx + 2] = blockL->data[idx + 2 + offset] / scale;
       bL[idx + 3] = blockL->data[idx + 3 + offset] / scale;
    }
  }

  static float lastInputSample = 0;

  for (unsigned idx = 0; idx < NUM_SAMPLES; idx++)
  {
    float sample, tmp;
    
    // 1.) Pre-emphasis filter
    tmp = bL[idx];
    sample = pre_a0 * tmp + pre_a1 * lastInputSample + pre_b1 * tmp;
    lastInputSample = tmp;    
    
    // 2.) convert to PLL value and save.
    fm_tx_buffer[idx + offset] = calcPLLnmult(sample);  
  } 

#endif

  //Flush cache to memory, so that DMA sees the new data
  arm_dcache_flush_delete(&fm_tx_buffer[offset], sizeof(fm_tx_buffer) / 2 );
}

inline
static void processStereo(const float blockL[I_NUM_SAMPLES], const float blockR[I_NUM_SAMPLES], const unsigned offset)
{
  static double pilot_acc = 0;
  const double  pilot_inc = FM_PILOT * TWO_PI / I_SAMPLERATE  ; // increment per sample for 19kHz pilot tone & AUDIO_SAMPLE_RATE_EXACT*INTERPOLATION

  float sample_L, sample_R, sample;  
  float LminusR, LplusR;
  
  for (unsigned idx = 0; idx < I_NUM_SAMPLES; idx++)
  {
    sample_L = blockL[idx];
    sample_R = blockR[idx];
    
    // 3.) Create MPX signal
    LminusR  =  (sample_L - sample_R);      // generate LEFT minus RIGHT signal
    LplusR   =  (sample_L + sample_R);      // generate LEFT plus RIGHT signal
    pilot_acc = pilot_acc + pilot_inc;
    sample = LplusR + (LminusR * arm_sin_f32(2.0f * (float)pilot_acc));  // generate MPX signal with LEFT minus right DSB signal around 2*19kHz = 38kHz
    sample = sample * 0.85f;                                      // 85% signal
    sample = sample + 0.1f * arm_sin_f32(pilot_acc);              // 10% pilot tone at 19kHz

#if FM_STEREO_RDS //Interpolation is good enough to add RDS
    // 4.) Add RDS sample:
    sample = sample + 0.5f * rds_sample() * arm_sin_f32(3.0f * (float)pilot_acc); // 5% RDS (rds_sample() maxvalue 0.1)
#endif

    // 5.) wrap around pilot tone phase accumulator
    if (pilot_acc >= TWO_PI) pilot_acc -= TWO_PI;
    
    // 6.) convert to PLL value and save.
    fm_tx_buffer[idx + offset] = calcPLLnmult(sample);
  }
}

#endif
