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

#include "output_fm.h"

#if defined(__IMXRT1062__) && FM_STEREO_RDS

//RDS generation according to CuteSDR, Moe Wheatley
//https://versaweb.dl.sourceforge.net/project/cutesdr/doc/CuteSDR102.pdf

#define MAX_RDS_DATA   100  //max number of blocks that can be stored

#define RDS_FREQUENCY  57000.0
#define RDS_BITRATE    (RDS_FREQUENCY / 48.0)
#define RDSBUF_SIZE    ((int)(I_SAMPLERATE / RDS_BITRATE) * 2 + 1)

static struct {

  float Time;
  float D1;
  float D2;
  int BufLength;
  int BufPos;
  int LastBit;
  uint32_t BitPtr;
  float PulseCoef[RDSBUF_SIZE];
  uint32_t DataBuf[MAX_RDS_DATA];

  int data_PI;        // Program indentification
  int data_PTY;       // Program Type
  int flag_TP;        // Traffic Program
  int flag_TA;        // Traffic Annoucenement
  int flag_MS;
  char data_PS[8]; // Program Service name
  char data_RT[64]; // Radio Text
  bool flag_RTChanged;       // set when different text
  bool data_updated;
  bool group_transmitted;
} rds;

FLASHMEM void AudioOutputFM::setPI(uint16_t _PI)
{
  rds.data_PI = _PI;
  rds.data_updated = true;
}

FLASHMEM void AudioOutputFM::setPTY(uint8_t PTY)
{
  rds.data_PTY = PTY & 0x1F;
  rds.data_updated = true;
}

FLASHMEM void AudioOutputFM::setTP(bool TP)
{

  rds.flag_TP = TP ? 1 : 0;
  if (!rds.flag_TP) rds.flag_TA = false;
  rds.data_updated = true;
}

FLASHMEM void AudioOutputFM::setTA(bool TA)
{
  if (rds.flag_TP)
    rds.flag_TA = TA ? 1 : 0;
  else
    rds.flag_TA = false;
  rds.data_updated = true;
}

FLASHMEM void AudioOutputFM::setPS(const char* PS)
{
  int i = 0;
  while (i < 8 && *PS != 0) rds.data_PS[i++] = *PS++;
  while (i < 8) rds.data_PS[i++] = ' ';
  rds.data_updated = true;
}

size_t AudioOutputFM::write(uint8_t c) {
  static int idx = 0;
  static char lastChar = '\0';

  if (lastChar == '\r' && c == '\n')  return 1;
  lastChar = (char)c;

  rds.flag_RTChanged = true;
  rds.data_updated = true;

  if (c == 0x0a) {
    idx = 0;
    memset((void*)&rds.data_RT, 0, sizeof(rds.data_RT));
  }
  if (idx < 63) {
    rds.data_RT[idx] = c;
    idx++;
    if ( (c != '\r') && (idx < 64) && ((idx & 0x1f) != 0x10) ) rds.data_RT[idx] = '\r';
    return 1;
  }
  return 0;
};

size_t AudioOutputFM::write(const uint8_t *buffer, size_t size)
{
  for (unsigned i = 0; i < size; i++) if (!write(buffer[i])) break;
  return size;
};


bool AudioOutputFM::transmitted()
{
  return rds.group_transmitted;
}

//Generatormatrix from RDS spec for creating check sum word
static const uint32_t CHKWORDGEN[16] = {
  0x077,//0001110111
  0x2E7,//1011100111
  0x3AF,//1110101111
  0x30B,//1100001011
  0x359,//1101011001
  0x370,//1101110000
  0x1B8,//0110111000
  0x0DC,//0011011100
  0x06E,//0001101110
  0x037,//0000110111
  0x2C7,//1011000111
  0x3BF,//1110111111
  0x303,//1100000011
  0x35D,//1101011101
  0x372,//1101110010
  0x1B9 //0110111001
};

#define OFFSET_WORD_BLOCK_A   0x0FC
#define OFFSET_WORD_BLOCK_B   0x198
#define OFFSET_WORD_BLOCK_C   0x168
#define OFFSET_WORD_BLOCK_CP  0x350
#define OFFSET_WORD_BLOCK_D   0x1B4
#define GROUPB_BIT            0x0800  //bit position in BlockB for determining group A or B msgs

#define NUMBITS_CRC    10
#define NUMBITS_MSG    16
#define NUMBITS_BLOCK  (NUMBITS_CRC + NUMBITS_MSG)

void CreateRdsGroup(uint16_t Blk1, uint16_t Blk2, uint16_t Blk3, uint16_t Blk4);
void rds_update();


FLASHMEM
void rds_begin() {

  memset((void*)&rds, 0, sizeof(rds));

  //create impulse response of bi-phase bits
  // This is basically the time domain shape of a single bi-phase bit
  // as defined for RDS and is close to a single cycle sine wave in shape

  float RdsPulseLength = (I_SAMPLERATE / RDS_BITRATE);
  int Period = (int)(RdsPulseLength + 0.5f);

  for (int i = 0; i <= Period; i++)
  {
    double t = (double)i / I_SAMPLERATE;
    double x = t * RDS_BITRATE;
    rds.PulseCoef[i + Period] = .75 * cos(2.0 * TWO_PI * x) * ( (1.0 / (1.0 / x - x * 64.0)) - (1.0 / (9.0 / x - x * 64)) );
    rds.PulseCoef[Period - i] = - rds.PulseCoef[i + Period];
  }

  rds.BitPtr = (1 << 25);

  rds.data_PI = 0xfffe;
  rds.data_PTY = 1;
  strncpy(rds.data_PS, "TFM    ", sizeof(rds.data_PS) - 1 );
  strncpy(rds.data_RT, "TeensyFMTransmitter compiled:" __DATE__ " " __TIME__ , sizeof(rds.data_RT) );
  rds.flag_RTChanged = true;
  rds.data_updated = true;
  rds_update();  
}


//Creates the Program Service groups:
inline void rds_groups_ps ()
{
  uint16_t b2, b4;
  unsigned di;
  const unsigned group = 0x01; // 0B

  //Group 0B for PS
  //PS: EN50067 Page 22
  //DI: EN50067 Page 41

  for (unsigned i = 0; i < 4; i++) {
    if (i == 0) di = 0b100;
    else di = i;
    b2 = (group << 11) | ((rds.flag_TP & 0x01) << 10) | ((rds.data_PTY & 0x1f) << 5) | ((rds.flag_TA & 0x01) << 4) | ((rds.flag_MS & 0x01) << 3) | (di);
    char c1 = rds.data_PS[i * 2];
    char c2 = rds.data_PS[i * 2 + 1];
    b4 = ((uint16_t)c1 << 8) | c2;
    CreateRdsGroup(rds.data_PI, b2, rds.data_PI, b4);
    //Serial.printf("%04x %04x %04x %04x\n",rds.data_PI, b2, rds.data_PI, b4);
  }
}

//Creates the Radio Text groups:
inline void rds_groups_rt()
{
  static unsigned RT_AB = 0;
  uint16_t b2, b3, b4;
  const unsigned group = 0x04; // 2B

  //Group 2A for RT
  //RT: EN50067 Page 25


  if (rds.flag_RTChanged) RT_AB = !RT_AB & 0x01;
  int len = strlen(rds.data_RT);
  for (int i = 0; i < (len / 4 + 1); i++) {
    b2 = (group << 11) | ((rds.flag_TP & 0x01) << 10) | ((rds.data_PTY & 0x1f) << 5) | ((RT_AB & 0x01) << 4) | i;
    char c1 = rds.data_RT[i * 4];
    char c2 = rds.data_RT[i * 4 + 1];
    b3 = ((uint16_t)c1 << 8) | c2;
    char c3 = rds.data_RT[i * 4 + 2];
    char c4 = rds.data_RT[i * 4 + 3];
    b4 = ((uint16_t)c3 << 8) | c4;
    CreateRdsGroup(rds.data_PI, b2, b3, b4);
  }
}

void rds_update()
{
  if (!rds.data_updated) return;

  rds.BufLength = 0;

  rds_groups_ps();
  rds_groups_rt();

  rds.group_transmitted = false;
  rds.data_updated = false;
}

/////////////////////////////////////////////////////////////////////////////////
//  Create data blk with chkword from 16 bit 'Data' with 'BlockOffset'.
// Returns 26 bit block with check word.
/////////////////////////////////////////////////////////////////////////////////
uint32_t CreateBlockWithCheckword(uint16_t Data, uint32_t BlockOffset)
{
  uint32_t block = (uint32_t)Data << 10; //put 16 msg data bits into block
  for (int i = 0; i < NUMBITS_MSG; i++)
  { //do matrix operation on data bits 15 to 0
    //Since generator matrix is in a systematic form
    //(first 16 columns are a diagonal identity matrix),
    //just XOR from table where message bit is a one.
    if (Data & 0x8000) //if msg bit 15 is 1, XOR with generator matrix value
      block = block ^ CHKWORDGEN[i];
    Data <<= 1;   //go to next bit position
  }
  block = block ^ BlockOffset;  //add in block offset word
  return block;
}

/////////////////////////////////////////////////////////////////////////////////
//  Create a 4 block group from the parameters and create check words
//Group data is placed in m_RdsDataBuf[] for continuous transmitting
/////////////////////////////////////////////////////////////////////////////////
void CreateRdsGroup(uint16_t Blk1, uint16_t Blk2, uint16_t Blk3, uint16_t Blk4)
{
  rds.DataBuf[rds.BufLength++] = CreateBlockWithCheckword(Blk1, OFFSET_WORD_BLOCK_A);
  rds.DataBuf[rds.BufLength++] = CreateBlockWithCheckword(Blk2, OFFSET_WORD_BLOCK_B);
  if (Blk2 & GROUPB_BIT)
    rds.DataBuf[rds.BufLength++] = CreateBlockWithCheckword(Blk3, OFFSET_WORD_BLOCK_CP);
  else
    rds.DataBuf[rds.BufLength++] = CreateBlockWithCheckword(Blk3, OFFSET_WORD_BLOCK_C);
  rds.DataBuf[rds.BufLength++] = CreateBlockWithCheckword(Blk4, OFFSET_WORD_BLOCK_D);
}

/////////////////////////////////////////////////////////////////////////////////
//  Gets next data bit from m_RdsDataBuf[] and converts to a + or - 1.0 value
// used by CreateRdsSamples() to produce the modulation waveform
/////////////////////////////////////////////////////////////////////////////////
float CreateNextRdsBit()
{
  int bit;
  //get next bit from 26 bit wide buffer msbit first
  if ( rds.BitPtr & rds.DataBuf[rds.BufPos] )
    bit = 1;
  else
    bit = 0;

  rds.BitPtr >>= 1;  //shift bit pointer  to next bit
  if (0 == rds.BitPtr)
  {
    //reached end of 26bit word so go to next word in m_RdsDataBuf[]
    rds.BitPtr = (1 << 25);
    rds.BufPos++;
    if (rds.BufPos >= rds.BufLength) {
      rds.BufPos = 0;  //reached end of m_RdsDataBuf[] so start over
      rds.group_transmitted = true;
    }
  }

  //differential encode output bit by XOR with previous output bit
  //return +1.0 for a '1' and -1.0 for a '0'
  if ( rds.LastBit ^ bit )
  {
    rds.LastBit = 1;
    return 1.0f;
  }
  else
  {
    rds.LastBit = 0;
    return -1.0f;
  }
}

static const float rdsperiod = 1.0 / RDS_BITRATE;
static const float rds2period = 2.0 / RDS_BITRATE;
static const float RdsSamplePeriod = 1.0 / I_SAMPLERATE;

float rds_sample()
{

  int n1;
  int n2;

  n1 = (int)( rds.Time * (float)I_SAMPLERATE);  //create integer index

  //calculate index positions of both pointers
  if (rds.Time > rdsperiod)
    n2 = (int)( (rds.Time - rdsperiod) * (float)I_SAMPLERATE);
  else
    n2 = (int)( (rds.Time + rdsperiod) * (float)I_SAMPLERATE);

  //manage running floating point time position
  rds.Time += RdsSamplePeriod;
  if (rds.Time >= rds2period) rds.Time -= rds2period;

  //if a pointer wraps to zero, get next new data bit value
  if (0 == n1) rds.D1 = CreateNextRdsBit();
  if (0 == n2) rds.D2 = CreateNextRdsBit();

  //get both table values and add together for output sample value
  return rds.D1 * rds.PulseCoef[n1] + rds.D2 * rds.PulseCoef[n2];
}

#endif
