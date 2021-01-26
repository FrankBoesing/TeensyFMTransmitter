
#include "output_fm.h"

#if 0

//https://versaweb.dl.sourceforge.net/project/cutesdr/doc/CuteSDR102.pdf

#define NUMBITS 1
#define RDS_BITRATE 1
#define NUMBITS_MSG 1
const m_RdsTime 

//Generatormatrix from RDS spec for creating check sum word
static
const uint32_t CHKWORDGEN[16] = {
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

/////////////////////////////////////////////////////////////////////////////////
//      Create data blk with chkword from 16 bit 'Data' with 'BlockOffset'.
// Returns 26 bit block with check word.
////////////////////////////////////////////////////////////////////////////////
uint32_t CreateBlockWithCheckword(uint32_t Data, uint32_t BlockOffset)
{
  uint32_t block = (uint32_t) Data << 10; //put 16 msg databits into block
  for (unsigned i = 0; i < NUMBITS_MSG; i++)
  {
    //do matrix operation on databits 15 to 0
    //Since generator matrix is in systematic form
    //(first 16 columns area diagonal identity matrix),
    //just XOR from table where message bit is a one.
    if (Data & 0x8000) // if msg bit 15 is 1, XOR with generator matrix value
      block = block ^ CHKWORDGEN[i];
    Data <<= 1; //go to next bit position
  }
  block = block ^ BlockOffset; //add in block offset word
  return block;
}

void CreateRdsSamples(int InLength, float *pBuf)
{
  int n1;
  int n2;
  double rdsperiod = 1.0 / RDS_BITRATE;
  double rds2period = 2.0 / RDS_BITRATE;
  for (int i = 0; i < InLength; i++)
  {
    n1 = (int)(m_RdsTime * m_SampleRate); //create integer index
    //calculate index positions of both pointers
    if (m_RdsTime > rdsperiod)
      n2 = (int)((m_RdsTime - rdsperiod) * m_SampleRate);
    else
      n2 = (int)((m_RdsTime + rdsperiod) * m_SampleRate);
    //if a pointer wraps to zero, get next new data bit value
    if (0 == n1) m_RdsD1 = CreateNextRdsBit();
    if (0 == n2) m_RdsD2 = CreateNextRdsBit();
    //get both table values and add together for output sample value
    pBuf[i] = m_RdsD1 * m_RdsPulseCoef[n1] + m_RdsD2 * m_RdsPulseCoef[n2];
    //manage running floating point time position
    m_RdsTime += m_RdsSamplePeriod;
    if (m_RdsTime >= rds2period) m_RdsTime -= rds2period;
  }
}

#endif
