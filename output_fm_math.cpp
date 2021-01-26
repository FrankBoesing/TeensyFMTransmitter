
#include "output_fm.h"

// taken from DD4WHs SDR

#define PIH 1.5707963267948966192313216916398



double cotan(double i) 
{ return(1 / tan(i)); }


FLASHMEM
double Izero (double x)
{
  double x2 = x / 2.0;
  double summe = 1.0;
  double ds = 1.0;
  double di = 1.0;
  double errorlimit = 1e-9;
  double tmp;
  do {
    tmp = x2 / di;
    tmp *= tmp;
    ds *= tmp;
    summe += ds;
    di += 1.0;
  }   while (ds >= errorlimit * summe);
  return (summe);
}


FLASHMEM
double m_sinc(int m, double fc)
{ // fc is f_cut/(Fsamp/2)
  // m is between -M and M step 2
  //
  double x = m * PIH;
  if (m == 0)
    return 1.0;
  else
    return sin(x * fc) / (fc * x);
}

FLASHMEM
void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float fc, float Astop, int type, float dfc, float Fsamprate)
// pointer to coefficients variable, no. of coefficients to calculate, frequency where it happens, stopband attenuation in dB,
// filter type, half-filter bandwidth (only for bandpass and notch)
{ // modified by WMXZ and DD4WH after
  // Wheatley, M. (2011): CuteSDR Technical Manual. www.metronix.com, pages 118 - 120, FIR with Kaiser-Bessel Window
  // assess required number of coefficients by
  //     numCoeffs = (Astop - 8.0) / (2.285 * TPI * normFtrans);
  // selecting high-pass, numCoeffs is forced to an even number for better frequency response

  double Beta;
  double izb;
  double fcf = fc;
  int nc = numCoeffs;
  fc = fc / Fsamprate;
  dfc = dfc / Fsamprate;
  // calculate Kaiser-Bessel window shape factor beta from stop-band attenuation
  if (Astop < 20.96)
    Beta = 0.0;
  else if (Astop >= 50.0)
    Beta = 0.1102 * (Astop - 8.71);
  else
    Beta = 0.5842 * pow((Astop - 20.96), 0.4) + 0.07886 * (Astop - 20.96);

  memset(coeffs_I, 0, numCoeffs * sizeof(float) );

  izb = Izero (Beta);
  if (type == 0) // low pass filter
    //     {  fcf = fc;
  { fcf = fc * 2.0;
    nc =  numCoeffs;
  }
  else if (type == 1) // high-pass filter
  { fcf = -fc;
    nc =  2 * (numCoeffs / 2);
  }
  else if ((type == 2) || (type == 3)) // band-pass filter
  {
    fcf = dfc;
    nc =  2 * (numCoeffs / 2); // maybe not needed
  }
  else if (type == 4) // Hilbert transform
  {
    nc =  2 * (numCoeffs / 2);
    // clear coefficients
    for (int ii = 0; ii < 2 * (nc - 1); ii++) coeffs_I[ii] = 0;
    // set real delay
    coeffs_I[nc] = 1;

    // set imaginary Hilbert coefficients
    for (int ii = 1; ii < (nc + 1); ii += 2)
    {
      if (2 * ii == nc) continue;
      double x = (double)(2 * ii - nc) / (double)nc;
      double w = Izero(Beta * sqrt(1.0f - x * x)) / izb; // Kaiser window
      coeffs_I[2 * ii + 1] = 1.0f / (PIH * (double)(ii - nc / 2)) * w ;
    }
    return;
  }

  for (int ii = - nc, jj = 0; ii < nc; ii += 2, jj++)
  {
    double x = (double)ii / (double)nc;
    double w = Izero(Beta * sqrt(1.0f - x * x)) / izb; // Kaiser window
    coeffs_I[jj] = fcf * m_sinc(ii, fcf) * w;

  }

  if (type == 1)
  {
    coeffs_I[nc / 2] += 1;
  }
  else if (type == 2)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= 2.0f * cos(PIH * (2 * jj - nc) * fc);
  }
  else if (type == 3)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= -2.0f * cos(PIH * (2 * jj - nc) * fc);
    coeffs_I[nc / 2] += 1;
  }

} // END calc_FIR_coeffs
