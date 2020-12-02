#ifndef RTD_MATH_ADS1248_H_
#define RTD_MATH_ADS1248_H_
/*************************************************************************************************************************************************/
/*!     Rtd_Math.h
*
*       Header file for Rtd_Match software
*
*       October 2013
*
*/
/**************************************************************************************************************************************************
*       Copyright � 2013 Texas Instruments Incorporated - http://www.ti.com/                                                                      *
***************************************************************************************************************************************************
*  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: *
*                                                                                                                                                 *
*    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.                 *
*                                                                                                                                                 *
*    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the        *
*    documentation and/or other materials provided with the distribution.                                                                         *
*                                                                                                                                                 *
*    Neither the name of Texas Instruments Incorporated nor the names of its contributors may be used to endorse or promote products derived      *
*    from this software without specific prior written permission.                                                                                *
*                                                                                                                                                 *
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     *
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         *
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    *
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                           *
**************************************************************************************************************************************************/

extern float ads1248_gain;
extern float ads1248_rref;
#define MIN_TEMP_MEAS   -200
#define MAX_TEMP_MEAS    851

#define NUM_MEAS_POINTS  (MAX_TEMP_MEAS - MIN_TEMP_MEAS + 1)

#if 0
typedef struct
{
    short  TempValue;
    float  R_RTD;
} Interpolation_Table_t;
#endif


/*************************************************************************************************************/
/*                              PROTOTYPES                                                                   */
/*************************************************************************************************************/

#ifdef __CPLUSPLUS
extern "C" {
#endif

float ads1248_interpolateTemperatureValue (long code);
short ads1248_findInterpolationIndex (float r_value);
float ads1248_calculateRValue (long code);

#ifdef __CPLUSPLUS
}
#endif


#endif /* RTD_MATH_H_ */
