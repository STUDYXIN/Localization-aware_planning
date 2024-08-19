/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 15; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 10 + 9];

acadoWorkspace.state[150] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[151] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[152] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[153] = acadoVariables.u[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 10] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 10 + 10];
acadoWorkspace.d[lRun1 * 10 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 10 + 11];
acadoWorkspace.d[lRun1 * 10 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 10 + 12];
acadoWorkspace.d[lRun1 * 10 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 10 + 13];
acadoWorkspace.d[lRun1 * 10 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 10 + 14];
acadoWorkspace.d[lRun1 * 10 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 10 + 15];
acadoWorkspace.d[lRun1 * 10 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 10 + 16];
acadoWorkspace.d[lRun1 * 10 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 10 + 17];
acadoWorkspace.d[lRun1 * 10 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 10 + 18];
acadoWorkspace.d[lRun1 * 10 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 10 + 19];

acadoWorkspace.evGx[lRun1 * 100] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 100 + 1] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 100 + 2] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 100 + 3] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 100 + 4] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 100 + 5] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 100 + 6] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 100 + 7] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 100 + 8] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 100 + 9] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 100 + 10] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 100 + 11] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 100 + 12] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 100 + 13] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 100 + 14] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 100 + 15] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 100 + 16] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 100 + 17] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 100 + 18] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 100 + 19] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 100 + 20] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 100 + 21] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 100 + 22] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 100 + 23] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 100 + 24] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 100 + 25] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 100 + 26] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 100 + 27] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 100 + 28] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 100 + 29] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 100 + 30] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 100 + 31] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 100 + 32] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 100 + 33] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 100 + 34] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 100 + 35] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 100 + 36] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 100 + 37] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 100 + 38] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 100 + 39] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 100 + 40] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 100 + 41] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 100 + 42] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 100 + 43] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 100 + 44] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 100 + 45] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 100 + 46] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 100 + 47] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 100 + 48] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 100 + 49] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 100 + 50] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 100 + 51] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 100 + 52] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 100 + 53] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 100 + 54] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 100 + 55] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 100 + 56] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 100 + 57] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 100 + 58] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 100 + 59] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 100 + 60] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 100 + 61] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 100 + 62] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 100 + 63] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 100 + 64] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 100 + 65] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 100 + 66] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 100 + 67] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 100 + 68] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 100 + 69] = acadoWorkspace.state[79];
acadoWorkspace.evGx[lRun1 * 100 + 70] = acadoWorkspace.state[80];
acadoWorkspace.evGx[lRun1 * 100 + 71] = acadoWorkspace.state[81];
acadoWorkspace.evGx[lRun1 * 100 + 72] = acadoWorkspace.state[82];
acadoWorkspace.evGx[lRun1 * 100 + 73] = acadoWorkspace.state[83];
acadoWorkspace.evGx[lRun1 * 100 + 74] = acadoWorkspace.state[84];
acadoWorkspace.evGx[lRun1 * 100 + 75] = acadoWorkspace.state[85];
acadoWorkspace.evGx[lRun1 * 100 + 76] = acadoWorkspace.state[86];
acadoWorkspace.evGx[lRun1 * 100 + 77] = acadoWorkspace.state[87];
acadoWorkspace.evGx[lRun1 * 100 + 78] = acadoWorkspace.state[88];
acadoWorkspace.evGx[lRun1 * 100 + 79] = acadoWorkspace.state[89];
acadoWorkspace.evGx[lRun1 * 100 + 80] = acadoWorkspace.state[90];
acadoWorkspace.evGx[lRun1 * 100 + 81] = acadoWorkspace.state[91];
acadoWorkspace.evGx[lRun1 * 100 + 82] = acadoWorkspace.state[92];
acadoWorkspace.evGx[lRun1 * 100 + 83] = acadoWorkspace.state[93];
acadoWorkspace.evGx[lRun1 * 100 + 84] = acadoWorkspace.state[94];
acadoWorkspace.evGx[lRun1 * 100 + 85] = acadoWorkspace.state[95];
acadoWorkspace.evGx[lRun1 * 100 + 86] = acadoWorkspace.state[96];
acadoWorkspace.evGx[lRun1 * 100 + 87] = acadoWorkspace.state[97];
acadoWorkspace.evGx[lRun1 * 100 + 88] = acadoWorkspace.state[98];
acadoWorkspace.evGx[lRun1 * 100 + 89] = acadoWorkspace.state[99];
acadoWorkspace.evGx[lRun1 * 100 + 90] = acadoWorkspace.state[100];
acadoWorkspace.evGx[lRun1 * 100 + 91] = acadoWorkspace.state[101];
acadoWorkspace.evGx[lRun1 * 100 + 92] = acadoWorkspace.state[102];
acadoWorkspace.evGx[lRun1 * 100 + 93] = acadoWorkspace.state[103];
acadoWorkspace.evGx[lRun1 * 100 + 94] = acadoWorkspace.state[104];
acadoWorkspace.evGx[lRun1 * 100 + 95] = acadoWorkspace.state[105];
acadoWorkspace.evGx[lRun1 * 100 + 96] = acadoWorkspace.state[106];
acadoWorkspace.evGx[lRun1 * 100 + 97] = acadoWorkspace.state[107];
acadoWorkspace.evGx[lRun1 * 100 + 98] = acadoWorkspace.state[108];
acadoWorkspace.evGx[lRun1 * 100 + 99] = acadoWorkspace.state[109];

acadoWorkspace.evGu[lRun1 * 40] = acadoWorkspace.state[110];
acadoWorkspace.evGu[lRun1 * 40 + 1] = acadoWorkspace.state[111];
acadoWorkspace.evGu[lRun1 * 40 + 2] = acadoWorkspace.state[112];
acadoWorkspace.evGu[lRun1 * 40 + 3] = acadoWorkspace.state[113];
acadoWorkspace.evGu[lRun1 * 40 + 4] = acadoWorkspace.state[114];
acadoWorkspace.evGu[lRun1 * 40 + 5] = acadoWorkspace.state[115];
acadoWorkspace.evGu[lRun1 * 40 + 6] = acadoWorkspace.state[116];
acadoWorkspace.evGu[lRun1 * 40 + 7] = acadoWorkspace.state[117];
acadoWorkspace.evGu[lRun1 * 40 + 8] = acadoWorkspace.state[118];
acadoWorkspace.evGu[lRun1 * 40 + 9] = acadoWorkspace.state[119];
acadoWorkspace.evGu[lRun1 * 40 + 10] = acadoWorkspace.state[120];
acadoWorkspace.evGu[lRun1 * 40 + 11] = acadoWorkspace.state[121];
acadoWorkspace.evGu[lRun1 * 40 + 12] = acadoWorkspace.state[122];
acadoWorkspace.evGu[lRun1 * 40 + 13] = acadoWorkspace.state[123];
acadoWorkspace.evGu[lRun1 * 40 + 14] = acadoWorkspace.state[124];
acadoWorkspace.evGu[lRun1 * 40 + 15] = acadoWorkspace.state[125];
acadoWorkspace.evGu[lRun1 * 40 + 16] = acadoWorkspace.state[126];
acadoWorkspace.evGu[lRun1 * 40 + 17] = acadoWorkspace.state[127];
acadoWorkspace.evGu[lRun1 * 40 + 18] = acadoWorkspace.state[128];
acadoWorkspace.evGu[lRun1 * 40 + 19] = acadoWorkspace.state[129];
acadoWorkspace.evGu[lRun1 * 40 + 20] = acadoWorkspace.state[130];
acadoWorkspace.evGu[lRun1 * 40 + 21] = acadoWorkspace.state[131];
acadoWorkspace.evGu[lRun1 * 40 + 22] = acadoWorkspace.state[132];
acadoWorkspace.evGu[lRun1 * 40 + 23] = acadoWorkspace.state[133];
acadoWorkspace.evGu[lRun1 * 40 + 24] = acadoWorkspace.state[134];
acadoWorkspace.evGu[lRun1 * 40 + 25] = acadoWorkspace.state[135];
acadoWorkspace.evGu[lRun1 * 40 + 26] = acadoWorkspace.state[136];
acadoWorkspace.evGu[lRun1 * 40 + 27] = acadoWorkspace.state[137];
acadoWorkspace.evGu[lRun1 * 40 + 28] = acadoWorkspace.state[138];
acadoWorkspace.evGu[lRun1 * 40 + 29] = acadoWorkspace.state[139];
acadoWorkspace.evGu[lRun1 * 40 + 30] = acadoWorkspace.state[140];
acadoWorkspace.evGu[lRun1 * 40 + 31] = acadoWorkspace.state[141];
acadoWorkspace.evGu[lRun1 * 40 + 32] = acadoWorkspace.state[142];
acadoWorkspace.evGu[lRun1 * 40 + 33] = acadoWorkspace.state[143];
acadoWorkspace.evGu[lRun1 * 40 + 34] = acadoWorkspace.state[144];
acadoWorkspace.evGu[lRun1 * 40 + 35] = acadoWorkspace.state[145];
acadoWorkspace.evGu[lRun1 * 40 + 36] = acadoWorkspace.state[146];
acadoWorkspace.evGu[lRun1 * 40 + 37] = acadoWorkspace.state[147];
acadoWorkspace.evGu[lRun1 * 40 + 38] = acadoWorkspace.state[148];
acadoWorkspace.evGu[lRun1 * 40 + 39] = acadoWorkspace.state[149];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
out[10] = u[0];
out[11] = u[1];
out[12] = u[2];
out[13] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = xd[9];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 15; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 14] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 14 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 14 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 14 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 14 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 14 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 14 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 14 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 14 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 14 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 14 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 14 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 14 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 14 + 13] = acadoWorkspace.objValueOut[13];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.x[155];
acadoWorkspace.objValueIn[6] = acadoVariables.x[156];
acadoWorkspace.objValueIn[7] = acadoVariables.x[157];
acadoWorkspace.objValueIn[8] = acadoVariables.x[158];
acadoWorkspace.objValueIn[9] = acadoVariables.x[159];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9];

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[24] + Gx1[17]*Gu1[28] + Gx1[18]*Gu1[32] + Gx1[19]*Gu1[36];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[21] + Gx1[16]*Gu1[25] + Gx1[17]*Gu1[29] + Gx1[18]*Gu1[33] + Gx1[19]*Gu1[37];
Gu2[6] = + Gx1[10]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[18] + Gx1[15]*Gu1[22] + Gx1[16]*Gu1[26] + Gx1[17]*Gu1[30] + Gx1[18]*Gu1[34] + Gx1[19]*Gu1[38];
Gu2[7] = + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[15] + Gx1[14]*Gu1[19] + Gx1[15]*Gu1[23] + Gx1[16]*Gu1[27] + Gx1[17]*Gu1[31] + Gx1[18]*Gu1[35] + Gx1[19]*Gu1[39];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[12] + Gx1[24]*Gu1[16] + Gx1[25]*Gu1[20] + Gx1[26]*Gu1[24] + Gx1[27]*Gu1[28] + Gx1[28]*Gu1[32] + Gx1[29]*Gu1[36];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[23]*Gu1[13] + Gx1[24]*Gu1[17] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[25] + Gx1[27]*Gu1[29] + Gx1[28]*Gu1[33] + Gx1[29]*Gu1[37];
Gu2[10] = + Gx1[20]*Gu1[2] + Gx1[21]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[23]*Gu1[14] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[26] + Gx1[27]*Gu1[30] + Gx1[28]*Gu1[34] + Gx1[29]*Gu1[38];
Gu2[11] = + Gx1[20]*Gu1[3] + Gx1[21]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[27] + Gx1[27]*Gu1[31] + Gx1[28]*Gu1[35] + Gx1[29]*Gu1[39];
Gu2[12] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[34]*Gu1[16] + Gx1[35]*Gu1[20] + Gx1[36]*Gu1[24] + Gx1[37]*Gu1[28] + Gx1[38]*Gu1[32] + Gx1[39]*Gu1[36];
Gu2[13] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[34]*Gu1[17] + Gx1[35]*Gu1[21] + Gx1[36]*Gu1[25] + Gx1[37]*Gu1[29] + Gx1[38]*Gu1[33] + Gx1[39]*Gu1[37];
Gu2[14] = + Gx1[30]*Gu1[2] + Gx1[31]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[34]*Gu1[18] + Gx1[35]*Gu1[22] + Gx1[36]*Gu1[26] + Gx1[37]*Gu1[30] + Gx1[38]*Gu1[34] + Gx1[39]*Gu1[38];
Gu2[15] = + Gx1[30]*Gu1[3] + Gx1[31]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[34]*Gu1[19] + Gx1[35]*Gu1[23] + Gx1[36]*Gu1[27] + Gx1[37]*Gu1[31] + Gx1[38]*Gu1[35] + Gx1[39]*Gu1[39];
Gu2[16] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[4] + Gx1[42]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[45]*Gu1[20] + Gx1[46]*Gu1[24] + Gx1[47]*Gu1[28] + Gx1[48]*Gu1[32] + Gx1[49]*Gu1[36];
Gu2[17] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[5] + Gx1[42]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[45]*Gu1[21] + Gx1[46]*Gu1[25] + Gx1[47]*Gu1[29] + Gx1[48]*Gu1[33] + Gx1[49]*Gu1[37];
Gu2[18] = + Gx1[40]*Gu1[2] + Gx1[41]*Gu1[6] + Gx1[42]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[45]*Gu1[22] + Gx1[46]*Gu1[26] + Gx1[47]*Gu1[30] + Gx1[48]*Gu1[34] + Gx1[49]*Gu1[38];
Gu2[19] = + Gx1[40]*Gu1[3] + Gx1[41]*Gu1[7] + Gx1[42]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[45]*Gu1[23] + Gx1[46]*Gu1[27] + Gx1[47]*Gu1[31] + Gx1[48]*Gu1[35] + Gx1[49]*Gu1[39];
Gu2[20] = + Gx1[50]*Gu1[0] + Gx1[51]*Gu1[4] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[12] + Gx1[54]*Gu1[16] + Gx1[55]*Gu1[20] + Gx1[56]*Gu1[24] + Gx1[57]*Gu1[28] + Gx1[58]*Gu1[32] + Gx1[59]*Gu1[36];
Gu2[21] = + Gx1[50]*Gu1[1] + Gx1[51]*Gu1[5] + Gx1[52]*Gu1[9] + Gx1[53]*Gu1[13] + Gx1[54]*Gu1[17] + Gx1[55]*Gu1[21] + Gx1[56]*Gu1[25] + Gx1[57]*Gu1[29] + Gx1[58]*Gu1[33] + Gx1[59]*Gu1[37];
Gu2[22] = + Gx1[50]*Gu1[2] + Gx1[51]*Gu1[6] + Gx1[52]*Gu1[10] + Gx1[53]*Gu1[14] + Gx1[54]*Gu1[18] + Gx1[55]*Gu1[22] + Gx1[56]*Gu1[26] + Gx1[57]*Gu1[30] + Gx1[58]*Gu1[34] + Gx1[59]*Gu1[38];
Gu2[23] = + Gx1[50]*Gu1[3] + Gx1[51]*Gu1[7] + Gx1[52]*Gu1[11] + Gx1[53]*Gu1[15] + Gx1[54]*Gu1[19] + Gx1[55]*Gu1[23] + Gx1[56]*Gu1[27] + Gx1[57]*Gu1[31] + Gx1[58]*Gu1[35] + Gx1[59]*Gu1[39];
Gu2[24] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36];
Gu2[25] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37];
Gu2[26] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38];
Gu2[27] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39];
Gu2[28] = + Gx1[70]*Gu1[0] + Gx1[71]*Gu1[4] + Gx1[72]*Gu1[8] + Gx1[73]*Gu1[12] + Gx1[74]*Gu1[16] + Gx1[75]*Gu1[20] + Gx1[76]*Gu1[24] + Gx1[77]*Gu1[28] + Gx1[78]*Gu1[32] + Gx1[79]*Gu1[36];
Gu2[29] = + Gx1[70]*Gu1[1] + Gx1[71]*Gu1[5] + Gx1[72]*Gu1[9] + Gx1[73]*Gu1[13] + Gx1[74]*Gu1[17] + Gx1[75]*Gu1[21] + Gx1[76]*Gu1[25] + Gx1[77]*Gu1[29] + Gx1[78]*Gu1[33] + Gx1[79]*Gu1[37];
Gu2[30] = + Gx1[70]*Gu1[2] + Gx1[71]*Gu1[6] + Gx1[72]*Gu1[10] + Gx1[73]*Gu1[14] + Gx1[74]*Gu1[18] + Gx1[75]*Gu1[22] + Gx1[76]*Gu1[26] + Gx1[77]*Gu1[30] + Gx1[78]*Gu1[34] + Gx1[79]*Gu1[38];
Gu2[31] = + Gx1[70]*Gu1[3] + Gx1[71]*Gu1[7] + Gx1[72]*Gu1[11] + Gx1[73]*Gu1[15] + Gx1[74]*Gu1[19] + Gx1[75]*Gu1[23] + Gx1[76]*Gu1[27] + Gx1[77]*Gu1[31] + Gx1[78]*Gu1[35] + Gx1[79]*Gu1[39];
Gu2[32] = + Gx1[80]*Gu1[0] + Gx1[81]*Gu1[4] + Gx1[82]*Gu1[8] + Gx1[83]*Gu1[12] + Gx1[84]*Gu1[16] + Gx1[85]*Gu1[20] + Gx1[86]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[89]*Gu1[36];
Gu2[33] = + Gx1[80]*Gu1[1] + Gx1[81]*Gu1[5] + Gx1[82]*Gu1[9] + Gx1[83]*Gu1[13] + Gx1[84]*Gu1[17] + Gx1[85]*Gu1[21] + Gx1[86]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[89]*Gu1[37];
Gu2[34] = + Gx1[80]*Gu1[2] + Gx1[81]*Gu1[6] + Gx1[82]*Gu1[10] + Gx1[83]*Gu1[14] + Gx1[84]*Gu1[18] + Gx1[85]*Gu1[22] + Gx1[86]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[89]*Gu1[38];
Gu2[35] = + Gx1[80]*Gu1[3] + Gx1[81]*Gu1[7] + Gx1[82]*Gu1[11] + Gx1[83]*Gu1[15] + Gx1[84]*Gu1[19] + Gx1[85]*Gu1[23] + Gx1[86]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[89]*Gu1[39];
Gu2[36] = + Gx1[90]*Gu1[0] + Gx1[91]*Gu1[4] + Gx1[92]*Gu1[8] + Gx1[93]*Gu1[12] + Gx1[94]*Gu1[16] + Gx1[95]*Gu1[20] + Gx1[96]*Gu1[24] + Gx1[97]*Gu1[28] + Gx1[98]*Gu1[32] + Gx1[99]*Gu1[36];
Gu2[37] = + Gx1[90]*Gu1[1] + Gx1[91]*Gu1[5] + Gx1[92]*Gu1[9] + Gx1[93]*Gu1[13] + Gx1[94]*Gu1[17] + Gx1[95]*Gu1[21] + Gx1[96]*Gu1[25] + Gx1[97]*Gu1[29] + Gx1[98]*Gu1[33] + Gx1[99]*Gu1[37];
Gu2[38] = + Gx1[90]*Gu1[2] + Gx1[91]*Gu1[6] + Gx1[92]*Gu1[10] + Gx1[93]*Gu1[14] + Gx1[94]*Gu1[18] + Gx1[95]*Gu1[22] + Gx1[96]*Gu1[26] + Gx1[97]*Gu1[30] + Gx1[98]*Gu1[34] + Gx1[99]*Gu1[38];
Gu2[39] = + Gx1[90]*Gu1[3] + Gx1[91]*Gu1[7] + Gx1[92]*Gu1[11] + Gx1[93]*Gu1[15] + Gx1[94]*Gu1[19] + Gx1[95]*Gu1[23] + Gx1[96]*Gu1[27] + Gx1[97]*Gu1[31] + Gx1[98]*Gu1[35] + Gx1[99]*Gu1[39];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36];
acadoWorkspace.H[(iRow * 240) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37];
acadoWorkspace.H[(iRow * 240) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38];
acadoWorkspace.H[(iRow * 240) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 244] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + (real_t)2.9999999999999999e-01;
acadoWorkspace.H[iRow * 244 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37];
acadoWorkspace.H[iRow * 244 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38];
acadoWorkspace.H[iRow * 244 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39];
acadoWorkspace.H[iRow * 244 + 60] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36];
acadoWorkspace.H[iRow * 244 + 61] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + (real_t)2.9999999999999999e-01;
acadoWorkspace.H[iRow * 244 + 62] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38];
acadoWorkspace.H[iRow * 244 + 63] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39];
acadoWorkspace.H[iRow * 244 + 120] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36];
acadoWorkspace.H[iRow * 244 + 121] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37];
acadoWorkspace.H[iRow * 244 + 122] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + (real_t)2.9999999999999999e-01;
acadoWorkspace.H[iRow * 244 + 123] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39];
acadoWorkspace.H[iRow * 244 + 180] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36];
acadoWorkspace.H[iRow * 244 + 181] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37];
acadoWorkspace.H[iRow * 244 + 182] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38];
acadoWorkspace.H[iRow * 244 + 183] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + (real_t)5.0000000000000000e+00;
acadoWorkspace.H[iRow * 244] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 244 + 61] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 244 + 122] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 244 + 183] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[10]*Gu1[4] + Gx1[20]*Gu1[8] + Gx1[30]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[50]*Gu1[20] + Gx1[60]*Gu1[24] + Gx1[70]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[90]*Gu1[36];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[10]*Gu1[5] + Gx1[20]*Gu1[9] + Gx1[30]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[50]*Gu1[21] + Gx1[60]*Gu1[25] + Gx1[70]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[90]*Gu1[37];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[10]*Gu1[6] + Gx1[20]*Gu1[10] + Gx1[30]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[50]*Gu1[22] + Gx1[60]*Gu1[26] + Gx1[70]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[90]*Gu1[38];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[10]*Gu1[7] + Gx1[20]*Gu1[11] + Gx1[30]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[50]*Gu1[23] + Gx1[60]*Gu1[27] + Gx1[70]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[90]*Gu1[39];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[21]*Gu1[8] + Gx1[31]*Gu1[12] + Gx1[41]*Gu1[16] + Gx1[51]*Gu1[20] + Gx1[61]*Gu1[24] + Gx1[71]*Gu1[28] + Gx1[81]*Gu1[32] + Gx1[91]*Gu1[36];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[21]*Gu1[9] + Gx1[31]*Gu1[13] + Gx1[41]*Gu1[17] + Gx1[51]*Gu1[21] + Gx1[61]*Gu1[25] + Gx1[71]*Gu1[29] + Gx1[81]*Gu1[33] + Gx1[91]*Gu1[37];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[21]*Gu1[10] + Gx1[31]*Gu1[14] + Gx1[41]*Gu1[18] + Gx1[51]*Gu1[22] + Gx1[61]*Gu1[26] + Gx1[71]*Gu1[30] + Gx1[81]*Gu1[34] + Gx1[91]*Gu1[38];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[21]*Gu1[11] + Gx1[31]*Gu1[15] + Gx1[41]*Gu1[19] + Gx1[51]*Gu1[23] + Gx1[61]*Gu1[27] + Gx1[71]*Gu1[31] + Gx1[81]*Gu1[35] + Gx1[91]*Gu1[39];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[12]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[32]*Gu1[12] + Gx1[42]*Gu1[16] + Gx1[52]*Gu1[20] + Gx1[62]*Gu1[24] + Gx1[72]*Gu1[28] + Gx1[82]*Gu1[32] + Gx1[92]*Gu1[36];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[12]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[32]*Gu1[13] + Gx1[42]*Gu1[17] + Gx1[52]*Gu1[21] + Gx1[62]*Gu1[25] + Gx1[72]*Gu1[29] + Gx1[82]*Gu1[33] + Gx1[92]*Gu1[37];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[12]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[32]*Gu1[14] + Gx1[42]*Gu1[18] + Gx1[52]*Gu1[22] + Gx1[62]*Gu1[26] + Gx1[72]*Gu1[30] + Gx1[82]*Gu1[34] + Gx1[92]*Gu1[38];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[12]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[32]*Gu1[15] + Gx1[42]*Gu1[19] + Gx1[52]*Gu1[23] + Gx1[62]*Gu1[27] + Gx1[72]*Gu1[31] + Gx1[82]*Gu1[35] + Gx1[92]*Gu1[39];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[23]*Gu1[8] + Gx1[33]*Gu1[12] + Gx1[43]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[63]*Gu1[24] + Gx1[73]*Gu1[28] + Gx1[83]*Gu1[32] + Gx1[93]*Gu1[36];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[23]*Gu1[9] + Gx1[33]*Gu1[13] + Gx1[43]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[63]*Gu1[25] + Gx1[73]*Gu1[29] + Gx1[83]*Gu1[33] + Gx1[93]*Gu1[37];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[23]*Gu1[10] + Gx1[33]*Gu1[14] + Gx1[43]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[63]*Gu1[26] + Gx1[73]*Gu1[30] + Gx1[83]*Gu1[34] + Gx1[93]*Gu1[38];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[23]*Gu1[11] + Gx1[33]*Gu1[15] + Gx1[43]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[63]*Gu1[27] + Gx1[73]*Gu1[31] + Gx1[83]*Gu1[35] + Gx1[93]*Gu1[39];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[24]*Gu1[8] + Gx1[34]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[54]*Gu1[20] + Gx1[64]*Gu1[24] + Gx1[74]*Gu1[28] + Gx1[84]*Gu1[32] + Gx1[94]*Gu1[36];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[24]*Gu1[9] + Gx1[34]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[54]*Gu1[21] + Gx1[64]*Gu1[25] + Gx1[74]*Gu1[29] + Gx1[84]*Gu1[33] + Gx1[94]*Gu1[37];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[24]*Gu1[10] + Gx1[34]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[54]*Gu1[22] + Gx1[64]*Gu1[26] + Gx1[74]*Gu1[30] + Gx1[84]*Gu1[34] + Gx1[94]*Gu1[38];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[24]*Gu1[11] + Gx1[34]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[54]*Gu1[23] + Gx1[64]*Gu1[27] + Gx1[74]*Gu1[31] + Gx1[84]*Gu1[35] + Gx1[94]*Gu1[39];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[25]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[45]*Gu1[16] + Gx1[55]*Gu1[20] + Gx1[65]*Gu1[24] + Gx1[75]*Gu1[28] + Gx1[85]*Gu1[32] + Gx1[95]*Gu1[36];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[25]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[45]*Gu1[17] + Gx1[55]*Gu1[21] + Gx1[65]*Gu1[25] + Gx1[75]*Gu1[29] + Gx1[85]*Gu1[33] + Gx1[95]*Gu1[37];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[25]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[45]*Gu1[18] + Gx1[55]*Gu1[22] + Gx1[65]*Gu1[26] + Gx1[75]*Gu1[30] + Gx1[85]*Gu1[34] + Gx1[95]*Gu1[38];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[25]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[45]*Gu1[19] + Gx1[55]*Gu1[23] + Gx1[65]*Gu1[27] + Gx1[75]*Gu1[31] + Gx1[85]*Gu1[35] + Gx1[95]*Gu1[39];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[16]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[36]*Gu1[12] + Gx1[46]*Gu1[16] + Gx1[56]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[76]*Gu1[28] + Gx1[86]*Gu1[32] + Gx1[96]*Gu1[36];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[16]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[36]*Gu1[13] + Gx1[46]*Gu1[17] + Gx1[56]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[76]*Gu1[29] + Gx1[86]*Gu1[33] + Gx1[96]*Gu1[37];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[16]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[36]*Gu1[14] + Gx1[46]*Gu1[18] + Gx1[56]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[76]*Gu1[30] + Gx1[86]*Gu1[34] + Gx1[96]*Gu1[38];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[16]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[36]*Gu1[15] + Gx1[46]*Gu1[19] + Gx1[56]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[76]*Gu1[31] + Gx1[86]*Gu1[35] + Gx1[96]*Gu1[39];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[27]*Gu1[8] + Gx1[37]*Gu1[12] + Gx1[47]*Gu1[16] + Gx1[57]*Gu1[20] + Gx1[67]*Gu1[24] + Gx1[77]*Gu1[28] + Gx1[87]*Gu1[32] + Gx1[97]*Gu1[36];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[27]*Gu1[9] + Gx1[37]*Gu1[13] + Gx1[47]*Gu1[17] + Gx1[57]*Gu1[21] + Gx1[67]*Gu1[25] + Gx1[77]*Gu1[29] + Gx1[87]*Gu1[33] + Gx1[97]*Gu1[37];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[27]*Gu1[10] + Gx1[37]*Gu1[14] + Gx1[47]*Gu1[18] + Gx1[57]*Gu1[22] + Gx1[67]*Gu1[26] + Gx1[77]*Gu1[30] + Gx1[87]*Gu1[34] + Gx1[97]*Gu1[38];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[27]*Gu1[11] + Gx1[37]*Gu1[15] + Gx1[47]*Gu1[19] + Gx1[57]*Gu1[23] + Gx1[67]*Gu1[27] + Gx1[77]*Gu1[31] + Gx1[87]*Gu1[35] + Gx1[97]*Gu1[39];
Gu2[32] = + Gx1[8]*Gu1[0] + Gx1[18]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[38]*Gu1[12] + Gx1[48]*Gu1[16] + Gx1[58]*Gu1[20] + Gx1[68]*Gu1[24] + Gx1[78]*Gu1[28] + Gx1[88]*Gu1[32] + Gx1[98]*Gu1[36];
Gu2[33] = + Gx1[8]*Gu1[1] + Gx1[18]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[38]*Gu1[13] + Gx1[48]*Gu1[17] + Gx1[58]*Gu1[21] + Gx1[68]*Gu1[25] + Gx1[78]*Gu1[29] + Gx1[88]*Gu1[33] + Gx1[98]*Gu1[37];
Gu2[34] = + Gx1[8]*Gu1[2] + Gx1[18]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[38]*Gu1[14] + Gx1[48]*Gu1[18] + Gx1[58]*Gu1[22] + Gx1[68]*Gu1[26] + Gx1[78]*Gu1[30] + Gx1[88]*Gu1[34] + Gx1[98]*Gu1[38];
Gu2[35] = + Gx1[8]*Gu1[3] + Gx1[18]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[38]*Gu1[15] + Gx1[48]*Gu1[19] + Gx1[58]*Gu1[23] + Gx1[68]*Gu1[27] + Gx1[78]*Gu1[31] + Gx1[88]*Gu1[35] + Gx1[98]*Gu1[39];
Gu2[36] = + Gx1[9]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[49]*Gu1[16] + Gx1[59]*Gu1[20] + Gx1[69]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[89]*Gu1[32] + Gx1[99]*Gu1[36];
Gu2[37] = + Gx1[9]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[49]*Gu1[17] + Gx1[59]*Gu1[21] + Gx1[69]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[89]*Gu1[33] + Gx1[99]*Gu1[37];
Gu2[38] = + Gx1[9]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[49]*Gu1[18] + Gx1[59]*Gu1[22] + Gx1[69]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[89]*Gu1[34] + Gx1[99]*Gu1[38];
Gu2[39] = + Gx1[9]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[49]*Gu1[19] + Gx1[59]*Gu1[23] + Gx1[69]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[89]*Gu1[35] + Gx1[99]*Gu1[39];
}

void acado_multQEW2( real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + (real_t)2.0000000000000000e+02*Gu1[0] + Gu2[0];
Gu3[1] = + (real_t)2.0000000000000000e+02*Gu1[1] + Gu2[1];
Gu3[2] = + (real_t)2.0000000000000000e+02*Gu1[2] + Gu2[2];
Gu3[3] = + (real_t)2.0000000000000000e+02*Gu1[3] + Gu2[3];
Gu3[4] = + (real_t)2.0000000000000000e+02*Gu1[4] + Gu2[4];
Gu3[5] = + (real_t)2.0000000000000000e+02*Gu1[5] + Gu2[5];
Gu3[6] = + (real_t)2.0000000000000000e+02*Gu1[6] + Gu2[6];
Gu3[7] = + (real_t)2.0000000000000000e+02*Gu1[7] + Gu2[7];
Gu3[8] = + (real_t)5.0000000000000000e+02*Gu1[8] + Gu2[8];
Gu3[9] = + (real_t)5.0000000000000000e+02*Gu1[9] + Gu2[9];
Gu3[10] = + (real_t)5.0000000000000000e+02*Gu1[10] + Gu2[10];
Gu3[11] = + (real_t)5.0000000000000000e+02*Gu1[11] + Gu2[11];
Gu3[12] = +Gu1[12] + Gu2[12];
Gu3[13] = +Gu1[13] + Gu2[13];
Gu3[14] = +Gu1[14] + Gu2[14];
Gu3[15] = +Gu1[15] + Gu2[15];
Gu3[16] = +Gu1[16] + Gu2[16];
Gu3[17] = +Gu1[17] + Gu2[17];
Gu3[18] = +Gu1[18] + Gu2[18];
Gu3[19] = +Gu1[19] + Gu2[19];
Gu3[20] = +Gu1[20] + Gu2[20];
Gu3[21] = +Gu1[21] + Gu2[21];
Gu3[22] = +Gu1[22] + Gu2[22];
Gu3[23] = +Gu1[23] + Gu2[23];
Gu3[24] = +Gu1[24] + Gu2[24];
Gu3[25] = +Gu1[25] + Gu2[25];
Gu3[26] = +Gu1[26] + Gu2[26];
Gu3[27] = +Gu1[27] + Gu2[27];
Gu3[28] = + (real_t)1.0000000000000000e+02*Gu1[28] + Gu2[28];
Gu3[29] = + (real_t)1.0000000000000000e+02*Gu1[29] + Gu2[29];
Gu3[30] = + (real_t)1.0000000000000000e+02*Gu1[30] + Gu2[30];
Gu3[31] = + (real_t)1.0000000000000000e+02*Gu1[31] + Gu2[31];
Gu3[32] = + (real_t)1.0000000000000000e+02*Gu1[32] + Gu2[32];
Gu3[33] = + (real_t)1.0000000000000000e+02*Gu1[33] + Gu2[33];
Gu3[34] = + (real_t)1.0000000000000000e+02*Gu1[34] + Gu2[34];
Gu3[35] = + (real_t)1.0000000000000000e+02*Gu1[35] + Gu2[35];
Gu3[36] = + (real_t)1.0000000000000000e+02*Gu1[36] + Gu2[36];
Gu3[37] = + (real_t)1.0000000000000000e+02*Gu1[37] + Gu2[37];
Gu3[38] = + (real_t)1.0000000000000000e+02*Gu1[38] + Gu2[38];
Gu3[39] = + (real_t)1.0000000000000000e+02*Gu1[39] + Gu2[39];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[10]*w11[1] + Gx1[20]*w11[2] + Gx1[30]*w11[3] + Gx1[40]*w11[4] + Gx1[50]*w11[5] + Gx1[60]*w11[6] + Gx1[70]*w11[7] + Gx1[80]*w11[8] + Gx1[90]*w11[9] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[11]*w11[1] + Gx1[21]*w11[2] + Gx1[31]*w11[3] + Gx1[41]*w11[4] + Gx1[51]*w11[5] + Gx1[61]*w11[6] + Gx1[71]*w11[7] + Gx1[81]*w11[8] + Gx1[91]*w11[9] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[12]*w11[1] + Gx1[22]*w11[2] + Gx1[32]*w11[3] + Gx1[42]*w11[4] + Gx1[52]*w11[5] + Gx1[62]*w11[6] + Gx1[72]*w11[7] + Gx1[82]*w11[8] + Gx1[92]*w11[9] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[13]*w11[1] + Gx1[23]*w11[2] + Gx1[33]*w11[3] + Gx1[43]*w11[4] + Gx1[53]*w11[5] + Gx1[63]*w11[6] + Gx1[73]*w11[7] + Gx1[83]*w11[8] + Gx1[93]*w11[9] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[14]*w11[1] + Gx1[24]*w11[2] + Gx1[34]*w11[3] + Gx1[44]*w11[4] + Gx1[54]*w11[5] + Gx1[64]*w11[6] + Gx1[74]*w11[7] + Gx1[84]*w11[8] + Gx1[94]*w11[9] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[15]*w11[1] + Gx1[25]*w11[2] + Gx1[35]*w11[3] + Gx1[45]*w11[4] + Gx1[55]*w11[5] + Gx1[65]*w11[6] + Gx1[75]*w11[7] + Gx1[85]*w11[8] + Gx1[95]*w11[9] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[16]*w11[1] + Gx1[26]*w11[2] + Gx1[36]*w11[3] + Gx1[46]*w11[4] + Gx1[56]*w11[5] + Gx1[66]*w11[6] + Gx1[76]*w11[7] + Gx1[86]*w11[8] + Gx1[96]*w11[9] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[17]*w11[1] + Gx1[27]*w11[2] + Gx1[37]*w11[3] + Gx1[47]*w11[4] + Gx1[57]*w11[5] + Gx1[67]*w11[6] + Gx1[77]*w11[7] + Gx1[87]*w11[8] + Gx1[97]*w11[9] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[18]*w11[1] + Gx1[28]*w11[2] + Gx1[38]*w11[3] + Gx1[48]*w11[4] + Gx1[58]*w11[5] + Gx1[68]*w11[6] + Gx1[78]*w11[7] + Gx1[88]*w11[8] + Gx1[98]*w11[9] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[19]*w11[1] + Gx1[29]*w11[2] + Gx1[39]*w11[3] + Gx1[49]*w11[4] + Gx1[59]*w11[5] + Gx1[69]*w11[6] + Gx1[79]*w11[7] + Gx1[89]*w11[8] + Gx1[99]*w11[9] + w12[9];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9];
}

void acado_macQSbarW2( real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + (real_t)2.0000000000000000e+02*w11[0] + w12[0];
w13[1] = + (real_t)2.0000000000000000e+02*w11[1] + w12[1];
w13[2] = + (real_t)5.0000000000000000e+02*w11[2] + w12[2];
w13[3] = +w11[3] + w12[3];
w13[4] = +w11[4] + w12[4];
w13[5] = +w11[5] + w12[5];
w13[6] = +w11[6] + w12[6];
w13[7] = + (real_t)1.0000000000000000e+02*w11[7] + w12[7];
w13[8] = + (real_t)1.0000000000000000e+02*w11[8] + w12[8];
w13[9] = + (real_t)1.0000000000000000e+02*w11[9] + w12[9];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9];
w12[1] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4] + Gx1[15]*w11[5] + Gx1[16]*w11[6] + Gx1[17]*w11[7] + Gx1[18]*w11[8] + Gx1[19]*w11[9];
w12[2] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4] + Gx1[25]*w11[5] + Gx1[26]*w11[6] + Gx1[27]*w11[7] + Gx1[28]*w11[8] + Gx1[29]*w11[9];
w12[3] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5] + Gx1[36]*w11[6] + Gx1[37]*w11[7] + Gx1[38]*w11[8] + Gx1[39]*w11[9];
w12[4] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7] + Gx1[48]*w11[8] + Gx1[49]*w11[9];
w12[5] += + Gx1[50]*w11[0] + Gx1[51]*w11[1] + Gx1[52]*w11[2] + Gx1[53]*w11[3] + Gx1[54]*w11[4] + Gx1[55]*w11[5] + Gx1[56]*w11[6] + Gx1[57]*w11[7] + Gx1[58]*w11[8] + Gx1[59]*w11[9];
w12[6] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9];
w12[7] += + Gx1[70]*w11[0] + Gx1[71]*w11[1] + Gx1[72]*w11[2] + Gx1[73]*w11[3] + Gx1[74]*w11[4] + Gx1[75]*w11[5] + Gx1[76]*w11[6] + Gx1[77]*w11[7] + Gx1[78]*w11[8] + Gx1[79]*w11[9];
w12[8] += + Gx1[80]*w11[0] + Gx1[81]*w11[1] + Gx1[82]*w11[2] + Gx1[83]*w11[3] + Gx1[84]*w11[4] + Gx1[85]*w11[5] + Gx1[86]*w11[6] + Gx1[87]*w11[7] + Gx1[88]*w11[8] + Gx1[89]*w11[9];
w12[9] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9];
w12[1] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4] + Gx1[15]*w11[5] + Gx1[16]*w11[6] + Gx1[17]*w11[7] + Gx1[18]*w11[8] + Gx1[19]*w11[9];
w12[2] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4] + Gx1[25]*w11[5] + Gx1[26]*w11[6] + Gx1[27]*w11[7] + Gx1[28]*w11[8] + Gx1[29]*w11[9];
w12[3] += + Gx1[30]*w11[0] + Gx1[31]*w11[1] + Gx1[32]*w11[2] + Gx1[33]*w11[3] + Gx1[34]*w11[4] + Gx1[35]*w11[5] + Gx1[36]*w11[6] + Gx1[37]*w11[7] + Gx1[38]*w11[8] + Gx1[39]*w11[9];
w12[4] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7] + Gx1[48]*w11[8] + Gx1[49]*w11[9];
w12[5] += + Gx1[50]*w11[0] + Gx1[51]*w11[1] + Gx1[52]*w11[2] + Gx1[53]*w11[3] + Gx1[54]*w11[4] + Gx1[55]*w11[5] + Gx1[56]*w11[6] + Gx1[57]*w11[7] + Gx1[58]*w11[8] + Gx1[59]*w11[9];
w12[6] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9];
w12[7] += + Gx1[70]*w11[0] + Gx1[71]*w11[1] + Gx1[72]*w11[2] + Gx1[73]*w11[3] + Gx1[74]*w11[4] + Gx1[75]*w11[5] + Gx1[76]*w11[6] + Gx1[77]*w11[7] + Gx1[78]*w11[8] + Gx1[79]*w11[9];
w12[8] += + Gx1[80]*w11[0] + Gx1[81]*w11[1] + Gx1[82]*w11[2] + Gx1[83]*w11[3] + Gx1[84]*w11[4] + Gx1[85]*w11[5] + Gx1[86]*w11[6] + Gx1[87]*w11[7] + Gx1[88]*w11[8] + Gx1[89]*w11[9];
w12[9] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
w12[8] += + Gu1[32]*U1[0] + Gu1[33]*U1[1] + Gu1[34]*U1[2] + Gu1[35]*U1[3];
w12[9] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 240 + 60) + (iRow * 4)];
acadoWorkspace.H[(iRow * 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 4)];
acadoWorkspace.H[(iRow * 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 240 + 180) + (iRow * 4)];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4)] = acadoWorkspace.H[(iCol * 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 240 + 60) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 240 + 60) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 240 + 180) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4)] = acadoWorkspace.H[(iCol * 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 240 + 60) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 240 + 180) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4)] = acadoWorkspace.H[(iCol * 240) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 240 + 60) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 240 + 180) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 240 + 180) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)2.9999999999999999e-01*Dy1[10];
RDy1[1] = + (real_t)2.9999999999999999e-01*Dy1[11];
RDy1[2] = + (real_t)2.9999999999999999e-01*Dy1[12];
RDy1[3] = + (real_t)5.0000000000000000e+00*Dy1[13];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)2.0000000000000000e+02*Dy1[0];
QDy1[1] = + (real_t)2.0000000000000000e+02*Dy1[1];
QDy1[2] = + (real_t)5.0000000000000000e+02*Dy1[2];
QDy1[3] = +Dy1[3];
QDy1[4] = +Dy1[4];
QDy1[5] = +Dy1[5];
QDy1[6] = +Dy1[6];
QDy1[7] = + (real_t)1.0000000000000000e+02*Dy1[7];
QDy1[8] = + (real_t)1.0000000000000000e+02*Dy1[8];
QDy1[9] = + (real_t)1.0000000000000000e+02*Dy1[9];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)2.0000000000000000e+02*Gx1[0];
Gx2[1] = + (real_t)2.0000000000000000e+02*Gx1[1];
Gx2[2] = + (real_t)2.0000000000000000e+02*Gx1[2];
Gx2[3] = + (real_t)2.0000000000000000e+02*Gx1[3];
Gx2[4] = + (real_t)2.0000000000000000e+02*Gx1[4];
Gx2[5] = + (real_t)2.0000000000000000e+02*Gx1[5];
Gx2[6] = + (real_t)2.0000000000000000e+02*Gx1[6];
Gx2[7] = + (real_t)2.0000000000000000e+02*Gx1[7];
Gx2[8] = + (real_t)2.0000000000000000e+02*Gx1[8];
Gx2[9] = + (real_t)2.0000000000000000e+02*Gx1[9];
Gx2[10] = + (real_t)2.0000000000000000e+02*Gx1[10];
Gx2[11] = + (real_t)2.0000000000000000e+02*Gx1[11];
Gx2[12] = + (real_t)2.0000000000000000e+02*Gx1[12];
Gx2[13] = + (real_t)2.0000000000000000e+02*Gx1[13];
Gx2[14] = + (real_t)2.0000000000000000e+02*Gx1[14];
Gx2[15] = + (real_t)2.0000000000000000e+02*Gx1[15];
Gx2[16] = + (real_t)2.0000000000000000e+02*Gx1[16];
Gx2[17] = + (real_t)2.0000000000000000e+02*Gx1[17];
Gx2[18] = + (real_t)2.0000000000000000e+02*Gx1[18];
Gx2[19] = + (real_t)2.0000000000000000e+02*Gx1[19];
Gx2[20] = + (real_t)5.0000000000000000e+02*Gx1[20];
Gx2[21] = + (real_t)5.0000000000000000e+02*Gx1[21];
Gx2[22] = + (real_t)5.0000000000000000e+02*Gx1[22];
Gx2[23] = + (real_t)5.0000000000000000e+02*Gx1[23];
Gx2[24] = + (real_t)5.0000000000000000e+02*Gx1[24];
Gx2[25] = + (real_t)5.0000000000000000e+02*Gx1[25];
Gx2[26] = + (real_t)5.0000000000000000e+02*Gx1[26];
Gx2[27] = + (real_t)5.0000000000000000e+02*Gx1[27];
Gx2[28] = + (real_t)5.0000000000000000e+02*Gx1[28];
Gx2[29] = + (real_t)5.0000000000000000e+02*Gx1[29];
Gx2[30] = +Gx1[30];
Gx2[31] = +Gx1[31];
Gx2[32] = +Gx1[32];
Gx2[33] = +Gx1[33];
Gx2[34] = +Gx1[34];
Gx2[35] = +Gx1[35];
Gx2[36] = +Gx1[36];
Gx2[37] = +Gx1[37];
Gx2[38] = +Gx1[38];
Gx2[39] = +Gx1[39];
Gx2[40] = +Gx1[40];
Gx2[41] = +Gx1[41];
Gx2[42] = +Gx1[42];
Gx2[43] = +Gx1[43];
Gx2[44] = +Gx1[44];
Gx2[45] = +Gx1[45];
Gx2[46] = +Gx1[46];
Gx2[47] = +Gx1[47];
Gx2[48] = +Gx1[48];
Gx2[49] = +Gx1[49];
Gx2[50] = +Gx1[50];
Gx2[51] = +Gx1[51];
Gx2[52] = +Gx1[52];
Gx2[53] = +Gx1[53];
Gx2[54] = +Gx1[54];
Gx2[55] = +Gx1[55];
Gx2[56] = +Gx1[56];
Gx2[57] = +Gx1[57];
Gx2[58] = +Gx1[58];
Gx2[59] = +Gx1[59];
Gx2[60] = +Gx1[60];
Gx2[61] = +Gx1[61];
Gx2[62] = +Gx1[62];
Gx2[63] = +Gx1[63];
Gx2[64] = +Gx1[64];
Gx2[65] = +Gx1[65];
Gx2[66] = +Gx1[66];
Gx2[67] = +Gx1[67];
Gx2[68] = +Gx1[68];
Gx2[69] = +Gx1[69];
Gx2[70] = + (real_t)1.0000000000000000e+02*Gx1[70];
Gx2[71] = + (real_t)1.0000000000000000e+02*Gx1[71];
Gx2[72] = + (real_t)1.0000000000000000e+02*Gx1[72];
Gx2[73] = + (real_t)1.0000000000000000e+02*Gx1[73];
Gx2[74] = + (real_t)1.0000000000000000e+02*Gx1[74];
Gx2[75] = + (real_t)1.0000000000000000e+02*Gx1[75];
Gx2[76] = + (real_t)1.0000000000000000e+02*Gx1[76];
Gx2[77] = + (real_t)1.0000000000000000e+02*Gx1[77];
Gx2[78] = + (real_t)1.0000000000000000e+02*Gx1[78];
Gx2[79] = + (real_t)1.0000000000000000e+02*Gx1[79];
Gx2[80] = + (real_t)1.0000000000000000e+02*Gx1[80];
Gx2[81] = + (real_t)1.0000000000000000e+02*Gx1[81];
Gx2[82] = + (real_t)1.0000000000000000e+02*Gx1[82];
Gx2[83] = + (real_t)1.0000000000000000e+02*Gx1[83];
Gx2[84] = + (real_t)1.0000000000000000e+02*Gx1[84];
Gx2[85] = + (real_t)1.0000000000000000e+02*Gx1[85];
Gx2[86] = + (real_t)1.0000000000000000e+02*Gx1[86];
Gx2[87] = + (real_t)1.0000000000000000e+02*Gx1[87];
Gx2[88] = + (real_t)1.0000000000000000e+02*Gx1[88];
Gx2[89] = + (real_t)1.0000000000000000e+02*Gx1[89];
Gx2[90] = + (real_t)1.0000000000000000e+02*Gx1[90];
Gx2[91] = + (real_t)1.0000000000000000e+02*Gx1[91];
Gx2[92] = + (real_t)1.0000000000000000e+02*Gx1[92];
Gx2[93] = + (real_t)1.0000000000000000e+02*Gx1[93];
Gx2[94] = + (real_t)1.0000000000000000e+02*Gx1[94];
Gx2[95] = + (real_t)1.0000000000000000e+02*Gx1[95];
Gx2[96] = + (real_t)1.0000000000000000e+02*Gx1[96];
Gx2[97] = + (real_t)1.0000000000000000e+02*Gx1[97];
Gx2[98] = + (real_t)1.0000000000000000e+02*Gx1[98];
Gx2[99] = + (real_t)1.0000000000000000e+02*Gx1[99];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)2.0000000000000000e+02*Gu1[0];
Gu2[1] = + (real_t)2.0000000000000000e+02*Gu1[1];
Gu2[2] = + (real_t)2.0000000000000000e+02*Gu1[2];
Gu2[3] = + (real_t)2.0000000000000000e+02*Gu1[3];
Gu2[4] = + (real_t)2.0000000000000000e+02*Gu1[4];
Gu2[5] = + (real_t)2.0000000000000000e+02*Gu1[5];
Gu2[6] = + (real_t)2.0000000000000000e+02*Gu1[6];
Gu2[7] = + (real_t)2.0000000000000000e+02*Gu1[7];
Gu2[8] = + (real_t)5.0000000000000000e+02*Gu1[8];
Gu2[9] = + (real_t)5.0000000000000000e+02*Gu1[9];
Gu2[10] = + (real_t)5.0000000000000000e+02*Gu1[10];
Gu2[11] = + (real_t)5.0000000000000000e+02*Gu1[11];
Gu2[12] = +Gu1[12];
Gu2[13] = +Gu1[13];
Gu2[14] = +Gu1[14];
Gu2[15] = +Gu1[15];
Gu2[16] = +Gu1[16];
Gu2[17] = +Gu1[17];
Gu2[18] = +Gu1[18];
Gu2[19] = +Gu1[19];
Gu2[20] = +Gu1[20];
Gu2[21] = +Gu1[21];
Gu2[22] = +Gu1[22];
Gu2[23] = +Gu1[23];
Gu2[24] = +Gu1[24];
Gu2[25] = +Gu1[25];
Gu2[26] = +Gu1[26];
Gu2[27] = +Gu1[27];
Gu2[28] = + (real_t)1.0000000000000000e+02*Gu1[28];
Gu2[29] = + (real_t)1.0000000000000000e+02*Gu1[29];
Gu2[30] = + (real_t)1.0000000000000000e+02*Gu1[30];
Gu2[31] = + (real_t)1.0000000000000000e+02*Gu1[31];
Gu2[32] = + (real_t)1.0000000000000000e+02*Gu1[32];
Gu2[33] = + (real_t)1.0000000000000000e+02*Gu1[33];
Gu2[34] = + (real_t)1.0000000000000000e+02*Gu1[34];
Gu2[35] = + (real_t)1.0000000000000000e+02*Gu1[35];
Gu2[36] = + (real_t)1.0000000000000000e+02*Gu1[36];
Gu2[37] = + (real_t)1.0000000000000000e+02*Gu1[37];
Gu2[38] = + (real_t)1.0000000000000000e+02*Gu1[38];
Gu2[39] = + (real_t)1.0000000000000000e+02*Gu1[39];
}

void acado_condensePrep(  )
{
int lRun1;
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 440 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 440 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 520 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.E[ 560 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 560 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 520 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 440 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.W1, 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 120 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 80 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 40 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.E[ 600 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.E[ 640 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.E[ 680 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.E[ 680 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 760 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.E[ 760 ]), &(acadoWorkspace.E[ 800 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.E[ 840 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.E[ 880 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 880 ]), &(acadoWorkspace.E[ 920 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 920 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1000 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 1000 ]), &(acadoWorkspace.E[ 1040 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 1040 ]), &(acadoWorkspace.E[ 1080 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1120 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 1120 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1080 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1040 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1000 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 960 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 920 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 880 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 840 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 800 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 760 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.W1, 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 720 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 680 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 640 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 600 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 1160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.E[ 1160 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.E[ 1240 ]), &(acadoWorkspace.E[ 1280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.E[ 1280 ]), &(acadoWorkspace.E[ 1320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.E[ 1360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 1360 ]), &(acadoWorkspace.E[ 1400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 1400 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 1480 ]), &(acadoWorkspace.E[ 1520 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 1520 ]), &(acadoWorkspace.E[ 1560 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.E[ 1600 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 1600 ]), &(acadoWorkspace.E[ 1640 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 1640 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1600 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1560 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1520 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1440 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.W1, 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.E[ 1680 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.E[ 1720 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.E[ 1720 ]), &(acadoWorkspace.E[ 1760 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.E[ 1760 ]), &(acadoWorkspace.E[ 1800 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.E[ 1840 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 1840 ]), &(acadoWorkspace.E[ 1880 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 1880 ]), &(acadoWorkspace.E[ 1920 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.E[ 1960 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 1960 ]), &(acadoWorkspace.E[ 2000 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 2000 ]), &(acadoWorkspace.E[ 2040 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.E[ 2080 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 2080 ]), &(acadoWorkspace.E[ 2120 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 2120 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2080 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2040 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2000 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1960 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1920 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1880 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1840 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1800 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1760 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.W1, 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1720 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 1680 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.E[ 2160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.E[ 2200 ]), &(acadoWorkspace.E[ 2240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 2240 ]), &(acadoWorkspace.E[ 2280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.E[ 2320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 2320 ]), &(acadoWorkspace.E[ 2360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 2360 ]), &(acadoWorkspace.E[ 2400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.E[ 2440 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 2440 ]), &(acadoWorkspace.E[ 2480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 2480 ]), &(acadoWorkspace.E[ 2520 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 2520 ]), &(acadoWorkspace.E[ 2560 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 2560 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2520 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2440 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.W1, 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 200 ]), &(acadoWorkspace.E[ 2600 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.E[ 2600 ]), &(acadoWorkspace.E[ 2640 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.E[ 2680 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 2680 ]), &(acadoWorkspace.E[ 2720 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 2720 ]), &(acadoWorkspace.E[ 2760 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 2760 ]), &(acadoWorkspace.E[ 2800 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 2800 ]), &(acadoWorkspace.E[ 2840 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 2840 ]), &(acadoWorkspace.E[ 2880 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.E[ 2920 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 2920 ]), &(acadoWorkspace.E[ 2960 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 2960 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2920 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2880 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2840 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2800 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2760 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2720 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2680 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2640 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 2600 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.E[ 3000 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.E[ 3000 ]), &(acadoWorkspace.E[ 3040 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 3040 ]), &(acadoWorkspace.E[ 3080 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 3080 ]), &(acadoWorkspace.E[ 3120 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.E[ 3160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 3160 ]), &(acadoWorkspace.E[ 3200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 3200 ]), &(acadoWorkspace.E[ 3240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 3240 ]), &(acadoWorkspace.E[ 3280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 3280 ]), &(acadoWorkspace.E[ 3320 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 3320 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3120 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3080 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3040 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3000 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 280 ]), &(acadoWorkspace.E[ 3360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.E[ 3400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 3400 ]), &(acadoWorkspace.E[ 3440 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 3440 ]), &(acadoWorkspace.E[ 3480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 3480 ]), &(acadoWorkspace.E[ 3520 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 3520 ]), &(acadoWorkspace.E[ 3560 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 3560 ]), &(acadoWorkspace.E[ 3600 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.E[ 3640 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 3640 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3600 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3560 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3520 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3440 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 320 ]), &(acadoWorkspace.E[ 3680 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.E[ 3680 ]), &(acadoWorkspace.E[ 3720 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 3720 ]), &(acadoWorkspace.E[ 3760 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 3760 ]), &(acadoWorkspace.E[ 3800 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 3800 ]), &(acadoWorkspace.E[ 3840 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 3840 ]), &(acadoWorkspace.E[ 3880 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 3880 ]), &(acadoWorkspace.E[ 3920 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 3920 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3880 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3840 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3800 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3760 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3720 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3680 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.E[ 3960 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.E[ 3960 ]), &(acadoWorkspace.E[ 4000 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 4000 ]), &(acadoWorkspace.E[ 4040 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 4040 ]), &(acadoWorkspace.E[ 4080 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 4080 ]), &(acadoWorkspace.E[ 4120 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 4120 ]), &(acadoWorkspace.E[ 4160 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 4160 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 9 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4120 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 9 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4080 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 9 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4040 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 9 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4000 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10, 9 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 3960 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.W1, 9 );

/* Column: 10 */
acado_moveGuE( &(acadoWorkspace.evGu[ 400 ]), &(acadoWorkspace.E[ 4200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.E[ 4200 ]), &(acadoWorkspace.E[ 4240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 4240 ]), &(acadoWorkspace.E[ 4280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 4280 ]), &(acadoWorkspace.E[ 4320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 4320 ]), &(acadoWorkspace.E[ 4360 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 4360 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 10 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 10 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 10 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11, 10 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.W1, 10 );

/* Column: 11 */
acado_moveGuE( &(acadoWorkspace.evGu[ 440 ]), &(acadoWorkspace.E[ 4400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.E[ 4400 ]), &(acadoWorkspace.E[ 4440 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 4440 ]), &(acadoWorkspace.E[ 4480 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 4480 ]), &(acadoWorkspace.E[ 4520 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 4520 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 11 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4480 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 11 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4440 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12, 11 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.W1, 11 );

/* Column: 12 */
acado_moveGuE( &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.E[ 4560 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.E[ 4560 ]), &(acadoWorkspace.E[ 4600 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 4600 ]), &(acadoWorkspace.E[ 4640 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 4640 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 12 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4600 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13, 12 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4560 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.W1, 12 );

/* Column: 13 */
acado_moveGuE( &(acadoWorkspace.evGu[ 520 ]), &(acadoWorkspace.E[ 4680 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.E[ 4680 ]), &(acadoWorkspace.E[ 4720 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 4720 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14, 13 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ 4680 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.W1, 13 );

/* Column: 14 */
acado_moveGuE( &(acadoWorkspace.evGu[ 560 ]), &(acadoWorkspace.E[ 4760 ]) );

acado_multQN1Gu( &(acadoWorkspace.E[ 4760 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.W1, 14 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );

for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 10] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)1.0000000000000000e-03 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)1.0000000000000000e-03 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)1.0000000000000000e-03 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)1.0000000000000000e-03 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)1.0000000000000000e-03 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)1.0000000000000000e-03 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)1.0000000000000000e-03 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)1.0000000000000000e-03 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)1.0000000000000000e-03 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)1.0000000000000000e-03 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)1.0000000000000000e-03 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)1.0000000000000000e-03 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)1.0000000000000000e-03 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)1.0000000000000000e-03 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-6.0000000000000000e+00 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)1.0000000000000000e-03 - acadoVariables.u[59];
acadoWorkspace.ub[0] = (real_t)6.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)6.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)6.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)6.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)6.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)6.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)6.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)6.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)6.0000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)6.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)6.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)6.0000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+00 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)6.0000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)6.0000000000000000e+00 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)6.0000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+00 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)6.0000000000000000e+00 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)6.0000000000000000e+00 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)6.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+00 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)6.0000000000000000e+00 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)6.0000000000000000e+00 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)6.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+00 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)6.0000000000000000e+00 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)6.0000000000000000e+00 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)6.0000000000000000e+00 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+00 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)6.0000000000000000e+00 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)6.0000000000000000e+00 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)6.0000000000000000e+00 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+00 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)6.0000000000000000e+00 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)6.0000000000000000e+00 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)6.0000000000000000e+00 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+00 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)6.0000000000000000e+00 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)6.0000000000000000e+00 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)6.0000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+00 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)6.0000000000000000e+00 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)6.0000000000000000e+00 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)6.0000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.0000000000000000e+00 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)6.0000000000000000e+00 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)6.0000000000000000e+00 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)6.0000000000000000e+00 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+00 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)6.0000000000000000e+00 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)6.0000000000000000e+00 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)6.0000000000000000e+00 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+00 - acadoVariables.u[59];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
for (lRun1 = 0; lRun1 < 210; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];
acadoWorkspace.DyN[9] -= acadoVariables.yN[9];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.g[ 56 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 98 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 182 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.QDy[ 140 ]) );

acadoWorkspace.QDy[150] = + (real_t)2.0000000000000000e+02*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[151] = + (real_t)2.0000000000000000e+02*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[152] = + (real_t)5.0000000000000000e+02*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[153] = +acadoWorkspace.DyN[3];
acadoWorkspace.QDy[154] = +acadoWorkspace.DyN[4];
acadoWorkspace.QDy[155] = +acadoWorkspace.DyN[5];
acadoWorkspace.QDy[156] = +acadoWorkspace.DyN[6];
acadoWorkspace.QDy[157] = + (real_t)1.0000000000000000e+02*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[158] = + (real_t)1.0000000000000000e+02*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[159] = + (real_t)1.0000000000000000e+02*acadoWorkspace.DyN[9];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 150 ]) );

acadoWorkspace.w1[0] = + (real_t)2.0000000000000000e+02*acadoWorkspace.sbar[150] + acadoWorkspace.QDy[150];
acadoWorkspace.w1[1] = + (real_t)2.0000000000000000e+02*acadoWorkspace.sbar[151] + acadoWorkspace.QDy[151];
acadoWorkspace.w1[2] = + (real_t)5.0000000000000000e+02*acadoWorkspace.sbar[152] + acadoWorkspace.QDy[152];
acadoWorkspace.w1[3] = +acadoWorkspace.sbar[153] + acadoWorkspace.QDy[153];
acadoWorkspace.w1[4] = +acadoWorkspace.sbar[154] + acadoWorkspace.QDy[154];
acadoWorkspace.w1[5] = +acadoWorkspace.sbar[155] + acadoWorkspace.QDy[155];
acadoWorkspace.w1[6] = +acadoWorkspace.sbar[156] + acadoWorkspace.QDy[156];
acadoWorkspace.w1[7] = + (real_t)1.0000000000000000e+02*acadoWorkspace.sbar[157] + acadoWorkspace.QDy[157];
acadoWorkspace.w1[8] = + (real_t)1.0000000000000000e+02*acadoWorkspace.sbar[158] + acadoWorkspace.QDy[158];
acadoWorkspace.w1[9] = + (real_t)1.0000000000000000e+02*acadoWorkspace.sbar[159] + acadoWorkspace.QDy[159];
acado_macBTw1( &(acadoWorkspace.evGu[ 560 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 130 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 130 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 440 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 110 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 110 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1000 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 900 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 800 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 280 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 700 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 70 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 70 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 500 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 50 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 50 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 300 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 10] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 50 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.evGu[ 200 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 50 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 70 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.evGu[ 280 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 70 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 800 ]), &(acadoWorkspace.evGu[ 320 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 900 ]), &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1000 ]), &(acadoWorkspace.evGu[ 400 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 110 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1100 ]), &(acadoWorkspace.evGu[ 440 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 110 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1200 ]), &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1300 ]), &(acadoWorkspace.evGu[ 520 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1400 ]), &(acadoWorkspace.evGu[ 560 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 150 ]) );
for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 15; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 10];
acadoWorkspace.state[1] = acadoVariables.x[index * 10 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 10 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 10 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 10 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 10 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 10 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 10 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 10 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 10 + 9];
acadoWorkspace.state[150] = acadoVariables.u[index * 4];
acadoWorkspace.state[151] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[152] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[153] = acadoVariables.u[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 10 + 10] = acadoWorkspace.state[0];
acadoVariables.x[index * 10 + 11] = acadoWorkspace.state[1];
acadoVariables.x[index * 10 + 12] = acadoWorkspace.state[2];
acadoVariables.x[index * 10 + 13] = acadoWorkspace.state[3];
acadoVariables.x[index * 10 + 14] = acadoWorkspace.state[4];
acadoVariables.x[index * 10 + 15] = acadoWorkspace.state[5];
acadoVariables.x[index * 10 + 16] = acadoWorkspace.state[6];
acadoVariables.x[index * 10 + 17] = acadoWorkspace.state[7];
acadoVariables.x[index * 10 + 18] = acadoWorkspace.state[8];
acadoVariables.x[index * 10 + 19] = acadoWorkspace.state[9];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 15; ++index)
{
acadoVariables.x[index * 10] = acadoVariables.x[index * 10 + 10];
acadoVariables.x[index * 10 + 1] = acadoVariables.x[index * 10 + 11];
acadoVariables.x[index * 10 + 2] = acadoVariables.x[index * 10 + 12];
acadoVariables.x[index * 10 + 3] = acadoVariables.x[index * 10 + 13];
acadoVariables.x[index * 10 + 4] = acadoVariables.x[index * 10 + 14];
acadoVariables.x[index * 10 + 5] = acadoVariables.x[index * 10 + 15];
acadoVariables.x[index * 10 + 6] = acadoVariables.x[index * 10 + 16];
acadoVariables.x[index * 10 + 7] = acadoVariables.x[index * 10 + 17];
acadoVariables.x[index * 10 + 8] = acadoVariables.x[index * 10 + 18];
acadoVariables.x[index * 10 + 9] = acadoVariables.x[index * 10 + 19];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
acadoVariables.x[153] = xEnd[3];
acadoVariables.x[154] = xEnd[4];
acadoVariables.x[155] = xEnd[5];
acadoVariables.x[156] = xEnd[6];
acadoVariables.x[157] = xEnd[7];
acadoVariables.x[158] = xEnd[8];
acadoVariables.x[159] = xEnd[9];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
acadoWorkspace.state[3] = acadoVariables.x[153];
acadoWorkspace.state[4] = acadoVariables.x[154];
acadoWorkspace.state[5] = acadoVariables.x[155];
acadoWorkspace.state[6] = acadoVariables.x[156];
acadoWorkspace.state[7] = acadoVariables.x[157];
acadoWorkspace.state[8] = acadoVariables.x[158];
acadoWorkspace.state[9] = acadoVariables.x[159];
if (uEnd != 0)
{
acadoWorkspace.state[150] = uEnd[0];
acadoWorkspace.state[151] = uEnd[1];
acadoWorkspace.state[152] = uEnd[2];
acadoWorkspace.state[153] = uEnd[3];
}
else
{
acadoWorkspace.state[150] = acadoVariables.u[56];
acadoWorkspace.state[151] = acadoVariables.u[57];
acadoWorkspace.state[152] = acadoVariables.u[58];
acadoWorkspace.state[153] = acadoVariables.u[59];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
acadoVariables.x[153] = acadoWorkspace.state[3];
acadoVariables.x[154] = acadoWorkspace.state[4];
acadoVariables.x[155] = acadoWorkspace.state[5];
acadoVariables.x[156] = acadoWorkspace.state[6];
acadoVariables.x[157] = acadoWorkspace.state[7];
acadoVariables.x[158] = acadoWorkspace.state[8];
acadoVariables.x[159] = acadoWorkspace.state[9];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 14; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[56] = uEnd[0];
acadoVariables.u[57] = uEnd[1];
acadoVariables.u[58] = uEnd[2];
acadoVariables.u[59] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 14 */
real_t tmpDy[ 14 ];

/** Row vector of size: 10 */
real_t tmpDyN[ 10 ];

for (lRun1 = 0; lRun1 < 15; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 10 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 14] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 14];
acadoWorkspace.Dy[lRun1 * 14 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 14 + 1];
acadoWorkspace.Dy[lRun1 * 14 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 14 + 2];
acadoWorkspace.Dy[lRun1 * 14 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 14 + 3];
acadoWorkspace.Dy[lRun1 * 14 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 14 + 4];
acadoWorkspace.Dy[lRun1 * 14 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 14 + 5];
acadoWorkspace.Dy[lRun1 * 14 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 14 + 6];
acadoWorkspace.Dy[lRun1 * 14 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 14 + 7];
acadoWorkspace.Dy[lRun1 * 14 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 14 + 8];
acadoWorkspace.Dy[lRun1 * 14 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 14 + 9];
acadoWorkspace.Dy[lRun1 * 14 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 14 + 10];
acadoWorkspace.Dy[lRun1 * 14 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 14 + 11];
acadoWorkspace.Dy[lRun1 * 14 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 14 + 12];
acadoWorkspace.Dy[lRun1 * 14 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 14 + 13];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.x[155];
acadoWorkspace.objValueIn[6] = acadoVariables.x[156];
acadoWorkspace.objValueIn[7] = acadoVariables.x[157];
acadoWorkspace.objValueIn[8] = acadoVariables.x[158];
acadoWorkspace.objValueIn[9] = acadoVariables.x[159];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9] - acadoVariables.yN[9];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 15; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 14]*(real_t)2.0000000000000000e+02;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 14 + 1]*(real_t)2.0000000000000000e+02;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 14 + 2]*(real_t)5.0000000000000000e+02;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 14 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 14 + 4];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 14 + 5];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 14 + 6];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 14 + 7]*(real_t)1.0000000000000000e+02;
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 14 + 8]*(real_t)1.0000000000000000e+02;
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 14 + 9]*(real_t)1.0000000000000000e+02;
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 14 + 10]*(real_t)2.9999999999999999e-01;
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 14 + 11]*(real_t)2.9999999999999999e-01;
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 14 + 12]*(real_t)2.9999999999999999e-01;
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 14 + 13]*(real_t)5.0000000000000000e+00;
objVal += + acadoWorkspace.Dy[lRun1 * 14]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 14 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 14 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 14 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 14 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 14 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 14 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 14 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 14 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 14 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 14 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 14 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 14 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 14 + 13]*tmpDy[13];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)2.0000000000000000e+02;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)2.0000000000000000e+02;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)5.0000000000000000e+02;
tmpDyN[3] = + acadoWorkspace.DyN[3];
tmpDyN[4] = + acadoWorkspace.DyN[4];
tmpDyN[5] = + acadoWorkspace.DyN[5];
tmpDyN[6] = + acadoWorkspace.DyN[6];
tmpDyN[7] = + acadoWorkspace.DyN[7]*(real_t)1.0000000000000000e+02;
tmpDyN[8] = + acadoWorkspace.DyN[8]*(real_t)1.0000000000000000e+02;
tmpDyN[9] = + acadoWorkspace.DyN[9]*(real_t)1.0000000000000000e+02;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8] + acadoWorkspace.DyN[9]*tmpDyN[9];

objVal *= 0.5;
return objVal;
}

