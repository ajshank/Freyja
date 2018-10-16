/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'quad_model/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"

PmfMessageId quad_model_a487e7e2_1_deriv(const double *state, const double
  *input, const double *inputDot, const double *inputDdot, const double
  *discreteState, double *deriv, double *errorResult, NeuDiagnosticManager
  *neDiagMgr)
{
  int ii[9];
  double xx[160];
  (void) inputDot;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = state[10];
  xx[5] = state[11];
  xx[6] = state[12];
  pm_math_quatDeriv(xx + 0, xx + 4, xx + 7);
  xx[11] = 1.0;
  xx[12] = 2.0;
  xx[13] = state[5] * state[5];
  xx[14] = state[6] * state[6];
  xx[16] = xx[11] - xx[12] * (xx[13] + xx[14]);
  xx[15] = state[4] * state[5];
  xx[17] = state[3] * state[6];
  xx[19] = xx[12] * (xx[15] - xx[17]);
  xx[18] = state[3] * state[5];
  xx[20] = state[4] * state[6];
  xx[22] = xx[12] * (xx[18] + xx[20]);
  xx[23] = xx[16];
  xx[24] = xx[19];
  xx[25] = xx[22];
  xx[21] = 0.55;
  xx[26] = xx[21] * xx[16];
  xx[16] = xx[21] * xx[19];
  xx[19] = xx[21] * xx[22];
  xx[27] = xx[26];
  xx[28] = xx[16];
  xx[29] = xx[19];
  xx[22] = xx[12] * (xx[17] + xx[15]);
  xx[15] = xx[21] * xx[22];
  xx[17] = state[4] * state[4];
  xx[30] = xx[11] - xx[12] * (xx[14] + xx[17]);
  xx[14] = xx[21] * xx[30];
  xx[31] = state[5] * state[6];
  xx[32] = state[3] * state[4];
  xx[34] = xx[12] * (xx[31] - xx[32]);
  xx[33] = xx[21] * xx[34];
  xx[35] = xx[15];
  xx[36] = xx[14];
  xx[37] = xx[33];
  xx[38] = pm_math_dot3(xx + 23, xx + 35);
  xx[39] = xx[12] * (xx[20] - xx[18]);
  xx[18] = xx[21] * xx[39];
  xx[20] = xx[12] * (xx[32] + xx[31]);
  xx[31] = xx[21] * xx[20];
  xx[32] = xx[11] - xx[12] * (xx[17] + xx[13]);
  xx[13] = xx[21] * xx[32];
  xx[40] = xx[18];
  xx[41] = xx[31];
  xx[42] = xx[13];
  xx[17] = pm_math_dot3(xx + 23, xx + 40);
  xx[43] = 0.0;
  xx[44] = xx[22];
  xx[45] = xx[30];
  xx[46] = xx[34];
  xx[22] = pm_math_dot3(xx + 44, xx + 40);
  xx[48] = xx[39];
  xx[49] = xx[20];
  xx[50] = xx[32];
  xx[20] = 1.5;
  xx[52] = pm_math_dot3(xx + 23, xx + 27);
  xx[53] = xx[38];
  xx[54] = xx[17];
  xx[55] = xx[43];
  xx[56] = xx[43];
  xx[57] = xx[43];
  xx[58] = xx[38];
  xx[59] = pm_math_dot3(xx + 44, xx + 35);
  xx[60] = xx[22];
  xx[61] = xx[43];
  xx[62] = xx[43];
  xx[63] = xx[43];
  xx[64] = xx[17];
  xx[65] = xx[22];
  xx[66] = pm_math_dot3(xx + 48, xx + 40);
  xx[67] = xx[43];
  xx[68] = xx[43];
  xx[69] = xx[43];
  xx[70] = xx[43];
  xx[71] = xx[43];
  xx[72] = xx[43];
  xx[73] = xx[20];
  xx[74] = xx[43];
  xx[75] = xx[43];
  xx[76] = xx[43];
  xx[77] = xx[43];
  xx[78] = xx[43];
  xx[79] = xx[43];
  xx[80] = xx[20];
  xx[81] = xx[43];
  xx[82] = xx[43];
  xx[83] = xx[43];
  xx[84] = xx[43];
  xx[85] = xx[43];
  xx[86] = xx[43];
  xx[87] = xx[11];
  ii[0] = 0;
  ii[0] = factorSymmetric(xx + 52, 6, xx + 89, xx + 34, ii + 1, ii + 8);
  if (ii[8] != 0) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:DegenerateMass",
      "'quad_model/HUMMINGBIRD' has a degenerate mass distribution on its follower side.",
      neDiagMgr);
  }

  xx[27] = - state[3];
  xx[28] = - state[4];
  xx[29] = - state[5];
  xx[30] = - state[6];
  xx[89] = state[7];
  xx[90] = state[8];
  xx[91] = state[9];
  pm_math_quatInverseXform(xx + 27, xx + 89, xx + 92);
  pm_math_cross3(xx + 4, xx + 92, xx + 27);
  xx[89] = - xx[92];
  xx[90] = - xx[93];
  xx[91] = - xx[94];
  pm_math_cross3(xx + 4, xx + 89, xx + 92);
  xx[89] = xx[21] * (xx[27] + xx[92]);
  xx[90] = xx[21] * (xx[28] + xx[93]);
  xx[91] = xx[21] * (xx[29] + xx[94]);
  xx[27] = input[3];
  xx[28] = input[4];
  xx[29] = input[5];
  pm_math_quatInverseXform(xx + 0, xx + 27, xx + 94);
  xx[0] = xx[20] * state[10];
  xx[1] = xx[20] * state[11];
  xx[2] = state[12];
  pm_math_cross3(xx + 4, xx + 0, xx + 27);
  xx[0] = input[0] - pm_math_dot3(xx + 23, xx + 89);
  xx[1] = input[1] - pm_math_dot3(xx + 44, xx + 89);
  xx[2] = input[2] - pm_math_dot3(xx + 48, xx + 89);
  xx[3] = xx[94] - xx[27];
  xx[4] = xx[95] - xx[28];
  xx[5] = xx[96] - xx[29];
  solveSymmetric(xx + 52, xx + 34, ii + 1, xx + 0, 6, 1, ii[0], xx + 44, xx + 88);
  xx[88] = xx[43];
  xx[89] = xx[43];
  xx[90] = xx[43];
  xx[91] = xx[20];
  xx[92] = xx[43];
  xx[93] = xx[43];
  xx[94] = xx[43];
  xx[95] = xx[43];
  xx[96] = xx[43];
  xx[97] = xx[43];
  xx[98] = xx[20];
  xx[99] = xx[43];
  xx[100] = xx[43];
  xx[101] = xx[43];
  xx[102] = xx[43];
  xx[103] = xx[43];
  xx[104] = xx[43];
  xx[105] = xx[11];
  xx[106] = xx[26];
  xx[107] = xx[15];
  xx[108] = xx[18];
  xx[109] = xx[43];
  xx[110] = xx[43];
  xx[111] = xx[43];
  xx[112] = xx[16];
  xx[113] = xx[14];
  xx[114] = xx[31];
  xx[115] = xx[43];
  xx[116] = xx[43];
  xx[117] = xx[43];
  xx[118] = xx[19];
  xx[119] = xx[33];
  xx[120] = xx[13];
  xx[121] = xx[43];
  xx[122] = xx[43];
  xx[123] = xx[43];
  solveSymmetric(xx + 52, xx + 34, ii + 1, xx + 88, 6, 6, ii[0], xx + 124, xx +
                 0);
  xx[0] = xx[142];
  xx[1] = xx[148];
  xx[2] = xx[154];
  xx[3] = 9.806649999999999;
  xx[4] = xx[3] * state[4];
  xx[6] = xx[3] * state[5];
  xx[15] = xx[12] * (xx[4] * state[6] - xx[6] * state[3]);
  xx[16] = xx[12] * (xx[4] * state[3] + xx[6] * state[6]);
  xx[17] = xx[3] - xx[12] * (xx[4] * state[4] + xx[6] * state[5]);
  xx[3] = xx[143];
  xx[4] = xx[149];
  xx[5] = xx[155];
  xx[11] = xx[144];
  xx[12] = xx[150];
  xx[13] = xx[156];
  xx[21] = xx[145];
  xx[22] = xx[151];
  xx[23] = xx[157];
  xx[24] = xx[146];
  xx[25] = xx[152];
  xx[26] = xx[158];
  xx[27] = xx[147];
  xx[28] = xx[153];
  xx[29] = xx[159];
  deriv[0] = state[7];
  deriv[1] = state[8];
  deriv[2] = state[9];
  deriv[3] = xx[7];
  deriv[4] = xx[8];
  deriv[5] = xx[9];
  deriv[6] = xx[10];
  deriv[7] = xx[44] - pm_math_dot3(xx + 0, xx + 15);
  deriv[8] = xx[45] - pm_math_dot3(xx + 3, xx + 15);
  deriv[9] = xx[46] - pm_math_dot3(xx + 11, xx + 15);
  deriv[10] = xx[47] - pm_math_dot3(xx + 21, xx + 15);
  deriv[11] = xx[48] - pm_math_dot3(xx + 24, xx + 15);
  deriv[12] = xx[49] - pm_math_dot3(xx + 27, xx + 15);
  errorResult[0] = xx[43];
  return NULL;
}