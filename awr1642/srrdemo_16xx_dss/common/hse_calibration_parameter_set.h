/*
 * hse_calibration_parameter_set.h
 *
 *  Created on: 06.11.2020
 *      Author: Steven
 */

#ifndef COMMON_HSE_CALIBRATION_PARAMETER_SET_H_
#define COMMON_HSE_CALIBRATION_PARAMETER_SET_H_

// RADAR ID SELECT, -1 = default (no calibration set usage)
#define HSE_RADAR_CALIB_PARAMSET_SELECT 1

// Radar ID 5586800134
#if HSE_RADAR_CALIB_PARAMSET_SELECT == 0
#define HSE_CALIB_PARAMSET_RANGE_BIAS  0.1165600f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_REAL -0.46582f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_IMAG -0.49887f
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_REAL -0.62988f
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_IMAG -0.27975f
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_REAL -0.60995f
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_IMAG -0.39777f
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_REAL -0.61441f
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_IMAG -0.47299f
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_REAL -0.69440f
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_IMAG -0.49570f
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_REAL -0.86789f
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_IMAG -0.19562f
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_REAL -0.89160f
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_IMAG -0.35620f
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_REAL -0.90100f
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_IMAG -0.43384f

// Radar 1 5542800373 (Multi Radarsystem)
#elif HSE_RADAR_CALIB_PARAMSET_SELECT == 1
#define HSE_CALIB_PARAMSET_RANGE_BIAS 0.122001060185185f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_REAL -0.384827592592593f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_IMAG -0.849931944444445f
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_REAL -0.796293333333333f
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_IMAG -0.355695925925926f
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_REAL -0.780615462962963f
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_IMAG -0.391220648148148f
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_REAL -0.756695555555555f
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_IMAG 0.04111712962963f
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_REAL -0.419851203703704f
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_IMAG 0.366307222222222f
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_REAL -0.252070185185185f
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_IMAG 0.463987592592592f
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_REAL -0.214651851851852f
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_IMAG 0.448162685185185f
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_REAL -0.003191203703704f
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_IMAG 0.465178333333333f

//Radar 2 5586800229 (Multi Radarsystem)
#elif HSE_RADAR_CALIB_PARAMSET_SELECT == 2
#define HSE_CALIB_PARAMSET_RANGE_BIAS 0.071672674226804f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_REAL -0.860259484536082f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_IMAG -0.224636288659794f
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_REAL -0.860786494845361f
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_IMAG  0.071846907216495f
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_REAL -0.77775793814433f
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_IMAG  0.112306597938144f
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_REAL -0.973763195876289f
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_IMAG  0.224337113402062f
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_REAL -0.319144639175258f
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_IMAG 0.629530206185567f
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_REAL -0.000259381443299f
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_IMAG  0.603111030927835f
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_REAL 0.021849484536083f
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_IMAG  0.62578381443299f
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_REAL   0.131079381443299f
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_IMAG   0.779119587628866f

#else // Default (no calibration set usage)
#define HSE_CALIB_PARAMSET_RANGE_BIAS 0.0f
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_0_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_1_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_2_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_3_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_4_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_5_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_6_COMP_IMAG 0U
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_REAL 1U
#define HSE_CALIB_PARAMSET_PHASE_RX_7_COMP_IMAG 0U
#endif

#endif /* COMMON_HSE_CALIBRATION_PARAMETER_SET_H_ */
