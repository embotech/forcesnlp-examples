/*
forces : A fast customized optimization solver.

Copyright (C) 2013-2016 EMBOTECH GMBH [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME forces_simulinkBlockcompact

#include "simstruc.h"



/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


/* include FORCES functions and defs */
#include "../include/forces.h" 

#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef forcesINTERFACE_FLOAT forcesNMPC_FLOAT;





/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
     if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
	 return; /* Parameter mismatch will be reported by Simulink */
     }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 12 in total */
    if (!ssSetNumInputPorts(S, 12)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 6, 21);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 6, 160);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 8, 160);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 8, 20);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 8, 20);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 8, 20);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 8, 20);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 6, 6);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 6, 1);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 6, 1);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 6, 1);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 10, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/	
	/* Input Port 11 */
    ssSetInputPortMatrixDimensions(S,  11, 6, 1);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 166, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
 if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
  ssSetInputPortDataType( S, 0, SS_DOUBLE);
 ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	int i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *c = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *C = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *H = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *f = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *lb = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *ub = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *z = (const real_T*) ssGetInputPortSignal(S,6);
	const real_T *H21 = (const real_T*) ssGetInputPortSignal(S,7);
	const real_T *f21 = (const real_T*) ssGetInputPortSignal(S,8);
	const real_T *lb21 = (const real_T*) ssGetInputPortSignal(S,9);
	const real_T *ub21 = (const real_T*) ssGetInputPortSignal(S,10);
	const real_T *z_init_20 = (const real_T*) ssGetInputPortSignal(S,11);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	forces_params params;
	forces_output output;
	forces_info info;	
	int exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<64; i++){ params.H1[i] = (double) H[i]; }
	for( i=0; i<8; i++){ params.f1[i] = (double) f[i]; }
	for( i=0; i<8; i++){ params.lb1[i] = (double) lb[i]; }
	for( i=0; i<8; i++){ params.ub1[i] = (double) ub[i]; }
	for( i=0; i<48; i++){ params.C1[i] = (double) C[i]; }
	for( i=0; i<6; i++){ params.d1[i] = (double) c[i]; }
	j=64; for( i=0; i<64; i++){ params.H2[i] = (double) H[j++]; }
	j=8; for( i=0; i<8; i++){ params.f2[i] = (double) f[j++]; }
	j=8; for( i=0; i<8; i++){ params.lb2[i] = (double) lb[j++]; }
	j=8; for( i=0; i<8; i++){ params.ub2[i] = (double) ub[j++]; }
	j=48; for( i=0; i<48; i++){ params.C2[i] = (double) C[j++]; }
	j=6; for( i=0; i<6; i++){ params.d2[i] = (double) c[j++]; }
	j=128; for( i=0; i<64; i++){ params.H3[i] = (double) H[j++]; }
	j=16; for( i=0; i<8; i++){ params.f3[i] = (double) f[j++]; }
	j=16; for( i=0; i<8; i++){ params.lb3[i] = (double) lb[j++]; }
	j=16; for( i=0; i<8; i++){ params.ub3[i] = (double) ub[j++]; }
	j=96; for( i=0; i<48; i++){ params.C3[i] = (double) C[j++]; }
	j=12; for( i=0; i<6; i++){ params.d3[i] = (double) c[j++]; }
	j=192; for( i=0; i<64; i++){ params.H4[i] = (double) H[j++]; }
	j=24; for( i=0; i<8; i++){ params.f4[i] = (double) f[j++]; }
	j=24; for( i=0; i<8; i++){ params.lb4[i] = (double) lb[j++]; }
	j=24; for( i=0; i<8; i++){ params.ub4[i] = (double) ub[j++]; }
	j=144; for( i=0; i<48; i++){ params.C4[i] = (double) C[j++]; }
	j=18; for( i=0; i<6; i++){ params.d4[i] = (double) c[j++]; }
	j=256; for( i=0; i<64; i++){ params.H5[i] = (double) H[j++]; }
	j=32; for( i=0; i<8; i++){ params.f5[i] = (double) f[j++]; }
	j=32; for( i=0; i<8; i++){ params.lb5[i] = (double) lb[j++]; }
	j=32; for( i=0; i<8; i++){ params.ub5[i] = (double) ub[j++]; }
	j=192; for( i=0; i<48; i++){ params.C5[i] = (double) C[j++]; }
	j=24; for( i=0; i<6; i++){ params.d5[i] = (double) c[j++]; }
	j=320; for( i=0; i<64; i++){ params.H6[i] = (double) H[j++]; }
	j=40; for( i=0; i<8; i++){ params.f6[i] = (double) f[j++]; }
	j=40; for( i=0; i<8; i++){ params.lb6[i] = (double) lb[j++]; }
	j=40; for( i=0; i<8; i++){ params.ub6[i] = (double) ub[j++]; }
	j=240; for( i=0; i<48; i++){ params.C6[i] = (double) C[j++]; }
	j=30; for( i=0; i<6; i++){ params.d6[i] = (double) c[j++]; }
	j=384; for( i=0; i<64; i++){ params.H7[i] = (double) H[j++]; }
	j=48; for( i=0; i<8; i++){ params.f7[i] = (double) f[j++]; }
	j=48; for( i=0; i<8; i++){ params.lb7[i] = (double) lb[j++]; }
	j=48; for( i=0; i<8; i++){ params.ub7[i] = (double) ub[j++]; }
	j=288; for( i=0; i<48; i++){ params.C7[i] = (double) C[j++]; }
	j=36; for( i=0; i<6; i++){ params.d7[i] = (double) c[j++]; }
	j=448; for( i=0; i<64; i++){ params.H8[i] = (double) H[j++]; }
	j=56; for( i=0; i<8; i++){ params.f8[i] = (double) f[j++]; }
	j=56; for( i=0; i<8; i++){ params.lb8[i] = (double) lb[j++]; }
	j=56; for( i=0; i<8; i++){ params.ub8[i] = (double) ub[j++]; }
	j=336; for( i=0; i<48; i++){ params.C8[i] = (double) C[j++]; }
	j=42; for( i=0; i<6; i++){ params.d8[i] = (double) c[j++]; }
	j=512; for( i=0; i<64; i++){ params.H9[i] = (double) H[j++]; }
	j=64; for( i=0; i<8; i++){ params.f9[i] = (double) f[j++]; }
	j=64; for( i=0; i<8; i++){ params.lb9[i] = (double) lb[j++]; }
	j=64; for( i=0; i<8; i++){ params.ub9[i] = (double) ub[j++]; }
	j=384; for( i=0; i<48; i++){ params.C9[i] = (double) C[j++]; }
	j=48; for( i=0; i<6; i++){ params.d9[i] = (double) c[j++]; }
	j=576; for( i=0; i<64; i++){ params.H10[i] = (double) H[j++]; }
	j=72; for( i=0; i<8; i++){ params.f10[i] = (double) f[j++]; }
	j=72; for( i=0; i<8; i++){ params.lb10[i] = (double) lb[j++]; }
	j=72; for( i=0; i<8; i++){ params.ub10[i] = (double) ub[j++]; }
	j=432; for( i=0; i<48; i++){ params.C10[i] = (double) C[j++]; }
	j=54; for( i=0; i<6; i++){ params.d10[i] = (double) c[j++]; }
	j=640; for( i=0; i<64; i++){ params.H11[i] = (double) H[j++]; }
	j=80; for( i=0; i<8; i++){ params.f11[i] = (double) f[j++]; }
	j=80; for( i=0; i<8; i++){ params.lb11[i] = (double) lb[j++]; }
	j=80; for( i=0; i<8; i++){ params.ub11[i] = (double) ub[j++]; }
	j=480; for( i=0; i<48; i++){ params.C11[i] = (double) C[j++]; }
	j=60; for( i=0; i<6; i++){ params.d11[i] = (double) c[j++]; }
	j=704; for( i=0; i<64; i++){ params.H12[i] = (double) H[j++]; }
	j=88; for( i=0; i<8; i++){ params.f12[i] = (double) f[j++]; }
	j=88; for( i=0; i<8; i++){ params.lb12[i] = (double) lb[j++]; }
	j=88; for( i=0; i<8; i++){ params.ub12[i] = (double) ub[j++]; }
	j=528; for( i=0; i<48; i++){ params.C12[i] = (double) C[j++]; }
	j=66; for( i=0; i<6; i++){ params.d12[i] = (double) c[j++]; }
	j=768; for( i=0; i<64; i++){ params.H13[i] = (double) H[j++]; }
	j=96; for( i=0; i<8; i++){ params.f13[i] = (double) f[j++]; }
	j=96; for( i=0; i<8; i++){ params.lb13[i] = (double) lb[j++]; }
	j=96; for( i=0; i<8; i++){ params.ub13[i] = (double) ub[j++]; }
	j=576; for( i=0; i<48; i++){ params.C13[i] = (double) C[j++]; }
	j=72; for( i=0; i<6; i++){ params.d13[i] = (double) c[j++]; }
	j=832; for( i=0; i<64; i++){ params.H14[i] = (double) H[j++]; }
	j=104; for( i=0; i<8; i++){ params.f14[i] = (double) f[j++]; }
	j=104; for( i=0; i<8; i++){ params.lb14[i] = (double) lb[j++]; }
	j=104; for( i=0; i<8; i++){ params.ub14[i] = (double) ub[j++]; }
	j=624; for( i=0; i<48; i++){ params.C14[i] = (double) C[j++]; }
	j=78; for( i=0; i<6; i++){ params.d14[i] = (double) c[j++]; }
	j=896; for( i=0; i<64; i++){ params.H15[i] = (double) H[j++]; }
	j=112; for( i=0; i<8; i++){ params.f15[i] = (double) f[j++]; }
	j=112; for( i=0; i<8; i++){ params.lb15[i] = (double) lb[j++]; }
	j=112; for( i=0; i<8; i++){ params.ub15[i] = (double) ub[j++]; }
	j=672; for( i=0; i<48; i++){ params.C15[i] = (double) C[j++]; }
	j=84; for( i=0; i<6; i++){ params.d15[i] = (double) c[j++]; }
	j=960; for( i=0; i<64; i++){ params.H16[i] = (double) H[j++]; }
	j=120; for( i=0; i<8; i++){ params.f16[i] = (double) f[j++]; }
	j=120; for( i=0; i<8; i++){ params.lb16[i] = (double) lb[j++]; }
	j=120; for( i=0; i<8; i++){ params.ub16[i] = (double) ub[j++]; }
	j=720; for( i=0; i<48; i++){ params.C16[i] = (double) C[j++]; }
	j=90; for( i=0; i<6; i++){ params.d16[i] = (double) c[j++]; }
	j=1024; for( i=0; i<64; i++){ params.H17[i] = (double) H[j++]; }
	j=128; for( i=0; i<8; i++){ params.f17[i] = (double) f[j++]; }
	j=128; for( i=0; i<8; i++){ params.lb17[i] = (double) lb[j++]; }
	j=128; for( i=0; i<8; i++){ params.ub17[i] = (double) ub[j++]; }
	j=768; for( i=0; i<48; i++){ params.C17[i] = (double) C[j++]; }
	j=96; for( i=0; i<6; i++){ params.d17[i] = (double) c[j++]; }
	j=1088; for( i=0; i<64; i++){ params.H18[i] = (double) H[j++]; }
	j=136; for( i=0; i<8; i++){ params.f18[i] = (double) f[j++]; }
	j=136; for( i=0; i<8; i++){ params.lb18[i] = (double) lb[j++]; }
	j=136; for( i=0; i<8; i++){ params.ub18[i] = (double) ub[j++]; }
	j=816; for( i=0; i<48; i++){ params.C18[i] = (double) C[j++]; }
	j=102; for( i=0; i<6; i++){ params.d18[i] = (double) c[j++]; }
	j=1152; for( i=0; i<64; i++){ params.H19[i] = (double) H[j++]; }
	j=144; for( i=0; i<8; i++){ params.f19[i] = (double) f[j++]; }
	j=144; for( i=0; i<8; i++){ params.lb19[i] = (double) lb[j++]; }
	j=144; for( i=0; i<8; i++){ params.ub19[i] = (double) ub[j++]; }
	j=864; for( i=0; i<48; i++){ params.C19[i] = (double) C[j++]; }
	j=108; for( i=0; i<6; i++){ params.d19[i] = (double) c[j++]; }
	j=1216; for( i=0; i<64; i++){ params.H20[i] = (double) H[j++]; }
	j=152; for( i=0; i<8; i++){ params.f20[i] = (double) f[j++]; }
	j=152; for( i=0; i<8; i++){ params.lb20[i] = (double) lb[j++]; }
	j=152; for( i=0; i<8; i++){ params.ub20[i] = (double) ub[j++]; }
	j=912; for( i=0; i<48; i++){ params.C20[i] = (double) C[j++]; }
	j=114; for( i=0; i<6; i++){ params.d20[i] = (double) c[j++]; }
	for( i=0; i<36; i++){ params.H21[i] = (double) H21[i]; }
	for( i=0; i<6; i++){ params.f21[i] = (double) f21[i]; }
	for( i=0; i<6; i++){ params.lb21[i] = (double) lb21[i]; }
	for( i=0; i<6; i++){ params.ub21[i] = (double) ub21[i]; }
	j=120; for( i=0; i<6; i++){ params.d21[i] = (double) c[j++]; }
	for( i=0; i<8; i++){ params.z_init_00[i] = (double) z[i]; }
	j=8; for( i=0; i<8; i++){ params.z_init_01[i] = (double) z[j++]; }
	j=16; for( i=0; i<8; i++){ params.z_init_02[i] = (double) z[j++]; }
	j=24; for( i=0; i<8; i++){ params.z_init_03[i] = (double) z[j++]; }
	j=32; for( i=0; i<8; i++){ params.z_init_04[i] = (double) z[j++]; }
	j=40; for( i=0; i<8; i++){ params.z_init_05[i] = (double) z[j++]; }
	j=48; for( i=0; i<8; i++){ params.z_init_06[i] = (double) z[j++]; }
	j=56; for( i=0; i<8; i++){ params.z_init_07[i] = (double) z[j++]; }
	j=64; for( i=0; i<8; i++){ params.z_init_08[i] = (double) z[j++]; }
	j=72; for( i=0; i<8; i++){ params.z_init_09[i] = (double) z[j++]; }
	j=80; for( i=0; i<8; i++){ params.z_init_10[i] = (double) z[j++]; }
	j=88; for( i=0; i<8; i++){ params.z_init_11[i] = (double) z[j++]; }
	j=96; for( i=0; i<8; i++){ params.z_init_12[i] = (double) z[j++]; }
	j=104; for( i=0; i<8; i++){ params.z_init_13[i] = (double) z[j++]; }
	j=112; for( i=0; i<8; i++){ params.z_init_14[i] = (double) z[j++]; }
	j=120; for( i=0; i<8; i++){ params.z_init_15[i] = (double) z[j++]; }
	j=128; for( i=0; i<8; i++){ params.z_init_16[i] = (double) z[j++]; }
	j=136; for( i=0; i<8; i++){ params.z_init_17[i] = (double) z[j++]; }
	j=144; for( i=0; i<8; i++){ params.z_init_18[i] = (double) z[j++]; }
	j=152; for( i=0; i<8; i++){ params.z_init_19[i] = (double) z[j++]; }
	for( i=0; i<6; i++){ params.z_init_20[i] = (double) z_init_20[i]; }
	

	

    #if forces_SET_PRINTLEVEL > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) {
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = forces_solve(&params, &output, &info, fp );

	#if forces_SET_PRINTLEVEL > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) {
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<8; i++){ outputs[i] = (real_T) output.out1[i]; }
	k=8; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out2[i]; }
	k=16; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out3[i]; }
	k=24; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out4[i]; }
	k=32; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out5[i]; }
	k=40; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out6[i]; }
	k=48; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out7[i]; }
	k=56; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out8[i]; }
	k=64; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out9[i]; }
	k=72; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out10[i]; }
	k=80; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out11[i]; }
	k=88; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out12[i]; }
	k=96; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out13[i]; }
	k=104; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out14[i]; }
	k=112; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out15[i]; }
	k=120; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out16[i]; }
	k=128; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out17[i]; }
	k=136; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out18[i]; }
	k=144; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out19[i]; }
	k=152; for( i=0; i<8; i++){ outputs[k++] = (real_T) output.out20[i]; }
	k=160; for( i=0; i<6; i++){ outputs[k++] = (real_T) output.out21[i]; }
	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


