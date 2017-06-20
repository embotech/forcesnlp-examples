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
    ssSetInputPortMatrixDimensions(S,  0, 17, 31);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 17, 630);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 21, 630);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 21, 30);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 15, 30);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 15, 30);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 21, 30);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 17, 17);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 17, 1);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 11, 1);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 11, 1);
    ssSetInputPortDataType(S, 10, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 10, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 10, 1); /*direct input signal access*/	
	/* Input Port 11 */
    ssSetInputPortMatrixDimensions(S,  11, 17, 1);
    ssSetInputPortDataType(S, 11, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 11, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 11, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 647, 1);
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
	const real_T *H31 = (const real_T*) ssGetInputPortSignal(S,7);
	const real_T *f31 = (const real_T*) ssGetInputPortSignal(S,8);
	const real_T *lb31 = (const real_T*) ssGetInputPortSignal(S,9);
	const real_T *ub31 = (const real_T*) ssGetInputPortSignal(S,10);
	const real_T *z_init_30 = (const real_T*) ssGetInputPortSignal(S,11);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	forces_params params;
	forces_output output;
	forces_info info;	
	int exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<441; i++){ params.H1[i] = (double) H[i]; }
	for( i=0; i<21; i++){ params.f1[i] = (double) f[i]; }
	for( i=0; i<15; i++){ params.lb1[i] = (double) lb[i]; }
	for( i=0; i<15; i++){ params.ub1[i] = (double) ub[i]; }
	for( i=0; i<357; i++){ params.C1[i] = (double) C[i]; }
	for( i=0; i<17; i++){ params.d1[i] = (double) c[i]; }
	j=441; for( i=0; i<441; i++){ params.H2[i] = (double) H[j++]; }
	j=21; for( i=0; i<21; i++){ params.f2[i] = (double) f[j++]; }
	j=15; for( i=0; i<15; i++){ params.lb2[i] = (double) lb[j++]; }
	j=15; for( i=0; i<15; i++){ params.ub2[i] = (double) ub[j++]; }
	j=357; for( i=0; i<357; i++){ params.C2[i] = (double) C[j++]; }
	j=17; for( i=0; i<17; i++){ params.d2[i] = (double) c[j++]; }
	j=882; for( i=0; i<441; i++){ params.H3[i] = (double) H[j++]; }
	j=42; for( i=0; i<21; i++){ params.f3[i] = (double) f[j++]; }
	j=30; for( i=0; i<15; i++){ params.lb3[i] = (double) lb[j++]; }
	j=30; for( i=0; i<15; i++){ params.ub3[i] = (double) ub[j++]; }
	j=714; for( i=0; i<357; i++){ params.C3[i] = (double) C[j++]; }
	j=34; for( i=0; i<17; i++){ params.d3[i] = (double) c[j++]; }
	j=1323; for( i=0; i<441; i++){ params.H4[i] = (double) H[j++]; }
	j=63; for( i=0; i<21; i++){ params.f4[i] = (double) f[j++]; }
	j=45; for( i=0; i<15; i++){ params.lb4[i] = (double) lb[j++]; }
	j=45; for( i=0; i<15; i++){ params.ub4[i] = (double) ub[j++]; }
	j=1071; for( i=0; i<357; i++){ params.C4[i] = (double) C[j++]; }
	j=51; for( i=0; i<17; i++){ params.d4[i] = (double) c[j++]; }
	j=1764; for( i=0; i<441; i++){ params.H5[i] = (double) H[j++]; }
	j=84; for( i=0; i<21; i++){ params.f5[i] = (double) f[j++]; }
	j=60; for( i=0; i<15; i++){ params.lb5[i] = (double) lb[j++]; }
	j=60; for( i=0; i<15; i++){ params.ub5[i] = (double) ub[j++]; }
	j=1428; for( i=0; i<357; i++){ params.C5[i] = (double) C[j++]; }
	j=68; for( i=0; i<17; i++){ params.d5[i] = (double) c[j++]; }
	j=2205; for( i=0; i<441; i++){ params.H6[i] = (double) H[j++]; }
	j=105; for( i=0; i<21; i++){ params.f6[i] = (double) f[j++]; }
	j=75; for( i=0; i<15; i++){ params.lb6[i] = (double) lb[j++]; }
	j=75; for( i=0; i<15; i++){ params.ub6[i] = (double) ub[j++]; }
	j=1785; for( i=0; i<357; i++){ params.C6[i] = (double) C[j++]; }
	j=85; for( i=0; i<17; i++){ params.d6[i] = (double) c[j++]; }
	j=2646; for( i=0; i<441; i++){ params.H7[i] = (double) H[j++]; }
	j=126; for( i=0; i<21; i++){ params.f7[i] = (double) f[j++]; }
	j=90; for( i=0; i<15; i++){ params.lb7[i] = (double) lb[j++]; }
	j=90; for( i=0; i<15; i++){ params.ub7[i] = (double) ub[j++]; }
	j=2142; for( i=0; i<357; i++){ params.C7[i] = (double) C[j++]; }
	j=102; for( i=0; i<17; i++){ params.d7[i] = (double) c[j++]; }
	j=3087; for( i=0; i<441; i++){ params.H8[i] = (double) H[j++]; }
	j=147; for( i=0; i<21; i++){ params.f8[i] = (double) f[j++]; }
	j=105; for( i=0; i<15; i++){ params.lb8[i] = (double) lb[j++]; }
	j=105; for( i=0; i<15; i++){ params.ub8[i] = (double) ub[j++]; }
	j=2499; for( i=0; i<357; i++){ params.C8[i] = (double) C[j++]; }
	j=119; for( i=0; i<17; i++){ params.d8[i] = (double) c[j++]; }
	j=3528; for( i=0; i<441; i++){ params.H9[i] = (double) H[j++]; }
	j=168; for( i=0; i<21; i++){ params.f9[i] = (double) f[j++]; }
	j=120; for( i=0; i<15; i++){ params.lb9[i] = (double) lb[j++]; }
	j=120; for( i=0; i<15; i++){ params.ub9[i] = (double) ub[j++]; }
	j=2856; for( i=0; i<357; i++){ params.C9[i] = (double) C[j++]; }
	j=136; for( i=0; i<17; i++){ params.d9[i] = (double) c[j++]; }
	j=3969; for( i=0; i<441; i++){ params.H10[i] = (double) H[j++]; }
	j=189; for( i=0; i<21; i++){ params.f10[i] = (double) f[j++]; }
	j=135; for( i=0; i<15; i++){ params.lb10[i] = (double) lb[j++]; }
	j=135; for( i=0; i<15; i++){ params.ub10[i] = (double) ub[j++]; }
	j=3213; for( i=0; i<357; i++){ params.C10[i] = (double) C[j++]; }
	j=153; for( i=0; i<17; i++){ params.d10[i] = (double) c[j++]; }
	j=4410; for( i=0; i<441; i++){ params.H11[i] = (double) H[j++]; }
	j=210; for( i=0; i<21; i++){ params.f11[i] = (double) f[j++]; }
	j=150; for( i=0; i<15; i++){ params.lb11[i] = (double) lb[j++]; }
	j=150; for( i=0; i<15; i++){ params.ub11[i] = (double) ub[j++]; }
	j=3570; for( i=0; i<357; i++){ params.C11[i] = (double) C[j++]; }
	j=170; for( i=0; i<17; i++){ params.d11[i] = (double) c[j++]; }
	j=4851; for( i=0; i<441; i++){ params.H12[i] = (double) H[j++]; }
	j=231; for( i=0; i<21; i++){ params.f12[i] = (double) f[j++]; }
	j=165; for( i=0; i<15; i++){ params.lb12[i] = (double) lb[j++]; }
	j=165; for( i=0; i<15; i++){ params.ub12[i] = (double) ub[j++]; }
	j=3927; for( i=0; i<357; i++){ params.C12[i] = (double) C[j++]; }
	j=187; for( i=0; i<17; i++){ params.d12[i] = (double) c[j++]; }
	j=5292; for( i=0; i<441; i++){ params.H13[i] = (double) H[j++]; }
	j=252; for( i=0; i<21; i++){ params.f13[i] = (double) f[j++]; }
	j=180; for( i=0; i<15; i++){ params.lb13[i] = (double) lb[j++]; }
	j=180; for( i=0; i<15; i++){ params.ub13[i] = (double) ub[j++]; }
	j=4284; for( i=0; i<357; i++){ params.C13[i] = (double) C[j++]; }
	j=204; for( i=0; i<17; i++){ params.d13[i] = (double) c[j++]; }
	j=5733; for( i=0; i<441; i++){ params.H14[i] = (double) H[j++]; }
	j=273; for( i=0; i<21; i++){ params.f14[i] = (double) f[j++]; }
	j=195; for( i=0; i<15; i++){ params.lb14[i] = (double) lb[j++]; }
	j=195; for( i=0; i<15; i++){ params.ub14[i] = (double) ub[j++]; }
	j=4641; for( i=0; i<357; i++){ params.C14[i] = (double) C[j++]; }
	j=221; for( i=0; i<17; i++){ params.d14[i] = (double) c[j++]; }
	j=6174; for( i=0; i<441; i++){ params.H15[i] = (double) H[j++]; }
	j=294; for( i=0; i<21; i++){ params.f15[i] = (double) f[j++]; }
	j=210; for( i=0; i<15; i++){ params.lb15[i] = (double) lb[j++]; }
	j=210; for( i=0; i<15; i++){ params.ub15[i] = (double) ub[j++]; }
	j=4998; for( i=0; i<357; i++){ params.C15[i] = (double) C[j++]; }
	j=238; for( i=0; i<17; i++){ params.d15[i] = (double) c[j++]; }
	j=6615; for( i=0; i<441; i++){ params.H16[i] = (double) H[j++]; }
	j=315; for( i=0; i<21; i++){ params.f16[i] = (double) f[j++]; }
	j=225; for( i=0; i<15; i++){ params.lb16[i] = (double) lb[j++]; }
	j=225; for( i=0; i<15; i++){ params.ub16[i] = (double) ub[j++]; }
	j=5355; for( i=0; i<357; i++){ params.C16[i] = (double) C[j++]; }
	j=255; for( i=0; i<17; i++){ params.d16[i] = (double) c[j++]; }
	j=7056; for( i=0; i<441; i++){ params.H17[i] = (double) H[j++]; }
	j=336; for( i=0; i<21; i++){ params.f17[i] = (double) f[j++]; }
	j=240; for( i=0; i<15; i++){ params.lb17[i] = (double) lb[j++]; }
	j=240; for( i=0; i<15; i++){ params.ub17[i] = (double) ub[j++]; }
	j=5712; for( i=0; i<357; i++){ params.C17[i] = (double) C[j++]; }
	j=272; for( i=0; i<17; i++){ params.d17[i] = (double) c[j++]; }
	j=7497; for( i=0; i<441; i++){ params.H18[i] = (double) H[j++]; }
	j=357; for( i=0; i<21; i++){ params.f18[i] = (double) f[j++]; }
	j=255; for( i=0; i<15; i++){ params.lb18[i] = (double) lb[j++]; }
	j=255; for( i=0; i<15; i++){ params.ub18[i] = (double) ub[j++]; }
	j=6069; for( i=0; i<357; i++){ params.C18[i] = (double) C[j++]; }
	j=289; for( i=0; i<17; i++){ params.d18[i] = (double) c[j++]; }
	j=7938; for( i=0; i<441; i++){ params.H19[i] = (double) H[j++]; }
	j=378; for( i=0; i<21; i++){ params.f19[i] = (double) f[j++]; }
	j=270; for( i=0; i<15; i++){ params.lb19[i] = (double) lb[j++]; }
	j=270; for( i=0; i<15; i++){ params.ub19[i] = (double) ub[j++]; }
	j=6426; for( i=0; i<357; i++){ params.C19[i] = (double) C[j++]; }
	j=306; for( i=0; i<17; i++){ params.d19[i] = (double) c[j++]; }
	j=8379; for( i=0; i<441; i++){ params.H20[i] = (double) H[j++]; }
	j=399; for( i=0; i<21; i++){ params.f20[i] = (double) f[j++]; }
	j=285; for( i=0; i<15; i++){ params.lb20[i] = (double) lb[j++]; }
	j=285; for( i=0; i<15; i++){ params.ub20[i] = (double) ub[j++]; }
	j=6783; for( i=0; i<357; i++){ params.C20[i] = (double) C[j++]; }
	j=323; for( i=0; i<17; i++){ params.d20[i] = (double) c[j++]; }
	j=8820; for( i=0; i<441; i++){ params.H21[i] = (double) H[j++]; }
	j=420; for( i=0; i<21; i++){ params.f21[i] = (double) f[j++]; }
	j=300; for( i=0; i<15; i++){ params.lb21[i] = (double) lb[j++]; }
	j=300; for( i=0; i<15; i++){ params.ub21[i] = (double) ub[j++]; }
	j=7140; for( i=0; i<357; i++){ params.C21[i] = (double) C[j++]; }
	j=340; for( i=0; i<17; i++){ params.d21[i] = (double) c[j++]; }
	j=9261; for( i=0; i<441; i++){ params.H22[i] = (double) H[j++]; }
	j=441; for( i=0; i<21; i++){ params.f22[i] = (double) f[j++]; }
	j=315; for( i=0; i<15; i++){ params.lb22[i] = (double) lb[j++]; }
	j=315; for( i=0; i<15; i++){ params.ub22[i] = (double) ub[j++]; }
	j=7497; for( i=0; i<357; i++){ params.C22[i] = (double) C[j++]; }
	j=357; for( i=0; i<17; i++){ params.d22[i] = (double) c[j++]; }
	j=9702; for( i=0; i<441; i++){ params.H23[i] = (double) H[j++]; }
	j=462; for( i=0; i<21; i++){ params.f23[i] = (double) f[j++]; }
	j=330; for( i=0; i<15; i++){ params.lb23[i] = (double) lb[j++]; }
	j=330; for( i=0; i<15; i++){ params.ub23[i] = (double) ub[j++]; }
	j=7854; for( i=0; i<357; i++){ params.C23[i] = (double) C[j++]; }
	j=374; for( i=0; i<17; i++){ params.d23[i] = (double) c[j++]; }
	j=10143; for( i=0; i<441; i++){ params.H24[i] = (double) H[j++]; }
	j=483; for( i=0; i<21; i++){ params.f24[i] = (double) f[j++]; }
	j=345; for( i=0; i<15; i++){ params.lb24[i] = (double) lb[j++]; }
	j=345; for( i=0; i<15; i++){ params.ub24[i] = (double) ub[j++]; }
	j=8211; for( i=0; i<357; i++){ params.C24[i] = (double) C[j++]; }
	j=391; for( i=0; i<17; i++){ params.d24[i] = (double) c[j++]; }
	j=10584; for( i=0; i<441; i++){ params.H25[i] = (double) H[j++]; }
	j=504; for( i=0; i<21; i++){ params.f25[i] = (double) f[j++]; }
	j=360; for( i=0; i<15; i++){ params.lb25[i] = (double) lb[j++]; }
	j=360; for( i=0; i<15; i++){ params.ub25[i] = (double) ub[j++]; }
	j=8568; for( i=0; i<357; i++){ params.C25[i] = (double) C[j++]; }
	j=408; for( i=0; i<17; i++){ params.d25[i] = (double) c[j++]; }
	j=11025; for( i=0; i<441; i++){ params.H26[i] = (double) H[j++]; }
	j=525; for( i=0; i<21; i++){ params.f26[i] = (double) f[j++]; }
	j=375; for( i=0; i<15; i++){ params.lb26[i] = (double) lb[j++]; }
	j=375; for( i=0; i<15; i++){ params.ub26[i] = (double) ub[j++]; }
	j=8925; for( i=0; i<357; i++){ params.C26[i] = (double) C[j++]; }
	j=425; for( i=0; i<17; i++){ params.d26[i] = (double) c[j++]; }
	j=11466; for( i=0; i<441; i++){ params.H27[i] = (double) H[j++]; }
	j=546; for( i=0; i<21; i++){ params.f27[i] = (double) f[j++]; }
	j=390; for( i=0; i<15; i++){ params.lb27[i] = (double) lb[j++]; }
	j=390; for( i=0; i<15; i++){ params.ub27[i] = (double) ub[j++]; }
	j=9282; for( i=0; i<357; i++){ params.C27[i] = (double) C[j++]; }
	j=442; for( i=0; i<17; i++){ params.d27[i] = (double) c[j++]; }
	j=11907; for( i=0; i<441; i++){ params.H28[i] = (double) H[j++]; }
	j=567; for( i=0; i<21; i++){ params.f28[i] = (double) f[j++]; }
	j=405; for( i=0; i<15; i++){ params.lb28[i] = (double) lb[j++]; }
	j=405; for( i=0; i<15; i++){ params.ub28[i] = (double) ub[j++]; }
	j=9639; for( i=0; i<357; i++){ params.C28[i] = (double) C[j++]; }
	j=459; for( i=0; i<17; i++){ params.d28[i] = (double) c[j++]; }
	j=12348; for( i=0; i<441; i++){ params.H29[i] = (double) H[j++]; }
	j=588; for( i=0; i<21; i++){ params.f29[i] = (double) f[j++]; }
	j=420; for( i=0; i<15; i++){ params.lb29[i] = (double) lb[j++]; }
	j=420; for( i=0; i<15; i++){ params.ub29[i] = (double) ub[j++]; }
	j=9996; for( i=0; i<357; i++){ params.C29[i] = (double) C[j++]; }
	j=476; for( i=0; i<17; i++){ params.d29[i] = (double) c[j++]; }
	j=12789; for( i=0; i<441; i++){ params.H30[i] = (double) H[j++]; }
	j=609; for( i=0; i<21; i++){ params.f30[i] = (double) f[j++]; }
	j=435; for( i=0; i<15; i++){ params.lb30[i] = (double) lb[j++]; }
	j=435; for( i=0; i<15; i++){ params.ub30[i] = (double) ub[j++]; }
	j=10353; for( i=0; i<357; i++){ params.C30[i] = (double) C[j++]; }
	j=493; for( i=0; i<17; i++){ params.d30[i] = (double) c[j++]; }
	for( i=0; i<289; i++){ params.H31[i] = (double) H31[i]; }
	for( i=0; i<17; i++){ params.f31[i] = (double) f31[i]; }
	for( i=0; i<11; i++){ params.lb31[i] = (double) lb31[i]; }
	for( i=0; i<11; i++){ params.ub31[i] = (double) ub31[i]; }
	j=510; for( i=0; i<17; i++){ params.d31[i] = (double) c[j++]; }
	for( i=0; i<21; i++){ params.z_init_00[i] = (double) z[i]; }
	j=21; for( i=0; i<21; i++){ params.z_init_01[i] = (double) z[j++]; }
	j=42; for( i=0; i<21; i++){ params.z_init_02[i] = (double) z[j++]; }
	j=63; for( i=0; i<21; i++){ params.z_init_03[i] = (double) z[j++]; }
	j=84; for( i=0; i<21; i++){ params.z_init_04[i] = (double) z[j++]; }
	j=105; for( i=0; i<21; i++){ params.z_init_05[i] = (double) z[j++]; }
	j=126; for( i=0; i<21; i++){ params.z_init_06[i] = (double) z[j++]; }
	j=147; for( i=0; i<21; i++){ params.z_init_07[i] = (double) z[j++]; }
	j=168; for( i=0; i<21; i++){ params.z_init_08[i] = (double) z[j++]; }
	j=189; for( i=0; i<21; i++){ params.z_init_09[i] = (double) z[j++]; }
	j=210; for( i=0; i<21; i++){ params.z_init_10[i] = (double) z[j++]; }
	j=231; for( i=0; i<21; i++){ params.z_init_11[i] = (double) z[j++]; }
	j=252; for( i=0; i<21; i++){ params.z_init_12[i] = (double) z[j++]; }
	j=273; for( i=0; i<21; i++){ params.z_init_13[i] = (double) z[j++]; }
	j=294; for( i=0; i<21; i++){ params.z_init_14[i] = (double) z[j++]; }
	j=315; for( i=0; i<21; i++){ params.z_init_15[i] = (double) z[j++]; }
	j=336; for( i=0; i<21; i++){ params.z_init_16[i] = (double) z[j++]; }
	j=357; for( i=0; i<21; i++){ params.z_init_17[i] = (double) z[j++]; }
	j=378; for( i=0; i<21; i++){ params.z_init_18[i] = (double) z[j++]; }
	j=399; for( i=0; i<21; i++){ params.z_init_19[i] = (double) z[j++]; }
	j=420; for( i=0; i<21; i++){ params.z_init_20[i] = (double) z[j++]; }
	j=441; for( i=0; i<21; i++){ params.z_init_21[i] = (double) z[j++]; }
	j=462; for( i=0; i<21; i++){ params.z_init_22[i] = (double) z[j++]; }
	j=483; for( i=0; i<21; i++){ params.z_init_23[i] = (double) z[j++]; }
	j=504; for( i=0; i<21; i++){ params.z_init_24[i] = (double) z[j++]; }
	j=525; for( i=0; i<21; i++){ params.z_init_25[i] = (double) z[j++]; }
	j=546; for( i=0; i<21; i++){ params.z_init_26[i] = (double) z[j++]; }
	j=567; for( i=0; i<21; i++){ params.z_init_27[i] = (double) z[j++]; }
	j=588; for( i=0; i<21; i++){ params.z_init_28[i] = (double) z[j++]; }
	j=609; for( i=0; i<21; i++){ params.z_init_29[i] = (double) z[j++]; }
	for( i=0; i<17; i++){ params.z_init_30[i] = (double) z_init_30[i]; }
	

	

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
	for( i=0; i<21; i++){ outputs[i] = (real_T) output.out1[i]; }
	k=21; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out2[i]; }
	k=42; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out3[i]; }
	k=63; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out4[i]; }
	k=84; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out5[i]; }
	k=105; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out6[i]; }
	k=126; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out7[i]; }
	k=147; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out8[i]; }
	k=168; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out9[i]; }
	k=189; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out10[i]; }
	k=210; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out11[i]; }
	k=231; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out12[i]; }
	k=252; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out13[i]; }
	k=273; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out14[i]; }
	k=294; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out15[i]; }
	k=315; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out16[i]; }
	k=336; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out17[i]; }
	k=357; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out18[i]; }
	k=378; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out19[i]; }
	k=399; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out20[i]; }
	k=420; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out21[i]; }
	k=441; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out22[i]; }
	k=462; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out23[i]; }
	k=483; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out24[i]; }
	k=504; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out25[i]; }
	k=525; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out26[i]; }
	k=546; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out27[i]; }
	k=567; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out28[i]; }
	k=588; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out29[i]; }
	k=609; for( i=0; i<21; i++){ outputs[k++] = (real_T) output.out30[i]; }
	k=630; for( i=0; i<17; i++){ outputs[k++] = (real_T) output.out31[i]; }
	
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


