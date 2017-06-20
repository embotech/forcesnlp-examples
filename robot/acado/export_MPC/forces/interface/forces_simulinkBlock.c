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
#define S_FUNCTION_NAME forces_simulinkBlock

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

	/* initialize input ports - there are 146 in total */
    if (!ssSetNumInputPorts(S, 146)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 8, 8);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 8, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 8, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 8, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/	
	/* Input Port 4 */
    ssSetInputPortMatrixDimensions(S,  4, 6, 8);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 4, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 4, 1); /*direct input signal access*/	
	/* Input Port 5 */
    ssSetInputPortMatrixDimensions(S,  5, 6, 1);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 5, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 5, 1); /*direct input signal access*/	
	/* Input Port 6 */
    ssSetInputPortMatrixDimensions(S,  6, 8, 8);
    ssSetInputPortDataType(S, 6, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 6, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 6, 1); /*direct input signal access*/	
	/* Input Port 7 */
    ssSetInputPortMatrixDimensions(S,  7, 8, 1);
    ssSetInputPortDataType(S, 7, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 7, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 7, 1); /*direct input signal access*/	
	/* Input Port 8 */
    ssSetInputPortMatrixDimensions(S,  8, 8, 1);
    ssSetInputPortDataType(S, 8, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 8, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 8, 1); /*direct input signal access*/	
	/* Input Port 9 */
    ssSetInputPortMatrixDimensions(S,  9, 8, 1);
    ssSetInputPortDataType(S, 9, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 9, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 9, 1); /*direct input signal access*/	
	/* Input Port 10 */
    ssSetInputPortMatrixDimensions(S,  10, 6, 8);
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
	/* Input Port 12 */
    ssSetInputPortMatrixDimensions(S,  12, 8, 8);
    ssSetInputPortDataType(S, 12, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 12, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 12, 1); /*direct input signal access*/	
	/* Input Port 13 */
    ssSetInputPortMatrixDimensions(S,  13, 8, 1);
    ssSetInputPortDataType(S, 13, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 13, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 13, 1); /*direct input signal access*/	
	/* Input Port 14 */
    ssSetInputPortMatrixDimensions(S,  14, 8, 1);
    ssSetInputPortDataType(S, 14, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 14, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 14, 1); /*direct input signal access*/	
	/* Input Port 15 */
    ssSetInputPortMatrixDimensions(S,  15, 8, 1);
    ssSetInputPortDataType(S, 15, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 15, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 15, 1); /*direct input signal access*/	
	/* Input Port 16 */
    ssSetInputPortMatrixDimensions(S,  16, 6, 8);
    ssSetInputPortDataType(S, 16, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 16, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 16, 1); /*direct input signal access*/	
	/* Input Port 17 */
    ssSetInputPortMatrixDimensions(S,  17, 6, 1);
    ssSetInputPortDataType(S, 17, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 17, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 17, 1); /*direct input signal access*/	
	/* Input Port 18 */
    ssSetInputPortMatrixDimensions(S,  18, 8, 8);
    ssSetInputPortDataType(S, 18, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 18, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 18, 1); /*direct input signal access*/	
	/* Input Port 19 */
    ssSetInputPortMatrixDimensions(S,  19, 8, 1);
    ssSetInputPortDataType(S, 19, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 19, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 19, 1); /*direct input signal access*/	
	/* Input Port 20 */
    ssSetInputPortMatrixDimensions(S,  20, 8, 1);
    ssSetInputPortDataType(S, 20, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 20, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 20, 1); /*direct input signal access*/	
	/* Input Port 21 */
    ssSetInputPortMatrixDimensions(S,  21, 8, 1);
    ssSetInputPortDataType(S, 21, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 21, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 21, 1); /*direct input signal access*/	
	/* Input Port 22 */
    ssSetInputPortMatrixDimensions(S,  22, 6, 8);
    ssSetInputPortDataType(S, 22, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 22, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 22, 1); /*direct input signal access*/	
	/* Input Port 23 */
    ssSetInputPortMatrixDimensions(S,  23, 6, 1);
    ssSetInputPortDataType(S, 23, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 23, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 23, 1); /*direct input signal access*/	
	/* Input Port 24 */
    ssSetInputPortMatrixDimensions(S,  24, 8, 8);
    ssSetInputPortDataType(S, 24, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 24, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 24, 1); /*direct input signal access*/	
	/* Input Port 25 */
    ssSetInputPortMatrixDimensions(S,  25, 8, 1);
    ssSetInputPortDataType(S, 25, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 25, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 25, 1); /*direct input signal access*/	
	/* Input Port 26 */
    ssSetInputPortMatrixDimensions(S,  26, 8, 1);
    ssSetInputPortDataType(S, 26, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 26, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 26, 1); /*direct input signal access*/	
	/* Input Port 27 */
    ssSetInputPortMatrixDimensions(S,  27, 8, 1);
    ssSetInputPortDataType(S, 27, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 27, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 27, 1); /*direct input signal access*/	
	/* Input Port 28 */
    ssSetInputPortMatrixDimensions(S,  28, 6, 8);
    ssSetInputPortDataType(S, 28, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 28, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 28, 1); /*direct input signal access*/	
	/* Input Port 29 */
    ssSetInputPortMatrixDimensions(S,  29, 6, 1);
    ssSetInputPortDataType(S, 29, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 29, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 29, 1); /*direct input signal access*/	
	/* Input Port 30 */
    ssSetInputPortMatrixDimensions(S,  30, 8, 8);
    ssSetInputPortDataType(S, 30, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 30, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 30, 1); /*direct input signal access*/	
	/* Input Port 31 */
    ssSetInputPortMatrixDimensions(S,  31, 8, 1);
    ssSetInputPortDataType(S, 31, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 31, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 31, 1); /*direct input signal access*/	
	/* Input Port 32 */
    ssSetInputPortMatrixDimensions(S,  32, 8, 1);
    ssSetInputPortDataType(S, 32, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 32, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 32, 1); /*direct input signal access*/	
	/* Input Port 33 */
    ssSetInputPortMatrixDimensions(S,  33, 8, 1);
    ssSetInputPortDataType(S, 33, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 33, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 33, 1); /*direct input signal access*/	
	/* Input Port 34 */
    ssSetInputPortMatrixDimensions(S,  34, 6, 8);
    ssSetInputPortDataType(S, 34, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 34, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 34, 1); /*direct input signal access*/	
	/* Input Port 35 */
    ssSetInputPortMatrixDimensions(S,  35, 6, 1);
    ssSetInputPortDataType(S, 35, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 35, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 35, 1); /*direct input signal access*/	
	/* Input Port 36 */
    ssSetInputPortMatrixDimensions(S,  36, 8, 8);
    ssSetInputPortDataType(S, 36, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 36, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 36, 1); /*direct input signal access*/	
	/* Input Port 37 */
    ssSetInputPortMatrixDimensions(S,  37, 8, 1);
    ssSetInputPortDataType(S, 37, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 37, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 37, 1); /*direct input signal access*/	
	/* Input Port 38 */
    ssSetInputPortMatrixDimensions(S,  38, 8, 1);
    ssSetInputPortDataType(S, 38, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 38, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 38, 1); /*direct input signal access*/	
	/* Input Port 39 */
    ssSetInputPortMatrixDimensions(S,  39, 8, 1);
    ssSetInputPortDataType(S, 39, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 39, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 39, 1); /*direct input signal access*/	
	/* Input Port 40 */
    ssSetInputPortMatrixDimensions(S,  40, 6, 8);
    ssSetInputPortDataType(S, 40, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 40, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 40, 1); /*direct input signal access*/	
	/* Input Port 41 */
    ssSetInputPortMatrixDimensions(S,  41, 6, 1);
    ssSetInputPortDataType(S, 41, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 41, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 41, 1); /*direct input signal access*/	
	/* Input Port 42 */
    ssSetInputPortMatrixDimensions(S,  42, 8, 8);
    ssSetInputPortDataType(S, 42, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 42, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 42, 1); /*direct input signal access*/	
	/* Input Port 43 */
    ssSetInputPortMatrixDimensions(S,  43, 8, 1);
    ssSetInputPortDataType(S, 43, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 43, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 43, 1); /*direct input signal access*/	
	/* Input Port 44 */
    ssSetInputPortMatrixDimensions(S,  44, 8, 1);
    ssSetInputPortDataType(S, 44, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 44, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 44, 1); /*direct input signal access*/	
	/* Input Port 45 */
    ssSetInputPortMatrixDimensions(S,  45, 8, 1);
    ssSetInputPortDataType(S, 45, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 45, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 45, 1); /*direct input signal access*/	
	/* Input Port 46 */
    ssSetInputPortMatrixDimensions(S,  46, 6, 8);
    ssSetInputPortDataType(S, 46, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 46, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 46, 1); /*direct input signal access*/	
	/* Input Port 47 */
    ssSetInputPortMatrixDimensions(S,  47, 6, 1);
    ssSetInputPortDataType(S, 47, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 47, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 47, 1); /*direct input signal access*/	
	/* Input Port 48 */
    ssSetInputPortMatrixDimensions(S,  48, 8, 8);
    ssSetInputPortDataType(S, 48, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 48, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 48, 1); /*direct input signal access*/	
	/* Input Port 49 */
    ssSetInputPortMatrixDimensions(S,  49, 8, 1);
    ssSetInputPortDataType(S, 49, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 49, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 49, 1); /*direct input signal access*/	
	/* Input Port 50 */
    ssSetInputPortMatrixDimensions(S,  50, 8, 1);
    ssSetInputPortDataType(S, 50, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 50, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 50, 1); /*direct input signal access*/	
	/* Input Port 51 */
    ssSetInputPortMatrixDimensions(S,  51, 8, 1);
    ssSetInputPortDataType(S, 51, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 51, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 51, 1); /*direct input signal access*/	
	/* Input Port 52 */
    ssSetInputPortMatrixDimensions(S,  52, 6, 8);
    ssSetInputPortDataType(S, 52, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 52, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 52, 1); /*direct input signal access*/	
	/* Input Port 53 */
    ssSetInputPortMatrixDimensions(S,  53, 6, 1);
    ssSetInputPortDataType(S, 53, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 53, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 53, 1); /*direct input signal access*/	
	/* Input Port 54 */
    ssSetInputPortMatrixDimensions(S,  54, 8, 8);
    ssSetInputPortDataType(S, 54, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 54, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 54, 1); /*direct input signal access*/	
	/* Input Port 55 */
    ssSetInputPortMatrixDimensions(S,  55, 8, 1);
    ssSetInputPortDataType(S, 55, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 55, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 55, 1); /*direct input signal access*/	
	/* Input Port 56 */
    ssSetInputPortMatrixDimensions(S,  56, 8, 1);
    ssSetInputPortDataType(S, 56, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 56, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 56, 1); /*direct input signal access*/	
	/* Input Port 57 */
    ssSetInputPortMatrixDimensions(S,  57, 8, 1);
    ssSetInputPortDataType(S, 57, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 57, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 57, 1); /*direct input signal access*/	
	/* Input Port 58 */
    ssSetInputPortMatrixDimensions(S,  58, 6, 8);
    ssSetInputPortDataType(S, 58, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 58, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 58, 1); /*direct input signal access*/	
	/* Input Port 59 */
    ssSetInputPortMatrixDimensions(S,  59, 6, 1);
    ssSetInputPortDataType(S, 59, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 59, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 59, 1); /*direct input signal access*/	
	/* Input Port 60 */
    ssSetInputPortMatrixDimensions(S,  60, 8, 8);
    ssSetInputPortDataType(S, 60, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 60, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 60, 1); /*direct input signal access*/	
	/* Input Port 61 */
    ssSetInputPortMatrixDimensions(S,  61, 8, 1);
    ssSetInputPortDataType(S, 61, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 61, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 61, 1); /*direct input signal access*/	
	/* Input Port 62 */
    ssSetInputPortMatrixDimensions(S,  62, 8, 1);
    ssSetInputPortDataType(S, 62, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 62, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 62, 1); /*direct input signal access*/	
	/* Input Port 63 */
    ssSetInputPortMatrixDimensions(S,  63, 8, 1);
    ssSetInputPortDataType(S, 63, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 63, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 63, 1); /*direct input signal access*/	
	/* Input Port 64 */
    ssSetInputPortMatrixDimensions(S,  64, 6, 8);
    ssSetInputPortDataType(S, 64, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 64, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 64, 1); /*direct input signal access*/	
	/* Input Port 65 */
    ssSetInputPortMatrixDimensions(S,  65, 6, 1);
    ssSetInputPortDataType(S, 65, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 65, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 65, 1); /*direct input signal access*/	
	/* Input Port 66 */
    ssSetInputPortMatrixDimensions(S,  66, 8, 8);
    ssSetInputPortDataType(S, 66, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 66, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 66, 1); /*direct input signal access*/	
	/* Input Port 67 */
    ssSetInputPortMatrixDimensions(S,  67, 8, 1);
    ssSetInputPortDataType(S, 67, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 67, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 67, 1); /*direct input signal access*/	
	/* Input Port 68 */
    ssSetInputPortMatrixDimensions(S,  68, 8, 1);
    ssSetInputPortDataType(S, 68, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 68, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 68, 1); /*direct input signal access*/	
	/* Input Port 69 */
    ssSetInputPortMatrixDimensions(S,  69, 8, 1);
    ssSetInputPortDataType(S, 69, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 69, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 69, 1); /*direct input signal access*/	
	/* Input Port 70 */
    ssSetInputPortMatrixDimensions(S,  70, 6, 8);
    ssSetInputPortDataType(S, 70, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 70, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 70, 1); /*direct input signal access*/	
	/* Input Port 71 */
    ssSetInputPortMatrixDimensions(S,  71, 6, 1);
    ssSetInputPortDataType(S, 71, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 71, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 71, 1); /*direct input signal access*/	
	/* Input Port 72 */
    ssSetInputPortMatrixDimensions(S,  72, 8, 8);
    ssSetInputPortDataType(S, 72, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 72, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 72, 1); /*direct input signal access*/	
	/* Input Port 73 */
    ssSetInputPortMatrixDimensions(S,  73, 8, 1);
    ssSetInputPortDataType(S, 73, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 73, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 73, 1); /*direct input signal access*/	
	/* Input Port 74 */
    ssSetInputPortMatrixDimensions(S,  74, 8, 1);
    ssSetInputPortDataType(S, 74, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 74, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 74, 1); /*direct input signal access*/	
	/* Input Port 75 */
    ssSetInputPortMatrixDimensions(S,  75, 8, 1);
    ssSetInputPortDataType(S, 75, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 75, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 75, 1); /*direct input signal access*/	
	/* Input Port 76 */
    ssSetInputPortMatrixDimensions(S,  76, 6, 8);
    ssSetInputPortDataType(S, 76, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 76, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 76, 1); /*direct input signal access*/	
	/* Input Port 77 */
    ssSetInputPortMatrixDimensions(S,  77, 6, 1);
    ssSetInputPortDataType(S, 77, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 77, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 77, 1); /*direct input signal access*/	
	/* Input Port 78 */
    ssSetInputPortMatrixDimensions(S,  78, 8, 8);
    ssSetInputPortDataType(S, 78, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 78, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 78, 1); /*direct input signal access*/	
	/* Input Port 79 */
    ssSetInputPortMatrixDimensions(S,  79, 8, 1);
    ssSetInputPortDataType(S, 79, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 79, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 79, 1); /*direct input signal access*/	
	/* Input Port 80 */
    ssSetInputPortMatrixDimensions(S,  80, 8, 1);
    ssSetInputPortDataType(S, 80, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 80, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 80, 1); /*direct input signal access*/	
	/* Input Port 81 */
    ssSetInputPortMatrixDimensions(S,  81, 8, 1);
    ssSetInputPortDataType(S, 81, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 81, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 81, 1); /*direct input signal access*/	
	/* Input Port 82 */
    ssSetInputPortMatrixDimensions(S,  82, 6, 8);
    ssSetInputPortDataType(S, 82, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 82, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 82, 1); /*direct input signal access*/	
	/* Input Port 83 */
    ssSetInputPortMatrixDimensions(S,  83, 6, 1);
    ssSetInputPortDataType(S, 83, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 83, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 83, 1); /*direct input signal access*/	
	/* Input Port 84 */
    ssSetInputPortMatrixDimensions(S,  84, 8, 8);
    ssSetInputPortDataType(S, 84, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 84, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 84, 1); /*direct input signal access*/	
	/* Input Port 85 */
    ssSetInputPortMatrixDimensions(S,  85, 8, 1);
    ssSetInputPortDataType(S, 85, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 85, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 85, 1); /*direct input signal access*/	
	/* Input Port 86 */
    ssSetInputPortMatrixDimensions(S,  86, 8, 1);
    ssSetInputPortDataType(S, 86, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 86, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 86, 1); /*direct input signal access*/	
	/* Input Port 87 */
    ssSetInputPortMatrixDimensions(S,  87, 8, 1);
    ssSetInputPortDataType(S, 87, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 87, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 87, 1); /*direct input signal access*/	
	/* Input Port 88 */
    ssSetInputPortMatrixDimensions(S,  88, 6, 8);
    ssSetInputPortDataType(S, 88, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 88, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 88, 1); /*direct input signal access*/	
	/* Input Port 89 */
    ssSetInputPortMatrixDimensions(S,  89, 6, 1);
    ssSetInputPortDataType(S, 89, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 89, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 89, 1); /*direct input signal access*/	
	/* Input Port 90 */
    ssSetInputPortMatrixDimensions(S,  90, 8, 8);
    ssSetInputPortDataType(S, 90, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 90, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 90, 1); /*direct input signal access*/	
	/* Input Port 91 */
    ssSetInputPortMatrixDimensions(S,  91, 8, 1);
    ssSetInputPortDataType(S, 91, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 91, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 91, 1); /*direct input signal access*/	
	/* Input Port 92 */
    ssSetInputPortMatrixDimensions(S,  92, 8, 1);
    ssSetInputPortDataType(S, 92, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 92, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 92, 1); /*direct input signal access*/	
	/* Input Port 93 */
    ssSetInputPortMatrixDimensions(S,  93, 8, 1);
    ssSetInputPortDataType(S, 93, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 93, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 93, 1); /*direct input signal access*/	
	/* Input Port 94 */
    ssSetInputPortMatrixDimensions(S,  94, 6, 8);
    ssSetInputPortDataType(S, 94, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 94, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 94, 1); /*direct input signal access*/	
	/* Input Port 95 */
    ssSetInputPortMatrixDimensions(S,  95, 6, 1);
    ssSetInputPortDataType(S, 95, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 95, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 95, 1); /*direct input signal access*/	
	/* Input Port 96 */
    ssSetInputPortMatrixDimensions(S,  96, 8, 8);
    ssSetInputPortDataType(S, 96, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 96, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 96, 1); /*direct input signal access*/	
	/* Input Port 97 */
    ssSetInputPortMatrixDimensions(S,  97, 8, 1);
    ssSetInputPortDataType(S, 97, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 97, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 97, 1); /*direct input signal access*/	
	/* Input Port 98 */
    ssSetInputPortMatrixDimensions(S,  98, 8, 1);
    ssSetInputPortDataType(S, 98, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 98, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 98, 1); /*direct input signal access*/	
	/* Input Port 99 */
    ssSetInputPortMatrixDimensions(S,  99, 8, 1);
    ssSetInputPortDataType(S, 99, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 99, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 99, 1); /*direct input signal access*/	
	/* Input Port 100 */
    ssSetInputPortMatrixDimensions(S,  100, 6, 8);
    ssSetInputPortDataType(S, 100, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 100, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 100, 1); /*direct input signal access*/	
	/* Input Port 101 */
    ssSetInputPortMatrixDimensions(S,  101, 6, 1);
    ssSetInputPortDataType(S, 101, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 101, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 101, 1); /*direct input signal access*/	
	/* Input Port 102 */
    ssSetInputPortMatrixDimensions(S,  102, 8, 8);
    ssSetInputPortDataType(S, 102, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 102, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 102, 1); /*direct input signal access*/	
	/* Input Port 103 */
    ssSetInputPortMatrixDimensions(S,  103, 8, 1);
    ssSetInputPortDataType(S, 103, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 103, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 103, 1); /*direct input signal access*/	
	/* Input Port 104 */
    ssSetInputPortMatrixDimensions(S,  104, 8, 1);
    ssSetInputPortDataType(S, 104, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 104, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 104, 1); /*direct input signal access*/	
	/* Input Port 105 */
    ssSetInputPortMatrixDimensions(S,  105, 8, 1);
    ssSetInputPortDataType(S, 105, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 105, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 105, 1); /*direct input signal access*/	
	/* Input Port 106 */
    ssSetInputPortMatrixDimensions(S,  106, 6, 8);
    ssSetInputPortDataType(S, 106, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 106, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 106, 1); /*direct input signal access*/	
	/* Input Port 107 */
    ssSetInputPortMatrixDimensions(S,  107, 6, 1);
    ssSetInputPortDataType(S, 107, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 107, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 107, 1); /*direct input signal access*/	
	/* Input Port 108 */
    ssSetInputPortMatrixDimensions(S,  108, 8, 8);
    ssSetInputPortDataType(S, 108, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 108, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 108, 1); /*direct input signal access*/	
	/* Input Port 109 */
    ssSetInputPortMatrixDimensions(S,  109, 8, 1);
    ssSetInputPortDataType(S, 109, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 109, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 109, 1); /*direct input signal access*/	
	/* Input Port 110 */
    ssSetInputPortMatrixDimensions(S,  110, 8, 1);
    ssSetInputPortDataType(S, 110, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 110, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 110, 1); /*direct input signal access*/	
	/* Input Port 111 */
    ssSetInputPortMatrixDimensions(S,  111, 8, 1);
    ssSetInputPortDataType(S, 111, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 111, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 111, 1); /*direct input signal access*/	
	/* Input Port 112 */
    ssSetInputPortMatrixDimensions(S,  112, 6, 8);
    ssSetInputPortDataType(S, 112, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 112, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 112, 1); /*direct input signal access*/	
	/* Input Port 113 */
    ssSetInputPortMatrixDimensions(S,  113, 6, 1);
    ssSetInputPortDataType(S, 113, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 113, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 113, 1); /*direct input signal access*/	
	/* Input Port 114 */
    ssSetInputPortMatrixDimensions(S,  114, 8, 8);
    ssSetInputPortDataType(S, 114, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 114, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 114, 1); /*direct input signal access*/	
	/* Input Port 115 */
    ssSetInputPortMatrixDimensions(S,  115, 8, 1);
    ssSetInputPortDataType(S, 115, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 115, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 115, 1); /*direct input signal access*/	
	/* Input Port 116 */
    ssSetInputPortMatrixDimensions(S,  116, 8, 1);
    ssSetInputPortDataType(S, 116, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 116, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 116, 1); /*direct input signal access*/	
	/* Input Port 117 */
    ssSetInputPortMatrixDimensions(S,  117, 8, 1);
    ssSetInputPortDataType(S, 117, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 117, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 117, 1); /*direct input signal access*/	
	/* Input Port 118 */
    ssSetInputPortMatrixDimensions(S,  118, 6, 8);
    ssSetInputPortDataType(S, 118, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 118, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 118, 1); /*direct input signal access*/	
	/* Input Port 119 */
    ssSetInputPortMatrixDimensions(S,  119, 6, 1);
    ssSetInputPortDataType(S, 119, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 119, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 119, 1); /*direct input signal access*/	
	/* Input Port 120 */
    ssSetInputPortMatrixDimensions(S,  120, 6, 6);
    ssSetInputPortDataType(S, 120, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 120, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 120, 1); /*direct input signal access*/	
	/* Input Port 121 */
    ssSetInputPortMatrixDimensions(S,  121, 6, 1);
    ssSetInputPortDataType(S, 121, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 121, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 121, 1); /*direct input signal access*/	
	/* Input Port 122 */
    ssSetInputPortMatrixDimensions(S,  122, 6, 1);
    ssSetInputPortDataType(S, 122, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 122, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 122, 1); /*direct input signal access*/	
	/* Input Port 123 */
    ssSetInputPortMatrixDimensions(S,  123, 6, 1);
    ssSetInputPortDataType(S, 123, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 123, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 123, 1); /*direct input signal access*/	
	/* Input Port 124 */
    ssSetInputPortMatrixDimensions(S,  124, 6, 1);
    ssSetInputPortDataType(S, 124, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 124, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 124, 1); /*direct input signal access*/	
	/* Input Port 125 */
    ssSetInputPortMatrixDimensions(S,  125, 8, 1);
    ssSetInputPortDataType(S, 125, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 125, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 125, 1); /*direct input signal access*/	
	/* Input Port 126 */
    ssSetInputPortMatrixDimensions(S,  126, 8, 1);
    ssSetInputPortDataType(S, 126, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 126, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 126, 1); /*direct input signal access*/	
	/* Input Port 127 */
    ssSetInputPortMatrixDimensions(S,  127, 8, 1);
    ssSetInputPortDataType(S, 127, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 127, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 127, 1); /*direct input signal access*/	
	/* Input Port 128 */
    ssSetInputPortMatrixDimensions(S,  128, 8, 1);
    ssSetInputPortDataType(S, 128, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 128, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 128, 1); /*direct input signal access*/	
	/* Input Port 129 */
    ssSetInputPortMatrixDimensions(S,  129, 8, 1);
    ssSetInputPortDataType(S, 129, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 129, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 129, 1); /*direct input signal access*/	
	/* Input Port 130 */
    ssSetInputPortMatrixDimensions(S,  130, 8, 1);
    ssSetInputPortDataType(S, 130, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 130, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 130, 1); /*direct input signal access*/	
	/* Input Port 131 */
    ssSetInputPortMatrixDimensions(S,  131, 8, 1);
    ssSetInputPortDataType(S, 131, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 131, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 131, 1); /*direct input signal access*/	
	/* Input Port 132 */
    ssSetInputPortMatrixDimensions(S,  132, 8, 1);
    ssSetInputPortDataType(S, 132, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 132, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 132, 1); /*direct input signal access*/	
	/* Input Port 133 */
    ssSetInputPortMatrixDimensions(S,  133, 8, 1);
    ssSetInputPortDataType(S, 133, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 133, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 133, 1); /*direct input signal access*/	
	/* Input Port 134 */
    ssSetInputPortMatrixDimensions(S,  134, 8, 1);
    ssSetInputPortDataType(S, 134, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 134, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 134, 1); /*direct input signal access*/	
	/* Input Port 135 */
    ssSetInputPortMatrixDimensions(S,  135, 8, 1);
    ssSetInputPortDataType(S, 135, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 135, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 135, 1); /*direct input signal access*/	
	/* Input Port 136 */
    ssSetInputPortMatrixDimensions(S,  136, 8, 1);
    ssSetInputPortDataType(S, 136, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 136, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 136, 1); /*direct input signal access*/	
	/* Input Port 137 */
    ssSetInputPortMatrixDimensions(S,  137, 8, 1);
    ssSetInputPortDataType(S, 137, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 137, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 137, 1); /*direct input signal access*/	
	/* Input Port 138 */
    ssSetInputPortMatrixDimensions(S,  138, 8, 1);
    ssSetInputPortDataType(S, 138, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 138, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 138, 1); /*direct input signal access*/	
	/* Input Port 139 */
    ssSetInputPortMatrixDimensions(S,  139, 8, 1);
    ssSetInputPortDataType(S, 139, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 139, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 139, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 139, 1); /*direct input signal access*/	
	/* Input Port 140 */
    ssSetInputPortMatrixDimensions(S,  140, 8, 1);
    ssSetInputPortDataType(S, 140, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 140, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 140, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 140, 1); /*direct input signal access*/	
	/* Input Port 141 */
    ssSetInputPortMatrixDimensions(S,  141, 8, 1);
    ssSetInputPortDataType(S, 141, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 141, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 141, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 141, 1); /*direct input signal access*/	
	/* Input Port 142 */
    ssSetInputPortMatrixDimensions(S,  142, 8, 1);
    ssSetInputPortDataType(S, 142, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 142, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 142, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 142, 1); /*direct input signal access*/	
	/* Input Port 143 */
    ssSetInputPortMatrixDimensions(S,  143, 8, 1);
    ssSetInputPortDataType(S, 143, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 143, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 143, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 143, 1); /*direct input signal access*/	
	/* Input Port 144 */
    ssSetInputPortMatrixDimensions(S,  144, 8, 1);
    ssSetInputPortDataType(S, 144, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 144, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 144, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 144, 1); /*direct input signal access*/	
	/* Input Port 145 */
    ssSetInputPortMatrixDimensions(S,  145, 6, 1);
    ssSetInputPortDataType(S, 145, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 145, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 145, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 145, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 21 in total */
    if (!ssSetNumOutputPorts(S, 21)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 8, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 1 */
    ssSetOutputPortMatrixDimensions(S,  1, 8, 1);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 2 */
    ssSetOutputPortMatrixDimensions(S,  2, 8, 1);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 3 */
    ssSetOutputPortMatrixDimensions(S,  3, 8, 1);
    ssSetOutputPortDataType(S, 3, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 4 */
    ssSetOutputPortMatrixDimensions(S,  4, 8, 1);
    ssSetOutputPortDataType(S, 4, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 5 */
    ssSetOutputPortMatrixDimensions(S,  5, 8, 1);
    ssSetOutputPortDataType(S, 5, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 6 */
    ssSetOutputPortMatrixDimensions(S,  6, 8, 1);
    ssSetOutputPortDataType(S, 6, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 7 */
    ssSetOutputPortMatrixDimensions(S,  7, 8, 1);
    ssSetOutputPortDataType(S, 7, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 8 */
    ssSetOutputPortMatrixDimensions(S,  8, 8, 1);
    ssSetOutputPortDataType(S, 8, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 9 */
    ssSetOutputPortMatrixDimensions(S,  9, 8, 1);
    ssSetOutputPortDataType(S, 9, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 10 */
    ssSetOutputPortMatrixDimensions(S,  10, 8, 1);
    ssSetOutputPortDataType(S, 10, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 11 */
    ssSetOutputPortMatrixDimensions(S,  11, 8, 1);
    ssSetOutputPortDataType(S, 11, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 12 */
    ssSetOutputPortMatrixDimensions(S,  12, 8, 1);
    ssSetOutputPortDataType(S, 12, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 13 */
    ssSetOutputPortMatrixDimensions(S,  13, 8, 1);
    ssSetOutputPortDataType(S, 13, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 14 */
    ssSetOutputPortMatrixDimensions(S,  14, 8, 1);
    ssSetOutputPortDataType(S, 14, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 15 */
    ssSetOutputPortMatrixDimensions(S,  15, 8, 1);
    ssSetOutputPortDataType(S, 15, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 16 */
    ssSetOutputPortMatrixDimensions(S,  16, 8, 1);
    ssSetOutputPortDataType(S, 16, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 17 */
    ssSetOutputPortMatrixDimensions(S,  17, 8, 1);
    ssSetOutputPortDataType(S, 17, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 18 */
    ssSetOutputPortMatrixDimensions(S,  18, 8, 1);
    ssSetOutputPortDataType(S, 18, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 19 */
    ssSetOutputPortMatrixDimensions(S,  19, 8, 1);
    ssSetOutputPortDataType(S, 19, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 20 */
    ssSetOutputPortMatrixDimensions(S,  20, 6, 1);
    ssSetOutputPortDataType(S, 20, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */


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
	const real_T *H1 = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *f1 = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *lb1 = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *ub1 = (const real_T*) ssGetInputPortSignal(S,3);
	const real_T *C1 = (const real_T*) ssGetInputPortSignal(S,4);
	const real_T *d1 = (const real_T*) ssGetInputPortSignal(S,5);
	const real_T *H2 = (const real_T*) ssGetInputPortSignal(S,6);
	const real_T *f2 = (const real_T*) ssGetInputPortSignal(S,7);
	const real_T *lb2 = (const real_T*) ssGetInputPortSignal(S,8);
	const real_T *ub2 = (const real_T*) ssGetInputPortSignal(S,9);
	const real_T *C2 = (const real_T*) ssGetInputPortSignal(S,10);
	const real_T *d2 = (const real_T*) ssGetInputPortSignal(S,11);
	const real_T *H3 = (const real_T*) ssGetInputPortSignal(S,12);
	const real_T *f3 = (const real_T*) ssGetInputPortSignal(S,13);
	const real_T *lb3 = (const real_T*) ssGetInputPortSignal(S,14);
	const real_T *ub3 = (const real_T*) ssGetInputPortSignal(S,15);
	const real_T *C3 = (const real_T*) ssGetInputPortSignal(S,16);
	const real_T *d3 = (const real_T*) ssGetInputPortSignal(S,17);
	const real_T *H4 = (const real_T*) ssGetInputPortSignal(S,18);
	const real_T *f4 = (const real_T*) ssGetInputPortSignal(S,19);
	const real_T *lb4 = (const real_T*) ssGetInputPortSignal(S,20);
	const real_T *ub4 = (const real_T*) ssGetInputPortSignal(S,21);
	const real_T *C4 = (const real_T*) ssGetInputPortSignal(S,22);
	const real_T *d4 = (const real_T*) ssGetInputPortSignal(S,23);
	const real_T *H5 = (const real_T*) ssGetInputPortSignal(S,24);
	const real_T *f5 = (const real_T*) ssGetInputPortSignal(S,25);
	const real_T *lb5 = (const real_T*) ssGetInputPortSignal(S,26);
	const real_T *ub5 = (const real_T*) ssGetInputPortSignal(S,27);
	const real_T *C5 = (const real_T*) ssGetInputPortSignal(S,28);
	const real_T *d5 = (const real_T*) ssGetInputPortSignal(S,29);
	const real_T *H6 = (const real_T*) ssGetInputPortSignal(S,30);
	const real_T *f6 = (const real_T*) ssGetInputPortSignal(S,31);
	const real_T *lb6 = (const real_T*) ssGetInputPortSignal(S,32);
	const real_T *ub6 = (const real_T*) ssGetInputPortSignal(S,33);
	const real_T *C6 = (const real_T*) ssGetInputPortSignal(S,34);
	const real_T *d6 = (const real_T*) ssGetInputPortSignal(S,35);
	const real_T *H7 = (const real_T*) ssGetInputPortSignal(S,36);
	const real_T *f7 = (const real_T*) ssGetInputPortSignal(S,37);
	const real_T *lb7 = (const real_T*) ssGetInputPortSignal(S,38);
	const real_T *ub7 = (const real_T*) ssGetInputPortSignal(S,39);
	const real_T *C7 = (const real_T*) ssGetInputPortSignal(S,40);
	const real_T *d7 = (const real_T*) ssGetInputPortSignal(S,41);
	const real_T *H8 = (const real_T*) ssGetInputPortSignal(S,42);
	const real_T *f8 = (const real_T*) ssGetInputPortSignal(S,43);
	const real_T *lb8 = (const real_T*) ssGetInputPortSignal(S,44);
	const real_T *ub8 = (const real_T*) ssGetInputPortSignal(S,45);
	const real_T *C8 = (const real_T*) ssGetInputPortSignal(S,46);
	const real_T *d8 = (const real_T*) ssGetInputPortSignal(S,47);
	const real_T *H9 = (const real_T*) ssGetInputPortSignal(S,48);
	const real_T *f9 = (const real_T*) ssGetInputPortSignal(S,49);
	const real_T *lb9 = (const real_T*) ssGetInputPortSignal(S,50);
	const real_T *ub9 = (const real_T*) ssGetInputPortSignal(S,51);
	const real_T *C9 = (const real_T*) ssGetInputPortSignal(S,52);
	const real_T *d9 = (const real_T*) ssGetInputPortSignal(S,53);
	const real_T *H10 = (const real_T*) ssGetInputPortSignal(S,54);
	const real_T *f10 = (const real_T*) ssGetInputPortSignal(S,55);
	const real_T *lb10 = (const real_T*) ssGetInputPortSignal(S,56);
	const real_T *ub10 = (const real_T*) ssGetInputPortSignal(S,57);
	const real_T *C10 = (const real_T*) ssGetInputPortSignal(S,58);
	const real_T *d10 = (const real_T*) ssGetInputPortSignal(S,59);
	const real_T *H11 = (const real_T*) ssGetInputPortSignal(S,60);
	const real_T *f11 = (const real_T*) ssGetInputPortSignal(S,61);
	const real_T *lb11 = (const real_T*) ssGetInputPortSignal(S,62);
	const real_T *ub11 = (const real_T*) ssGetInputPortSignal(S,63);
	const real_T *C11 = (const real_T*) ssGetInputPortSignal(S,64);
	const real_T *d11 = (const real_T*) ssGetInputPortSignal(S,65);
	const real_T *H12 = (const real_T*) ssGetInputPortSignal(S,66);
	const real_T *f12 = (const real_T*) ssGetInputPortSignal(S,67);
	const real_T *lb12 = (const real_T*) ssGetInputPortSignal(S,68);
	const real_T *ub12 = (const real_T*) ssGetInputPortSignal(S,69);
	const real_T *C12 = (const real_T*) ssGetInputPortSignal(S,70);
	const real_T *d12 = (const real_T*) ssGetInputPortSignal(S,71);
	const real_T *H13 = (const real_T*) ssGetInputPortSignal(S,72);
	const real_T *f13 = (const real_T*) ssGetInputPortSignal(S,73);
	const real_T *lb13 = (const real_T*) ssGetInputPortSignal(S,74);
	const real_T *ub13 = (const real_T*) ssGetInputPortSignal(S,75);
	const real_T *C13 = (const real_T*) ssGetInputPortSignal(S,76);
	const real_T *d13 = (const real_T*) ssGetInputPortSignal(S,77);
	const real_T *H14 = (const real_T*) ssGetInputPortSignal(S,78);
	const real_T *f14 = (const real_T*) ssGetInputPortSignal(S,79);
	const real_T *lb14 = (const real_T*) ssGetInputPortSignal(S,80);
	const real_T *ub14 = (const real_T*) ssGetInputPortSignal(S,81);
	const real_T *C14 = (const real_T*) ssGetInputPortSignal(S,82);
	const real_T *d14 = (const real_T*) ssGetInputPortSignal(S,83);
	const real_T *H15 = (const real_T*) ssGetInputPortSignal(S,84);
	const real_T *f15 = (const real_T*) ssGetInputPortSignal(S,85);
	const real_T *lb15 = (const real_T*) ssGetInputPortSignal(S,86);
	const real_T *ub15 = (const real_T*) ssGetInputPortSignal(S,87);
	const real_T *C15 = (const real_T*) ssGetInputPortSignal(S,88);
	const real_T *d15 = (const real_T*) ssGetInputPortSignal(S,89);
	const real_T *H16 = (const real_T*) ssGetInputPortSignal(S,90);
	const real_T *f16 = (const real_T*) ssGetInputPortSignal(S,91);
	const real_T *lb16 = (const real_T*) ssGetInputPortSignal(S,92);
	const real_T *ub16 = (const real_T*) ssGetInputPortSignal(S,93);
	const real_T *C16 = (const real_T*) ssGetInputPortSignal(S,94);
	const real_T *d16 = (const real_T*) ssGetInputPortSignal(S,95);
	const real_T *H17 = (const real_T*) ssGetInputPortSignal(S,96);
	const real_T *f17 = (const real_T*) ssGetInputPortSignal(S,97);
	const real_T *lb17 = (const real_T*) ssGetInputPortSignal(S,98);
	const real_T *ub17 = (const real_T*) ssGetInputPortSignal(S,99);
	const real_T *C17 = (const real_T*) ssGetInputPortSignal(S,100);
	const real_T *d17 = (const real_T*) ssGetInputPortSignal(S,101);
	const real_T *H18 = (const real_T*) ssGetInputPortSignal(S,102);
	const real_T *f18 = (const real_T*) ssGetInputPortSignal(S,103);
	const real_T *lb18 = (const real_T*) ssGetInputPortSignal(S,104);
	const real_T *ub18 = (const real_T*) ssGetInputPortSignal(S,105);
	const real_T *C18 = (const real_T*) ssGetInputPortSignal(S,106);
	const real_T *d18 = (const real_T*) ssGetInputPortSignal(S,107);
	const real_T *H19 = (const real_T*) ssGetInputPortSignal(S,108);
	const real_T *f19 = (const real_T*) ssGetInputPortSignal(S,109);
	const real_T *lb19 = (const real_T*) ssGetInputPortSignal(S,110);
	const real_T *ub19 = (const real_T*) ssGetInputPortSignal(S,111);
	const real_T *C19 = (const real_T*) ssGetInputPortSignal(S,112);
	const real_T *d19 = (const real_T*) ssGetInputPortSignal(S,113);
	const real_T *H20 = (const real_T*) ssGetInputPortSignal(S,114);
	const real_T *f20 = (const real_T*) ssGetInputPortSignal(S,115);
	const real_T *lb20 = (const real_T*) ssGetInputPortSignal(S,116);
	const real_T *ub20 = (const real_T*) ssGetInputPortSignal(S,117);
	const real_T *C20 = (const real_T*) ssGetInputPortSignal(S,118);
	const real_T *d20 = (const real_T*) ssGetInputPortSignal(S,119);
	const real_T *H21 = (const real_T*) ssGetInputPortSignal(S,120);
	const real_T *f21 = (const real_T*) ssGetInputPortSignal(S,121);
	const real_T *lb21 = (const real_T*) ssGetInputPortSignal(S,122);
	const real_T *ub21 = (const real_T*) ssGetInputPortSignal(S,123);
	const real_T *d21 = (const real_T*) ssGetInputPortSignal(S,124);
	const real_T *z_init_00 = (const real_T*) ssGetInputPortSignal(S,125);
	const real_T *z_init_01 = (const real_T*) ssGetInputPortSignal(S,126);
	const real_T *z_init_02 = (const real_T*) ssGetInputPortSignal(S,127);
	const real_T *z_init_03 = (const real_T*) ssGetInputPortSignal(S,128);
	const real_T *z_init_04 = (const real_T*) ssGetInputPortSignal(S,129);
	const real_T *z_init_05 = (const real_T*) ssGetInputPortSignal(S,130);
	const real_T *z_init_06 = (const real_T*) ssGetInputPortSignal(S,131);
	const real_T *z_init_07 = (const real_T*) ssGetInputPortSignal(S,132);
	const real_T *z_init_08 = (const real_T*) ssGetInputPortSignal(S,133);
	const real_T *z_init_09 = (const real_T*) ssGetInputPortSignal(S,134);
	const real_T *z_init_10 = (const real_T*) ssGetInputPortSignal(S,135);
	const real_T *z_init_11 = (const real_T*) ssGetInputPortSignal(S,136);
	const real_T *z_init_12 = (const real_T*) ssGetInputPortSignal(S,137);
	const real_T *z_init_13 = (const real_T*) ssGetInputPortSignal(S,138);
	const real_T *z_init_14 = (const real_T*) ssGetInputPortSignal(S,139);
	const real_T *z_init_15 = (const real_T*) ssGetInputPortSignal(S,140);
	const real_T *z_init_16 = (const real_T*) ssGetInputPortSignal(S,141);
	const real_T *z_init_17 = (const real_T*) ssGetInputPortSignal(S,142);
	const real_T *z_init_18 = (const real_T*) ssGetInputPortSignal(S,143);
	const real_T *z_init_19 = (const real_T*) ssGetInputPortSignal(S,144);
	const real_T *z_init_20 = (const real_T*) ssGetInputPortSignal(S,145);
	
    real_T *out1 = (real_T*) ssGetOutputPortSignal(S,0);
	real_T *out2 = (real_T*) ssGetOutputPortSignal(S,1);
	real_T *out3 = (real_T*) ssGetOutputPortSignal(S,2);
	real_T *out4 = (real_T*) ssGetOutputPortSignal(S,3);
	real_T *out5 = (real_T*) ssGetOutputPortSignal(S,4);
	real_T *out6 = (real_T*) ssGetOutputPortSignal(S,5);
	real_T *out7 = (real_T*) ssGetOutputPortSignal(S,6);
	real_T *out8 = (real_T*) ssGetOutputPortSignal(S,7);
	real_T *out9 = (real_T*) ssGetOutputPortSignal(S,8);
	real_T *out10 = (real_T*) ssGetOutputPortSignal(S,9);
	real_T *out11 = (real_T*) ssGetOutputPortSignal(S,10);
	real_T *out12 = (real_T*) ssGetOutputPortSignal(S,11);
	real_T *out13 = (real_T*) ssGetOutputPortSignal(S,12);
	real_T *out14 = (real_T*) ssGetOutputPortSignal(S,13);
	real_T *out15 = (real_T*) ssGetOutputPortSignal(S,14);
	real_T *out16 = (real_T*) ssGetOutputPortSignal(S,15);
	real_T *out17 = (real_T*) ssGetOutputPortSignal(S,16);
	real_T *out18 = (real_T*) ssGetOutputPortSignal(S,17);
	real_T *out19 = (real_T*) ssGetOutputPortSignal(S,18);
	real_T *out20 = (real_T*) ssGetOutputPortSignal(S,19);
	real_T *out21 = (real_T*) ssGetOutputPortSignal(S,20);
	
	

	/* Solver data */
	forces_params params;
	forces_output output;
	forces_info info;	
	int exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<64; i++){ params.H1[i] = (double) H1[i]; }
	for( i=0; i<8; i++){ params.f1[i] = (double) f1[i]; }
	for( i=0; i<8; i++){ params.lb1[i] = (double) lb1[i]; }
	for( i=0; i<8; i++){ params.ub1[i] = (double) ub1[i]; }
	for( i=0; i<48; i++){ params.C1[i] = (double) C1[i]; }
	for( i=0; i<6; i++){ params.d1[i] = (double) d1[i]; }
	for( i=0; i<64; i++){ params.H2[i] = (double) H2[i]; }
	for( i=0; i<8; i++){ params.f2[i] = (double) f2[i]; }
	for( i=0; i<8; i++){ params.lb2[i] = (double) lb2[i]; }
	for( i=0; i<8; i++){ params.ub2[i] = (double) ub2[i]; }
	for( i=0; i<48; i++){ params.C2[i] = (double) C2[i]; }
	for( i=0; i<6; i++){ params.d2[i] = (double) d2[i]; }
	for( i=0; i<64; i++){ params.H3[i] = (double) H3[i]; }
	for( i=0; i<8; i++){ params.f3[i] = (double) f3[i]; }
	for( i=0; i<8; i++){ params.lb3[i] = (double) lb3[i]; }
	for( i=0; i<8; i++){ params.ub3[i] = (double) ub3[i]; }
	for( i=0; i<48; i++){ params.C3[i] = (double) C3[i]; }
	for( i=0; i<6; i++){ params.d3[i] = (double) d3[i]; }
	for( i=0; i<64; i++){ params.H4[i] = (double) H4[i]; }
	for( i=0; i<8; i++){ params.f4[i] = (double) f4[i]; }
	for( i=0; i<8; i++){ params.lb4[i] = (double) lb4[i]; }
	for( i=0; i<8; i++){ params.ub4[i] = (double) ub4[i]; }
	for( i=0; i<48; i++){ params.C4[i] = (double) C4[i]; }
	for( i=0; i<6; i++){ params.d4[i] = (double) d4[i]; }
	for( i=0; i<64; i++){ params.H5[i] = (double) H5[i]; }
	for( i=0; i<8; i++){ params.f5[i] = (double) f5[i]; }
	for( i=0; i<8; i++){ params.lb5[i] = (double) lb5[i]; }
	for( i=0; i<8; i++){ params.ub5[i] = (double) ub5[i]; }
	for( i=0; i<48; i++){ params.C5[i] = (double) C5[i]; }
	for( i=0; i<6; i++){ params.d5[i] = (double) d5[i]; }
	for( i=0; i<64; i++){ params.H6[i] = (double) H6[i]; }
	for( i=0; i<8; i++){ params.f6[i] = (double) f6[i]; }
	for( i=0; i<8; i++){ params.lb6[i] = (double) lb6[i]; }
	for( i=0; i<8; i++){ params.ub6[i] = (double) ub6[i]; }
	for( i=0; i<48; i++){ params.C6[i] = (double) C6[i]; }
	for( i=0; i<6; i++){ params.d6[i] = (double) d6[i]; }
	for( i=0; i<64; i++){ params.H7[i] = (double) H7[i]; }
	for( i=0; i<8; i++){ params.f7[i] = (double) f7[i]; }
	for( i=0; i<8; i++){ params.lb7[i] = (double) lb7[i]; }
	for( i=0; i<8; i++){ params.ub7[i] = (double) ub7[i]; }
	for( i=0; i<48; i++){ params.C7[i] = (double) C7[i]; }
	for( i=0; i<6; i++){ params.d7[i] = (double) d7[i]; }
	for( i=0; i<64; i++){ params.H8[i] = (double) H8[i]; }
	for( i=0; i<8; i++){ params.f8[i] = (double) f8[i]; }
	for( i=0; i<8; i++){ params.lb8[i] = (double) lb8[i]; }
	for( i=0; i<8; i++){ params.ub8[i] = (double) ub8[i]; }
	for( i=0; i<48; i++){ params.C8[i] = (double) C8[i]; }
	for( i=0; i<6; i++){ params.d8[i] = (double) d8[i]; }
	for( i=0; i<64; i++){ params.H9[i] = (double) H9[i]; }
	for( i=0; i<8; i++){ params.f9[i] = (double) f9[i]; }
	for( i=0; i<8; i++){ params.lb9[i] = (double) lb9[i]; }
	for( i=0; i<8; i++){ params.ub9[i] = (double) ub9[i]; }
	for( i=0; i<48; i++){ params.C9[i] = (double) C9[i]; }
	for( i=0; i<6; i++){ params.d9[i] = (double) d9[i]; }
	for( i=0; i<64; i++){ params.H10[i] = (double) H10[i]; }
	for( i=0; i<8; i++){ params.f10[i] = (double) f10[i]; }
	for( i=0; i<8; i++){ params.lb10[i] = (double) lb10[i]; }
	for( i=0; i<8; i++){ params.ub10[i] = (double) ub10[i]; }
	for( i=0; i<48; i++){ params.C10[i] = (double) C10[i]; }
	for( i=0; i<6; i++){ params.d10[i] = (double) d10[i]; }
	for( i=0; i<64; i++){ params.H11[i] = (double) H11[i]; }
	for( i=0; i<8; i++){ params.f11[i] = (double) f11[i]; }
	for( i=0; i<8; i++){ params.lb11[i] = (double) lb11[i]; }
	for( i=0; i<8; i++){ params.ub11[i] = (double) ub11[i]; }
	for( i=0; i<48; i++){ params.C11[i] = (double) C11[i]; }
	for( i=0; i<6; i++){ params.d11[i] = (double) d11[i]; }
	for( i=0; i<64; i++){ params.H12[i] = (double) H12[i]; }
	for( i=0; i<8; i++){ params.f12[i] = (double) f12[i]; }
	for( i=0; i<8; i++){ params.lb12[i] = (double) lb12[i]; }
	for( i=0; i<8; i++){ params.ub12[i] = (double) ub12[i]; }
	for( i=0; i<48; i++){ params.C12[i] = (double) C12[i]; }
	for( i=0; i<6; i++){ params.d12[i] = (double) d12[i]; }
	for( i=0; i<64; i++){ params.H13[i] = (double) H13[i]; }
	for( i=0; i<8; i++){ params.f13[i] = (double) f13[i]; }
	for( i=0; i<8; i++){ params.lb13[i] = (double) lb13[i]; }
	for( i=0; i<8; i++){ params.ub13[i] = (double) ub13[i]; }
	for( i=0; i<48; i++){ params.C13[i] = (double) C13[i]; }
	for( i=0; i<6; i++){ params.d13[i] = (double) d13[i]; }
	for( i=0; i<64; i++){ params.H14[i] = (double) H14[i]; }
	for( i=0; i<8; i++){ params.f14[i] = (double) f14[i]; }
	for( i=0; i<8; i++){ params.lb14[i] = (double) lb14[i]; }
	for( i=0; i<8; i++){ params.ub14[i] = (double) ub14[i]; }
	for( i=0; i<48; i++){ params.C14[i] = (double) C14[i]; }
	for( i=0; i<6; i++){ params.d14[i] = (double) d14[i]; }
	for( i=0; i<64; i++){ params.H15[i] = (double) H15[i]; }
	for( i=0; i<8; i++){ params.f15[i] = (double) f15[i]; }
	for( i=0; i<8; i++){ params.lb15[i] = (double) lb15[i]; }
	for( i=0; i<8; i++){ params.ub15[i] = (double) ub15[i]; }
	for( i=0; i<48; i++){ params.C15[i] = (double) C15[i]; }
	for( i=0; i<6; i++){ params.d15[i] = (double) d15[i]; }
	for( i=0; i<64; i++){ params.H16[i] = (double) H16[i]; }
	for( i=0; i<8; i++){ params.f16[i] = (double) f16[i]; }
	for( i=0; i<8; i++){ params.lb16[i] = (double) lb16[i]; }
	for( i=0; i<8; i++){ params.ub16[i] = (double) ub16[i]; }
	for( i=0; i<48; i++){ params.C16[i] = (double) C16[i]; }
	for( i=0; i<6; i++){ params.d16[i] = (double) d16[i]; }
	for( i=0; i<64; i++){ params.H17[i] = (double) H17[i]; }
	for( i=0; i<8; i++){ params.f17[i] = (double) f17[i]; }
	for( i=0; i<8; i++){ params.lb17[i] = (double) lb17[i]; }
	for( i=0; i<8; i++){ params.ub17[i] = (double) ub17[i]; }
	for( i=0; i<48; i++){ params.C17[i] = (double) C17[i]; }
	for( i=0; i<6; i++){ params.d17[i] = (double) d17[i]; }
	for( i=0; i<64; i++){ params.H18[i] = (double) H18[i]; }
	for( i=0; i<8; i++){ params.f18[i] = (double) f18[i]; }
	for( i=0; i<8; i++){ params.lb18[i] = (double) lb18[i]; }
	for( i=0; i<8; i++){ params.ub18[i] = (double) ub18[i]; }
	for( i=0; i<48; i++){ params.C18[i] = (double) C18[i]; }
	for( i=0; i<6; i++){ params.d18[i] = (double) d18[i]; }
	for( i=0; i<64; i++){ params.H19[i] = (double) H19[i]; }
	for( i=0; i<8; i++){ params.f19[i] = (double) f19[i]; }
	for( i=0; i<8; i++){ params.lb19[i] = (double) lb19[i]; }
	for( i=0; i<8; i++){ params.ub19[i] = (double) ub19[i]; }
	for( i=0; i<48; i++){ params.C19[i] = (double) C19[i]; }
	for( i=0; i<6; i++){ params.d19[i] = (double) d19[i]; }
	for( i=0; i<64; i++){ params.H20[i] = (double) H20[i]; }
	for( i=0; i<8; i++){ params.f20[i] = (double) f20[i]; }
	for( i=0; i<8; i++){ params.lb20[i] = (double) lb20[i]; }
	for( i=0; i<8; i++){ params.ub20[i] = (double) ub20[i]; }
	for( i=0; i<48; i++){ params.C20[i] = (double) C20[i]; }
	for( i=0; i<6; i++){ params.d20[i] = (double) d20[i]; }
	for( i=0; i<36; i++){ params.H21[i] = (double) H21[i]; }
	for( i=0; i<6; i++){ params.f21[i] = (double) f21[i]; }
	for( i=0; i<6; i++){ params.lb21[i] = (double) lb21[i]; }
	for( i=0; i<6; i++){ params.ub21[i] = (double) ub21[i]; }
	for( i=0; i<6; i++){ params.d21[i] = (double) d21[i]; }
	for( i=0; i<8; i++){ params.z_init_00[i] = (double) z_init_00[i]; }
	for( i=0; i<8; i++){ params.z_init_01[i] = (double) z_init_01[i]; }
	for( i=0; i<8; i++){ params.z_init_02[i] = (double) z_init_02[i]; }
	for( i=0; i<8; i++){ params.z_init_03[i] = (double) z_init_03[i]; }
	for( i=0; i<8; i++){ params.z_init_04[i] = (double) z_init_04[i]; }
	for( i=0; i<8; i++){ params.z_init_05[i] = (double) z_init_05[i]; }
	for( i=0; i<8; i++){ params.z_init_06[i] = (double) z_init_06[i]; }
	for( i=0; i<8; i++){ params.z_init_07[i] = (double) z_init_07[i]; }
	for( i=0; i<8; i++){ params.z_init_08[i] = (double) z_init_08[i]; }
	for( i=0; i<8; i++){ params.z_init_09[i] = (double) z_init_09[i]; }
	for( i=0; i<8; i++){ params.z_init_10[i] = (double) z_init_10[i]; }
	for( i=0; i<8; i++){ params.z_init_11[i] = (double) z_init_11[i]; }
	for( i=0; i<8; i++){ params.z_init_12[i] = (double) z_init_12[i]; }
	for( i=0; i<8; i++){ params.z_init_13[i] = (double) z_init_13[i]; }
	for( i=0; i<8; i++){ params.z_init_14[i] = (double) z_init_14[i]; }
	for( i=0; i<8; i++){ params.z_init_15[i] = (double) z_init_15[i]; }
	for( i=0; i<8; i++){ params.z_init_16[i] = (double) z_init_16[i]; }
	for( i=0; i<8; i++){ params.z_init_17[i] = (double) z_init_17[i]; }
	for( i=0; i<8; i++){ params.z_init_18[i] = (double) z_init_18[i]; }
	for( i=0; i<8; i++){ params.z_init_19[i] = (double) z_init_19[i]; }
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
	for( i=0; i<8; i++){ out1[i] = (real_T) output.out1[i]; }
	for( i=0; i<8; i++){ out2[i] = (real_T) output.out2[i]; }
	for( i=0; i<8; i++){ out3[i] = (real_T) output.out3[i]; }
	for( i=0; i<8; i++){ out4[i] = (real_T) output.out4[i]; }
	for( i=0; i<8; i++){ out5[i] = (real_T) output.out5[i]; }
	for( i=0; i<8; i++){ out6[i] = (real_T) output.out6[i]; }
	for( i=0; i<8; i++){ out7[i] = (real_T) output.out7[i]; }
	for( i=0; i<8; i++){ out8[i] = (real_T) output.out8[i]; }
	for( i=0; i<8; i++){ out9[i] = (real_T) output.out9[i]; }
	for( i=0; i<8; i++){ out10[i] = (real_T) output.out10[i]; }
	for( i=0; i<8; i++){ out11[i] = (real_T) output.out11[i]; }
	for( i=0; i<8; i++){ out12[i] = (real_T) output.out12[i]; }
	for( i=0; i<8; i++){ out13[i] = (real_T) output.out13[i]; }
	for( i=0; i<8; i++){ out14[i] = (real_T) output.out14[i]; }
	for( i=0; i<8; i++){ out15[i] = (real_T) output.out15[i]; }
	for( i=0; i<8; i++){ out16[i] = (real_T) output.out16[i]; }
	for( i=0; i<8; i++){ out17[i] = (real_T) output.out17[i]; }
	for( i=0; i<8; i++){ out18[i] = (real_T) output.out18[i]; }
	for( i=0; i<8; i++){ out19[i] = (real_T) output.out19[i]; }
	for( i=0; i<8; i++){ out20[i] = (real_T) output.out20[i]; }
	for( i=0; i<6; i++){ out21[i] = (real_T) output.out21[i]; }
	
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


