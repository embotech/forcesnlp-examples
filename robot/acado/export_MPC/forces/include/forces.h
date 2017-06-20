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

#include <stdio.h>

#ifndef __forces_H__
#define __forces_H__

/* DATA TYPE ------------------------------------------------------------*/
typedef double forces_FLOAT;

typedef double forcesINTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef forces_SET_PRINTLEVEL
#define forces_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef forces_SET_TIMING
#define forces_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define forces_SET_MAXIT         (498)	

/* scaling factor of line search (affine direction) */
#define forces_SET_LS_SCALE_AFF  (forces_FLOAT)(0.9)      

/* scaling factor of line search (combined direction) */
#define forces_SET_LS_SCALE      (forces_FLOAT)(0.95)  

/* minimum required step size in each iteration */
#define forces_SET_LS_MINSTEP    (forces_FLOAT)(1E-08)

/* maximum step size (combined direction) */
#define forces_SET_LS_MAXSTEP    (forces_FLOAT)(0.995)

/* desired relative duality gap */
#define forces_SET_ACC_RDGAP     (forces_FLOAT)(0.0001)

/* desired maximum residual on equality constraints */
#define forces_SET_ACC_RESEQ     (forces_FLOAT)(1E-06)

/* desired maximum residual on inequality constraints */
#define forces_SET_ACC_RESINEQ   (forces_FLOAT)(1E-06)

/* desired maximum violation of complementarity */
#define forces_SET_ACC_KKTCOMPL  (forces_FLOAT)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define forces_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define forces_MAXITREACHED (0)

/* no progress in line search possible */
#define forces_NOPROGRESS   (-7)

/* fatal internal error - nans occurring */
#define forces_NAN  (-10)


/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct forces_params
{
    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H1[64];

    /* vector of size 8 */
    forces_FLOAT f1[8];

    /* vector of size 8 */
    forces_FLOAT lb1[8];

    /* vector of size 8 */
    forces_FLOAT ub1[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C1[48];

    /* vector of size 6 */
    forces_FLOAT d1[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H2[64];

    /* vector of size 8 */
    forces_FLOAT f2[8];

    /* vector of size 8 */
    forces_FLOAT lb2[8];

    /* vector of size 8 */
    forces_FLOAT ub2[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C2[48];

    /* vector of size 6 */
    forces_FLOAT d2[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H3[64];

    /* vector of size 8 */
    forces_FLOAT f3[8];

    /* vector of size 8 */
    forces_FLOAT lb3[8];

    /* vector of size 8 */
    forces_FLOAT ub3[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C3[48];

    /* vector of size 6 */
    forces_FLOAT d3[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H4[64];

    /* vector of size 8 */
    forces_FLOAT f4[8];

    /* vector of size 8 */
    forces_FLOAT lb4[8];

    /* vector of size 8 */
    forces_FLOAT ub4[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C4[48];

    /* vector of size 6 */
    forces_FLOAT d4[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H5[64];

    /* vector of size 8 */
    forces_FLOAT f5[8];

    /* vector of size 8 */
    forces_FLOAT lb5[8];

    /* vector of size 8 */
    forces_FLOAT ub5[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C5[48];

    /* vector of size 6 */
    forces_FLOAT d5[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H6[64];

    /* vector of size 8 */
    forces_FLOAT f6[8];

    /* vector of size 8 */
    forces_FLOAT lb6[8];

    /* vector of size 8 */
    forces_FLOAT ub6[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C6[48];

    /* vector of size 6 */
    forces_FLOAT d6[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H7[64];

    /* vector of size 8 */
    forces_FLOAT f7[8];

    /* vector of size 8 */
    forces_FLOAT lb7[8];

    /* vector of size 8 */
    forces_FLOAT ub7[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C7[48];

    /* vector of size 6 */
    forces_FLOAT d7[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H8[64];

    /* vector of size 8 */
    forces_FLOAT f8[8];

    /* vector of size 8 */
    forces_FLOAT lb8[8];

    /* vector of size 8 */
    forces_FLOAT ub8[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C8[48];

    /* vector of size 6 */
    forces_FLOAT d8[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H9[64];

    /* vector of size 8 */
    forces_FLOAT f9[8];

    /* vector of size 8 */
    forces_FLOAT lb9[8];

    /* vector of size 8 */
    forces_FLOAT ub9[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C9[48];

    /* vector of size 6 */
    forces_FLOAT d9[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H10[64];

    /* vector of size 8 */
    forces_FLOAT f10[8];

    /* vector of size 8 */
    forces_FLOAT lb10[8];

    /* vector of size 8 */
    forces_FLOAT ub10[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C10[48];

    /* vector of size 6 */
    forces_FLOAT d10[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H11[64];

    /* vector of size 8 */
    forces_FLOAT f11[8];

    /* vector of size 8 */
    forces_FLOAT lb11[8];

    /* vector of size 8 */
    forces_FLOAT ub11[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C11[48];

    /* vector of size 6 */
    forces_FLOAT d11[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H12[64];

    /* vector of size 8 */
    forces_FLOAT f12[8];

    /* vector of size 8 */
    forces_FLOAT lb12[8];

    /* vector of size 8 */
    forces_FLOAT ub12[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C12[48];

    /* vector of size 6 */
    forces_FLOAT d12[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H13[64];

    /* vector of size 8 */
    forces_FLOAT f13[8];

    /* vector of size 8 */
    forces_FLOAT lb13[8];

    /* vector of size 8 */
    forces_FLOAT ub13[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C13[48];

    /* vector of size 6 */
    forces_FLOAT d13[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H14[64];

    /* vector of size 8 */
    forces_FLOAT f14[8];

    /* vector of size 8 */
    forces_FLOAT lb14[8];

    /* vector of size 8 */
    forces_FLOAT ub14[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C14[48];

    /* vector of size 6 */
    forces_FLOAT d14[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H15[64];

    /* vector of size 8 */
    forces_FLOAT f15[8];

    /* vector of size 8 */
    forces_FLOAT lb15[8];

    /* vector of size 8 */
    forces_FLOAT ub15[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C15[48];

    /* vector of size 6 */
    forces_FLOAT d15[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H16[64];

    /* vector of size 8 */
    forces_FLOAT f16[8];

    /* vector of size 8 */
    forces_FLOAT lb16[8];

    /* vector of size 8 */
    forces_FLOAT ub16[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C16[48];

    /* vector of size 6 */
    forces_FLOAT d16[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H17[64];

    /* vector of size 8 */
    forces_FLOAT f17[8];

    /* vector of size 8 */
    forces_FLOAT lb17[8];

    /* vector of size 8 */
    forces_FLOAT ub17[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C17[48];

    /* vector of size 6 */
    forces_FLOAT d17[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H18[64];

    /* vector of size 8 */
    forces_FLOAT f18[8];

    /* vector of size 8 */
    forces_FLOAT lb18[8];

    /* vector of size 8 */
    forces_FLOAT ub18[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C18[48];

    /* vector of size 6 */
    forces_FLOAT d18[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H19[64];

    /* vector of size 8 */
    forces_FLOAT f19[8];

    /* vector of size 8 */
    forces_FLOAT lb19[8];

    /* vector of size 8 */
    forces_FLOAT ub19[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C19[48];

    /* vector of size 6 */
    forces_FLOAT d19[6];

    /* matrix of size [8 x 8] (column major format) */
    forces_FLOAT H20[64];

    /* vector of size 8 */
    forces_FLOAT f20[8];

    /* vector of size 8 */
    forces_FLOAT lb20[8];

    /* vector of size 8 */
    forces_FLOAT ub20[8];

    /* matrix of size [6 x 8] (column major format) */
    forces_FLOAT C20[48];

    /* vector of size 6 */
    forces_FLOAT d20[6];

    /* matrix of size [6 x 6] (column major format) */
    forces_FLOAT H21[36];

    /* vector of size 6 */
    forces_FLOAT f21[6];

    /* vector of size 6 */
    forces_FLOAT lb21[6];

    /* vector of size 6 */
    forces_FLOAT ub21[6];

    /* vector of size 6 */
    forces_FLOAT d21[6];

    /* vector of size 8 */
    forces_FLOAT z_init_00[8];

    /* vector of size 8 */
    forces_FLOAT z_init_01[8];

    /* vector of size 8 */
    forces_FLOAT z_init_02[8];

    /* vector of size 8 */
    forces_FLOAT z_init_03[8];

    /* vector of size 8 */
    forces_FLOAT z_init_04[8];

    /* vector of size 8 */
    forces_FLOAT z_init_05[8];

    /* vector of size 8 */
    forces_FLOAT z_init_06[8];

    /* vector of size 8 */
    forces_FLOAT z_init_07[8];

    /* vector of size 8 */
    forces_FLOAT z_init_08[8];

    /* vector of size 8 */
    forces_FLOAT z_init_09[8];

    /* vector of size 8 */
    forces_FLOAT z_init_10[8];

    /* vector of size 8 */
    forces_FLOAT z_init_11[8];

    /* vector of size 8 */
    forces_FLOAT z_init_12[8];

    /* vector of size 8 */
    forces_FLOAT z_init_13[8];

    /* vector of size 8 */
    forces_FLOAT z_init_14[8];

    /* vector of size 8 */
    forces_FLOAT z_init_15[8];

    /* vector of size 8 */
    forces_FLOAT z_init_16[8];

    /* vector of size 8 */
    forces_FLOAT z_init_17[8];

    /* vector of size 8 */
    forces_FLOAT z_init_18[8];

    /* vector of size 8 */
    forces_FLOAT z_init_19[8];

    /* vector of size 6 */
    forces_FLOAT z_init_20[6];

} forces_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct forces_output
{
    /* vector of size 8 */
    forces_FLOAT out1[8];

    /* vector of size 8 */
    forces_FLOAT out2[8];

    /* vector of size 8 */
    forces_FLOAT out3[8];

    /* vector of size 8 */
    forces_FLOAT out4[8];

    /* vector of size 8 */
    forces_FLOAT out5[8];

    /* vector of size 8 */
    forces_FLOAT out6[8];

    /* vector of size 8 */
    forces_FLOAT out7[8];

    /* vector of size 8 */
    forces_FLOAT out8[8];

    /* vector of size 8 */
    forces_FLOAT out9[8];

    /* vector of size 8 */
    forces_FLOAT out10[8];

    /* vector of size 8 */
    forces_FLOAT out11[8];

    /* vector of size 8 */
    forces_FLOAT out12[8];

    /* vector of size 8 */
    forces_FLOAT out13[8];

    /* vector of size 8 */
    forces_FLOAT out14[8];

    /* vector of size 8 */
    forces_FLOAT out15[8];

    /* vector of size 8 */
    forces_FLOAT out16[8];

    /* vector of size 8 */
    forces_FLOAT out17[8];

    /* vector of size 8 */
    forces_FLOAT out18[8];

    /* vector of size 8 */
    forces_FLOAT out19[8];

    /* vector of size 8 */
    forces_FLOAT out20[8];

    /* vector of size 6 */
    forces_FLOAT out21[6];

} forces_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct forces_info
{
    /* iteration number */
    int it;

	/* number of iterations needed to optimality (branch-and-bound) */
	int it2opt;
	
    /* inf-norm of equality constraint residuals */
    forces_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    forces_FLOAT res_ineq;

    /* primal objective */
    forces_FLOAT pobj;	
	
    /* dual objective */
    forces_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    forces_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    forces_FLOAT rdgap;		

    /* duality measure */
    forces_FLOAT mu;

	/* duality measure (after affine step) */
    forces_FLOAT mu_aff;
	
    /* centering parameter */
    forces_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    forces_FLOAT step_aff;
    
    /* step size (combined direction) */
    forces_FLOAT step_cc;    

	/* solvertime */
	forces_FLOAT solvetime;   

} forces_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
#ifdef _cplusplus
extern "C" {
#endif
int forces_solve(forces_params* params, forces_output* output, forces_info* info, FILE* fs);

#ifdef _cplusplus
}
#endif

#endif