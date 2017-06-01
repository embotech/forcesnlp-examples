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
#define forces_SET_MAXIT         (1941)	

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
    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H1[441];

    /* vector of size 21 */
    forces_FLOAT f1[21];

    /* vector of size 15 */
    forces_FLOAT lb1[15];

    /* vector of size 15 */
    forces_FLOAT ub1[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C1[357];

    /* vector of size 17 */
    forces_FLOAT d1[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H2[441];

    /* vector of size 21 */
    forces_FLOAT f2[21];

    /* vector of size 15 */
    forces_FLOAT lb2[15];

    /* vector of size 15 */
    forces_FLOAT ub2[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C2[357];

    /* vector of size 17 */
    forces_FLOAT d2[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H3[441];

    /* vector of size 21 */
    forces_FLOAT f3[21];

    /* vector of size 15 */
    forces_FLOAT lb3[15];

    /* vector of size 15 */
    forces_FLOAT ub3[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C3[357];

    /* vector of size 17 */
    forces_FLOAT d3[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H4[441];

    /* vector of size 21 */
    forces_FLOAT f4[21];

    /* vector of size 15 */
    forces_FLOAT lb4[15];

    /* vector of size 15 */
    forces_FLOAT ub4[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C4[357];

    /* vector of size 17 */
    forces_FLOAT d4[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H5[441];

    /* vector of size 21 */
    forces_FLOAT f5[21];

    /* vector of size 15 */
    forces_FLOAT lb5[15];

    /* vector of size 15 */
    forces_FLOAT ub5[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C5[357];

    /* vector of size 17 */
    forces_FLOAT d5[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H6[441];

    /* vector of size 21 */
    forces_FLOAT f6[21];

    /* vector of size 15 */
    forces_FLOAT lb6[15];

    /* vector of size 15 */
    forces_FLOAT ub6[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C6[357];

    /* vector of size 17 */
    forces_FLOAT d6[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H7[441];

    /* vector of size 21 */
    forces_FLOAT f7[21];

    /* vector of size 15 */
    forces_FLOAT lb7[15];

    /* vector of size 15 */
    forces_FLOAT ub7[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C7[357];

    /* vector of size 17 */
    forces_FLOAT d7[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H8[441];

    /* vector of size 21 */
    forces_FLOAT f8[21];

    /* vector of size 15 */
    forces_FLOAT lb8[15];

    /* vector of size 15 */
    forces_FLOAT ub8[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C8[357];

    /* vector of size 17 */
    forces_FLOAT d8[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H9[441];

    /* vector of size 21 */
    forces_FLOAT f9[21];

    /* vector of size 15 */
    forces_FLOAT lb9[15];

    /* vector of size 15 */
    forces_FLOAT ub9[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C9[357];

    /* vector of size 17 */
    forces_FLOAT d9[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H10[441];

    /* vector of size 21 */
    forces_FLOAT f10[21];

    /* vector of size 15 */
    forces_FLOAT lb10[15];

    /* vector of size 15 */
    forces_FLOAT ub10[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C10[357];

    /* vector of size 17 */
    forces_FLOAT d10[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H11[441];

    /* vector of size 21 */
    forces_FLOAT f11[21];

    /* vector of size 15 */
    forces_FLOAT lb11[15];

    /* vector of size 15 */
    forces_FLOAT ub11[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C11[357];

    /* vector of size 17 */
    forces_FLOAT d11[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H12[441];

    /* vector of size 21 */
    forces_FLOAT f12[21];

    /* vector of size 15 */
    forces_FLOAT lb12[15];

    /* vector of size 15 */
    forces_FLOAT ub12[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C12[357];

    /* vector of size 17 */
    forces_FLOAT d12[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H13[441];

    /* vector of size 21 */
    forces_FLOAT f13[21];

    /* vector of size 15 */
    forces_FLOAT lb13[15];

    /* vector of size 15 */
    forces_FLOAT ub13[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C13[357];

    /* vector of size 17 */
    forces_FLOAT d13[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H14[441];

    /* vector of size 21 */
    forces_FLOAT f14[21];

    /* vector of size 15 */
    forces_FLOAT lb14[15];

    /* vector of size 15 */
    forces_FLOAT ub14[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C14[357];

    /* vector of size 17 */
    forces_FLOAT d14[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H15[441];

    /* vector of size 21 */
    forces_FLOAT f15[21];

    /* vector of size 15 */
    forces_FLOAT lb15[15];

    /* vector of size 15 */
    forces_FLOAT ub15[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C15[357];

    /* vector of size 17 */
    forces_FLOAT d15[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H16[441];

    /* vector of size 21 */
    forces_FLOAT f16[21];

    /* vector of size 15 */
    forces_FLOAT lb16[15];

    /* vector of size 15 */
    forces_FLOAT ub16[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C16[357];

    /* vector of size 17 */
    forces_FLOAT d16[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H17[441];

    /* vector of size 21 */
    forces_FLOAT f17[21];

    /* vector of size 15 */
    forces_FLOAT lb17[15];

    /* vector of size 15 */
    forces_FLOAT ub17[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C17[357];

    /* vector of size 17 */
    forces_FLOAT d17[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H18[441];

    /* vector of size 21 */
    forces_FLOAT f18[21];

    /* vector of size 15 */
    forces_FLOAT lb18[15];

    /* vector of size 15 */
    forces_FLOAT ub18[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C18[357];

    /* vector of size 17 */
    forces_FLOAT d18[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H19[441];

    /* vector of size 21 */
    forces_FLOAT f19[21];

    /* vector of size 15 */
    forces_FLOAT lb19[15];

    /* vector of size 15 */
    forces_FLOAT ub19[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C19[357];

    /* vector of size 17 */
    forces_FLOAT d19[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H20[441];

    /* vector of size 21 */
    forces_FLOAT f20[21];

    /* vector of size 15 */
    forces_FLOAT lb20[15];

    /* vector of size 15 */
    forces_FLOAT ub20[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C20[357];

    /* vector of size 17 */
    forces_FLOAT d20[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H21[441];

    /* vector of size 21 */
    forces_FLOAT f21[21];

    /* vector of size 15 */
    forces_FLOAT lb21[15];

    /* vector of size 15 */
    forces_FLOAT ub21[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C21[357];

    /* vector of size 17 */
    forces_FLOAT d21[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H22[441];

    /* vector of size 21 */
    forces_FLOAT f22[21];

    /* vector of size 15 */
    forces_FLOAT lb22[15];

    /* vector of size 15 */
    forces_FLOAT ub22[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C22[357];

    /* vector of size 17 */
    forces_FLOAT d22[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H23[441];

    /* vector of size 21 */
    forces_FLOAT f23[21];

    /* vector of size 15 */
    forces_FLOAT lb23[15];

    /* vector of size 15 */
    forces_FLOAT ub23[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C23[357];

    /* vector of size 17 */
    forces_FLOAT d23[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H24[441];

    /* vector of size 21 */
    forces_FLOAT f24[21];

    /* vector of size 15 */
    forces_FLOAT lb24[15];

    /* vector of size 15 */
    forces_FLOAT ub24[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C24[357];

    /* vector of size 17 */
    forces_FLOAT d24[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H25[441];

    /* vector of size 21 */
    forces_FLOAT f25[21];

    /* vector of size 15 */
    forces_FLOAT lb25[15];

    /* vector of size 15 */
    forces_FLOAT ub25[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C25[357];

    /* vector of size 17 */
    forces_FLOAT d25[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H26[441];

    /* vector of size 21 */
    forces_FLOAT f26[21];

    /* vector of size 15 */
    forces_FLOAT lb26[15];

    /* vector of size 15 */
    forces_FLOAT ub26[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C26[357];

    /* vector of size 17 */
    forces_FLOAT d26[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H27[441];

    /* vector of size 21 */
    forces_FLOAT f27[21];

    /* vector of size 15 */
    forces_FLOAT lb27[15];

    /* vector of size 15 */
    forces_FLOAT ub27[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C27[357];

    /* vector of size 17 */
    forces_FLOAT d27[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H28[441];

    /* vector of size 21 */
    forces_FLOAT f28[21];

    /* vector of size 15 */
    forces_FLOAT lb28[15];

    /* vector of size 15 */
    forces_FLOAT ub28[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C28[357];

    /* vector of size 17 */
    forces_FLOAT d28[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H29[441];

    /* vector of size 21 */
    forces_FLOAT f29[21];

    /* vector of size 15 */
    forces_FLOAT lb29[15];

    /* vector of size 15 */
    forces_FLOAT ub29[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C29[357];

    /* vector of size 17 */
    forces_FLOAT d29[17];

    /* matrix of size [21 x 21] (column major format) */
    forces_FLOAT H30[441];

    /* vector of size 21 */
    forces_FLOAT f30[21];

    /* vector of size 15 */
    forces_FLOAT lb30[15];

    /* vector of size 15 */
    forces_FLOAT ub30[15];

    /* matrix of size [17 x 21] (column major format) */
    forces_FLOAT C30[357];

    /* vector of size 17 */
    forces_FLOAT d30[17];

    /* matrix of size [17 x 17] (column major format) */
    forces_FLOAT H31[289];

    /* vector of size 17 */
    forces_FLOAT f31[17];

    /* vector of size 11 */
    forces_FLOAT lb31[11];

    /* vector of size 11 */
    forces_FLOAT ub31[11];

    /* vector of size 17 */
    forces_FLOAT d31[17];

    /* vector of size 21 */
    forces_FLOAT z_init_00[21];

    /* vector of size 21 */
    forces_FLOAT z_init_01[21];

    /* vector of size 21 */
    forces_FLOAT z_init_02[21];

    /* vector of size 21 */
    forces_FLOAT z_init_03[21];

    /* vector of size 21 */
    forces_FLOAT z_init_04[21];

    /* vector of size 21 */
    forces_FLOAT z_init_05[21];

    /* vector of size 21 */
    forces_FLOAT z_init_06[21];

    /* vector of size 21 */
    forces_FLOAT z_init_07[21];

    /* vector of size 21 */
    forces_FLOAT z_init_08[21];

    /* vector of size 21 */
    forces_FLOAT z_init_09[21];

    /* vector of size 21 */
    forces_FLOAT z_init_10[21];

    /* vector of size 21 */
    forces_FLOAT z_init_11[21];

    /* vector of size 21 */
    forces_FLOAT z_init_12[21];

    /* vector of size 21 */
    forces_FLOAT z_init_13[21];

    /* vector of size 21 */
    forces_FLOAT z_init_14[21];

    /* vector of size 21 */
    forces_FLOAT z_init_15[21];

    /* vector of size 21 */
    forces_FLOAT z_init_16[21];

    /* vector of size 21 */
    forces_FLOAT z_init_17[21];

    /* vector of size 21 */
    forces_FLOAT z_init_18[21];

    /* vector of size 21 */
    forces_FLOAT z_init_19[21];

    /* vector of size 21 */
    forces_FLOAT z_init_20[21];

    /* vector of size 21 */
    forces_FLOAT z_init_21[21];

    /* vector of size 21 */
    forces_FLOAT z_init_22[21];

    /* vector of size 21 */
    forces_FLOAT z_init_23[21];

    /* vector of size 21 */
    forces_FLOAT z_init_24[21];

    /* vector of size 21 */
    forces_FLOAT z_init_25[21];

    /* vector of size 21 */
    forces_FLOAT z_init_26[21];

    /* vector of size 21 */
    forces_FLOAT z_init_27[21];

    /* vector of size 21 */
    forces_FLOAT z_init_28[21];

    /* vector of size 21 */
    forces_FLOAT z_init_29[21];

    /* vector of size 17 */
    forces_FLOAT z_init_30[17];

} forces_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct forces_output
{
    /* vector of size 21 */
    forces_FLOAT out1[21];

    /* vector of size 21 */
    forces_FLOAT out2[21];

    /* vector of size 21 */
    forces_FLOAT out3[21];

    /* vector of size 21 */
    forces_FLOAT out4[21];

    /* vector of size 21 */
    forces_FLOAT out5[21];

    /* vector of size 21 */
    forces_FLOAT out6[21];

    /* vector of size 21 */
    forces_FLOAT out7[21];

    /* vector of size 21 */
    forces_FLOAT out8[21];

    /* vector of size 21 */
    forces_FLOAT out9[21];

    /* vector of size 21 */
    forces_FLOAT out10[21];

    /* vector of size 21 */
    forces_FLOAT out11[21];

    /* vector of size 21 */
    forces_FLOAT out12[21];

    /* vector of size 21 */
    forces_FLOAT out13[21];

    /* vector of size 21 */
    forces_FLOAT out14[21];

    /* vector of size 21 */
    forces_FLOAT out15[21];

    /* vector of size 21 */
    forces_FLOAT out16[21];

    /* vector of size 21 */
    forces_FLOAT out17[21];

    /* vector of size 21 */
    forces_FLOAT out18[21];

    /* vector of size 21 */
    forces_FLOAT out19[21];

    /* vector of size 21 */
    forces_FLOAT out20[21];

    /* vector of size 21 */
    forces_FLOAT out21[21];

    /* vector of size 21 */
    forces_FLOAT out22[21];

    /* vector of size 21 */
    forces_FLOAT out23[21];

    /* vector of size 21 */
    forces_FLOAT out24[21];

    /* vector of size 21 */
    forces_FLOAT out25[21];

    /* vector of size 21 */
    forces_FLOAT out26[21];

    /* vector of size 21 */
    forces_FLOAT out27[21];

    /* vector of size 21 */
    forces_FLOAT out28[21];

    /* vector of size 21 */
    forces_FLOAT out29[21];

    /* vector of size 21 */
    forces_FLOAT out30[21];

    /* vector of size 17 */
    forces_FLOAT out31[17];

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