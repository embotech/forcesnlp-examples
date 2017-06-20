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

#include "mex.h"
#include "math.h"
#include "../include/forces.h"
#include <stdio.h>



/* copy functions */
void copyCArrayToM(double *src, double *dest, int dim) {
    while (dim--) {
        *dest++ = (double)*src++;
    }
}
void copyMArrayToC(double *src, double *dest, int dim) {
    while (dim--) {
        *dest++ = (double) (*src++) ;
    }
}




/* Some memory for mex-function */
forces_params params;
forces_output output;
forces_info info;

/* THE mex-function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0];
	double *pvalue;
	int i;
	int exitflag;
	const char *fname;
	const char *outputnames[31] = {"out1","out2","out3","out4","out5","out6","out7","out8","out9","out10","out11","out12","out13","out14","out15","out16","out17","out18","out19","out20","out21","out22","out23","out24","out25","out26","out27","out28","out29","out30","out31"};
	const char *infofields[16] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "dobj",  "dgap", "rdgap",  "mu",  "mu_aff",  "sigma",  "lsit_aff",  "lsit_cc",  "step_aff",   "step_cc",  "solvetime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1) {
        mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help forces_mex' for details.");
    }    
	if (nlhs > 3) {
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help forces_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) {
		mexErrMsgTxt("PARAMS must be a structure.");
	}

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "H1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H1 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H1 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H1, 441);

	par = mxGetField(PARAMS, 0, "f1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f1 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f1 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f1, 21);

	par = mxGetField(PARAMS, 0, "lb1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb1 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb1 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb1, 15);

	par = mxGetField(PARAMS, 0, "ub1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub1 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub1 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub1, 15);

	par = mxGetField(PARAMS, 0, "C1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C1 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C1 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C1, 357);

	par = mxGetField(PARAMS, 0, "d1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d1 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d1 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d1, 17);

	par = mxGetField(PARAMS, 0, "H2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H2 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H2 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H2, 441);

	par = mxGetField(PARAMS, 0, "f2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f2 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f2 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f2, 21);

	par = mxGetField(PARAMS, 0, "lb2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb2 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb2 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb2, 15);

	par = mxGetField(PARAMS, 0, "ub2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub2 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub2 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub2, 15);

	par = mxGetField(PARAMS, 0, "C2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C2 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C2 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C2, 357);

	par = mxGetField(PARAMS, 0, "d2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d2 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d2 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d2, 17);

	par = mxGetField(PARAMS, 0, "H3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H3 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H3 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H3, 441);

	par = mxGetField(PARAMS, 0, "f3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f3 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f3 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f3, 21);

	par = mxGetField(PARAMS, 0, "lb3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb3 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb3 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb3, 15);

	par = mxGetField(PARAMS, 0, "ub3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub3 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub3 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub3, 15);

	par = mxGetField(PARAMS, 0, "C3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C3 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C3 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C3, 357);

	par = mxGetField(PARAMS, 0, "d3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d3 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d3 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d3, 17);

	par = mxGetField(PARAMS, 0, "H4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H4 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H4 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H4, 441);

	par = mxGetField(PARAMS, 0, "f4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f4 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f4 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f4, 21);

	par = mxGetField(PARAMS, 0, "lb4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb4 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb4 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb4, 15);

	par = mxGetField(PARAMS, 0, "ub4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub4 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub4 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub4, 15);

	par = mxGetField(PARAMS, 0, "C4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C4 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C4 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C4, 357);

	par = mxGetField(PARAMS, 0, "d4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d4 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d4 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d4, 17);

	par = mxGetField(PARAMS, 0, "H5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H5 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H5 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H5, 441);

	par = mxGetField(PARAMS, 0, "f5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f5 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f5 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f5, 21);

	par = mxGetField(PARAMS, 0, "lb5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb5 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb5 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb5, 15);

	par = mxGetField(PARAMS, 0, "ub5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub5 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub5 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub5, 15);

	par = mxGetField(PARAMS, 0, "C5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C5 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C5 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C5, 357);

	par = mxGetField(PARAMS, 0, "d5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d5 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d5 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d5, 17);

	par = mxGetField(PARAMS, 0, "H6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H6 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H6 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H6, 441);

	par = mxGetField(PARAMS, 0, "f6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f6 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f6 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f6, 21);

	par = mxGetField(PARAMS, 0, "lb6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb6 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb6 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb6, 15);

	par = mxGetField(PARAMS, 0, "ub6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub6 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub6 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub6, 15);

	par = mxGetField(PARAMS, 0, "C6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C6 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C6 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C6, 357);

	par = mxGetField(PARAMS, 0, "d6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d6 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d6 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d6, 17);

	par = mxGetField(PARAMS, 0, "H7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H7 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H7 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H7, 441);

	par = mxGetField(PARAMS, 0, "f7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f7 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f7 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f7, 21);

	par = mxGetField(PARAMS, 0, "lb7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb7 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb7 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb7, 15);

	par = mxGetField(PARAMS, 0, "ub7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub7 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub7 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub7, 15);

	par = mxGetField(PARAMS, 0, "C7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C7 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C7 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C7, 357);

	par = mxGetField(PARAMS, 0, "d7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d7 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d7 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d7, 17);

	par = mxGetField(PARAMS, 0, "H8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H8 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H8 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H8, 441);

	par = mxGetField(PARAMS, 0, "f8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f8 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f8 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f8, 21);

	par = mxGetField(PARAMS, 0, "lb8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb8 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb8 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb8, 15);

	par = mxGetField(PARAMS, 0, "ub8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub8 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub8 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub8, 15);

	par = mxGetField(PARAMS, 0, "C8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C8 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C8 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C8, 357);

	par = mxGetField(PARAMS, 0, "d8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d8 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d8 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d8, 17);

	par = mxGetField(PARAMS, 0, "H9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H9 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H9 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H9, 441);

	par = mxGetField(PARAMS, 0, "f9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f9 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f9 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f9, 21);

	par = mxGetField(PARAMS, 0, "lb9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb9 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb9 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb9, 15);

	par = mxGetField(PARAMS, 0, "ub9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub9 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub9 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub9, 15);

	par = mxGetField(PARAMS, 0, "C9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C9 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C9 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C9, 357);

	par = mxGetField(PARAMS, 0, "d9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d9 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d9 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d9, 17);

	par = mxGetField(PARAMS, 0, "H10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H10 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H10 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H10, 441);

	par = mxGetField(PARAMS, 0, "f10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f10 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f10 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f10, 21);

	par = mxGetField(PARAMS, 0, "lb10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb10 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb10 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb10, 15);

	par = mxGetField(PARAMS, 0, "ub10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub10 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub10 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub10, 15);

	par = mxGetField(PARAMS, 0, "C10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C10 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C10 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C10, 357);

	par = mxGetField(PARAMS, 0, "d10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d10 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d10 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d10, 17);

	par = mxGetField(PARAMS, 0, "H11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H11 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H11 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H11, 441);

	par = mxGetField(PARAMS, 0, "f11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f11 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f11 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f11, 21);

	par = mxGetField(PARAMS, 0, "lb11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb11 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb11 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb11, 15);

	par = mxGetField(PARAMS, 0, "ub11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub11 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub11 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub11, 15);

	par = mxGetField(PARAMS, 0, "C11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C11 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C11 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C11, 357);

	par = mxGetField(PARAMS, 0, "d11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d11 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d11 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d11, 17);

	par = mxGetField(PARAMS, 0, "H12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H12 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H12 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H12, 441);

	par = mxGetField(PARAMS, 0, "f12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f12 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f12 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f12, 21);

	par = mxGetField(PARAMS, 0, "lb12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb12 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb12 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb12, 15);

	par = mxGetField(PARAMS, 0, "ub12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub12 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub12 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub12, 15);

	par = mxGetField(PARAMS, 0, "C12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C12 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C12 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C12, 357);

	par = mxGetField(PARAMS, 0, "d12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d12 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d12 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d12, 17);

	par = mxGetField(PARAMS, 0, "H13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H13 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H13 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H13, 441);

	par = mxGetField(PARAMS, 0, "f13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f13 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f13 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f13, 21);

	par = mxGetField(PARAMS, 0, "lb13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb13 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb13 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb13, 15);

	par = mxGetField(PARAMS, 0, "ub13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub13 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub13 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub13, 15);

	par = mxGetField(PARAMS, 0, "C13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C13 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C13 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C13, 357);

	par = mxGetField(PARAMS, 0, "d13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d13 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d13 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d13, 17);

	par = mxGetField(PARAMS, 0, "H14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H14 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H14 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H14, 441);

	par = mxGetField(PARAMS, 0, "f14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f14 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f14 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f14, 21);

	par = mxGetField(PARAMS, 0, "lb14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb14 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb14 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb14, 15);

	par = mxGetField(PARAMS, 0, "ub14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub14 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub14 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub14, 15);

	par = mxGetField(PARAMS, 0, "C14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C14 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C14 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C14, 357);

	par = mxGetField(PARAMS, 0, "d14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d14 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d14 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d14, 17);

	par = mxGetField(PARAMS, 0, "H15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H15 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H15 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H15, 441);

	par = mxGetField(PARAMS, 0, "f15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f15 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f15 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f15, 21);

	par = mxGetField(PARAMS, 0, "lb15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb15 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb15 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb15, 15);

	par = mxGetField(PARAMS, 0, "ub15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub15 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub15 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub15, 15);

	par = mxGetField(PARAMS, 0, "C15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C15 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C15 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C15, 357);

	par = mxGetField(PARAMS, 0, "d15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d15 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d15 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d15, 17);

	par = mxGetField(PARAMS, 0, "H16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H16 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H16 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H16, 441);

	par = mxGetField(PARAMS, 0, "f16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f16 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f16 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f16, 21);

	par = mxGetField(PARAMS, 0, "lb16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb16 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb16 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb16, 15);

	par = mxGetField(PARAMS, 0, "ub16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub16 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub16 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub16, 15);

	par = mxGetField(PARAMS, 0, "C16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C16 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C16 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C16, 357);

	par = mxGetField(PARAMS, 0, "d16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d16 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d16 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d16, 17);

	par = mxGetField(PARAMS, 0, "H17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H17 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H17 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H17, 441);

	par = mxGetField(PARAMS, 0, "f17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f17 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f17 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f17, 21);

	par = mxGetField(PARAMS, 0, "lb17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb17 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb17 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb17, 15);

	par = mxGetField(PARAMS, 0, "ub17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub17 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub17 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub17, 15);

	par = mxGetField(PARAMS, 0, "C17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C17 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C17 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C17, 357);

	par = mxGetField(PARAMS, 0, "d17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d17 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d17 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d17, 17);

	par = mxGetField(PARAMS, 0, "H18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H18 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H18 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H18, 441);

	par = mxGetField(PARAMS, 0, "f18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f18 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f18 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f18, 21);

	par = mxGetField(PARAMS, 0, "lb18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb18 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb18 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb18, 15);

	par = mxGetField(PARAMS, 0, "ub18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub18 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub18 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub18, 15);

	par = mxGetField(PARAMS, 0, "C18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C18 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C18 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C18, 357);

	par = mxGetField(PARAMS, 0, "d18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d18 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d18 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d18, 17);

	par = mxGetField(PARAMS, 0, "H19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H19 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H19 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H19, 441);

	par = mxGetField(PARAMS, 0, "f19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f19 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f19 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f19, 21);

	par = mxGetField(PARAMS, 0, "lb19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb19 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb19 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb19, 15);

	par = mxGetField(PARAMS, 0, "ub19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub19 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub19 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub19, 15);

	par = mxGetField(PARAMS, 0, "C19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C19 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C19 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C19, 357);

	par = mxGetField(PARAMS, 0, "d19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d19 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d19 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d19, 17);

	par = mxGetField(PARAMS, 0, "H20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H20 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H20 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H20, 441);

	par = mxGetField(PARAMS, 0, "f20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f20 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f20 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f20, 21);

	par = mxGetField(PARAMS, 0, "lb20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb20 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb20 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb20, 15);

	par = mxGetField(PARAMS, 0, "ub20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub20 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub20 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub20, 15);

	par = mxGetField(PARAMS, 0, "C20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C20 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C20 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C20, 357);

	par = mxGetField(PARAMS, 0, "d20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d20 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d20 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d20, 17);

	par = mxGetField(PARAMS, 0, "H21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H21 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H21 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H21, 441);

	par = mxGetField(PARAMS, 0, "f21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f21 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f21 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f21, 21);

	par = mxGetField(PARAMS, 0, "lb21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb21 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb21 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb21, 15);

	par = mxGetField(PARAMS, 0, "ub21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub21 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub21 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub21, 15);

	par = mxGetField(PARAMS, 0, "C21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C21 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C21 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C21, 357);

	par = mxGetField(PARAMS, 0, "d21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d21 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d21 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d21, 17);

	par = mxGetField(PARAMS, 0, "H22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H22 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H22 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H22, 441);

	par = mxGetField(PARAMS, 0, "f22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f22 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f22 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f22, 21);

	par = mxGetField(PARAMS, 0, "lb22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb22 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb22 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb22, 15);

	par = mxGetField(PARAMS, 0, "ub22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub22 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub22 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub22, 15);

	par = mxGetField(PARAMS, 0, "C22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C22 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C22 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C22, 357);

	par = mxGetField(PARAMS, 0, "d22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d22 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d22 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d22, 17);

	par = mxGetField(PARAMS, 0, "H23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H23 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H23 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H23, 441);

	par = mxGetField(PARAMS, 0, "f23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f23 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f23 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f23, 21);

	par = mxGetField(PARAMS, 0, "lb23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb23 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb23 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb23, 15);

	par = mxGetField(PARAMS, 0, "ub23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub23 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub23 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub23, 15);

	par = mxGetField(PARAMS, 0, "C23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C23 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C23 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C23, 357);

	par = mxGetField(PARAMS, 0, "d23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d23 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d23 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d23, 17);

	par = mxGetField(PARAMS, 0, "H24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H24 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H24 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H24, 441);

	par = mxGetField(PARAMS, 0, "f24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f24 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f24 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f24, 21);

	par = mxGetField(PARAMS, 0, "lb24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb24 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb24 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb24, 15);

	par = mxGetField(PARAMS, 0, "ub24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub24 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub24 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub24, 15);

	par = mxGetField(PARAMS, 0, "C24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C24 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C24 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C24, 357);

	par = mxGetField(PARAMS, 0, "d24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d24 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d24 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d24, 17);

	par = mxGetField(PARAMS, 0, "H25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H25 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H25 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H25, 441);

	par = mxGetField(PARAMS, 0, "f25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f25 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f25 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f25, 21);

	par = mxGetField(PARAMS, 0, "lb25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb25 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb25 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb25, 15);

	par = mxGetField(PARAMS, 0, "ub25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub25 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub25 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub25, 15);

	par = mxGetField(PARAMS, 0, "C25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C25 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C25 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C25, 357);

	par = mxGetField(PARAMS, 0, "d25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d25 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d25 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d25, 17);

	par = mxGetField(PARAMS, 0, "H26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H26 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H26 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H26, 441);

	par = mxGetField(PARAMS, 0, "f26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f26 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f26 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f26, 21);

	par = mxGetField(PARAMS, 0, "lb26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb26 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb26 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb26, 15);

	par = mxGetField(PARAMS, 0, "ub26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub26 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub26 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub26, 15);

	par = mxGetField(PARAMS, 0, "C26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C26 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C26 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C26, 357);

	par = mxGetField(PARAMS, 0, "d26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d26 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d26 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d26, 17);

	par = mxGetField(PARAMS, 0, "H27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H27 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H27 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H27, 441);

	par = mxGetField(PARAMS, 0, "f27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f27 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f27 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f27, 21);

	par = mxGetField(PARAMS, 0, "lb27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb27 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb27 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb27, 15);

	par = mxGetField(PARAMS, 0, "ub27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub27 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub27 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub27, 15);

	par = mxGetField(PARAMS, 0, "C27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C27 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C27 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C27, 357);

	par = mxGetField(PARAMS, 0, "d27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d27 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d27 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d27, 17);

	par = mxGetField(PARAMS, 0, "H28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H28 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H28 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H28, 441);

	par = mxGetField(PARAMS, 0, "f28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f28 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f28 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f28, 21);

	par = mxGetField(PARAMS, 0, "lb28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb28 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb28 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb28, 15);

	par = mxGetField(PARAMS, 0, "ub28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub28 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub28 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub28, 15);

	par = mxGetField(PARAMS, 0, "C28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C28 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C28 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C28, 357);

	par = mxGetField(PARAMS, 0, "d28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d28 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d28 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d28, 17);

	par = mxGetField(PARAMS, 0, "H29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H29 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H29 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H29, 441);

	par = mxGetField(PARAMS, 0, "f29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f29 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f29 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f29, 21);

	par = mxGetField(PARAMS, 0, "lb29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb29 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb29 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb29, 15);

	par = mxGetField(PARAMS, 0, "ub29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub29 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub29 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub29, 15);

	par = mxGetField(PARAMS, 0, "C29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C29 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C29 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C29, 357);

	par = mxGetField(PARAMS, 0, "d29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d29 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d29 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d29, 17);

	par = mxGetField(PARAMS, 0, "H30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H30 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.H30 must be of size [21 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H30, 441);

	par = mxGetField(PARAMS, 0, "f30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f30 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f30 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f30, 21);

	par = mxGetField(PARAMS, 0, "lb30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb30 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb30 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb30, 15);

	par = mxGetField(PARAMS, 0, "ub30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub30 must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub30 must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub30, 15);

	par = mxGetField(PARAMS, 0, "C30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C30 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 21 ) {
    mexErrMsgTxt("PARAMS.C30 must be of size [17 x 21]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C30, 357);

	par = mxGetField(PARAMS, 0, "d30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d30 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d30 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d30, 17);

	par = mxGetField(PARAMS, 0, "H31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H31 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 17 ) {
    mexErrMsgTxt("PARAMS.H31 must be of size [17 x 17]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H31, 289);

	par = mxGetField(PARAMS, 0, "f31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f31 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f31 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f31, 17);

	par = mxGetField(PARAMS, 0, "lb31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb31 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb31 must be of size [11 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb31, 11);

	par = mxGetField(PARAMS, 0, "ub31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub31 must be a double.");
    }
    if( mxGetM(par) != 11 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub31 must be of size [11 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub31, 11);

	par = mxGetField(PARAMS, 0, "d31");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d31 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d31 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d31 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d31, 17);

	par = mxGetField(PARAMS, 0, "z_init_00");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_00 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_00 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_00 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_00, 21);

	par = mxGetField(PARAMS, 0, "z_init_01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_01 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_01 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_01, 21);

	par = mxGetField(PARAMS, 0, "z_init_02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_02 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_02 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_02, 21);

	par = mxGetField(PARAMS, 0, "z_init_03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_03 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_03 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_03, 21);

	par = mxGetField(PARAMS, 0, "z_init_04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_04 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_04 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_04, 21);

	par = mxGetField(PARAMS, 0, "z_init_05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_05 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_05 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_05, 21);

	par = mxGetField(PARAMS, 0, "z_init_06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_06 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_06 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_06, 21);

	par = mxGetField(PARAMS, 0, "z_init_07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_07 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_07 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_07, 21);

	par = mxGetField(PARAMS, 0, "z_init_08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_08 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_08 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_08, 21);

	par = mxGetField(PARAMS, 0, "z_init_09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_09 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_09 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_09, 21);

	par = mxGetField(PARAMS, 0, "z_init_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_10 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_10 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_10, 21);

	par = mxGetField(PARAMS, 0, "z_init_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_11 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_11 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_11, 21);

	par = mxGetField(PARAMS, 0, "z_init_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_12 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_12 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_12, 21);

	par = mxGetField(PARAMS, 0, "z_init_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_13 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_13 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_13, 21);

	par = mxGetField(PARAMS, 0, "z_init_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_14 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_14 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_14, 21);

	par = mxGetField(PARAMS, 0, "z_init_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_15 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_15 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_15, 21);

	par = mxGetField(PARAMS, 0, "z_init_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_16 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_16 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_16, 21);

	par = mxGetField(PARAMS, 0, "z_init_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_17 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_17 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_17, 21);

	par = mxGetField(PARAMS, 0, "z_init_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_18 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_18 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_18, 21);

	par = mxGetField(PARAMS, 0, "z_init_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_19 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_19 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_19, 21);

	par = mxGetField(PARAMS, 0, "z_init_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_20 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_20 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_20, 21);

	par = mxGetField(PARAMS, 0, "z_init_21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_21 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_21 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_21, 21);

	par = mxGetField(PARAMS, 0, "z_init_22");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_22 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_22 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_22 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_22, 21);

	par = mxGetField(PARAMS, 0, "z_init_23");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_23 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_23 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_23 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_23, 21);

	par = mxGetField(PARAMS, 0, "z_init_24");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_24 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_24 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_24 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_24, 21);

	par = mxGetField(PARAMS, 0, "z_init_25");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_25 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_25 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_25 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_25, 21);

	par = mxGetField(PARAMS, 0, "z_init_26");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_26 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_26 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_26 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_26, 21);

	par = mxGetField(PARAMS, 0, "z_init_27");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_27 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_27 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_27 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_27, 21);

	par = mxGetField(PARAMS, 0, "z_init_28");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_28 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_28 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_28 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_28, 21);

	par = mxGetField(PARAMS, 0, "z_init_29");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_29 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_29 must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_29 must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_29, 21);

	par = mxGetField(PARAMS, 0, "z_init_30");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_30 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_30 must be a double.");
    }
    if( mxGetM(par) != 17 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_30 must be of size [17 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_30, 17);

	#if forces_SET_PRINTLEVEL > 0
		/* Prepare file for printfs */
		/*fp = freopen("stdout_temp","w+",stdout);*/
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) {
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */
	exitflag = forces_solve(&params, &output, &info, fp );

	/* close stdout */
	/* fclose(fp); */
	
	#if forces_SET_PRINTLEVEL > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) {
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 31, outputnames);
	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out1, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out1", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out2, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out2", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out3, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out3", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out4, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out4", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out5, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out5", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out6, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out6", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out7, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out7", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out8, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out8", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out9, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out9", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out10, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out10", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out11, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out11", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out12, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out12", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out13, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out13", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out14, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out14", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out15, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out15", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out16, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out16", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out17, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out17", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out18, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out18", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out19, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out19", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out20, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out20", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out21, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out21", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out22, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out22", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out23, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out23", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out24, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out24", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out25, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out25", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out26, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out26", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out27, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out27", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out28, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out28", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out29, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out29", outvar);

	outvar = mxCreateDoubleMatrix(21, 1, mxREAL);
	copyCArrayToM( output.out30, mxGetPr(outvar), 21);
	mxSetField(plhs[0], 0, "out30", outvar);

	outvar = mxCreateDoubleMatrix(17, 1, mxREAL);
	copyCArrayToM( output.out31, mxGetPr(outvar), 17);
	mxSetField(plhs[0], 0, "out31", outvar);	

	/* copy exitflag */
	if( nlhs > 1 )
	{
		plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
		        plhs[2] = mxCreateStructMatrix(1, 1, 16, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dobj;
		mxSetField(plhs[2], 0, "dobj", outvar);

		/* dgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dgap;
		mxSetField(plhs[2], 0, "dgap", outvar);

		/* rdgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rdgap;
		mxSetField(plhs[2], 0, "rdgap", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* mu_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu_aff;
		mxSetField(plhs[2], 0, "mu_aff", outvar);

		/* sigma */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.sigma;
		mxSetField(plhs[2], 0, "sigma", outvar);

		/* lsit_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_aff;
		mxSetField(plhs[2], 0, "lsit_aff", outvar);

		/* lsit_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_cc;
		mxSetField(plhs[2], 0, "lsit_cc", outvar);

		/* step_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_aff;
		mxSetField(plhs[2], 0, "step_aff", outvar);

		/* step_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_cc;
		mxSetField(plhs[2], 0, "step_cc", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);
	}
}