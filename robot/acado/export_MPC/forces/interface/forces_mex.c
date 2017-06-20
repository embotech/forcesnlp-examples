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
	const char *outputnames[21] = {"out1","out2","out3","out4","out5","out6","out7","out8","out9","out10","out11","out12","out13","out14","out15","out16","out17","out18","out19","out20","out21"};
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
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H1 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H1, 64);

	par = mxGetField(PARAMS, 0, "f1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f1 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f1 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f1, 8);

	par = mxGetField(PARAMS, 0, "lb1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb1 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb1 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb1, 8);

	par = mxGetField(PARAMS, 0, "ub1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub1 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub1 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub1, 8);

	par = mxGetField(PARAMS, 0, "C1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C1 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C1 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C1, 48);

	par = mxGetField(PARAMS, 0, "d1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d1 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d1 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d1, 6);

	par = mxGetField(PARAMS, 0, "H2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H2 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H2 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H2, 64);

	par = mxGetField(PARAMS, 0, "f2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f2 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f2 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f2, 8);

	par = mxGetField(PARAMS, 0, "lb2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb2 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb2 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb2, 8);

	par = mxGetField(PARAMS, 0, "ub2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub2 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub2 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub2, 8);

	par = mxGetField(PARAMS, 0, "C2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C2 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C2 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C2, 48);

	par = mxGetField(PARAMS, 0, "d2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d2 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d2 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d2, 6);

	par = mxGetField(PARAMS, 0, "H3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H3 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H3 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H3, 64);

	par = mxGetField(PARAMS, 0, "f3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f3 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f3 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f3, 8);

	par = mxGetField(PARAMS, 0, "lb3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb3 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb3 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb3, 8);

	par = mxGetField(PARAMS, 0, "ub3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub3 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub3 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub3, 8);

	par = mxGetField(PARAMS, 0, "C3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C3 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C3 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C3, 48);

	par = mxGetField(PARAMS, 0, "d3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d3 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d3 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d3, 6);

	par = mxGetField(PARAMS, 0, "H4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H4 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H4 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H4, 64);

	par = mxGetField(PARAMS, 0, "f4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f4 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f4 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f4, 8);

	par = mxGetField(PARAMS, 0, "lb4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb4 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb4 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb4, 8);

	par = mxGetField(PARAMS, 0, "ub4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub4 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub4 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub4, 8);

	par = mxGetField(PARAMS, 0, "C4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C4 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C4 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C4, 48);

	par = mxGetField(PARAMS, 0, "d4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d4 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d4 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d4, 6);

	par = mxGetField(PARAMS, 0, "H5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H5 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H5 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H5, 64);

	par = mxGetField(PARAMS, 0, "f5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f5 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f5 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f5, 8);

	par = mxGetField(PARAMS, 0, "lb5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb5 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb5 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb5, 8);

	par = mxGetField(PARAMS, 0, "ub5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub5 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub5 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub5, 8);

	par = mxGetField(PARAMS, 0, "C5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C5 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C5 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C5, 48);

	par = mxGetField(PARAMS, 0, "d5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d5 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d5 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d5, 6);

	par = mxGetField(PARAMS, 0, "H6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H6 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H6 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H6, 64);

	par = mxGetField(PARAMS, 0, "f6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f6 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f6 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f6, 8);

	par = mxGetField(PARAMS, 0, "lb6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb6 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb6 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb6, 8);

	par = mxGetField(PARAMS, 0, "ub6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub6 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub6 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub6, 8);

	par = mxGetField(PARAMS, 0, "C6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C6 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C6 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C6, 48);

	par = mxGetField(PARAMS, 0, "d6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d6 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d6 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d6, 6);

	par = mxGetField(PARAMS, 0, "H7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H7 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H7 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H7, 64);

	par = mxGetField(PARAMS, 0, "f7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f7 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f7 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f7, 8);

	par = mxGetField(PARAMS, 0, "lb7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb7 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb7 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb7, 8);

	par = mxGetField(PARAMS, 0, "ub7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub7 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub7 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub7, 8);

	par = mxGetField(PARAMS, 0, "C7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C7 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C7 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C7, 48);

	par = mxGetField(PARAMS, 0, "d7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d7 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d7 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d7, 6);

	par = mxGetField(PARAMS, 0, "H8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H8 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H8 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H8, 64);

	par = mxGetField(PARAMS, 0, "f8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f8 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f8 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f8, 8);

	par = mxGetField(PARAMS, 0, "lb8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb8 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb8 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb8, 8);

	par = mxGetField(PARAMS, 0, "ub8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub8 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub8 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub8, 8);

	par = mxGetField(PARAMS, 0, "C8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C8 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C8 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C8, 48);

	par = mxGetField(PARAMS, 0, "d8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d8 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d8 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d8, 6);

	par = mxGetField(PARAMS, 0, "H9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H9 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H9 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H9, 64);

	par = mxGetField(PARAMS, 0, "f9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f9 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f9 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f9, 8);

	par = mxGetField(PARAMS, 0, "lb9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb9 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb9 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb9, 8);

	par = mxGetField(PARAMS, 0, "ub9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub9 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub9 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub9, 8);

	par = mxGetField(PARAMS, 0, "C9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C9 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C9 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C9, 48);

	par = mxGetField(PARAMS, 0, "d9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d9 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d9 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d9, 6);

	par = mxGetField(PARAMS, 0, "H10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H10 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H10 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H10, 64);

	par = mxGetField(PARAMS, 0, "f10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f10 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f10 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f10, 8);

	par = mxGetField(PARAMS, 0, "lb10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb10 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb10 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb10, 8);

	par = mxGetField(PARAMS, 0, "ub10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub10 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub10 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub10, 8);

	par = mxGetField(PARAMS, 0, "C10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C10 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C10 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C10, 48);

	par = mxGetField(PARAMS, 0, "d10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d10 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d10 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d10, 6);

	par = mxGetField(PARAMS, 0, "H11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H11 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H11 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H11, 64);

	par = mxGetField(PARAMS, 0, "f11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f11 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f11 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f11, 8);

	par = mxGetField(PARAMS, 0, "lb11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb11 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb11 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb11, 8);

	par = mxGetField(PARAMS, 0, "ub11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub11 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub11 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub11, 8);

	par = mxGetField(PARAMS, 0, "C11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C11 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C11 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C11, 48);

	par = mxGetField(PARAMS, 0, "d11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d11 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d11 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d11, 6);

	par = mxGetField(PARAMS, 0, "H12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H12 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H12, 64);

	par = mxGetField(PARAMS, 0, "f12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f12 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f12, 8);

	par = mxGetField(PARAMS, 0, "lb12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb12 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb12, 8);

	par = mxGetField(PARAMS, 0, "ub12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub12 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub12, 8);

	par = mxGetField(PARAMS, 0, "C12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C12 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C12 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C12, 48);

	par = mxGetField(PARAMS, 0, "d12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d12 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d12 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d12, 6);

	par = mxGetField(PARAMS, 0, "H13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H13 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H13 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H13, 64);

	par = mxGetField(PARAMS, 0, "f13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f13 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f13 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f13, 8);

	par = mxGetField(PARAMS, 0, "lb13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb13 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb13 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb13, 8);

	par = mxGetField(PARAMS, 0, "ub13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub13 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub13 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub13, 8);

	par = mxGetField(PARAMS, 0, "C13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C13 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C13 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C13, 48);

	par = mxGetField(PARAMS, 0, "d13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d13 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d13 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d13, 6);

	par = mxGetField(PARAMS, 0, "H14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H14 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H14 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H14, 64);

	par = mxGetField(PARAMS, 0, "f14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f14 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f14 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f14, 8);

	par = mxGetField(PARAMS, 0, "lb14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb14 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb14 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb14, 8);

	par = mxGetField(PARAMS, 0, "ub14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub14 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub14 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub14, 8);

	par = mxGetField(PARAMS, 0, "C14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C14 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C14 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C14, 48);

	par = mxGetField(PARAMS, 0, "d14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d14 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d14 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d14, 6);

	par = mxGetField(PARAMS, 0, "H15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H15 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H15 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H15, 64);

	par = mxGetField(PARAMS, 0, "f15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f15 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f15 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f15, 8);

	par = mxGetField(PARAMS, 0, "lb15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb15 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb15 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb15, 8);

	par = mxGetField(PARAMS, 0, "ub15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub15 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub15 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub15, 8);

	par = mxGetField(PARAMS, 0, "C15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C15 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C15 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C15, 48);

	par = mxGetField(PARAMS, 0, "d15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d15 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d15 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d15, 6);

	par = mxGetField(PARAMS, 0, "H16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H16 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H16 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H16, 64);

	par = mxGetField(PARAMS, 0, "f16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f16 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f16 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f16, 8);

	par = mxGetField(PARAMS, 0, "lb16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb16 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb16 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb16, 8);

	par = mxGetField(PARAMS, 0, "ub16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub16 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub16 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub16, 8);

	par = mxGetField(PARAMS, 0, "C16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C16 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C16 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C16, 48);

	par = mxGetField(PARAMS, 0, "d16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d16 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d16 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d16, 6);

	par = mxGetField(PARAMS, 0, "H17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H17 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H17 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H17, 64);

	par = mxGetField(PARAMS, 0, "f17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f17 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f17 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f17, 8);

	par = mxGetField(PARAMS, 0, "lb17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb17 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb17 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb17, 8);

	par = mxGetField(PARAMS, 0, "ub17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub17 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub17 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub17, 8);

	par = mxGetField(PARAMS, 0, "C17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C17 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C17 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C17, 48);

	par = mxGetField(PARAMS, 0, "d17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d17 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d17 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d17, 6);

	par = mxGetField(PARAMS, 0, "H18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H18 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H18 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H18, 64);

	par = mxGetField(PARAMS, 0, "f18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f18 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f18 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f18, 8);

	par = mxGetField(PARAMS, 0, "lb18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb18 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb18 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb18, 8);

	par = mxGetField(PARAMS, 0, "ub18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub18 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub18 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub18, 8);

	par = mxGetField(PARAMS, 0, "C18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C18 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C18 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C18, 48);

	par = mxGetField(PARAMS, 0, "d18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d18 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d18 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d18, 6);

	par = mxGetField(PARAMS, 0, "H19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H19 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H19 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H19, 64);

	par = mxGetField(PARAMS, 0, "f19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f19 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f19 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f19, 8);

	par = mxGetField(PARAMS, 0, "lb19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb19 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb19 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb19, 8);

	par = mxGetField(PARAMS, 0, "ub19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub19 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub19 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub19, 8);

	par = mxGetField(PARAMS, 0, "C19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C19 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C19 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C19, 48);

	par = mxGetField(PARAMS, 0, "d19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d19 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d19 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d19, 6);

	par = mxGetField(PARAMS, 0, "H20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H20 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.H20 must be of size [8 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H20, 64);

	par = mxGetField(PARAMS, 0, "f20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f20 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f20 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f20, 8);

	par = mxGetField(PARAMS, 0, "lb20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb20 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb20 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb20, 8);

	par = mxGetField(PARAMS, 0, "ub20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub20 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub20 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub20, 8);

	par = mxGetField(PARAMS, 0, "C20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C20 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 8 ) {
    mexErrMsgTxt("PARAMS.C20 must be of size [6 x 8]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C20, 48);

	par = mxGetField(PARAMS, 0, "d20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d20 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d20 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d20, 6);

	par = mxGetField(PARAMS, 0, "H21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.H21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.H21 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 6 ) {
    mexErrMsgTxt("PARAMS.H21 must be of size [6 x 6]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.H21, 36);

	par = mxGetField(PARAMS, 0, "f21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f21 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f21 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f21, 6);

	par = mxGetField(PARAMS, 0, "lb21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb21 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb21 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb21, 6);

	par = mxGetField(PARAMS, 0, "ub21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub21 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub21 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub21, 6);

	par = mxGetField(PARAMS, 0, "d21");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.d21 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.d21 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.d21 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.d21, 6);

	par = mxGetField(PARAMS, 0, "z_init_00");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_00 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_00 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_00 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_00, 8);

	par = mxGetField(PARAMS, 0, "z_init_01");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_01 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_01 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_01 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_01, 8);

	par = mxGetField(PARAMS, 0, "z_init_02");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_02 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_02 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_02 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_02, 8);

	par = mxGetField(PARAMS, 0, "z_init_03");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_03 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_03 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_03 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_03, 8);

	par = mxGetField(PARAMS, 0, "z_init_04");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_04 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_04 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_04 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_04, 8);

	par = mxGetField(PARAMS, 0, "z_init_05");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_05 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_05 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_05 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_05, 8);

	par = mxGetField(PARAMS, 0, "z_init_06");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_06 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_06 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_06 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_06, 8);

	par = mxGetField(PARAMS, 0, "z_init_07");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_07 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_07 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_07 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_07, 8);

	par = mxGetField(PARAMS, 0, "z_init_08");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_08 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_08 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_08 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_08, 8);

	par = mxGetField(PARAMS, 0, "z_init_09");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_09 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_09 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_09 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_09, 8);

	par = mxGetField(PARAMS, 0, "z_init_10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_10 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_10 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_10, 8);

	par = mxGetField(PARAMS, 0, "z_init_11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_11 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_11 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_11, 8);

	par = mxGetField(PARAMS, 0, "z_init_12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_12 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_12, 8);

	par = mxGetField(PARAMS, 0, "z_init_13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_13 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_13 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_13, 8);

	par = mxGetField(PARAMS, 0, "z_init_14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_14 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_14 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_14, 8);

	par = mxGetField(PARAMS, 0, "z_init_15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_15 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_15 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_15, 8);

	par = mxGetField(PARAMS, 0, "z_init_16");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_16 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_16 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_16 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_16, 8);

	par = mxGetField(PARAMS, 0, "z_init_17");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_17 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_17 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_17 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_17, 8);

	par = mxGetField(PARAMS, 0, "z_init_18");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_18 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_18 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_18 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_18, 8);

	par = mxGetField(PARAMS, 0, "z_init_19");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_19 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_19 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_19 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_19, 8);

	par = mxGetField(PARAMS, 0, "z_init_20");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.z_init_20 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.z_init_20 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.z_init_20 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.z_init_20, 6);

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
	plhs[0] = mxCreateStructMatrix(1, 1, 21, outputnames);
	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out1, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out1", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out2, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out2", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out3, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out3", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out4, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out4", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out5, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out5", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out6, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out6", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out7, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out7", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out8, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out8", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out9, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out9", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out10, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out10", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out11, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out11", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out12, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out12", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out13, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out13", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out14, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out14", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out15, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out15", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out16, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out16", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out17, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out17", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out18, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out18", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out19, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out19", outvar);

	outvar = mxCreateDoubleMatrix(8, 1, mxREAL);
	copyCArrayToM( output.out20, mxGetPr(outvar), 8);
	mxSetField(plhs[0], 0, "out20", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.out21, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "out21", outvar);	

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