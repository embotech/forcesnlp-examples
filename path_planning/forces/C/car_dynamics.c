#include <math.h>

void car_dyanmics(double *x, double *c)
{
    c[0] = x[2] + (x[4]*cos(x[5]))/60 + (cos(x[1]/10 + x[5])*(x[0]/10 + x[4]))/60 + (cos(x[1]/20 + x[5])*(x[0]/20 + x[4]))/15;
    c[1] = x[3] + (x[4]*sin(x[5]))/60 + (sin(x[1]/10 + x[5])*(x[0]/10 + x[4]))/60 + (sin(x[1]/20 + x[5])*(x[0]/20 + x[4]))/15;
    c[2] = x[0]/10 + x[4];
    c[3] = x[1]/10 + x[5];
}

void car_dyanmics_jacobian(double *x, double *J)
{   
    /* NOTE: only non-zero values in dense matrix (column major) filled in */
    
    /* 1st column: indices 0..3 */
    J[0] = cos(x[1]/10 + x[5])/600 + cos(x[1]/20 + x[5])/300;
    J[1] = sin(x[1]/10 + x[5])/600 + sin(x[1]/20 + x[5])/300;
    J[2] = 0.1;
    
    /* 2nd column: indices 4..7 */
    J[4] = - (sin(x[1]/10 + x[5])*(x[0]/10 + x[4]))/600 - (sin(x[1]/20 + x[5])*(x[0]/20 + x[4]))/300;
    J[5] = (cos(x[1]/10 + x[5])*(x[0]/10 + x[4]))/600 + (cos(x[1]/20 + x[5])*(x[0]/20 + x[4]))/300;
    J[7] = 0.1;
    
    /* 3rd column: indices 8..11 */
    J[8] = 1;
            
    /* 4th column: indices 12..15 */
    J[13] = 1;
    
    /* 5th column: indices 16..19 */
    J[16] = cos(x[1]/10 + x[5])/60 + cos(x[1]/20 + x[5])/15 + cos(x[5])/60;
    J[17] = sin(x[1]/10 + x[5])/60 + sin(x[1]/20 + x[5])/15 + sin(x[5])/60;
    J[18] = 1;
    
    /* 6th column: indices 20..23 */
    J[20] = - (x[4]*sin(x[5]))/60 - (sin(x[1]/10 + x[5])*(x[0]/10 + x[4]))/60 - (sin(x[1]/20 + x[5])*(x[0]/20 + x[4]))/15;
    J[21] =   (x[4]*cos(x[5]))/60 + (cos(x[1]/10 + x[5])*(x[0]/10 + x[4]))/60 + (cos(x[1]/20 + x[5])*(x[0]/20 + x[4]))/15;
    J[23] = 1;
}
