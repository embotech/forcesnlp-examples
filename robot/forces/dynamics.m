function dx = dynamics(x,u)
% 2-link planar manipulator dynamics
th1     = x(1);
th1d    = x(2);
th2     = x(3);
th2d    = x(4);
tau1     = x(5);
tau2    = x(6);

tau1r = u(1);
tau2r = u(2);
dx = zeros(6,1);
mM1 	= 0.3;
mM2 	= 0.3;
kr1 	= 1.0;
kr2 	= 1.0;
rM1  = 0.05;
rM2  = 0.05;
Im1 	= 1/2*mM1*rM1*rM1;
Im2 	= 1/2*mM2*rM2*rM2;

ml1 	= 3.0;
ml2 	= 3.0;
l1 	= 1.0;
l2 	= 1.0;
Il1	= 1.0/12.0*ml1*l1*l1;
Il2	= 1.0/12.0*ml2*l2*l2;
% Il1	= 1.0/12.0*ml1*ml1*l1*l1;
% Il2	= 1.0/12.0*ml2*ml2*l2*l2;
a1 	= l1/2.0;
a2 	= l2/2.0;

g = 9.81;

alpha_1	= Il1 + ml1*l1*l1 + kr1*kr1*Im1 + Il2 + ml2*(a1*a1 + l2*l2 + 2*a1*l2*cos(th2) + Im2 + mM2*a1*a1);
alpha_2	= Il2 + ml2*(l2*l2 + a1*l2*cos(th2)) + kr2*Im2;
alpha_3 = -2.0*ml2*a1*l2*sin(th2);
alpha_4 = -ml2*a1*l2*sin(th2);
K_alpha = (ml1*l1 + mM2*a1 + ml2*a1)*g*cos(th1) + ml2*l2*g*cos(th1 + th2);

beta_1 	= Il2 + ml2*(l2*l2 + a1*l2*cos(th2)) + kr2*Im2;
beta_2 	= Il2 + ml2*l2*l2 + kr2*kr2*Im2;
beta_3 	= ml2*a1*l2*sin(th2);
K_beta 	= ml2*l2*g*cos(th1 + th2);

theta1dd_exp = 1.0/(alpha_1 - alpha_2*beta_1/beta_2) 	* (alpha_2/beta_2 * (K_beta + beta_3*th1d*th1d - tau2) - alpha_3*th1d*th2d - alpha_4*th2d - K_alpha + tau1);

dx = [  th1d;
        theta1dd_exp;
        th2d;
        1.0/beta_2*(tau2 - beta_1*theta1dd_exp - beta_3*th1d*th1d - K_beta);
        tau1r;
        tau2r ];
end