function [ dx ] = dynamics_ss( input )
rho = 1.23;
A = 0.1;
Cl = 0.25;
Cd = 0.3*Cl;
m = 10;
g = 9.81;
L = 0.5;
L2 = 1; 
J1 = 0.25;
J2 = 0.25;
J3 = 1;

V1 = 0;
V2 = 0;
V3 = 0;

q1 = 1;
q2 = 0;
q3 = 0;
q4 = 0;

Omega1 = 0;
Omega2 = 0;
Omega3 = 0;

W1 = input(1);
W2 = input(2);
W3 = input(3);
W4 = input(4);

alpha = 0.0;
rW1 = 0.0;
rW2 = 0.0;
rW3 = 0.0;
rW4 = 0.0;

dx = ...
    [  
    V1; 
    V2; 
    V3; 
    (A*Cl*rho*(2*q1*q3 + 2*q2*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m);
    -(A*Cl*rho*(2*q1*q2 - 2*q3*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m);
    (A*Cl*rho*(W1*W1 + W2*W2 + W3*W3 + W4*W4)*(q1*q1 - q2*q2 - q3*q3 + q4*q4))/(2*m) - g;
    - (Omega1*q2)/2 - (Omega2*q3)/2 - (Omega3*q4)/2 - (alpha*q1*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    (Omega1*q1)/2 - (Omega3*q3)/2 + (Omega2*q4)/2 - (alpha*q2*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    (Omega2*q1)/2 + (Omega3*q2)/2 - (Omega1*q4)/2 - (alpha*q3*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    (Omega3*q1)/2 - (Omega2*q2)/2 + (Omega1*q3)/2 - (alpha*q4*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    (J3*Omega2*Omega3 - J2*Omega2*Omega3 + (A*Cl*L*rho*(W2*W2 - W4*W4))/2)/J1;
    -(J3*Omega1*Omega3 - J1*Omega1*Omega3 + (A*Cl*L*rho*(W1*W1 - W3*W3))/2)/J2;
    (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*L2*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3;
    rW1;
    rW2;
    rW3;
    rW4];
end

