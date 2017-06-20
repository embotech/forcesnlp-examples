clc;
clear all;
close all;

Th = 1;
N = 30;

Ts = Th/N;
EXPORT = 0;

DifferentialState P1; 
DifferentialState P2; 
DifferentialState P3; 
DifferentialState V1;
DifferentialState V2;
DifferentialState V3;
DifferentialState q1;
DifferentialState q2;
DifferentialState q3;
DifferentialState q4;
DifferentialState Omega1;
DifferentialState Omega2;
DifferentialState Omega3;

Control W1;
Control W2;
Control W3;
Control W4;

n_XD = length(diffStates);
n_U = length(controls);

rho = 1.23;
A = 0.1;
Cl = 0.25;
Cd = 0.3*Cl;
m = 10;
g = 9.81;
L = 0.5;
L2 = 1; 
Jp = 1e-2;
xi = 1e-2;
J1 = 0.25;
J2 = 0.25;
J3 = 1;
gain = 1e-4;
alpha = 0.0;

f = dot([P1;P2;P3;V1;V2;V3;q1;q2;q3;q4;Omega1;Omega2;Omega3]) == ...
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
    (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*L2*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3];


hover_omega = 40;                    
ref = [0.0, 0.0, 0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,hover_omega,hover_omega,hover_omega,hover_omega];
h = [   P1 - ref(1); ...
        P2 - ref(2); ...
        P3 - ref(3); ...    
        V1 - ref(4); ...
        V2 - ref(5); ...
        V3 - ref(6); ...
        q1 - ref(7); ...
        q2 - ref(8); ...
        q3 - ref(9); ...
        q4 - ref(10); ...
        Omega1 - ref(11); ...
        Omega2 - ref(12); ...
        Omega3 - ref(13); ...
        W1 - ref(14); ...
        W2 - ref(15); ...
        W3 - ref(16); ...
        W4 - ref(17)];  ...

hN = [  P1 - ref(1); ...
        P2 - ref(2); ...
        P3 - ref(3); ...    
        V1 - ref(4); ...
        V2 - ref(5); ...
        V3 - ref(6); ...
        q1 - ref(7); ...
        q2 - ref(8); ...
        q3 - ref(9); ...
        q4 - ref(10); ...
        Omega1 - ref(11); ...
        Omega2 - ref(12); ...
        Omega3 - ref(13)];  
        

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 10;
sim = acado.SIMexport( Ts );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../integrate_quadcopter')
    cd ..
end

%% MPCexport
acadoSet('problemname', 'mpc');


ocp = acado.OCP( 0.0, N*Ts, N );

W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo( 20.0 <= W1 <= 60.0);
ocp.subjectTo( 20.0 <= W2 <= 60.0);
ocp.subjectTo( 20.0 <= W3 <= 60.0);
ocp.subjectTo( 20.0 <= W4 <= 60.0);
ocp.subjectTo( P3 >= -3);
ocp.setModel(f);

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_RK4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        5*N                 );
% mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
% mpc.set( 'QP_SOLVER',                   'QP_FORCES'    	);
% mpc.set( 'SPARSE_QP_SOLUTION', 'SPARSE_SOLVER' );

if EXPORT
    mpc.exportCode( 'export_MPC' );
    
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3')
    
    cd export_MPC
    make_acado_solver('../acado_MPCstep')
    cd ..
end

%% PARAMETERS SIMULATION
initial_angle_rad = 3.0;
X0 = [0.0 0.0 0.0, 0.0 0.0 0.0, cos(initial_angle_rad/2) sin(initial_angle_rad/2) 0.0 0.0, 0.0 0.0 0.0]';

Xref0 = zeros(13,1)';
input.x = repmat(Xref0,N+1,1);

Xref = repmat(Xref0,N,1);
input.od = [];

Uref = zeros(N,n_U);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);
  
input.W = diag([100 100 100 0.1 0.1 0.1 100 100 100 100 0.1 0.1 0.1 0.1 0.1 0.1 0.1]);
input.WN = diag([100 100 100 0.1 0.1 0.1 100 100 100 100 0.1 0.1 0.1]);

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 4;
Ns = Tf/Ts;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
log = [];

% Cost
Q = diag([100, 100, 100, 0.10, 0.1, 0.1, 100, 100, 100, 100,  0.1, 0.1, 0.1]);
R = diag([0.1, 0.1, 0.1, 0.1]);
cost = zeros(Ns,1);
ode45_intermediate_steps = 10;
cost_integration_grid = linspace(0,Ts,ode45_intermediate_steps);
cost_integration_step_size = Ts/ode45_intermediate_steps;

sqp_iter = [1,3,5,10];
n_sims = length(sqp_iter);
cost_v = zeros(Ns,n_sims);
timeitngs_v = zeros(n_sims,1);
log_v = zeros(Ns,n_sims);
log_iter_v = zeros(Ns,n_sims);

for jj=1:n_sims
    input.x = repmat(Xref0,N+1,1);
    input.u = Uref;
    state_sim = [];
    controls_MPC = [];
    state_sim = X0;
    iter = 0; time = 0;


for i=1:Ns
   
    % Solve NMPC OCP
    input.x0 = state_sim(:,end)';
    input.shifting.strategy = 1;
    output = acado_MPCstep(input);   
    
    log_v(i,jj) = (log_v(i,jj) +  output.info.cpuTime);
    log_iter_v(i,jj) = (log_iter_v(i,jj) +  output.info.nIterations);
    % Save the MPC step
    input.x = output.x;
    input.u = output.u;
    
    for kk=2:sqp_iter(jj)
    input.shifting.strategy = 0;
    output = acado_MPCstep(input);   
    
    log_v(i,jj) = (log_v(i,jj) +  output.info.cpuTime);
    log_iter_v(i,jj) = (log_iter_v(i,jj) +  output.info.nIterations);
    % Save the MPC step
    input.x = output.x;
    input.u = output.u;
    end
    
    
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    
    % Simulate system
    sim_input.x = state_sim(end,:).';

    sim_input.u = output.u(1,:).';

    [~,xtemp] = ode45( @(time, states) dynamics(states,output.u(1,:)), cost_integration_grid, state_sim(:,end) );
    
    % Compute cost
    for j = 1:size(xtemp,1)
        cost_v(i,jj) = cost_v(i,jj) + cost_integration_step_size*((xtemp(j,:) - ref(1:n_XD))*Q*(xtemp(j,:) - ref(1:n_XD))' + (output.u(1,:) - ref(n_XD+1:end))*R*(output.u(1,:) - ref(n_XD+1:end))');
    end
    
    state_sim = [state_sim,xtemp(end,:)'];
    
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' num2str(output.info.cpuTime*1e6) ' µs)'])
    time = [time nextTime];
end
end

%% Plot results
% State and input trajectories
figure()
subplot(4,2,[1 2])
plot(time,state_sim(1:3,:)')
xlabel('time [s]')
ylabel('position [m]')
legend('x','y','z')
grid on

eul_angles = zeros(3,length(time));
for i=1:length(time)
    eul_angles(:,i) = myquat2eul(state_sim(7:10,i));
end

subplot(4,2,[3 4])
plot(time,eul_angles)
xlabel('time [s] ')
ylabel('Euler angles [rad]')
legend('roll','pitch','yaw')
grid on

subplot(4,2,5)
stairs(time(1:end-1),controls_MPC(:,1))
xlabel('time')
ylabel('\omega_1 [rad/s]')
grid on

subplot(4,2,6)
stairs(time(1:end-1),controls_MPC(:,2))
xlabel('time')
ylabel('\omega_2 [rad/s]')
grid on

subplot(4,2,7)
stairs(time(1:end-1),controls_MPC(:,3))
xlabel('time')
ylabel('\omega_3 [rad/s]')
grid on

subplot(4,2,8)
stairs(time(1:end-1),controls_MPC(:,4))
xlabel('time')
ylabel('\omega_4 [rad/s]')
grid on

% savefig('../plots/quadcopter_acado_sim.fig')

% Timings
figure()
subplot(211)
semilogy(time(1:end-1), log_v)
xlabel('time [s]')
ylabel('CPU time [s]')
grid on


subplot(212)
plot(time(1:end-1), log_iter_v)
xlabel('time [s]')
ylabel('total qpOASES it')
grid on
legend('1 SQP step: 80.67 %', '3 SQP steps: 3.86 %', '5 SQP steps: 1.04 %', '10 SQP steps: 0.98 %')


max(log_v)

relative_sub = (sum(cost_v)' - 187.955*ones(n_sims,1))./187.955*100

% 
% log_v_qpoases = log_v;
% log_iter_v_qpoases = log_iter_v;
% save('forces_data.mat','log_v_qpoases','log_iter_v_qpoases')

