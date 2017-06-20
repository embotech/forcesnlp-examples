clc;
clear all;
close all;

Th = 1;
N = 30;

Ts = Th/N;
EXPORT = 1;

timing_iters = 1;
lambda = 1;
sqp_steps = 1; % 2 and 5...
input.shifting.strategy = 1;

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
DifferentialState W1; 
DifferentialState W2; 
DifferentialState W3; 
DifferentialState W4;

Control rateW1;
Control rateW2;
Control rateW3;
Control rateW4;

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
J1 = 0.25;
J2 = 0.25;
J3 = 1;

alpha = 0.0;

f = dot([P1;P2;P3;V1;V2;V3;q1;q2;q3;q4;Omega1;Omega2;Omega3;W1;W2;W3;W4]) == ...  
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
    rateW1;
    rateW2;
    rateW3;
    rateW4];

hover_omega = 39.939;                    
ref = [0.0, 0.0, 0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,hover_omega*ones(1,4), zeros(1,4)];
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
        W4 - ref(17);
        rateW1 - ref(18);
        rateW2 - ref(19);
        rateW3 - ref(20);
        rateW4 - ref(21)];  ...

hN = [  P1 - ref(1); ...
        P2 - ref(2); ...
        P3 - ref(3); ...    
        V1 - ref(4); ...
        V2 - ref(5); ...
        V3 - ref(6); ...
        q1 - ref(7); ...
        q2 - ref(8); ...
        q3 - ref(9); ...
        q4 - ref(10);    ...
        Omega1 - ref(11); ...
        Omega2 - ref(12); ...
        Omega3 - ref(13);
        W1 - ref(14);
        W2 - ref(15);
        W3 - ref(16);
        W4 - ref(17);];  
        

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

max_rate = 40;
ocp.subjectTo( -20.0 <= W1 <= 50.0);
ocp.subjectTo( -20.0 <= W2 <= 50.0);
ocp.subjectTo( -20.0 <= W3 <= 50.0);
ocp.subjectTo( -20.0 <= W4 <= 50.0);

ocp.subjectTo( -10 <= P1 <= 10 );
ocp.subjectTo( -10 <= P2 <= 10 );
ocp.subjectTo( -10 <= P3 <= 10 );

ocp.subjectTo( -100 <= V1 <= 100 );
ocp.subjectTo( -100 <= V2 <= 100 );
ocp.subjectTo( -100 <= V3 <= 100 );
% 
% ocp.subjectTo( -5 <= q1 <= 5 );
% ocp.subjectTo( -5 <= q2 <= 5 );
% ocp.subjectTo( -5 <= q3 <= 5 );

ocp.subjectTo( -40 <= Omega1 <= 40 );
ocp.subjectTo( -40 <= Omega1 <= 40 );
ocp.subjectTo( -40 <= Omega1 <= 40 );

ocp.subjectTo( -max_rate <= rateW1 <= max_rate);
ocp.subjectTo( -max_rate <= rateW2 <= max_rate);
ocp.subjectTo( -max_rate <= rateW3 <= max_rate);
ocp.subjectTo( -max_rate <= rateW4 <= max_rate);
% ocp.subjectTo( P3 >= -3);
ocp.setModel(f);

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_RK4'       );  
mpc.set( 'NUM_INTEGRATOR_STEPS',        1*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
% mpc.set( 'QP_SOLVER',                   'QP_FORCES'    	);
% mpc.set( 'SPARSE_QP_SOLUTION', 'SPARSE_SOLVER' );
% mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-4  );

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
X0 = [0.0 0.0 0.0, 0.0 0.0 0.0, cos(initial_angle_rad/2) sin(initial_angle_rad/2) 0.0 0.0, 0.0 0.0 0.0, hover_omega*ones(1,4)]';

Xref = [0.0 0.0 0.0, 0.0 0.0 0.0, 1 0 0.0 0.0, 0.0 0.0 0.0, hover_omega*ones(1,4)];
input.x = repmat(Xref,N+1,1);

Xref = repmat(Xref,N,1);
input.od = [];

Uref = 0*hover_omega*ones(N,n_U);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);
  
% input.W = diag([100 100 100 0.1 0.1 0.1 100 100 100 100 0.1 0.1 0.1 0.001 0.001 0.001 0.001 0.01 0.01 0.01 0.01]);
% input.WN = diag([100 100 100 0.1 0.1 0.1 100 100 100 100 0.1 0.1 0.1 0.001 0.001 0.001 0.001]);

% w_pos = 100; w_vel = 0.10; w_att = 100; w_omega = 0.10; w_in = 0.1; w_rate = 0.01; cost_scaling = 1; 

w_pos = 50; w_vel = 5.0; w_att = 20.00; w_omega = 5.0; w_in = 0.01; w_rate = 0.01; cost_scaling = 1; 

input.W = diag([w_pos*ones(3,1); w_vel*ones(3,1); w_att*ones(4,1); ...
    w_omega*ones(3,1); w_in*ones(4,1); w_rate*ones(4,1)]);

input.WN = diag([w_pos*ones(3,1); w_vel*ones(3,1); w_att*ones(4,1); ...
    w_omega*ones(3,1); w_in*ones(4,1)]);

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 8;
Ns = floor(Tf/Ts);
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
log = zeros(Ns,2);

% Cost
Q = diag([w_pos*ones(3,1); w_vel*ones(3,1); w_att*ones(4,1); w_omega*ones(3,1); w_in*ones(4,1)]);
R = diag(w_rate*ones(4,1));
cost = zeros(Ns,1);
ode45_intermediate_steps = 10;
cost_integration_grid = linspace(0,Ts,ode45_intermediate_steps);
cost_integration_step_size = Ts/ode45_intermediate_steps;
% do some initialization to avoid bad local minimum


for i=1:Ns
   
    % Solve NMPC OCP
    input.x0 = state_sim(:,end)';
    
    for k = 1:sqp_steps
        temp_time = Inf;
        if (k ==1)
            input.shifting.strategy = 1;
        else
            input.shifting.strategy = 0;
        end
        for j = 1:timing_iters
            output = acado_MPCstep(input);
            if temp_time > output.info.cpuTime*1e6;
                temp_time = output.info.cpuTime*1e6;
            end
        end
        
        step = norm([reshape(output.x, n_XD*(N+1), 1); reshape(output.u, n_U*(N),1) ] - [reshape(input.x, n_XD*(N+1), 1); reshape(input.u, n_U*(N),1) ]);
    
        input.x = input.x + lambda*(output.x - input.x);
        input.u = input.u + lambda*(output.u - input.u);
        log(i,1) = log(i,1) + output.info.nIterations;
        log(i,2) = log(i,2) + temp_time;
    end
    

    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
        
    % Simulate system
    sim_input.x = state_sim(end,:).';

    sim_input.u = output.u(1,:).';

    [~,xtemp] = ode45( @(time, states) dynamics(states,output.u(1,:)), cost_integration_grid, state_sim(:,end) );
    
    % Compute cost
    for j = 1:size(xtemp,1)
        cost(i) = cost(i) + cost_integration_step_size*((xtemp(j,:) - ref(1:n_XD))*Q*(xtemp(j,:) - ref(1:n_XD))' + (output.u(1,:) - ref(n_XD+1:end))*R*(output.u(1,:) - ref(n_XD+1:end))');
    end
    
    state_sim = [state_sim, xtemp(end,:)';];

    display(num2str(norm(xtemp(end,:)' - input.x(2,:).')))
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step: ' num2str(output.info.cpuTime*1e6) ' µs, status = ' num2str(output.info.status) ',  KKT tol = '  num2str(output.info.kktValue) '   step =' num2str(step) ')' ])
    time = [time nextTime];
%     if (output.info.status~=1) 
%         error('some problem...')
%     end
    
end

%% Plot results
% State and input trajectories
h1 = figure(1)
subplot(6,2,[1 2])
plot(time,state_sim(1:3,:)')
xlabel('time [s]')
ylabel('position [m]')
legend('x','y','z')
grid on

eul_angles = zeros(3,length(time));
for i=1:length(time)
    eul_angles(:,i) = myquat2eul(state_sim(7:10,i));
end

subplot(6,2,[3 4])
plot(time,eul_angles)
xlabel('time [s] ')
ylabel('Euler angles [rad]')
legend('roll','pitch','yaw')
grid on

subplot(6,2,5)
stairs(time(1:end),state_sim(15,:)')
xlabel('time')
legend('\omega_1')
grid on

subplot(6,2,6)
stairs(time(1:end),state_sim(15,:)')
xlabel('time')
legend('\omega_2')
grid on

subplot(6,2,7)
stairs(time(1:end),state_sim(16,:)')
xlabel('time')
legend('\omega_3')
grid on

subplot(6,2,8)
stairs(time(1:end),state_sim(17,:)')
xlabel('time')
legend('\omega_4')
grid on

subplot(6,2,9)
stairs(time(1:end-1),controls_MPC(:,1))
xlabel('time')
legend('\omega_1')
grid on

subplot(6,2,10)
stairs(time(1:end-1),controls_MPC(:,2))
xlabel('time')
legend('\omega_2')
grid on

subplot(6,2,11)
stairs(time(1:end-1),controls_MPC(:,3))
xlabel('time')
legend('\omega_3')
grid on

subplot(6,2,12)
stairs(time(1:end-1),controls_MPC(:,4))
xlabel('time')
legend('\omega_4')
grid on

% savefig(h1,'../plots/quadcopter_acado_sim.fig')

% Timings
h2 = figure(2)
subplot(2,1,1)
plot(time(1:end-1), log(:,1))
xlabel('time')
ylabel('iterations')
grid on

subplot(2,1,2)
semilogy(time(1:end-1), log(:,2)/1e6)
xlabel('time')
ylabel('CPU time')
grid on



% savefig(h2,'../plots/quadcopter_acado_stat.fig')

% h3 = figure()
% plot(sum(state_sim(7:10,:).^2))
% xlabel('time')
% ylabel('Quaternion norm')

% plot closed loop cost
% h4=figure(10)
% plot(time(1:end-1),cost);
% xlabel('time [s]')
% ylabel('cost')
% grid on

sum(cost)




