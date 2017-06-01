clc;
clear all;
close all;

T = 2.0;
N = 20;
Ts = T/N;
EXPORT  = 0;
COMPILE = 0;

Tsim = 20;
Ns = Tsim/(T/N);   % simulation length?

timing_iters = 1;

%% Differential Equation

DifferentialState th1;
DifferentialState th1d;
DifferentialState th2;
DifferentialState th2d;
DifferentialState tau1;
DifferentialState tau2;

Control tau1r tau2r;

% Motor parameters
mM1 	= 0.3;
mM2 	= 0.3;
kr1 	= 1.0;
kr2 	= 1.0;
rM1     = 0.05;
rM2     = 0.05;
Im1 	= 1.0/2.0*mM1*rM1*rM1;
Im2 	= 1.0/2.0*mM2*rM2*rM2;

% Link parameters:

ml1 	= 3.0;
ml2 	= 3.0;
l1 	= 1.0;
l2 	= 1.0;
Il1	= 1.0/12.0*ml1*ml1*l1*l1;
Il2	= 1.0/12.0*ml2*ml2*l2*l2;
a1 	= l1/2.0;
a2 	= l2/2.0;

% Others
g = 9.81;

% Bounds
% tau_max = 70;
% tau_min = -100;
% thd_min = -100;
% thd_max = 100;
% th_max = 100;
% th_min = -100;
% taur_max = 200;
% taur_min = -200;

tau_max = 70;
tau_min = -100;
thd_min = -100;
thd_max = 100;
th_max = pi;
th_min = -pi;
taur_max = 200;
taur_min = -200;


% DEFINE A DIFFERENTIAL EQUATION:
% -------------------------------
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

f_expl = [  dot(th1)    == th1d;
            dot(th1d) 	== theta1dd_exp;
            dot(th2)    == th2d;
            dot(th2d) 	== 1.0/beta_2*(tau2 - beta_1*theta1dd_exp - beta_3*th1d*th1d - K_beta);
            dot(tau1) 	== tau1r;
            dot(tau2) 	== tau2r];

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 2;
sim = acado.SIMexport( Ts );
sim.setModel(f_expl);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' );
    
    cd export_SIM
    make_acado_integrator('../integrate_dynamics')
    cd ..
end

% DEFINE LEAST SQUARE FUNCTION:
% -----------------------------
acadoSet('problemname', 'mpc');

h  = [th1, th1d, th2, th2d, tau1, tau2, tau1r, tau2r];


hN =  [ th1, th1d, th2, th2d, tau1, tau2 ];

% Optimal Control Problem

ocp = acado.OCP(0.0, T, N);

W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));

ocp.minimizeLSQ( W, h);
ocp.minimizeLSQEndTerm( WN,hN );


ocp.subjectTo( tau_min 	<= tau1 <= tau_max );
ocp.subjectTo( tau_min 	<= tau2 <= tau_max );

ocp.subjectTo( thd_min <= th1d <= thd_max);
ocp.subjectTo( thd_min <= th2d <= thd_max );

ocp.subjectTo( th_min 	<= th1 <= th_max  );   % nonlinear constraints (make RTI fail)
ocp.subjectTo( th_min 	<= th2 <= th_max  ); % nonlinear constraints (make RTI fail)

ocp.subjectTo( taur_min 	<= tau1r <= taur_max  );
ocp.subjectTo( taur_min 	<= tau2r <=  taur_max );

ocp.setModel(f_expl);

mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'INTEGRATOR_TYPE',             'INT_RK4'           );
mpc.set( 'NUM_INTEGRATOR_STEPS',        1*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',           'YES'              );

% mpc.set( 'QP_SOLVER',                   'QP_FORCES'    	);
% mpc.set( 'SPARSE_QP_SOLUTION', 'SPARSE_SOLVER' );
% mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-1 );

if EXPORT
    mpc.exportCode( 'export_MPC' );
end

if COMPILE
    
    cd export_MPC
    make_acado_solver('../acado_solver_mex')
    cd ..
end

%% PARAMETERS SIMULATION
X0 = [-0.4 0 0.4 0 0 0 ];
Xref = [1.2 0 -1.2 0 0 0 ];
input.x = [repmat(X0,N/2,1); repmat(X0,N/2+1 ,1)]; %repmat(Xref,N/2+1,1)];
input.od = [];

Uref = zeros(N,2);
input.u = Uref;

input.y = [repmat(Xref,N,1) Uref];
input.yN = Xref;

input.W = zeros(8,8); % LSQ coefficient matrix
% Q = diag([1000 1000 1000 1000 0.001 0.001]);
% R = diag([0.001 0.001]);

Q = diag([1000 0.1 1000 0.1 0.01 0.01]);
R = diag([0.01 0.01]);

input.W(1,1) = Q(1,1);
input.W(2,2) = Q(2,2);
input.W(3,3) = Q(3,3);
input.W(4,4) = Q(4,4);
% input.W(5,5) = 0.001;
% input.W(6,6) = 0.001;
% input.W(7,7) = 0.001;
% input.W(8,8) = 0.001;

input.W(5,5) = Q(5,5);
input.W(6,6) = Q(6,6);
input.W(7,7) = R(1,1);
input.W(8,8) = R(2,2);



input.WN = zeros(6,6); % LSQ coefficient matrix
input.WN(1,1) = Q(1,1);
input.WN(2,2) = Q(2,2);
input.WN(3,3) = Q(3,3);
input.WN(4,4) = Q(4,4);
% input.WN(5,5) = 0.001;
% input.WN(6,6) = 0.001;

input.WN(5,5) = Q(5,5);
input.WN(6,6) = Q(6,6);
input.shifting.strategy = 1;

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 30;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
timings = [];
qpiter  = [];

% Cost


cost = zeros(Ns,1);
ode45_intermediate_steps = 10;
cost_integration_grid = linspace(0,Ts,ode45_intermediate_steps);
cost_integration_step_size = Ts/ode45_intermediate_steps;

for i=1:Ns/2
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    if input.x0 >= [th_max thd_max th_max thd_max tau_max tau_max];
        error('bounds on the initial state violated!')
    end
    if input.x0 <= [th_min thd_min th_min thd_min tau_min tau_min];
        error('bounds on the initial state violated!')
    end
    input.x(1,:) = state_sim(end,:);
    temp_time = Inf;
    for j = 1:timing_iters
        output = acado_solver_mex(input);
        if temp_time > output.info.cpuTime*1e6;
            temp_time = output.info.cpuTime*1e6;
        end
    end
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    
    input.x = output.x;
    input.u = output.u;
    
    % Simulate system
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    
    [~,xtemp] = ode45( @(time, states) dynamics(states,output.u(1,:)), cost_integration_grid, state_sim(end,:) );
    
    % Compute cost
    for j = 1:length(xtemp)
        cost(i) = cost(i) + cost_integration_step_size*(xtemp(j,:)*Q*xtemp(j,:)' + output.u(1,:)*R*output.u(1,:)');
    end

    state_sim = [state_sim; xtemp(end,:)];
    
    iter = iter+1;
    nextTime = iter*Ts;
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step -- QP error: ' num2str(output.info.status) ',' ' ' char(2) ' KKT val: ' num2str(output.info.kktValue,'%1.2e') ',' ' ' char(2) ' CPU time: ' num2str(round(output.info.cpuTime*1e6)) ' µs)'])
    timings = [timings temp_time];
    qpiter  = [qpiter  output.info.nIterations];
    time = [time nextTime];
    
    %     visualize(time, state_sim, Xref, xmin, xmax, l);
end

Xref = -Xref;
input.y = [repmat(Xref,N,1) Uref];
input.yN = Xref;

for i=Ns/2+1:Ns
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
    input.x(1,:) = state_sim(end,:);
    
    temp_time = Inf;
    for j = 1:timing_iters
        output = acado_solver_mex(input);
        if temp_time > output.info.cpuTime*1e6;
            temp_time = output.info.cpuTime*1e6;
        end
    end
    
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    
    input.x = output.x;
    input.u = output.u;
    
    % Simulate system
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    sim_input.od = 0.2;
    [~,xtemp] = ode45( @(time, states) dynamics(states,output.u(1,:)), cost_integration_grid, state_sim(end,:) );
    
    % Compute cost
    for j = 1:length(xtemp)
        cost(i) = cost(i) + cost_integration_step_size*(xtemp(j,:)*Q*xtemp(j,:)' + output.u(1,:)*R*output.u(1,:)');
    end

    state_sim = [state_sim; xtemp(end,:)];
    
    iter = iter+1;
    nextTime = iter*Ts;
    disp(['current time: ' num2str(nextTime) '   ' char(9) ' (RTI step -- QP error: ' num2str(output.info.status) ',' ' ' char(2) ' KKT val: ' num2str(output.info.kktValue,'%1.2e') ',' ' ' char(2) ' CPU time: ' num2str(round(output.info.cpuTime*1e6)) ' µs)'])
    timings = [timings temp_time];
    qpiter  = [qpiter  output.info.nIterations];
    time = [time nextTime];
    
end

h1=figure();
subplot(4,1,1);
plot(time, state_sim(:,1)); hold on;
plot(time, state_sim(:,3)); hold on;
ylabel('joint angles [rad]');
xlabel('time [s]')
ylim([-1.5 1.5])
grid on

subplot(4,1,2);
plot(time(1:end),  state_sim(:,5)); hold on;
plot(time(1:end),  state_sim(:,6)); hold on;
plot([0 time(end)], [70 70], 'r--');
ylim([-20 80])
ylabel('torques [Nm]');
xlabel('time [s]')
grid on

subplot(4,1,3);
stairs(time(1:end-1), controls_MPC(:,1)); hold on;
plot([0 time(end)], [-200 -200], 'r--');
plot([0 time(end)], [200 200], 'r--');
ylabel('\tau_{r1}');
xlabel('time [s]')
ylim([-1.1*200 1.1*200])
grid on

subplot(4,1,4);
stairs(time(1:end-1), controls_MPC(:,2)); hold on;
plot([0 time(end)], [-200 -200], 'r--');
plot([0 time(end)], [200 200], 'r--');
ylabel('\tau_{r2}');
xlabel('time [s]')
ylim([-1.1*200 1.1*200])
grid on

% savefig(h1,'../plots/robot_acado_sim.fig')

h2=figure();
subplot(211)
plot(time(1:end-1), qpiter)
xlabel('time(s)')
ylabel('iterations')
grid on

subplot(212)
semilogy(time(1:end-1), timings/1e6)
xlabel('time(s)')
ylabel('time (us)')
grid on

% savefig(h2,'../plots/robot_acado_stat.fig')

% Compute and plot closed loop cost
% Ns = length(time) -1;
% cost = zeros(Ns,1);
% Q = diag([1000 1000 1000 1000 0.001 0.001]);
% R = diag([0.001 0.001]);
% cost(1,1) = state_sim(1,:)*Q*state_sim(1,:)' + controls_MPC(1,:)*R*controls_MPC(1,:)';
% for i=2:Ns
%     cost(i,1) = cost(i-1) + state_sim(i,:)*Q*state_sim(i,:)' + controls_MPC(i,:)*R*controls_MPC(i,:)';
% end
h3=figure(10)
plot(time(1:end-1),cost);
xlabel('time [s]')
ylabel('cost')
grid on

sum(cost)
% savefig(h3,'../plots/robot_acado_cost.fig')
