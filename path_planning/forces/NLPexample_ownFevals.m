% Example script for getting started with FORCES NLP solver.
%
%--------------------------------------------------------------------------
% NOTE: This example shows how to pass C functions implementing the
% function evaluations to FORCES. There is an automated way of creating
% these C functions, which is explained in the other example file named
% "NLPexample_autoFevals.m"
%--------------------------------------------------------------------------
%
% This example solves an optimization problem for a car with the simple
% continuous-time, nonlinear dynamics
%
%    dx/dt = v*cos(theta)
%    dy/dt = v*sin(theta)
%    dv/dt = F/m
%    dtheta/dt = s/I
%
% where x,y are the position, v the velocity in heading angle theta of the
% car. The inputs are F (accelerating force) and steering s.
%
% The car starts from standstill with a certain heading angle, and the
% optimization problem is to maximize progress in y direction while staying
% inside a non-convex feasible region.
%
% Quadratic costs for the acceleration force and steering are added to the
% objective to avoid excessive maneouvers.
%
% There are bounds on all variables.
%
% Variables are collected stage-wise into z = [F s x y v theta].
%
% See also FORCES_NLP
%
% (c) embotech GmbH, Zurich, Switzerland, 2013-16.

clear; clc; close all;
deg2rad = @(deg) deg/180*pi; % convert degrees into radians
rad2deg = @(rad) rad/pi*180; % convert radians into degrees

%% Problem dimensions
model.N = 50;            % horizon length
model.nvar = 6;          % number of variables
model.neq  = 4;          % number of equality constraints
model.nh = 2;            % number of inequality constraint functions

%% Define source file containing function evaluation code
model.extfuncs = 'C/myfevals.c';
% Note: we will add additional source files required below at code options!

% Indices on LHS of dynamical constraint - for efficiency reasons, make
% sure the matrix E has structure [0 I] where I is the identity matrix.
model.E = [zeros(4,2), eye(4)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs    |        states
%             F    s       x     y     v    theta
model.lb = [ -5,  -1,      -3,    0    0     0  ];
model.ub = [ +5,  +1,       0,    3    2    +pi ];

% General (differentiable) nonlinear inequalities hl <= h(x) <= hu
model.ineq = @(z)  [   z(3)^2 + z(4)^2; 
                    (z(3)+2)^2 + (z(4)-2.5)^2 ];
                
% Upper/lower bounds for inequalities
model.hu = [9,  +inf]';
model.hl = [1,  0.95^2]';

%% Initial and final conditions

% Initial condition on vehicle states
model.xinit = [-2, 0, 0, deg2rad(120)]'; % x=-2, y=0, v=0 (standstill), heading angle=120?
model.xinitidx = 3:6; % use this to specify on which variables initial conditions are imposed

% Final condition on vehicle velocity and heading angle
model.xfinal = [0, deg2rad(0)]'; % v final=0 (standstill), heading angle final=0?
model.xfinalidx = 5:6; % use this to specify on which variables final conditions are imposed


%% Define solver options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.maxit = 200;    % Maximum number of iterations
codeoptions.printlevel = 0; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 0;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

% change this to your server or leave uncommented for using the standard
% embotech server at https://www.embotech.com/codegen
% codeoptions.server = 'http://embotech-server2.ee.ethz.ch:8114/v1.5.beta'; 

% add additional source files required - separate by spaces if more than 1
codeoptions.nlp.other_srcs = 'C/car_dynamics.c';

%% Generate forces solver
FORCES_NLP(model, codeoptions);

%% Call solver
% Set initial guess to start solver from:
x0i=model.lb+(model.ub-model.lb)/2;
x0=repmat(x0i',model.N,1);
problem.x0=x0; 

% Set initial and final conditions. This is usually changing from problem
% instance to problem instance:
problem.xinit = model.xinit;
problem.xfinal = model.xfinal;

% Time to solve the NLP!
[output,exitflag,info] = FORCESNLPsolver(problem);

% Make sure the solver has exited properly. 
assert(exitflag == 1,'Some problem in FORCES solver');
fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);

%% Plot results
TEMP = zeros(model.nvar,model.N);
for i=1:model.N
    TEMP(:,i) = output.(['x',sprintf('%02d',i)]);
end
U = TEMP(1:2,:);
X = TEMP(3:6,:);

% plot trajectory
figure(1); clf;
plot(X(1,:),X(2,:),'b-'); hold on; 
rectangle('Position',[-sqrt(model.hl(1)) -sqrt(model.hl(1)) 2*sqrt(model.hl(1)) 2*sqrt(model.hl(1))],'Curvature',[1 1],'EdgeColor','r','LineStyle',':');
rectangle('Position',[-sqrt(model.hu(1)) -sqrt(model.hu(1)) 2*sqrt(model.hu(1)) 2*sqrt(model.hu(1))],'Curvature',[1 1],'EdgeColor','r','LineStyle',':');
rectangle('Position',[-2-sqrt(model.hl(2)) 2.5-sqrt(model.hl(2)) 2*sqrt(model.hl(2)) 2*sqrt(model.hl(2))],'Curvature',[1 1],'EdgeColor','r','LineStyle',':');
plot(model.xinit(1),model.xinit(2),'bx','LineWidth',3); 
title('position'); xlim([-2.5 0]); ylim([0 3]); xlabel('$y$ position'); ylabel('$z position');

% plot heading angle and velocity variables
figure(2); clf;
subplot(2,2,1); plot(X(3,:)); grid on; title('velocity'); hold on; 
xlabel('time [s]')
% plot([1 model.N], [model.ub(5) model.ub(5)]', 'r:');
% plot([1 model.N], [model.lb(5) model.lb(5)]', 'r:');
subplot(2,2,2); plot(rad2deg(X(4,:))); grid on; title('heading angle'); ylim([0, 180]); hold on; 
xlabel('time [s]')
% plot([1 model.N], rad2deg([model.ub(6) model.ub(6)])', 'r:');
% plot([1 model.N], rad2deg([model.lb(6) model.lb(6)])', 'r:');

% plot inputs
subplot(2,2,3); stairs(U(1,:)); grid on; title('acceleration force'); hold on; 
xlabel('time [s]')
% plot([1 model.N], [model.ub(1) model.ub(1)]', 'r:');
% plot([1 model.N], [model.lb(1) model.lb(1)]', 'r:');
subplot(2,2,4); stairs(U(2,:)); grid on; title('delta steering'); hold on; 
xlabel('time [s]')
% plot([1 model.N], [model.ub(2) model.ub(2)]', 'r:');
% plot([1 model.N], [model.lb(2) model.lb(2)]', 'r:');
