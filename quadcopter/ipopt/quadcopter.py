import sys
sys.path.append(r"/home/andrea/casadi-py27-np1.9.1-v2.4.2")
from casadi import *
from numpy import *
from scipy.linalg import *
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from math import atan2, asin
import pdb

N = 5      # Control discretization
T = 1.0     # End time
nx = 17
nu = 4
T_sim = 8.0
N_sim = int(ceil(T_sim/(T/N)))

# Declare variables (use scalar graph)
u  = SX.sym("u",nu)    # control
x  = SX.sym("x",nx)    # states

# Dynamics definition

rho = 1.23
A = 0.1
Cl = 0.25
Cd = 0.3*Cl
m = 10
g = 9.81
L = 0.5
Jp = 1e-2
xi = 1e-2
J1 = 0.25
J2 = 0.25
J3 = 1
gain = 1e-4
alpha = 0.0

P1 = x[0]
P2 = x[1]
P3 = x[2]
V1 = x[3]
V2 = x[4]
V3 = x[5]
q1 = x[6]
q2 = x[7]
q3 = x[8]
q4 = x[9]
Omega1 = x[10]
Omega2 = x[11]
Omega3 = x[12]
W1 = x[13]
W2 = x[14]
W3 = x[15]
W4 = x[16]

rW1 = u[0]
rW2 = u[1]
rW3 = u[2]
rW4 = u[3]


ode = vertcat([         V1,\
                        V2,\
                        V3,\
                        (A*Cl*rho*(2*q1*q3 + 2*q2*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m),\
                        -(A*Cl*rho*(2*q1*q2 - 2*q3*q4)*(W1*W1 + W2*W2 + W3*W3 + W4*W4))/(2*m),\
                        (A*Cl*rho*(W1*W1 + W2*W2 + W3*W3 + W4*W4)*(q1*q1 - q2*q2 - q3*q3 + q4*q4))/(2*m) - g,\
                        - (Omega1*q2)/2 - (Omega2*q3)/2 - (Omega3*q4)/2 - (alpha*q1*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4),\
                        (Omega1*q1)/2 - (Omega3*q3)/2 + (Omega2*q4)/2 - (alpha*q2*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4),\
                        (Omega2*q1)/2 + (Omega3*q2)/2 - (Omega1*q4)/2 - (alpha*q3*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4),\
                        (Omega3*q1)/2 - (Omega2*q2)/2 + (Omega1*q3)/2 - (alpha*q4*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1))/(q1*q1 + q2*q2 + q3*q3 + q4*q4),\
                        (J3*Omega2*Omega3 - J2*Omega2*Omega3 + (A*Cl*L*rho*(W2*W2 - W4*W4))/2)/J1,\
                        -(J3*Omega1*Omega3 - J1*Omega1*Omega3 + (A*Cl*L*rho*(W1*W1 - W3*W3))/2)/J2,\
                        (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3,\
                        rW1,\
                        rW2,\
                        rW3,\
                        rW4])

f = SXFunction([x,u],[ode])
f.init()

# RK4 with M steps
U = MX.sym("U",nu)
X = MX.sym("X",nx)

M = 1 ; DT = T/(N*M)
XF = X
QF = 0
for j in range(M):
    [k1] = f([XF,             U])
    [k2] = f([XF + DT/2 * k1, U])
    [k3] = f([XF + DT/2 * k2, U])
    [k4] = f([XF + DT   * k3, U])
    XF += DT/6*(k1   + 2*k2   + 2*k3   + k4)
F = MXFunction([X,U],[XF])
F.init()

# Formulate NLP (use matrix graph)
nv = nu*N + nx*(N+1)
v = MX.sym("v", nv)

# Get the state for each shooting interval
xk = [v[(nx + nu)*k : (nx + nu)*k + nx] for k in range(N+1)]

# Get the control for each shooting interval
uk = [v[(nx + nu)*k + nx:(nx + nu)*k + nx + nu] for k in range(N)]

# Variable bounds and initial guess
vmin = -100*ones(nv)
vmax =  100*ones(nv)
#vmin[nu*N + nx*(N):] = -5*ones(nx)
#vmax[nu*N + nx*(N):] = 5*ones(nx)

v0   =  zeros(nv)

# Control bounds
max_rate = 40

vmin[0::nx+nu] = -10
vmin[1::nx+nu] = -10
vmin[2::nx+nu] = -10
vmin[3::nx+nu] = -100
vmin[4::nx+nu] = -100
vmin[5::nx+nu] = -100
vmin[6::nx+nu] = -5
vmin[7::nx+nu] = -5
vmin[8::nx+nu] = -5
vmin[9::nx+nu] = -5
vmin[10::nx+nu] = -40
vmin[11::nx+nu] = -40
vmin[12::nx+nu] = -40
vmin[13::nx+nu] = -50
vmin[14::nx+nu] = -50
vmin[15::nx+nu] = -50
vmin[16::nx+nu] = -50

vmin[nx+0::nx+nu] = -max_rate
vmin[nx+1::nx+nu] = -max_rate
vmin[nx+2::nx+nu] = -max_rate
vmin[nx+3::nx+nu] = -max_rate

vmax[0::nx+nu] = 10
vmax[1::nx+nu] = 10
vmax[2::nx+nu] = 10
vmax[3::nx+nu] = 100
vmax[4::nx+nu] = 100
vmax[5::nx+nu] = 100
vmax[6::nx+nu] = 5
vmax[7::nx+nu] = 5
vmax[8::nx+nu] = 5
vmax[9::nx+nu] = 5
vmax[10::nx+nu] = 40
vmax[11::nx+nu] = 40
vmax[12::nx+nu] = 40
vmax[13::nx+nu] = 50
vmax[14::nx+nu] = 50
vmax[15::nx+nu] = 50
vmax[16::nx+nu] = 50

vmax[nx+0::nx+nu] = max_rate
vmax[nx+1::nx+nu] = max_rate
vmax[nx+2::nx+nu] = max_rate
vmax[nx+3::nx+nu] = max_rate

initial_angle_rad = 3.0

# Initial condition
hover_omega = 39.939
x0 = array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cos(initial_angle_rad/2.0), sin(initial_angle_rad/2.0), 0.0, 0.0, 0.0, 0.0, 0.0, hover_omega, hover_omega, hover_omega, hover_omega])

for i in range(nx):
	vmin[i] = vmax[i] = v0[i] = x0[i]

# Initial solution guess
v0 = 1.0*ones(nv)
# for k in range(N):
#     v0[(nx + nu)*k + nx:(nx + nu)*k + nx + nu] = [40, 40, 40, 40]

# Constraint function with bounds
g = []; gmin = []; gmax = []

# Objective function
J=0.0

# Build up a graph of integrator calls
w_pos = 50.0
w_vel = 5.0
w_att = 20.00
w_omega = 5.0
w_in = 0.01
w_rate = 0.01

H_end_diag = array([w_pos, w_pos, w_pos, w_vel, w_vel, w_vel, w_att, w_att, w_att, w_att, w_omega, w_omega, w_omega, w_in, w_in, w_in, w_in])
H_diag = array([w_pos, w_pos, w_pos, w_vel, w_vel, w_vel, w_att, w_att, w_att, w_att, w_omega, w_omega, w_omega, w_in, w_in, w_in, w_in, w_rate, w_rate, w_rate, w_rate])

for k in range(N):
    # Call the integrator
    [xf] = F([xk[k],uk[k]])

    # position
    J+= 1.0/2.0*H_diag[0]*pow(xk[k][0],2)
    J+= 1.0/2.0*H_diag[1]*pow(xk[k][1],2)
    J+= 1.0/2.0*H_diag[2]*pow(xk[k][2],2)

    # velocities
    J+= 1.0/2.0*H_diag[3]*pow(xk[k][3],2)
    J+= 1.0/2.0*H_diag[4]*pow(xk[k][4],2)
    J+= 1.0/2.0*H_diag[5]*pow(xk[k][5],2)

    # attitude
    J+= 1.0/2.0*H_diag[6]*pow(xk[k][6] - 1.0,2)
    J+= 1.0/2.0*H_diag[7]*pow(xk[k][7],2)
    J+= 1.0/2.0*H_diag[8]*pow(xk[k][8],2)
    J+= 1.0/2.0*H_diag[9]*pow(xk[k][9],2)

    # omega
    J+= 1.0/2.0*H_diag[10]*pow(xk[k][10],2)
    J+= 1.0/2.0*H_diag[11]*pow(xk[k][11],2)
    J+= 1.0/2.0*H_diag[12]*pow(xk[k][12],2)

    # inputs
    J+= 1.0/2.0*H_diag[13]*pow(xk[k][13] - hover_omega,2)
    J+= 1.0/2.0*H_diag[14]*pow(xk[k][14] - hover_omega,2)
    J+= 1.0/2.0*H_diag[15]*pow(xk[k][15] - hover_omega,2)
    J+= 1.0/2.0*H_diag[16]*pow(xk[k][16] - hover_omega,2)

    for j in range(nu):
        J+= 1.0/2.0*H_diag[nx+j]*pow(uk[k][j],2)

    g.append(xf - xk[k+1])
    gmin.append(zeros(nx))
    gmax.append(zeros(nx))

# Terminal cost
k = N
# position
J+= 1.0/2.0*H_end_diag[0]*pow(xk[k][0],2)
J+= 1.0/2.0*H_end_diag[1]*pow(xk[k][1],2)
J+= 1.0/2.0*H_end_diag[2]*pow(xk[k][2],2)

# velocities
J+= 1.0/2.0*H_end_diag[3]*pow(xk[k][3],2)
J+= 1.0/2.0*H_end_diag[4]*pow(xk[k][4],2)
J+= 1.0/2.0*H_end_diag[5]*pow(xk[k][5],2)

# attitude
J+= 1.0/2.0*H_end_diag[6]*pow(xk[k][6] - 1.0,2)
J+= 1.0/2.0*H_end_diag[7]*pow(xk[k][7],2)
J+= 1.0/2.0*H_end_diag[8]*pow(xk[k][8],2)
J+= 1.0/2.0*H_end_diag[9]*pow(xk[k][9],2)

# omega
J+= 1.0/2.0*H_end_diag[10]*pow(xk[k][10],2)
J+= 1.0/2.0*H_end_diag[11]*pow(xk[k][11],2)
J+= 1.0/2.0*H_end_diag[12]*pow(xk[k][12],2)

# inputs
J+= 1.0/2.0*H_end_diag[13]*pow(xk[k][13] - hover_omega,2)
J+= 1.0/2.0*H_end_diag[14]*pow(xk[k][14] - hover_omega,2)
J+= 1.0/2.0*H_end_diag[15]*pow(xk[k][15] - hover_omega,2)
J+= 1.0/2.0*H_end_diag[16]*pow(xk[k][16] - hover_omega,2)

# Concatenate constraints
g = vertcat(g)
gmin = concatenate(gmin)
gmax = concatenate(gmax)

# Gauss-Newton hessian:
H = block_diag(H_diag)
H_end = block_diag(H_end_diag)
Hgn = kron(eye(N),H)
Hgn = block_diag(Hgn,H_end)

h=SXFunction(hessLagIn(), hessLagOut(hess=Hgn))

# Create NLP solver instance
opts = {'jit':True,"jit_options":{"flags":['-O0']}}
nlp = MXFunction('nlp',nlpIn(x=v),nlpOut(f=J,g=g),opts)
# nlp = MXFunction('nlp',nlpIn(x=v),nlpOut(f=J,g=g))
solver = NlpSolver("nlp_solver", "ipopt", nlp)

# nlp = MXFunction(nlpIn(x=v),nlpOut(f=J,g=g))
# solver = NlpSolver("ipopt", nlp)

#solver.setOption("tol",1.0e-4)
#solver.setOption("constr_viol_tol",1.0e-4)
#solver.setOption("compl_inf_tol",1.0e-4)
#solver.setOption("dual_inf_tol",1.0e-4)
#solver.setOption("accept_every_trial_step","yes")
solver.setOption("limited_memory_update_type","bfgs")
solver.setOption("print_level",5)
solver.setOption("output_file","ipopt_log.txt")
# solver.setOption("linear_solver","ma57")

solver.init()

# Set bounds and initial guess
solver.setInput(v0,   "x0")
solver.setInput(vmin, "lbx")
solver.setInput(vmax, "ubx")
solver.setInput(gmin, "lbg")
solver.setInput(gmax, "ubg")

# Simulation loop
X = zeros((nx,N_sim))
U = zeros((nu,N_sim))
full_time = zeros(N_sim)
time      = zeros(N_sim)
iter      = zeros(N_sim)

#pdb.set_trace()
for i in range(N_sim):

	# Solve the problem
	solver.evaluate()
	stat = solver.getStats()
	full_time[i] = stat["t_mainloop.proc"]
	time[i] 	 = stat["t_mainloop.proc"] - stat["t_eval_f.proc"] - stat["t_eval_g.proc"] - stat["t_eval_grad_f.proc"] - stat["t_eval_jac_g.proc"] - stat["t_eval_h.proc"]
	iter[i] = stat["iter_count"]

	# Retrieve the solution
	v_opt = solver.getOutput("x")
	for j in range(nx):
		X[j,i] = v_opt[j]

	for j in range(nu):
		U[j,i] = v_opt[nx+j]

	# Update initial condition
	for j in range(nx):
		vmin[j] = vmax[j] = v_opt[nx+nu+j]

	solver.init()

	solver.setInput(v0,   "x0")
	solver.setInput(vmin, "lbx")
	solver.setInput(vmax, "ubx")
	solver.setInput(gmin, "lbg")
	solver.setInput(gmax, "ubg")

# Plot the results
plt.figure(1)
plt.clf()
plt.subplot(311)
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[0,:],'--')
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[1,:],'--')
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[2,:],'--')
plt.title("Quadcopter - Ipopt")
plt.xlabel('time')
plt.legend(['x','y','z'])
plt.grid()

# Convert quaternions to Euler angles (rad)
angles = zeros((3,N_sim))
for i in range(N_sim):
	q0 = X[6,i]
	q1 = X[7,i]
	q2 = X[8,i]
	q3 = X[9,i]

	angles[0,i] = atan2(2*(q0*q1 + q2*q3),(1 - 2*(q1**2 + q2**2)))
	angles[1,i] = asin(2*(q0*q2 - q3*q1))
	angles[2,i] = atan2(2*(q0*q3 + q1*q2),(1 - 2*(q2**2 + q3**2)))

plt.subplot(312)
plt.plot(linspace(0,DT*M*N_sim,N_sim),angles[0,:])
plt.plot(linspace(0,DT*M*N_sim,N_sim),angles[1,:])
plt.plot(linspace(0,DT*M*N_sim,N_sim),angles[2,:])
plt.xlabel('time')
plt.legend(['phi','theta','psi'])
plt.grid()

#plt.step(linspace(0,T,N),u_opt,'-.')
plt.subplot(313)
plt.step(linspace(0,DT*M*N_sim,N_sim),U[0,:])
plt.step(linspace(0,DT*M*N_sim,N_sim),U[1,:])
plt.step(linspace(0,DT*M*N_sim,N_sim),U[2,:])
plt.step(linspace(0,DT*M*N_sim,N_sim),U[3,:])
plt.xlabel('time')
plt.legend(['w1','w2','w3','w4'])
plt.grid()
#plt.legend(['force'])

plt.figure(2)
plt.subplot(211)
plt.plot(linspace(0,DT*M*N_sim,N_sim),time)
#plt.plot(linspace(0,T,N_sim),full_time,'--')
plt.xlabel('time')
plt.legend(['CPU time w/o f eval','total CPU time'])
plt.grid()

plt.subplot(212)
plt.plot(linspace(0,DT*M*N_sim,N_sim),iter)
plt.xlabel('time')
plt.legend(['iter'])
plt.grid()

# Store results
savez('quadcopter_sim_ipopt.mpy', X=X, U=U, time=time, full_time=full_time, iter=iter)
savetxt('X.txt',X)
savetxt('U.txt',U)
savetxt('time_N5.txt',time)
savetxt('full_time_N5.txt',full_time)
savetxt('iter_N5.txt',iter)
plt.show()
