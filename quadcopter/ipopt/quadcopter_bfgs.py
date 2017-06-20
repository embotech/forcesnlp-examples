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

N = 10      # Control discretization
T = 1.0     # End time
nx = 13
nu = 4
N_sim = 40

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

W1 = u[0]
W2 = u[1]
W3 = u[2]
W4 = u[3]



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
                        (J2*Omega1*Omega2 - J1*Omega1*Omega2 + (A*Cd*rho*(W1*W1 - W2*W2 + W3*W3 - W4*W4))/2)/J3])

f = SXFunction([x,u],[ode])
f.init()

# RK4 with M steps
U = MX.sym("U",nu)
X = MX.sym("X",nx)

M = 10 ; DT = T/(N*M)
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
vmin[nx+0::nx+nu] = 20.0
vmin[nx+1::nx+nu] = 20.0
vmin[nx+2::nx+nu] = 20.0
vmin[nx+3::nx+nu] = 20.0
vmax[nx+0::nx+nu] =  60.0
vmax[nx+1::nx+nu] =  60.0
vmax[nx+2::nx+nu] =  60.0
vmax[nx+3::nx+nu] =  60.0


initial_angle_rad = 3.0

# Initial condition
x0 = array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cos(initial_angle_rad/2.0), sin(initial_angle_rad/2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 

for i in range(nx):
	vmin[i] = vmax[i] = v0[i] = x0[i] 

# Initial solution guess
v0 = 0.0*ones(nv)
# for k in range(N):
#     v0[(nx + nu)*k + nx:(nx + nu)*k + nx + nu] = [40, 40, 40, 40] 

# Constraint function with bounds
g = []; gmin = []; gmax = []

# Objective function
J=0

# Build up a graph of integrator calls
H_end_diag = array([100, 100, 100, 0.10, 0.1, 0.1, 100, 100, 100, 100,  0.1, 0.1, 0.1])
H_diag = array([100, 100, 100, 0.10, 0.1, 0.1, 100, 100, 100, 100,  0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

for k in range(N):
    # Call the integrator
    [xf] = F([xk[k],uk[k]])
    for j in range(nx):
        J+= 1.0/2.0*H_diag[j]*pow(xk[k][j],2)
    for j in range(nu):
        J+= 1.0/2.0*H_diag[nx+j]*pow(uk[k][j] - 40,2) 
    g.append(xf - xk[k+1])
    gmin.append(zeros(nx))
    gmax.append(zeros(nx))

# Terminal cost
k = N
for j in range(nx):
    J+= 1.0/2.0*H_end_diag[j]*pow(xk[k][j],2)

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
nlp = MXFunction(nlpIn(x=v),nlpOut(f=J,g=g))
solver = NlpSolver("ipopt", nlp)

#solver.setOption("tol",1.0e-4)
#solver.setOption("constr_viol_tol",1.0e-4)
#solver.setOption("compl_inf_tol",1.0e-4)
#solver.setOption("dual_inf_tol",1.0e-4)
solver.setOption("hessian_approximation","limited-memory")
solver.setOption("limited_memory_update_type","bfgs")
solver.setOption("print_level",5)
solver.setOption("output_file","ipopt_log.txt")

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
savez('quadcopter_sim_ipopt_bfgs.mpy', X=X, U=U, time=time, full_time=full_time, iter=iter)
savetxt('X_bfgs.txt',X)
savetxt('U_bfgs.txt',U)
savetxt('time_bfgs.txt',time)
savetxt('full_time_bfgs.txt',full_time)
savetxt('iter_bfgs.txt',iter)
plt.show()

