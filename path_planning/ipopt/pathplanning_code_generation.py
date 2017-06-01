import sys
sys.path.append(r"/home/andrea/casadi-py27-np1.9.1-v2.4.3")
from casadi import *
from numpy import *
from scipy.linalg import *
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from math import atan2, asin
import pdb
from os import system

N = 50      # Control discretization
T = 5.0     # End time
nx = 4 
nu = 2
N_sim = 30

# Declare variables (use scalar graph)
u  = SX.sym("u",nu)    # control
x  = SX.sym("x",nx)    # states

# Dynamics definition
m = 1.0
I = 1.0
ode = vertcat([ 	x[2]*cos(x[3]),\
					x[2]*sin(x[3]),\
					u[0]/m,\
					u[1]/I	])

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

#F_casadi = MXFunction([X,U],[XF])
#F_casadi.init()

#name = "F_source"
#F_casadi.generate(name)
#obj_name = "F_external.so"
#system("gcc -fPIC -shared " + name + ".c -o " + obj_name)
#F = ExternalFunction(name,"./"+obj_name)

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

#  State bounds
vmin[0::nx+nu] = -3.0
vmin[1::nx+nu] = 0.0
vmin[2::nx+nu] = 0.0
vmin[3::nx+nu] = 0.0
vmax[0::nx+nu] = 0.0
vmax[1::nx+nu] = 3.0
vmax[2::nx+nu] = 2.0
vmax[3::nx+nu] = 3.14


# Control bounds
vmin[nx+0::nx+nu] = -5.0
vmin[nx+1::nx+nu] = -1.0
vmax[nx+0::nx+nu] =  5.0
vmax[nx+1::nx+nu] =  1.0

# Initial condition
x0 = array([-2.0,0,0,2.094]) 

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
for k in range(N):
	[xf] = F([xk[k],uk[k]])
	J+=-100.0*xk[k][1] + 0.01*pow(uk[k][0],2) + 0.01*pow(uk[k][1],2)
	g.append(xf - xk[k+1])	
	gmin.append(zeros(nx))
	gmax.append(zeros(nx))
	# add nonlinear constraints
	g.append(pow(xk[k][0],2) + pow(xk[k][1],2))
	g.append(pow(xk[k][0] + 2.0,2) + pow(xk[k][1]-2.5,2))
	gmax.append(array([9,Inf]))
	gmin.append(array([1, 0.95**2]))

# Terminal cost

# Concatenate constraints
g = vertcat(g)
gmin = concatenate(gmin)
gmax = concatenate(gmax)

# Create NLP solver instance
opts = {'jit':True}
nlp = MXFunction('nlp',nlpIn(x=v),nlpOut(f=J,g=g),opts)
solver = NlpSolver("nlp_solver", "ipopt",nlp)

#solver.setOption("tol",1.0e-4)
#solver.setOption("constr_viol_tol",1.0e-4)
#solver.setOption("compl_inf_tol",1.0e-4)
#solver.setOption("dual_inf_tol",1.0e-4)
#solver.setOption("accept_every_trial_step","yes")
#solver.setOption("limited_memory_update_type","bfgs")
solver.setOption("print_level",5)
solver.setOption("linear_solver","mumps")
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
plt.plot(X[0,:],X[1,:],'--')
plt.title("Trajectory")
plt.xlabel('time')
plt.grid()


