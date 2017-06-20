import sys
sys.path.append(r"/home/andrea/casadi-py27-np1.9.1-v2.4.2")
from casadi import *
from numpy import *
from scipy.linalg import *
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt

N = 20      # Control discretization
T = 2.0       # End time
nx = 6
nu = 2
Tsim = 20
N_sim = int(ceil(Tsim/(T/N)))
# Declare variables (use scalar graph)
u  = SX.sym("u",nu)    # control
x  = SX.sym("x",nx)  # states

# Motor parameters
mM1  = 0.3;
mM2  = 0.3;
kr1  = 1;
kr2  = 1;
rM1  = 0.05;
rM2  = 0.05;
Im1  = 1/2*mM1*rM1*rM1;
Im2  = 1/2*mM2*rM2*rM2;

#Link parameters:

ml1  = 3.0;
ml2  = 3.0;
l1   = 1.0;
l2   = 1.0;
Il1  = 1.0/12.0*ml1*ml1*l1*l1;
Il2  = 1.0/12.0*ml2*ml2*l2*l2;
a1   = l1/2.0;
a2   = l2/2.0;

# Others
g = 9.81;

#DEFINE A DIFFERENTIAL EQUATION:
#-------------------------------

th1     = x[0]
th1d    = x[1]
th2     = x[2]
th2d    = x[3]

tau1 = x[4]
tau2 = x[5]

tau1r = u[0]
tau2r = u[1]

alpha_1 = Il1 + ml1*l1*l1 + kr1*kr1*Im1 + Il2 + ml2*(a1*a1 + l2*l2 + 2*a1*l2*cos(th2) + Im2 + mM2*a1*a1);
alpha_2 = Il2 + ml2*(l2*l2 + a1*l2*cos(th2)) + kr2*Im2;
alpha_3 = -2.0*ml2*a1*l2*sin(th2);
alpha_4 = -ml2*a1*l2*sin(th2);
K_alpha = (ml1*l1 + mM2*a1 + ml2*a1)*g*cos(th1) + ml2*l2*g*cos(th1 + th2);

beta_1  = Il2 + ml2*(l2*l2 + a1*l2*cos(th2)) + kr2*Im2;
beta_2  = Il2 + ml2*l2*l2 + kr2*kr2*Im2;
beta_3  = ml2*a1*l2*sin(th2);
K_beta  = ml2*l2*g*cos(th1 + th2);

theta1dd_exp = 1.0/(alpha_1 - alpha_2*beta_1/beta_2)    * (alpha_2/beta_2 * (K_beta + beta_3*th1d*th1d - tau2) - alpha_3*th1d*th2d - alpha_4*th2d - K_alpha + tau1);

ode = vertcat([    th1d,\
                                                theta1dd_exp,\
                                                th2d,\
                                                1.0/beta_2*(tau2 - beta_1*theta1dd_exp - beta_3*th1d*th1d - K_beta),
                                                tau1r,
                                                tau2r])
f = SXFunction([x,u],[ode])
f.init()

# RK4 with M steps
U = MX.sym("U",nu)
X = MX.sym("X",nx)

M = 1; DT = T/(N*M)
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
vmin[nx+0::nx+nu] = -200.0
vmin[nx+1::nx+nu] = -200.0
vmax[nx+0::nx+nu] =  200.0
vmax[nx+1::nx+nu] =  200.0

# State bounds
vmin[0::nx+nu] = -100
vmin[1::nx+nu] = -100
vmin[2::nx+nu] = -100
vmin[3::nx+nu] = -100
vmin[4::nx+nu] = -100
vmin[5::nx+nu] = -100

vmax[0::nx+nu] =  100
vmax[1::nx+nu] =  100
vmax[2::nx+nu] =  100
vmax[3::nx+nu] =  100
vmax[4::nx+nu] =  70
vmax[5::nx+nu] =  70


# Initial condition
vmin[0] = vmax[0] = v0[0] = -0.4
vmin[1] = vmax[1] = v0[1] = 0.0
vmin[2] = vmax[2] = v0[2] = 0.4
vmin[3] = vmax[3] = v0[3] = 0.0
vmin[4] = vmax[4] = v0[4] = 0.0
vmin[5] = vmax[5] = v0[5] = 0.0

# Initial solution guess
v0 = 0.001*ones(nv)
v0 = 1*ones(nv)
# for k in range(N):
#     v0[(nx + nu)*k + nx:(nx + nu)*k + nx + nu] = [40, 40, 40, 40]

# Constraint function with bounds
g = []; gmin = []; gmax = []

# Objective function
J=0

# Build up a graph of integrator calls
#H_diag = array([1000, 1000, 1000, 1000, 0.001, 0.001, 0.001, 0.001])
#H_end_diag = array([1000, 1000, 1000, 1000, 0.001, 0.001])
H_diag = array([1000, 0.1000, 1000, 0.10, 0.01, 0.01, 0.01, 0.01])
H_end_diag = array([1000, 0.1000, 1000, 0.1000, 0.01, 0.01])

ref = array([1.2, 0.0, -1.2, 0.0, 0.0, 0.0])
for k in range(N):
  # Call the integrator
  [xf] = F([xk[k],uk[k]])

  J+= 1.0/2.0*H_diag[0]*pow(xk[k][0] - ref[0],2)
  J+= 1.0/2.0*H_diag[1]*pow(xk[k][1],2)
  J+= 1.0/2.0*H_diag[2]*pow(xk[k][2] - ref[2],2)
  J+= 1.0/2.0*H_diag[3]*pow(xk[k][3],2)
  J+= 1.0/2.0*H_diag[4]*pow(xk[k][4],2)
  J+= 1.0/2.0*H_diag[5]*pow(xk[k][5],2)

  for j in range(nu):
    J+= 1.0/2.0*H_diag[nx+j]*pow(uk[k][j] ,2)

  #Append continuity constraints
  g.append(xf - xk[k+1])
  gmin.append(zeros(nx))
  gmax.append(zeros(nx))

# Terminal cost
k = N

J+= 1.0/2.0*H_end_diag[0]*pow(xk[k][0] - ref[0],2)
J+= 1.0/2.0*H_end_diag[1]*pow(xk[k][1],2)
J+= 1.0/2.0*H_end_diag[2]*pow(xk[k][2] - ref[2],2)
J+= 1.0/2.0*H_end_diag[3]*pow(xk[k][3],2)

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
opts = {'jit':True, "jit_options":{"flags":['-O3']}}
# nlp = MXFunction('nlp',nlpIn(x=v),nlpOut(f=J,g=g),opts)
# solver = NlpSolver("nlp_solver","ipopt", nlp)
nlp = MXFunction(nlpIn(x=v),nlpOut(f=J,g=g))
solver = NlpSolver("ipopt", nlp)

#solver.setOption("tol",1.0e-4)
#solver.setOption("constr_viol_tol",1.0e-4)
#solver.setOption("compl_inf_tol",1.0e-4)
#solver.setOption("dual_inf_tol",1.0e-4)
#solver.setOption("accept_every_trial_step","yes")

solver.setOption("print_level",0)
solver.setOption("output_file","ipopt_log.txt")
solver.setOption("linear_solver","ma57")
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
for i in range(N_sim/2):

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

# Change reference
J = 0

ref = array([-1.2, 0.0, +1.2, 0.0, 0.0, 0.0])
for k in range(N):
  # Call the integrator
  [xf] = F([xk[k],uk[k]])

  J+= 1.0/2.0*H_diag[0]*pow(xk[k][0] - ref[0],2)
  J+= 1.0/2.0*H_diag[1]*pow(xk[k][1],2)
  J+= 1.0/2.0*H_diag[2]*pow(xk[k][2] - ref[2],2)
  J+= 1.0/2.0*H_diag[3]*pow(xk[k][3],2)
  J+= 1.0/2.0*H_diag[4]*pow(xk[k][4],2)
  J+= 1.0/2.0*H_diag[5]*pow(xk[k][5],2)

  for j in range(nu):
    J+= 1.0/2.0*H_diag[nx+j]*pow(uk[k][j] ,2)


# Terminal cost
k = N

J+= 1.0/2.0*H_end_diag[0]*pow(xk[k][0] - ref[0],2)
J+= 1.0/2.0*H_end_diag[1]*pow(xk[k][1],2)
J+= 1.0/2.0*H_end_diag[2]*pow(xk[k][2] - ref[2],2)
J+= 1.0/2.0*H_end_diag[3]*pow(xk[k][3],2)


# Initial condition
vmin[0] = vmax[0] = v0[0] = X[0,N_sim/2-1]
vmin[1] = vmax[1] = v0[1] = X[1,N_sim/2-1]
vmin[2] = vmax[2] = v0[2] = X[2,N_sim/2-1]
vmin[3] = vmax[3] = v0[3] = X[3,N_sim/2-1]
vmin[4] = vmax[4] = v0[4] = X[4,N_sim/2-1]
vmin[5] = vmax[5] = v0[5] = X[5,N_sim/2-1]

# Create NLP solver instance
opts = {'jit':True, "jit_options":{"flags":['-O2']}}
# nlp = MXFunction('nlp',nlpIn(x=v),nlpOut(f=J,g=g),opts)
#solver = NlpSolver("nlp_solver","ipopt", nlp)
nlp = MXFunction(nlpIn(x=v),nlpOut(f=J,g=g))
solver = NlpSolver("ipopt", nlp)


solver.setOption("print_level",0)
solver.setOption("output_file","ipopt_log.txt")

solver.init()

# Set bounds and initial guess
solver.setInput(v0,   "x0")
solver.setInput(vmin, "lbx")
solver.setInput(vmax, "ubx")
solver.setInput(gmin, "lbg")
solver.setInput(gmax, "ubg")

#pdb.set_trace()
for i in range(N_sim/2-1,N_sim):

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

# Plot results
plt.figure(1)
plt.clf()
plt.subplot(411)
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[0,:],'--')
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[2,:],'--')
plt.title("Robot - Ipopt")
plt.xlabel('time')
plt.legend(['theta1','theta2'])
plt.grid()

plt.subplot(412)
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[1,:],'--')
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[3,:],'--')
plt.xlabel('time')
plt.legend(['dtheta1','dtheta2'])
plt.grid()

plt.subplot(413)
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[4,:],'--')
plt.plot(linspace(0,DT*M*N_sim,N_sim),X[5,:],'--')
plt.xlabel('time')
plt.legend(['tau2','tau1'])
plt.grid()

#plt.step(linspace(0,T,N),u_opt,'-.')
plt.subplot(414)
plt.step(linspace(0,DT*M*N_sim,N_sim),U[0,:])
plt.step(linspace(0,DT*M*N_sim,N_sim),U[1,:])
plt.xlabel('time')
plt.legend(['tau1','tau2'])
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
savez('robot_sim_ipopt.mpy', X=X, U=U, time=time, full_time=full_time, iter=iter)
savetxt('X_new.txt',X)
savetxt('U_new.txt',U)
savetxt('time.txt',time)
savetxt('full_time.txt',full_time)
savetxt('iter.txt',iter)
plt.show()
