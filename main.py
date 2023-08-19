from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from aerial_system_model import export_uav_model
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import Function
from casadi import MX
from fancy_plots import fancy_plots_2, fancy_plots_1, fancy_plots_3, plot_states, plot_states_velocity, plot_states_reference, plot_control
import scipy.io
#from c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

Identification = scipy.io.loadmat('matrices_angular.mat') 
cent_a = Identification['cent_a']
C_a = Identification['C_a']

def euler_p_f(omega, euler):
    ## Create Elements for the matrix
    T_11 = 1
    T_12 = 0
    T_13 = -np.sin(euler[0])
    T_21 = 0
    T_22 = np.cos(euler[0])
    T_23 = np.sin(euler[0])*np.cos(euler[1])
    T_31 = 0
    T_32 = -np.sin(euler[0])
    T_33 = np.cos(euler[0])*np.cos(euler[1])
    T = np.array([[T_11, T_12, T_13], [T_21, T_22, T_23], [T_31, T_32, T_33]], dtype=np.double)

    # Dfinition Omega
    w = omega.reshape(3, 1)
    aux = T@w
    aux = aux.reshape(3, )

    return aux

def rbf(X, C, rbf_type, eps=None, k=None):
    X = X.reshape(1, 1)
    rbf_type = rbf_type.lower()
    
    if eps is None:
        eps = 1
    if k is None:
        k = 1
    
    Cbig = C
    Y = np.zeros((C.shape[1], X.shape[1]))
    
    for i in range(0, Cbig.shape[1]):
        C = Cbig[:, i].reshape(-1, 1)
        C = np.tile(C, (1, X.shape[1]))
        r_squared = np.sum((X - C) ** 2, axis=0)
        
        if rbf_type == 'thinplate':
            y = r_squared * np.log(np.sqrt(r_squared)) + 0.5 * np.sqrt(r_squared)
            y[np.isnan(y)] = 0
        elif rbf_type == 'gauss':
            y = np.exp(-eps ** 2 * r_squared)
        elif rbf_type == 'invquad':
            y = 1 / (1 + eps ** 2 * r_squared)
        elif rbf_type == 'invmultquad':
            y = 1 / np.sqrt(1 + eps ** 2 * r_squared)
        elif rbf_type == 'polyharmonic':
            y = r_squared ** (k / 2) * np.log(np.sqrt(r_squared))
            y[np.isnan(y)] = 0
        else:
            raise ValueError('RBF type not recognized')
        
        Y[i, :] = y
    Y = Y.reshape(Y.shape[0], )
    return Y

def lift_Fun_angular(x):
    x_lift = []
    for k in x: x_lift.append(k)
    for k in rbf(x[3], cent_a, "gauss"): x_lift.append(k)
    for k in rbf(x[4], cent_a, "gauss"): x_lift.append(k)
    for k in rbf(x[5], cent_a, "gauss"): x_lift.append(k)
    x_lift = np.array(x_lift)
    return x_lift

def f_angular_system(x, u, f_system):
    x_lift_k = f_system(x, u)
    y_output = C_a@x_lift_k
    out = np.array(y_output[:,0]).reshape((6,))
    return out

def create_ocp_solver_description(x0, N_horizon, t_horizon, phi_max, phi_min, theta_max, theta_min, psi_p_max, psi_p_min) -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    f_system, model = export_uav_model()

    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = 10 * np.diag([0, 0, 0, 20, 20, 10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x,th,dx,dth]
    R_mat = 1 * np.diag([(1/phi_max), (1/theta_max), (1/psi_p_max)])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Q_mat
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([phi_min, theta_min, psi_p_min])
    ocp.constraints.ubu = np.array([phi_max, theta_max, psi_p_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])

    ocp.constraints.x0 = x0

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "EXACT"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = t_horizon

    return ocp
def main():
    # Initial Values System
    # Read Matlab Data
    pose = scipy.io.loadmat('h_3.mat')
    velocity = scipy.io.loadmat('hp_3.mat')
    control_action = scipy.io.loadmat('T_ref_3.mat')
    time_simulation = scipy.io.loadmat('t_3.mat')


    h = pose['h']
    hp = velocity['hp']
    T_ref = control_action['T_ref']
    t = time_simulation['t']
    t = t.reshape(t.shape[1],)
    t_s = t[1] - t[0]
    # Prediction Time
    t_prediction= 2;

    # Nodes inside MPC
    N = np.arange(0, t_prediction + t_s, t_s)
    N_prediction = N.shape[0]
    print(N_prediction)


    # Sample time vector
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = t_s*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Vector Initial conditions
    x = np.zeros((6, t.shape[0]+1 - N_prediction), dtype = np.double)

    # Initial Control values
    u_control = np.zeros((3, t.shape[0] - N_prediction), dtype = np.double)
    #u_control[0, :] = T_ref[1,0:t.shape[0]-N_prediction]
    #u_control[1, :] = T_ref[2,0:t.shape[0]-N_prediction]
    #u_control[2, :] = T_ref[3,0:t.shape[0]-N_prediction]


    # Desired Values
    xref = np.zeros((15, t.shape[0]), dtype = np.double)
    xref[3,:] = 0.1 * np.sin(18*0.04*t)
    xref[4,:] =  0.1 * np.cos(18*0.08*t)
    xref[5,:] = 0.4 * np.sin (0.5* t)

    #xref[3, :] = -0.2 * np.sign(np.sin(0.5*t))
    #xref[4, :] = 0.2 * np.sign(np.sin(0.5*t))
    #xref[5, :] = 0.2 * np.sign(np.cos(0.5*t))

    # Euler Angles
    euler = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)
    euler_p = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Definition of euler angles
    euler[:, 0] = h[7:10, 0]

    # Get Euler dot throughout
    for k in range(0, euler.shape[1]):
        euler_p[:, k] = euler_p_f(hp[3:6, k], euler[:,k])

    ## Initial Condition
    x[0:3, 0] = h[7:10, 0]
    x[3:6, 0] = euler_p[:, 0]

    # Create Model of the system
    f_angular, model = export_uav_model()

    # Initial Condition system
    x_lift = lift_Fun_angular(x[:, 0])
    # Limits Control values
    phi_max = 0.5
    theta_max = 0.5
    psi_p_max = 0.5

    phi_min = -phi_max
    theta_min = -theta_max
    psi_p_min = -psi_p_max

    ## Optimization problem definition
    ocp = create_ocp_solver_description(x_lift, N_prediction, t_prediction, phi_max, phi_min, theta_max, theta_min, psi_p_max, psi_p_min)
    # Optimization Problem

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)


    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    # Initial States Acados
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(x_lift.shape))
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))
    # Simulation System


    for k in range(0, t.shape[0]- N_prediction):
        # Get Computational Time
        tic = time.time()
        x_lift = lift_Fun_angular(x[:, k])

        acados_ocp_solver.set(0, "lbx", x_lift)
        acados_ocp_solver.set(0, "ubx", x_lift)

        # Update yref
        for j in range(N_prediction):
            yref = xref[:,k+j]
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = xref[:,k+N_prediction]
        acados_ocp_solver.set(N_prediction, "yref", yref_N[0:12])
        # Get Computational Time
        status = acados_ocp_solver.solve()

        toc_solver = time.time()- tic

        # Get Control Signal
        u_control[:, k] = acados_ocp_solver.get(0, "u")


        # System Evolution
        x[:, k+1] = f_angular_system(x_lift, u_control[:, k], f_angular)

        delta_t[:, k] = toc_solver

    fig1, ax1, ax2, ax3 = fancy_plots_3()
    plot_states(fig1, ax1, ax2, ax3, x, h, t, "Angles_estimation")
    fig12, ax12, ax22, ax32 = fancy_plots_3()
    plot_states_velocity(fig12, ax12, ax22, ax32, x, euler_p, t, "velocity_estimation")
    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_states_reference(fig13, ax13, ax23, ax33, x, xref, t, "Reference")
    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_control(fig14, ax14, ax24, ax34, u_control, t, "Control")

    # Time Plot
    fig3, ax13 = fancy_plots_1()
    ## Axis definition necesary to fancy plots
    ax13.set_xlim((t[0], t[-1]))

    time_1, = ax13.plot(t[0:delta_t.shape[1]],delta_t[0,:],
                    color='#00429d', lw=2, ls="-")
    tsam1, = ax13.plot(t[0:t_sample.shape[1]],t_sample[0,:],
                    color='#9e4941', lw=2, ls="-.")

    ax13.set_ylabel(r"$[s]$", rotation='vertical')
    ax13.set_xlabel(r"$\textrm{Time}[s]$", labelpad=5)
    ax13.legend([time_1,tsam1],
            [r'$t_{compute}$',r'$t_{sample}$'],
            loc="best",
            frameon=True, fancybox=True, shadow=False, ncol=2,
            borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
            borderaxespad=0.3, columnspacing=2)
    ax13.grid(color='#949494', linestyle='-.', linewidth=0.5)

    fig3.savefig("time.eps")
    fig3.savefig("time.png")


    ## Axis definition necesary to fancy plots
    # Systems Results
    print(f'Mean iteration time with MLP Model: {1000*np.mean(delta_t):.1f}ms -- {1/np.mean(delta_t):.0f}Hz)')

if __name__ == '__main__':
    main()