from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from aerial_system_model_complete import export_uav_model
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import Function
from casadi import MX
from fancy_plots import fancy_plots_2, fancy_plots_1, fancy_plots_3, plot_states, plot_states_velocity, plot_states_reference, plot_control, plot_states_reference_angular
from fancy_plots  import fancy_plots_4, plot_control_full
import scipy.io
from nmpc import euler_p_f, rbf, lift_Fun_angular, lift_Fun_linear, lift_Fun, f_angular_system, create_ocp_solver_description
#from c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

## Load angular parameters
Identification = scipy.io.loadmat('matrices_complete.mat') 
cent_a = Identification['cent_a']
cent_l = Identification['cent_l']
cent_lz = Identification['cent_lz']
C = Identification['C']
A_aux = Identification['A'] 
B_aux = Identification['B'] 

def main():
    # Initial Values System
    # Read Matlab Data
    Data_1 = scipy.io.loadmat('Data_mujoco_1.mat')
    Data_2 = scipy.io.loadmat('Data_mujoco_2.mat')

    h = np.hstack((Data_1['h'], Data_2['h']))
    hp = np.hstack((Data_1['hp'], Data_2['hp']))
    T_ref = np.hstack((Data_1['T_ref'], Data_2['T_ref']))
    t = np.hstack((Data_1['t'], Data_2['t']))
    t = t.reshape(t.shape[1],)
    t_s = t[1] - t[0]
    t = np.zeros((T_ref.shape[1]), dtype=np.double)
    t[0] = 0.0

    # Create new time vector
    for i in range(0, T_ref.shape[1]-1):
        t[i+1] = t[i] + t_s

    # Prediction Time
    t_prediction= 2;

    # Nodes inside MPC
    N = np.arange(0, t_prediction + t_s, t_s)
    N_prediction = N.shape[0]


    # Sample time vector
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = t_s*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Vector Initial conditions
    x = np.zeros((9, t.shape[0]+1 - N_prediction), dtype = np.double)

    # Initial Control values
    u_control = np.zeros((4, t.shape[0] - N_prediction), dtype = np.double)
    u_control[0, :] = T_ref[0,0:t.shape[0]-N_prediction]
    u_control[1, :] = T_ref[1,0:t.shape[0]-N_prediction]
    u_control[2, :] = T_ref[2,0:t.shape[0]-N_prediction]
    u_control[3, :] = T_ref[3,0:t.shape[0]-N_prediction]


    # Desired Values
    xref = np.zeros((37, t.shape[0]), dtype = np.double)
    # Linear Velocities
    xref[0,:] = 0.5 * np.sign(np.sin(18*0.04*t))
    xref[1,:] =  0.5 * np.sign(np.cos(18*0.04*t))
    xref[2,:] = 0.6 * np.sign(np.sin (0.8* t))
    # Angular Velocities
    xref[18,:] = 0 * np.sin(18*0.04*t)
    xref[19,:] =  0 * np.cos(18*0.08*t)
    xref[20,:] = 0.3 * np.sin (0.5* t)

    #xref[0, :] = -0.1 * np.sign(np.sin(0.2*t))
    #xref[1, :] = 0.1 * np.sign(np.sin(0.2*t))
    #xref[2, :] = 0.1 * np.sign(np.cos(0.2*t))

    # Euler Angles
    euler = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)
    euler_p = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Definition of euler angles
    euler[:, 0] = h[7:10, 0]

    # Get Euler dot throughout
    for k in range(0, euler.shape[1]):
        euler_p[:, k] = euler_p_f(hp[3:6, k], euler[:,k])

    ## Initial Condition
    x[0:3, 0] = hp[0:3, 0]
    x[3:6, 0] = h[7:10, 0]
    x[6:9, 0] = euler_p[:, 0]

    # Create Model of the system
    f_complete, model = export_uav_model()

    # Initial Condition system
    x_lift = lift_Fun(x[:, 0], cent_a, cent_l, cent_lz)

    # Limits Control values
    z_max = 12
    phi_max = 0.3
    theta_max = 0.3
    psi_p_max = 0.3

    phi_min = -phi_max
    theta_min = -theta_max
    psi_p_min = -psi_p_max
    z_min = 3

    ### Optimization problem definition
    ocp = create_ocp_solver_description(x_lift, N_prediction, t_prediction, z_max, z_min, phi_max, phi_min, theta_max, theta_min, psi_p_max, psi_p_min)
    ## Optimization Problem

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json", build= True, generate= True)


    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    ## Initial States Acados
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(x_lift.shape))
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    ## Simulation System


    for k in range(0, t.shape[0]- N_prediction):
        # Get Computational Time
        tic = time.time()
        x_lift = lift_Fun(x[:, k], cent_a, cent_l, cent_lz)
        acados_ocp_solver.set(0, "lbx", x_lift)
        acados_ocp_solver.set(0, "ubx", x_lift)
        # Update yref
        for j in range(N_prediction):
            yref = xref[:,k+j]
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = xref[:,k+N_prediction]
        acados_ocp_solver.set(N_prediction, "yref", yref_N[0:33])
        # Get Computational Time
        status = acados_ocp_solver.solve()
        toc_solver = time.time()- tic
        # Get Control Signal
        u_control[:, k] = acados_ocp_solver.get(0, "u")
        # System Evolution
        x[:, k+1] = f_angular_system(x_lift, u_control[:, k], f_complete, C)
        delta_t[:, k] = toc_solver

    # System Figures
    plt.imshow(A_aux)
    plt.colorbar()

    
    plt.imshow(B_aux)
    plt.colorbar()

    fig13, ax13, ax23, ax33 = fancy_plots_3()
    plot_states_reference(fig13, ax13, ax23, ax33, x[0:3,:], xref[0:3,:], t, "Reference_linear")
    
    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_states_reference_angular(fig14, ax14, ax24, ax34, x[6:9,:], xref[18:21,:], t, "Reference_angular")

    fig15, ax15, ax25, ax35, ax45= fancy_plots_4()
    plot_control_full(fig15, ax15, ax25, ax35, ax45, u_control, t, "Control_actions")

    ### Time Plot
    fig3, ax13 = fancy_plots_1()
    #### Axis definition necesary to fancy plots
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


    ## Systems Results
    print(f'Mean iteration time with MLP Model: {1000*np.mean(delta_t):.1f}ms -- {1/np.mean(delta_t):.0f}Hz)')

if __name__ == '__main__':
    main()