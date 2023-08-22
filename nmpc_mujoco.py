
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
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
#from c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

## Load angular parameters
Identification = scipy.io.loadmat('matrices_complete.mat') 
cent_a = Identification['cent_a']
cent_l = Identification['cent_l']
cent_lz = Identification['cent_lz']
C = Identification['C']
A_aux = Identification['A'] 
B_aux = Identification['B'] 

# Global variables Odometry Drone
x_real = 0.0
y_real = 0.0
z_real = 0.0
vx_real = 0.0
vy_real = 0.0
vz_real = 0.0

# Angular velocities
qx_real = 0.0005
qy_real = 0.0
qz_real = 0.0
qw_real = 1.0
wx_real = 0.0
wy_real = 0.0
wz_real = 0.0

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

## Reference system
def get_reference(ref, ref_msg):
        ref_msg.linear.x = 0
        ref_msg.linear.y = 0
        ref_msg.linear.z = ref[0]

        ref_msg.angular.x = ref[1]
        ref_msg.angular.y = ref[2]
        ref_msg.angular.z = ref[3]
        return ref_msg

def send_reference(ref_msg, ref_pu):
    ref_pu.publish(ref_msg)
    return None

def odometry_call_back(odom_msg):
    global x_real, y_real, z_real, qx_real, qy_real, qz_real, qw_real, vx_real, vy_real, vz_real, wx_real, wy_real, wz_real
    # Read desired linear velocities from node
    x_real = odom_msg.pose.pose.position.x 
    y_real = odom_msg.pose.pose.position.y
    z_real = odom_msg.pose.pose.position.z
    vx_real = odom_msg.twist.twist.linear.x
    vy_real = odom_msg.twist.twist.linear.y
    vz_real = odom_msg.twist.twist.linear.z


    qx_real = odom_msg.pose.pose.orientation.x
    qy_real = odom_msg.pose.pose.orientation.y
    qz_real = odom_msg.pose.pose.orientation.z
    qw_real = odom_msg.pose.pose.orientation.w

    wx_real = odom_msg.twist.twist.angular.x
    wy_real = odom_msg.twist.twist.angular.y
    wz_real = odom_msg.twist.twist.angular.z
    return None

def get_system_states_ori_sensor():
    x = np.array([qx_real, qy_real, qz_real, qw_real], dtype=np.double)
    r = R.from_quat(x)
    return r.as_euler('xyz', degrees=False)

def get_system_states_sensor():
    quat = np.array([qx_real, qy_real, qz_real, qw_real], dtype=np.double)
    rot = R.from_quat(quat)
    euler = rot.as_euler('xyz', degrees=False)
    x = np.array([x_real, y_real, z_real, qw_real, qx_real, qy_real, qz_real, euler[0], euler[1], euler[2]], dtype=np.double)
    return x

# Get system velocities
def get_system_velocity_sensor():
    q = np.array([qx_real, qy_real, qz_real, qw_real], dtype=np.double)
    rot = R.from_quat(q)
    rot = rot.as_matrix()
    v_body = np.array([[vx_real], [vy_real], [vz_real]], dtype=np.double)
    v_world = rot@v_body
    x = np.array([v_world[0, 0], v_world[1, 0], v_world[2, 0], wx_real, wy_real, wz_real], dtype=np.double)
    return None

def main(control_pub):
    # Initial Values System
    # Read Matlab Data
    # Simulation time parameters
    t_s = 0.04
    tf = 40
    t = np.arange(0, tf+t_s, t_s, dtype=np.double)

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
    ref_drone = Twist()


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

    ## Complete states of the system
    h = np.zeros((10, t.shape[0]+1 - N_prediction), dtype = np.double)
    hp = np.zeros((6, t.shape[0]+1 - N_prediction), dtype = np.double)


    # Euler Angles
    euler = np.zeros((3, t.shape[0]+1 - N_prediction), dtype=np.double)
    euler_p = np.zeros((3, t.shape[0]+1 - N_prediction), dtype=np.double)

    for k in range(0, 50):
        tic = time.time()
        ## Get Contol Action or nothing
        ref_drone = get_reference([0, 0, 0, 0], ref_drone)
        send_reference(ref_drone, control_pub)

        # Loop_rate.sleep()
        while (time.time() - tic <= t_s):
                None
        toc = time.time() - tic 
        print("Init System")
    
    # Set initial Conditions
    h[:, 0] = get_system_states_sensor()
    hp[:, 0] = get_system_velocity_sensor()

    # Definition of euler angles
    euler[:, 0] = h[7:10, 0]

    # Get Euler dot throughout
    euler_p[:, 0] = euler_p_f(hp[3:6, 0], euler[:, 0])

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

        # Get Control Signal
        u_control[:, k] = acados_ocp_solver.get(0, "u")
        ref_drone = get_reference([0, 0, 0, 0], ref_drone)
        send_reference(ref_drone, control_pub)

        # System Evolution
        #x[:, k+1] = f_angular_system(x_lift, u_control[:, k], f_complete, C)
        # Loop_rate.sleep()
        while (time.time() - tic <= t_s):
                None
        toc_solver = time.time()- tic
        # Save Data
        h[:, k+1] = get_system_states_sensor()
        hp[:, k+1] = get_system_velocity_sensor()
        # Definition of euler angles
        euler[:, k+1] = h[7:10, k+1]

        # Get Euler dot throughout
        euler_p[:, k+1] = euler_p_f(hp[3:6, k+1], euler[:, k+1])

        ## Initial Condition
        x[0:3, k+1] = hp[0:3, k+1]
        x[3:6, k+1] = h[7:10, k+1]
        x[6:9, k+1] = euler_p[:, k+1]
        t_k = t_k + toc
        # Set zero Values
        delta_t[:, k] = toc_solver

    ref_drone = get_reference([0, 0, 0, 0], ref_drone)
    send_reference(ref_drone, control_pub)

    # System Figures
    plt.imshow(A_aux)
    plt.colorbar()
    plt.show()

    
    plt.imshow(B_aux)
    plt.colorbar()
    plt.show()

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
    try:
        # Initialization Node
        rospy.init_node("NMPC_controller",disable_signals=True, anonymous=True)

        # Publisher Info
        odomety_topic = "/odom"
        odometry_subscriber = rospy.Subscriber(odomety_topic, Odometry, odometry_call_back)

        # Subscribe Info
        velocity_topic = "/cmd_vel"
        velocity_publisher = rospy.Publisher(velocity_topic, Twist, queue_size = 10)


        # Main System
        main(velocity_publisher)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass