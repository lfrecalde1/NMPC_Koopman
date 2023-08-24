import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from aerial_system_model_complete import export_uav_model
import scipy.linalg
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

def lift_Fun_angular(x, cent_a):
    x_lift = []
    for k in x: x_lift.append(k)
    for k in rbf(x[3], cent_a, "gauss"): x_lift.append(k)
    for k in rbf(x[4], cent_a, "gauss"): x_lift.append(k)
    for k in rbf(x[5], cent_a, "gauss"): x_lift.append(k)
    x_lift = np.array(x_lift)
    return x_lift

def lift_Fun_linear(x, cent_l, cent_lz):
    x_lift = []
    for k in x: x_lift.append(k)
    for k in rbf(x[0], cent_l, "gauss"): x_lift.append(k)
    for k in rbf(x[1], cent_l, "gauss"): x_lift.append(k)
    for k in rbf(x[2], cent_lz, "gauss"): x_lift.append(k)
    x_lift = np.array(x_lift)
    return x_lift

def lift_Fun(x, cent_a, cent_l, cent_lz):
    a_lift = lift_Fun_angular(x[3:9], cent_a)
    v_lift = lift_Fun_linear(x[0:3], cent_l, cent_lz)

    complete = np.hstack((v_lift, a_lift))
    return complete


def f_angular_system(x, u, f_system, C):
    x_lift_k = f_system(x, u)
    y_output = C@x_lift_k
    out = np.array(y_output[:,0]).reshape((9,))
    return out

def create_ocp_solver_description(x0, N_horizon, t_horizon, z_max, z_min, phi_max, phi_min, theta_max, theta_min, psi_p_max, psi_p_min) -> AcadosOcp:
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
    Q_mat = np.zeros((nx, nx), dtype=np.double)
    Q_mat[0, 0] = 20
    Q_mat[1, 1] = 20
    Q_mat[2, 2] = 20

    Q_mat[15, 15] = 10
    Q_mat[16, 16] = 10
    Q_mat[17, 17] = 0.01

    Q_mat[18, 18] = 10
    Q_mat[19, 19] = 10
    Q_mat[20, 20] = 10
    R_mat = 1 * np.diag([(0.1/z_max), (1/phi_max), (1/theta_max), (1/psi_p_max)])

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
    ocp.constraints.lbu = np.array([z_min, phi_min, theta_min, psi_p_min])
    ocp.constraints.ubu = np.array([z_max, phi_max, theta_max, psi_p_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.constraints.x0 = x0

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "EXACT"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP

    # set prediction horizon
    ocp.solver_options.tf = t_horizon

    return ocp