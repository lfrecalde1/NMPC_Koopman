from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from casadi import Function
import numpy as np
import scipy.io
# Load Identification matrices
Identification = scipy.io.loadmat('matrices_complete.mat') 
A_aux = Identification['A'] 
B_aux = Identification['B'] 
G_aux = Identification['G'] 
nx = A_aux.shape[0]
nu = B_aux.shape[1]

def create_matrix(A, data):
    for i in range(0, data.shape[0]):
        for j in range(0, data.shape[1]):
            A[i, j] = data[i, j]
    return A
        

def export_uav_model():

    model_name = 'angular_ode_drone'

    # Model MatriceS
    A_a = MX.zeros(nx, nx)
    A_a = create_matrix(A_a, A_aux)

    B_a = MX.zeros(nx, nu)
    B_a = create_matrix(B_a, B_aux)

    G_a = MX.zeros(nx, 1)
    G_a = create_matrix(G_a, G_aux)

    ## Definition Symbolic Variables states

    x = MX.sym('x', nx, 1)

    ## Definition of control variables

    u = MX.sym('u', 4, 1)
    u_aux = vertcat(u[0, 0], u[1, 0], u[2, 0], u[3, 0])


    ## System Evolution
    x_k = A_a@x + B_a@u + G_a
    # populate structure
    model = AcadosModel()
    model.x = x
    model.u = u
    model.disc_dyn_expr = x_k
    model.name = model_name
    f_system = Function('system',[x, u], [x_k])
    return f_system, model