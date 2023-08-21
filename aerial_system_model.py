from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from casadi import Function
import numpy as np
import scipy.io
# Load Identification matrices
Identification = scipy.io.loadmat('matrices_angular.mat') 
A_a_aux = Identification['A_a'] 
B_a_aux = Identification['B_a'] 
nx = A_a_aux.shape[0]
nu = B_a_aux.shape[1]

def create_matrix(A, data):
    for i in range(0, data.shape[0]):
        for j in range(0, data.shape[1]):
            A[i, j] = data[i, j]
    return A
        


def export_uav_model():

    model_name = 'angular_ode_drone'

    # Model MatriceS
    A_a = MX.zeros(nx, nx)
    A_a = create_matrix(A_a, A_a_aux)

    B_a = MX.zeros(nx, nu)
    B_a = create_matrix(B_a, B_a_aux)

    ## Definition Symbolic Variables states

    x = MX.sym('x', nx, 1)
    
    ## Definition of control variables

    u = MX.sym('u', nu, 1)

    ## System Evolution
    x_k = A_a@x + B_a@u
    # populate structure
    model = AcadosModel()
    model.x = x
    model.u = u
    model.disc_dyn_expr = x_k
    model.name = model_name
    f_system = Function('system',[x, u], [x_k])
    return f_system, model