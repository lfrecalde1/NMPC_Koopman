from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from casadi import Function
import numpy as np
import scipy.io
# Load Identification matrices
Identification = scipy.io.loadmat('matrices_angular.mat') 
A_a_aux = Identification['A_a'] 
B_a_aux = Identification['B_a'] 

def create_matrix(A, data):
    for i in range(0, data.shape[0]):
        for j in range(0, data.shape[1]):
            A[i, j] = data[i, j]
    return A
        


def export_uav_model():

    model_name = 'angular_ode_drone'
    nx = 12
    nu = 3

    # Model MatriceS
    A_a = MX.zeros(nx, nx)
    A_a = create_matrix(A_a, A_a_aux)

    B_a = MX.zeros(nx, nu)
    B_a = create_matrix(B_a, B_a_aux)

    ## Definition Symbolic Variables states
    x1 = MX.sym('x1')
    x2 = MX.sym('x2')
    x3 = MX.sym('x3')
    x4 = MX.sym('x4')
    x5 = MX.sym('x5')
    x6 = MX.sym('x6')
    x7 = MX.sym('x7')
    x8 = MX.sym('x8')
    x9 = MX.sym('x9')
    x10 = MX.sym('x10')
    x11 = MX.sym('x11')
    x12 = MX.sym('x12')
    
    x = vertcat(x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12)


    ## Definition of control variables
    u1 = MX.sym('u1')
    u2 = MX.sym('u2')
    u3 = MX.sym('u3')

    u = vertcat(u1, u2, u3)

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