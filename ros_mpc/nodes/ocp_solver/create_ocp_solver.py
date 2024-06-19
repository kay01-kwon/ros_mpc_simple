from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from model import double_integrator
import scipy.linalg
import numpy as np
import matplotlib.pyplot as plt
import casadi as cs

# Initial state
X0 = np.array([0.0, 0.0, 0.0, 0.0])
T_horizon = 2.0
obs_center = np.array([2.5, 2.5])
r = 1+0.3

def create_ocp_solver() -> AcadosOcp:
    # Create ocp object to formulate the OCP
    ocp = AcadosOcp()

    di_model = double_integrator.DoubleIntegrator()
    model = di_model.getSystemModel()
    ocp.model = model

    N = 20
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu

    ny_e = nx

    # Set the number of shooting intervals
    ocp.dims.N = N

    # OCP objective function
    # Set cost
    # cost Q: p_x,p_y, v_x, v_y
    Q_mat = 2*np.diag([10, 10, 7, 7])
    # cost R: u
    R_mat = 2*5*np.diag([1, 1])

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vu = np.zeros((ny, nu))

    ocp.cost.Vx[:4,:4] = np.eye(4)

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((ny_e,))
    ocp.cost.W_e = Q_mat

    Umax = 2
    ocp.constraints.lbu = np.array([-Umax, -Umax])
    ocp.constraints.ubu = np.array([Umax, Umax])
    ocp.constraints.idxbu = np.array([0, 1])

    Kappa = np.array([64.0, 32.0])
    h_expr = get_h(model.x[0:2])
    dhdt_expr = get_dhdt(model.x[0:2],model.x[2:4])
    mu_expr = get_dh2dt2(model.x[0:2],model.x[2:4],model.u)
    ocp.model.con_h_expr = mu_expr + Kappa[0] * h_expr + Kappa[1] * dhdt_expr
    ocp.constraints.lh = np.array([0])
    ocp.constraints.uh = np.array([1e15])

    ocp.constraints.x0 = X0

    # set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp

def get_h(p):
    h = (p[0]-obs_center[0])**2 + (p[1]-obs_center[1])**2 - r**2
    return h

def get_dhdt(p,v):
    dhdt = 2*(p[0]-obs_center[0])*v[0] + (p[1]-obs_center[1])*v[1]
    return dhdt

def get_dh2dt2(p, v, u):
    Lf2h = 2*(v[0]**2 + v[1]**2)
    Lgfh = 2*(p[0] - obs_center[0])*u[0] + 2*(p[1] - obs_center[1])*u[1]
    d2hdt2 = Lf2h + Lgfh
    return d2hdt2

def get_h_value(p):
    h = (p[0]-obs_center[0])**2 + (p[1]-obs_center[1])**2 - r**2
    return h