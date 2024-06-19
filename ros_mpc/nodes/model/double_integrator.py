from acados_template import AcadosModel
import casadi as cs

class DoubleIntegrator:
    def __init__(self, mass = 1.0):

        self.model_name = "double_integrator"
        self.model = AcadosModel()
        # dpdt = v
        # dvdt = 1/m*u

        # Mass property
        self.m = mass

        self.p = cs.MX.sym('p',2)
        self.v = cs.MX.sym('v',2)
        self.x = cs.vertcat(self.p,self.v)

        self.u_x = cs.MX.sym('ux')
        self.u_y = cs.MX.sym('uy')
        self.u = cs.vertcat(self.u_x, self.u_y)

        self.dxdt = cs.MX.sym('dxdt',2)
        self.dvdt = cs.MX.sym('dvdt',2)

        self.xdot = cs.vertcat(self.dxdt, self.dvdt)

    def getSystemModel(self):

        self.f_expl = cs.vertcat(self.v,
                                 1/self.m*self.u_x,
                                 1/self.m*self.u_y)
        self.f_impl = self.xdot - self.f_expl

        self.model.f_impl_expr = self.f_impl
        self.model.f_expl_expr = self.f_expl
        self.model.x = self.x
        self.model.xdot = self.xdot
        self.model.u = self.u
        self.model.name = self.model_name

        return self.model

