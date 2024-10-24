import torch
import numpy as np 
from model import MatrixNet

class Adaptive_dis(torch.nn.Module):
    def __init__(self, device=None, udim=4, dt_ct=0.025, dt_fl = 0.1):
        super(Adaptive_dis, self).__init__()
        init_gain = 0.001
        self.qdim = 3
        self.qdotdim = 3
        self.xdim = self.qdim + self.qdotdim
        self.pfdim = 4 
        self.udim = udim
        
        self.J0_inv = 12/(12*(0.4**2+0.1**2))
        
        self.M0_inv = 1/12
        self.g = 9.81 
        self.dt1 = dt_ct
        self.dt2 = dt_fl

        # Neural networks
        self.h1_net = MatrixNet(self.qdim + self.pfdim, 400, self.qdotdim, shape=(self.qdotdim, 1), init_gain=init_gain).to(device)
        self.g1_net = MatrixNet(self.qdim + self.pfdim, 1000, self.qdotdim*self.udim, shape=(self.qdotdim, self.udim), init_gain=init_gain).to(device)
        self.h2_net = MatrixNet(self.qdim + self.pfdim, 400, self.qdotdim, shape=(self.qdotdim, 1), init_gain=init_gain).to(device)

        self.device = device
        self.nfe = 0

    def nominal_ct(self, x):
        with torch.enable_grad():
            self.nfe += 1
            bs = x.shape[0]
            qk, qk_dot, gravity, pfk, uk = torch.split(x, [self.qdim, self.qdotdim, 1, self.pfdim, self.udim], dim=1)
            rxf = -torch.unsqueeze(
                -pfk[:, 1] * uk[:, 0] + pfk[:, 0] * uk[:, 1] - pfk[:, 3] * uk[:, 2] + pfk[:, 2] * uk[:, 3], dim=1)
            Fx = torch.unsqueeze(uk[:, 0] + uk[:, 2], dim=1)
            Fz = torch.unsqueeze(uk[:, 1] + uk[:, 3], dim=1)

            qk_next = qk + qk_dot * self.dt1
            vxk_next = qk_dot[:, 0:1] + self.M0_inv * Fx * self.dt1
            vzk_next = qk_dot[:, 1:2] + (self.M0_inv * Fz - self.g) * self.dt1
            wk_next = qk_dot[:, 2:3] + (self.J0_inv * rxf) * self.dt1

            return torch.cat((qk_next, vxk_next, vzk_next, wk_next, gravity, pfk, uk), dim=1)

    def nominal_fl(self, x):
        with torch.enable_grad():
            self.nfe += 1
            bs = x.shape[0]
            qk, qk_dot, gravity, pfk, uk = torch.split(x, [self.qdim, self.qdotdim, 1, self.pfdim, self.udim], dim=1)
            qk_next = qk + qk_dot * self.dt2
            vxk_next = qk_dot[:, 0:1]
            vzk_next = qk_dot[:, 1:2] + (- self.g) * self.dt2
            wk_next = qk_dot[:, 2:3]

            return torch.cat((qk_next, vxk_next, vzk_next, wk_next, gravity, pfk, uk), dim=1)
    def contact_phase(self, x):
        with torch.enable_grad():
            self.nfe += 1
            bs = x.shape[0]
            qk, qk_dot, gravity, pfk, uk = torch.split(x, [self.qdim, self.qdotdim, 1, self.pfdim, self.udim], dim=1)
            q_pfk = torch.cat((qk, pfk), dim=1)

            # Neural networks 
            g_qk = self.g1_net(q_pfk)
            h_qk = torch.squeeze(self.h1_net(q_pfk), dim=2)
            Gk = torch.squeeze(g_qk @ torch.unsqueeze(uk, dim=2), dim=2)

            rxf = -torch.unsqueeze(-pfk[:, 1] * uk[:, 0] + pfk[:, 0] * uk[:, 1] - pfk[:, 3] * uk[:, 2] + pfk[:, 2] * uk[:, 3], dim=1)
            Fx = torch.unsqueeze(uk[:, 0] + uk[:, 2], dim=1)
            Fz = torch.unsqueeze(uk[:, 1] + uk[:, 3], dim=1)

            zeros = torch.zeros(bs, 1, dtype=torch.float32).to(self.device)
            g = torch.cat((zeros, gravity, zeros), dim=1)
            control = torch.cat((Fx, Fz, rxf), dim=1)
            M = torch.diag(torch.tensor([self.M0_inv, self.M0_inv, self.J0_inv])).to(self.device)
            f_nominal = (M @ control.t()).t() - g

            qk_next = qk + qk_dot * self.dt1
            qkdot_next = qk_dot + (f_nominal * self.dt1) + (Gk + h_qk)

            return torch.cat((qk_next, qkdot_next, gravity, pfk, uk), dim=1)
    def flight_phase(self, x):
        with torch.enable_grad():
            self.nfe += 1
            bs = x.shape[0]
            qk, qk_dot, gravity, pfk, uk = torch.split(x, [self.qdim, self.qdotdim, 1, self.pfdim, self.udim], dim=1)
            q_pfk = torch.cat((qk, pfk), dim=1)

            # Neural networks 
            h_qk = torch.squeeze(self.h2_net(q_pfk), dim=2)

            zeros = torch.zeros(bs, 1, dtype=torch.float32).to(self.device)
            g = torch.cat((zeros, gravity, zeros), dim=1)
            f_nominal = - g

            qk_next = qk + qk_dot * self.dt2
            qkdot_next = qk_dot + f_nominal * self.dt2 + h_qk

            return torch.cat((qk_next, qkdot_next, gravity, pfk, uk), dim=1)

    def predict(self, x, num_steps, mode=0):
        '''
        '''
        xseq = x[None,:,:]
        curx = x
        for i in range(num_steps):
            if (mode == 'norm_ct'):
                nextx = self.nominal_ct(curx)
            if (mode == 'norm_fl'):
                nextx = self.nominal_fl(curx)
            if(mode == 1):
                nextx = self.contact_phase(curx)
            if(mode == 2):
                nextx = self.flight_phase(curx)
            curx = nextx
            xseq = torch.cat((xseq, curx[None,:,:]), dim = 0)
        return xseq