
import torch
import os
from model import Adaptive_dis

directory = os.path.dirname(os.getcwd())

gpu = 0
device = 'cpu'

dt_mpc = 0.025
dt_mpc_fl = 0.1

model = Adaptive_dis(device=device, dt_ct=dt_mpc, dt_fl=dt_mpc_fl).to(device)
model = model.to(torch.float32)
path = directory + '/trained_model/adaptive/quadruped-full-11p-best.tar'

model.load_state_dict(torch.load(path, map_location=device))

# An example input you would normally provide to your model's forward() method.
example = torch.rand(1, 7, dtype= torch.float32)

# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
traced_script_module_h1 = torch.jit.trace(model.h1_net, example)
traced_script_module_g1 = torch.jit.trace(model.g1_net, example)
traced_script_module_h2 = torch.jit.trace(model.h2_net, example)

traced_script_module_h1.save("model_h1.pt")
traced_script_module_g1.save("model_g1.pt")
traced_script_module_h2.save("model_h2.pt")