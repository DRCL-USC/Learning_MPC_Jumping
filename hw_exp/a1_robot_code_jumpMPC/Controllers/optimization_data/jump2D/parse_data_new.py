import scipy.io
import numpy as np

data = scipy.io.loadmat("jumpingFull_A1_1ms_h00_d60_MDC_interp")
#data = scipy.io.loadmat("jumping_test_experiment_A1_d60_h00_mu0.4_082421_v1_interp")
#data = scipy.io.loadmat("jumping_backflip_cs503040_051522_motor_constraints_contact_timings_v3_interp") # successful backflip


data = {k:v for k, v in data.items() if k[0] != '_'}

parameter = data.keys()

for i in parameter:
	if(i == 'tau'):
		np.savetxt(("data_tau.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'Q'):
		np.savetxt(("data_Q.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'F'):
		np.savetxt(("data_F.csv".format(i)), data.get(i), delimiter=",")
        if(i == 'pf'):
 		np.savetxt(("data_pf.csv".format(i)), data.get(i), delimiter=",")
	if(i == 'vf'):
		np.savetxt(("data_vf.csv".format(i)), data.get(i), delimiter=",")
