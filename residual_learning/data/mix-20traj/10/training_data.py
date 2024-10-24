import matplotlib.pyplot as plt
import numpy as np



def plot_COM_pos_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='px') 
    ax.plot(t, f_1, label='pz') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_rpy_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,2]
    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*180/3.14, label='pitch') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_vel_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='vx') 
    ax.plot(t, f_1, label='vy') 
    ax.plot(t, f_2, label='vz') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_wy_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,5]
    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='wy') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


def plot_force_MPC_xz(file_name,title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_2 = data[:,][:,12] # rear leg x
    f_3 = data[:,][:,13] # rear leg z


    t = np.linspace(0,f_2.shape[0],f_2.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_2, label='rear_x_mpc') 
    ax.plot(t, f_3, label='rear_z_mpc') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_pf(file_name,title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_2 = data[:,][:,7] # front pf_x
    f_3 = data[:,][:,8] # front pf_z
    f_4 = data[:,][:,9] # rear pf_x
    f_5 = data[:,][:,10] # rear pf_z


    t = np.linspace(0,f_2.shape[0],f_2.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_2, label='front_pf_x') 
    ax.plot(t, f_3, label='front_pf_z') 
    ax.plot(t, f_4, label='rear_pf_x') 
    ax.plot(t, f_5, label='rear_pf_z') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

plot_COM_rpy_act('data.txt', 'pitch')
plot_wy_act('data.txt', 'wy')
plot_COM_pos_act('data.txt', 'COM_pos_act')
plot_force_MPC_xz('data.txt', 'GRF_xz')
plot_pf('data.txt', 'pf2CoM_w')



plt.show()
