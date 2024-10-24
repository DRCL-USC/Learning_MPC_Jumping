import os, sys

# Get the directory (up 1 level)
directory = os.path.dirname(os.getcwd())
print(f'directory = {directory}')
print(f'working_directory = {os.getcwd()}')

import os
import torch, argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle
from model import Adaptive_dis

import scienceplots

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
plt.rcParams['text.usetex'] = True


def get_args():
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('--dt', default='0.001', type=float, help='sampling time')
    parser.add_argument('--name', default='quadruped', type=str, help='only one option right now')
    parser.add_argument('--gpu', type=int, default=0)
    parser.set_defaults(feature=True)
    args, unknown = parser.parse_known_args()  # use parse_known_args instead of parse_args
    return args


def get_dataset(file_name):
    if os.path.exists(file_name):
        # Load the result from the pickle file
        with open(file_name, 'rb') as file:
            dataset = pickle.load(file)
        print('Loaded result from', file_name)
        return dataset


def plot(st_idx, x, x_hat_adaptive, x_hat_fixed, x_hat_nominal=None, dt_mpc=0.025, dt_mpc_fl = 0.1):
    horizon = x.shape[0]
    t = np.zeros(horizon)
    start_hybrid = 6
    for i in range(horizon):
        if i <= start_hybrid:
            t[i] = i * dt_mpc * 1000
        else:
            t[i] = t[start_hybrid] + (i - (start_hybrid)) * dt_mpc_fl * 1000

    with plt.style.context(['science']):
        
        t = t + st_idx
        plt.figure(figsize=[12.0, 12])  # Adjusted for 2 columns
        fontsize = 14
        label_fontsize = 22
        linewidth=2
        scatter_pts_size = 60

        titles = [r'$x$', r'$z$', r'$\phi$', r'$v_x$', r'$v_z$', r'$\omega$']
        y_labels = [r'$x$ \ [m]', r'$z$ \ [m]', r'$\phi$ \ [rad]', r'$v_x$ \ [m/s]', r'$v_z$ \ [m/s]', r'$\omega$ \ [rad/s]']

        indices = [(0, 'x (m)'), (1, 'z (m)'), (2, 'Pitch (rad)'), (3, 'vx (m/s)'), (4, 'vz (m/s)'), (5, 'omega (rad/s)')]

        plot_handles = []

        for i, (index, label) in enumerate(indices):
            # Calculate subplot number for 2 columns and 3 rows
            plt.subplot(3, 2, (i % 3) * 2 + (i // 3) + 1)

            # Add colored backgrounds
            print(f't = {t.shape}')
            plt.axvspan(t[0], t[6], facecolor='#73c7f1', alpha=0.8)  # First 6 samples
            plt.axvspan(t[-5], t[-1], facecolor='#f9d56b', alpha=0.8)  # Last 4 samples

            if i == 2:  # Special case for theta
                h1, = plt.plot(t, x[:, index], color='k', linestyle='--', linewidth=linewidth, label='Ground-truth')
                h2, = plt.plot(t, x_hat_adaptive[:, index], color='r', linewidth=linewidth, label='Variable-frequency learned model (Our method)')
                h3, = plt.plot(t, x_hat_fixed[:, index], color='b', linewidth=linewidth, label='Fixed-frequency learned model')
                h4, = plt.plot(t, x_hat_nominal[:, index], color='g', linewidth=linewidth, label='Variable-frequency nominal model')
                plt.scatter(t, x[:, index], color='k', s=scatter_pts_size)
                plt.scatter(t, x_hat_adaptive[:, index], color='r', s=scatter_pts_size)
                plt.scatter(t, x_hat_fixed[:, index], color='b', s=scatter_pts_size)
                plt.scatter(t, x_hat_nominal[:, index], color='g', s=scatter_pts_size)
            else:
                h1, = plt.plot(t, x[:, index], color='k', linestyle='--', linewidth=linewidth, label='Ground-truth')
                h2, = plt.plot(t, x_hat_adaptive[:, index], color='r', linewidth=linewidth, label='Variable-frequency learned model (Our method)')
                h3, = plt.plot(t, x_hat_fixed[:, index], color='b', linewidth=linewidth, label='Fixed-frequency learned model')
                h4, = plt.plot(t, x_hat_nominal[:, index], color='g', linewidth=linewidth, label='Variable-frequency nominal model')
                plt.scatter(t, x[:, index], color='k', s=scatter_pts_size)
                plt.scatter(t, x_hat_adaptive[:, index], color='r', s=scatter_pts_size)
                plt.scatter(t, x_hat_fixed[:, index], color='b', s=scatter_pts_size)
                plt.scatter(t, x_hat_nominal[:, index], color='g', s=scatter_pts_size)

            plt.xlabel(r'Time [ms]', fontsize=label_fontsize)
            plt.ylabel(y_labels[i], fontsize=label_fontsize)
            plt.grid(True)

            if i == 0:  # Only add handles for the first plot
                plot_handles = [h1, h2, h3, h4]

        plt.tight_layout(rect=[0, 0.08, 1, 0.95])
        plt.legend(plot_handles, ['Ground-truth', 'Variable-frequency learned model (Our method)', 'Fixed-frequency learned model', 'Variable-frequency nominal model'], 
           loc='upper center', bbox_to_anchor=(-0.1, -0.25), fontsize=fontsize, frameon=True, edgecolor='black', framealpha=0.7, ncol=2)
        plt.savefig('./result/rollout.pdf', bbox_inches='tight', pad_inches=0.1)
        plt.savefig('./result/rollout.png', bbox_inches='tight', pad_inches=0.1)
        plt.show()

def evaluate(args):

    args.dt_mpc = 0.025
    args.dt_mpc_fl = 0.1
    args.num_points = 6 + int(0.4 / args.dt_mpc_fl) + 1 # num_horizon + 1
    num_points = args.num_points
    
    # device = torch.device('cuda:' + str(args.gpu) if torch.cuda.is_available() else 'cpu')
    device = torch.device('cpu')


    # Load models
    path = directory + '/trained_model/adaptive/quadruped-full-11p-best.tar'
    model_adaptive = Adaptive_dis(device=device, dt_ct=args.dt_mpc, dt_fl=args.dt_mpc_fl).to(device)
    model_adaptive.load_state_dict(torch.load(path, map_location=device))

    path = directory + '/trained_model/fixed/quadruped-full-11p-best.tar'
    model_fixed = Adaptive_dis(device=device, dt_ct=args.dt_mpc, dt_fl=args.dt_mpc_fl).to(device)
    model_fixed.load_state_dict(torch.load(path, map_location=device))
    

    # Load dataset
    filename = '/home/ubuntu/Documents/GitHub/Learning_Quadruped_Jumping_Model/residual_learning/data/mix-20traj/fmpc_pd/full-1ms/dataset.pkl'
    dataset = get_dataset(file_name=filename)

    # Plot prediction: nominal vs grouth-truth vs learned
    stats = {'x_gt': [], 'x_hat_adaptive': [], 'x_hat_fixed': [], 'x_hat_nominal': [],}


    # Used in paper 
    single_traj_ct = dataset[-3, :800]
    single_traj_fl = dataset[-3, 800:]

    dT1 = int(args.dt_mpc / args.dt)
    dT2 = int(args.dt_mpc_fl / args.dt)
    dataset_ct = single_traj_ct[::dT1, :]
    dataset_fl = single_traj_fl[::dT2, :]
    dataset_s = np.concatenate((dataset_ct, dataset_fl), axis=0)
    num_points = args.num_points
    samples, dim = dataset_s.shape
    N = samples - num_points + 1
    new_dataset = np.zeros((num_points, N, dim))
    for i in range(N):
        new_dataset[:, i, :] = dataset_s[i:i + num_points, :]

    # torch
    single_traj = new_dataset

    # translation-invariant
    single_traj[1:, :, 0:2] -= np.expand_dims(single_traj[0, :, 0:2], axis=0)
    single_traj[0, :, 0:2] = single_traj[0, :, 0:2] - single_traj[0, :, 0:2]
    single_traj = torch.tensor(single_traj, requires_grad=True, dtype=torch.float32).to(device)

    x, _, _, _ = torch.split(single_traj, [6, 1, 4, 4], dim=2)  # (q, qdot, g, r, u)
    stats['x_gt'] = x.detach().cpu().numpy()
    x0 = single_traj[0, :, :] # initial state 
    x_hat_adaptive = torch.unsqueeze(x0, dim=0)
    x_hat_fixed = torch.unsqueeze(x0, dim=0)
    x_hat_nominal = torch.unsqueeze(x0, dim=0)
    stats['x_hat_adaptive'].append(x_hat_adaptive[-1, :, :])
    stats['x_hat_fixed'].append(x_hat_fixed[-1, :, :])
    stats['x_hat_nominal'].append(x_hat_adaptive[-1, :, :])
    start_hybrid = 6
    for i in range(num_points - 1):
        # learned model (adaptive) 
        if i < start_hybrid:
            x0 = torch.cat((x_hat_adaptive[i, :, 0:6], single_traj[i, :, 6:]), dim=1)
            x_hat_step = model_adaptive.predict(x=x0, num_steps=1, mode=1)
            x_hat_adaptive = torch.cat((x_hat_adaptive, torch.unsqueeze(x_hat_step[-1, :, :], dim=0)), dim=0)
        else:
            x0 = torch.cat((x_hat_adaptive[i, :, 0:6], single_traj[i, :, 6:]), dim=1)
            x0_ct, x0_fl = x0[:(N - 1 - (i - start_hybrid)), :], x0[N - 1 - (i - start_hybrid):, :]
            x0_ct_hat = model_adaptive.predict(x=x0_ct, num_steps=1, mode=1)  # prediction in contact
            x0_fl_hat = model_adaptive.predict(x=x0_fl, num_steps=1, mode=2)  # prediction in flight
            x0_ct_hat_only, x0_fl_hat_only = x0_ct_hat[-1, :, :], x0_fl_hat[-1, :, :]
            x0_hat_only = torch.cat((x0_ct_hat_only, x0_fl_hat_only), dim=0)
            x_hat_adaptive = torch.cat((x_hat_adaptive, torch.unsqueeze(x0_hat_only, dim=0)), dim=0)
        stats['x_hat_adaptive'].append(x_hat_adaptive[-1, ...])
    
    for i in range(num_points - 1):
        # learned model (fixed) 
        if i < start_hybrid:
            x0 = torch.cat((x_hat_fixed[i, :, 0:6], single_traj[i, :, 6:]), dim=1)
            x_hat_step = model_fixed.predict(x=x0, num_steps=1, mode=1)
            x_hat_fixed = torch.cat((x_hat_fixed, torch.unsqueeze(x_hat_step[-1, :, :], dim=0)), dim=0)
        else:
            x0 = torch.cat((x_hat_fixed[i, :, 0:6], single_traj[i, :, 6:]), dim=1)
            x0_ct, x0_fl = x0[:(N - 1 - (i - start_hybrid)), :], x0[N - 1 - (i - start_hybrid):, :]
            x0_ct_hat = model_fixed.predict(x=x0_ct, num_steps=1, mode=1)  # prediction in contact
            x0_fl_hat = model_fixed.predict(x=x0_fl, num_steps=1, mode=2)  # prediction in flight
            x0_ct_hat_only, x0_fl_hat_only = x0_ct_hat[-1, :, :], x0_fl_hat[-1, :, :]
            x0_hat_only = torch.cat((x0_ct_hat_only, x0_fl_hat_only), dim=0)
            x_hat_fixed = torch.cat((x_hat_fixed, torch.unsqueeze(x0_hat_only, dim=0)), dim=0)
        stats['x_hat_fixed'].append(x_hat_fixed[-1, ...])

    for i in range(num_points - 1):
        # nominal model
        if i < start_hybrid:
            x0 = torch.cat((x_hat_nominal[i, :, 0:6], single_traj[i, :, 6:]), dim=1)
            x_hat_step = model_adaptive.predict(x=x0, num_steps=1, mode='norm_ct')
            x_hat_nominal = torch.cat((x_hat_nominal, torch.unsqueeze(x_hat_step[-1, :, :], dim=0)), dim=0)
        else:
            x0 = torch.cat((x_hat_nominal[i, :, 0:6], single_traj[i, :, 6:]), dim=1)
            x0_ct, x0_fl = x0[:(N - 1 - (i - start_hybrid)), :], x0[N - 1 - (i - start_hybrid):, :]
            x0_ct_hat = model_adaptive.predict(x=x0_ct, num_steps=1, mode='norm_ct')  # prediction in contact
            x0_fl_hat = model_adaptive.predict(x=x0_fl, num_steps=1, mode='norm_fl')  # prediction in flight
            x0_ct_hat_only, x0_fl_hat_only = x0_ct_hat[-1, :, :], x0_fl_hat[-1, :, :]
            x0_hat_only = torch.cat((x0_ct_hat_only, x0_fl_hat_only), dim=0)
            x_hat_nominal = torch.cat((x_hat_nominal, torch.unsqueeze(x0_hat_only, dim=0)), dim=0)
        stats['x_hat_nominal'].append(x_hat_nominal[-1, ...])

    # Convert to numpy
    x_gt = stats['x_gt']
    x_hat_adaptive = torch.stack(stats['x_hat_adaptive']).detach().cpu().numpy()
    x_hat_fixed = torch.stack(stats['x_hat_fixed']).detach().cpu().numpy()
    x_hat_nominal = torch.stack(stats['x_hat_nominal']).detach().cpu().numpy()

    # We have N trajectories to roll out, select among [0, N-1]
    start_rollout_idx = N-1 #  5, N-1

    print(f'x_gt_start = {x_gt[:, start_rollout_idx, :]}')
    if start_rollout_idx <= N - 4: # in contact phase mode
        args.dt_mpc_fl = args.dt_mpc
    else:
        start_rollout_idx = 26 # 26 # consider full flight phase prediction

    st_idx = int(start_rollout_idx * args.dt_mpc * 1000)
    
    plot(st_idx, x=x_gt[:, start_rollout_idx, :], x_hat_adaptive=x_hat_adaptive[:, start_rollout_idx, :], 
         x_hat_fixed=x_hat_fixed[:, start_rollout_idx, :], x_hat_nominal=x_hat_nominal[:, start_rollout_idx, :],
         dt_mpc= args.dt_mpc, dt_mpc_fl=args.dt_mpc_fl)
    
if __name__ == "__main__":
    args = get_args()
    evaluate(args)