import os, sys

# Get the current working directory
directory = os.path.dirname(os.getcwd())
print(f'current_directory = {directory}')

import copy
import time
import torch, argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle
from model import Adaptive_dis
from utils import L2_loss


def get_args():
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('--learn_rate', default=2e-4, type=float, help='learning rate')
    parser.add_argument('--dt', default='0.001', type=float, help='sampling time')
    parser.add_argument('--total_steps', default=20000, type=int, help='number of gradient steps')
    parser.add_argument('--print_every', default=1000, type=int, help='number of gradient steps between prints')
    parser.add_argument('--name', default='quadruped', type=str, help='only one option right now')
    parser.add_argument('--verbose', dest='verbose', action='store_true', help='verbose?')
    parser.add_argument('--seed', default=0, type=int, help='random seed')
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--num_points', type=int, default=11,
                        help='number of evaluation points by the ODE solver, including the initial point')
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
def rearrange_full(args, dataset1, dataset2, num_points):
    dT1 = int(args.dt_mpc / args.dt)
    dT2 = int(args.dt_mpc_fl/ args.dt)
    # num_points take 11 points --> predict 10 horizon
    dataset_ct = dataset1[:, ::dT1, :]
    dataset_fl = dataset2[:, ::dT2, :]
    dataset_s = np.concatenate((dataset_ct, dataset_fl), axis = 1)
    num_traj, samples, dim = dataset_s.shape
    print(f'dataset_s.shape:', dataset_s.shape)

    # combine dataset1 and dataset2
    N = samples - num_points + 1
    # print(f'N = {N}')
    new_dataset = np.zeros((num_traj, num_points, N, dim))
    for traj in range(num_traj):
        for i in range(N):
            new_dataset[traj, :, i, :] = dataset_s[traj, i:i + num_points, :]

    return new_dataset

def train(args):
    g1_lambda = 1e-3
    h1_lambda = 1e-3
    h2_lambda = 1e-3
    args.dt_mpc = 0.025
    args.dt_mpc_fl =  0.1 
    args.num_points = 6 + int(0.4/args.dt_mpc_fl) + 1
    print(f'number_points = {args.num_points}')

    model = []
    stats = []

    # device = torch.device('cuda:' + str(args.gpu) if torch.cuda.is_available() else 'cpu')
    device = torch.device("cpu")


    # Set random seed
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    # model
    model = Adaptive_dis(device=device, dt_ct=args.dt_mpc, dt_fl=args.dt_mpc_fl).to(device)

    os.makedirs(directory + '/trained_model/adaptive') if not os.path.exists(
        directory + '/trained_model/adaptive') else None
            
    model_path = '/trained_model/adaptive/quadruped-full-11p-best.tar'
    if os.path.exists(model_path):
        model.load_state_dict(torch.load(model_path, map_location=device))
        print("Model loaded successfully.")
    else:
        print("Model file does not exist.")

    optim = torch.optim.Adam(model.parameters(), args.learn_rate, weight_decay=0.0)

    print(f'======================= TRAIN =======================')
    if device.type == 'cuda':
        cuda_properties = torch.cuda.get_device_properties(device)
        total_memory_in_GB = cuda_properties.total_memory / 1e9
        print(f'DEVICE        : {device}    |   GPU CAPACITY: {total_memory_in_GB:.2f} GB')
    else:
        print(f'DEVICE        : {device}')
    print(f'STEPS         : {args.total_steps}')
    print(f'LEARNING RATE : {args.learn_rate}')
    print(f'PRINT EVRY    : {args.print_every}')
    print(f'=====================================================')
    filename = '/home/ubuntu/Documents/GitHub/Learning_Quadruped_Jumping_Model/residual_learning/data/mix-20traj/fmpc_pd/full-1ms/dataset.pkl'

    dataset = get_dataset(file_name=filename)
    print(f'dataset original shape = {dataset.shape}')

    dataset = rearrange_full(args, dataset1=dataset[:, 0:800, :], dataset2=dataset[:, 800:, :],  num_points=args.num_points)

    num_traj, num_point, N, dim = dataset.shape
    print(f'N = {N}')
    
    # translation-invariant
    dataset[:, 1:, :, 0:2] -= np.expand_dims(dataset[:, 0, :, 0:2], axis=1)
    dataset[:, 0, :, 0:2] = dataset[:, 0, :, 0:2] - dataset[:, 0, :, 0:2]

    # Train and test datasets
    ratio = 0.8
    num_traj = dataset.shape[0]
    train_samples = int(ratio * num_traj)
    train_x = dataset[0:train_samples, ...]
    test_x = dataset[train_samples:, ...]
    train_x_cat = np.concatenate(train_x, axis=1)
    test_x_cat = np.concatenate(test_x, axis=1)

    stats = {'x': [], 'x_hat': [], 'u': [], }
    train_x_cat = torch.tensor(train_x_cat, requires_grad=True, dtype=torch.float32).to(device)
    test_x_cat = torch.tensor(test_x_cat, requires_grad=True, dtype=torch.float32).to(device)
    dt = torch.tensor([args.dt], dtype=torch.float32).to(device)

    # Training stats
    stats = {'train_loss': [], 'test_loss': []}

    best_loss = np.inf
    for step in range(0, args.total_steps + 1):

        start_time = time.time()
        # Predict for train data
        target_hat = None
        h1_net_l1_loss = 0
        g1_net_l1_loss = 0
        h2_net_l1_loss = 0
        horizon_adaptive = 6 # the horizon at which adaptive prediction starts
        for i in range(args.num_points - 1):
            # print(f' i = {i}')
            if (i == 0):
                x0 = train_x_cat[i, :, :]
                x_hat = model.predict(x=x0, num_steps=1, mode=1)
                target_hat = torch.unsqueeze(x_hat[-1, :, 0:6], dim=0)
                q_pf = torch.cat((x0[:, 0:3], x0[:, 7:11]), dim=1)
                h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()
            else:
                if (i < horizon_adaptive): # perform contact prediction
                    x0 = torch.cat((target_hat[i - 1, :, 0:6], train_x_cat[i, :, 6:]), dim=1)
                    x_hat = model.predict(x=x0, num_steps=1, mode=1)
                    target_hat = torch.cat((target_hat, torch.unsqueeze(x_hat[-1, :, 0:6], dim=0)), dim=0)

                    q_pf = torch.cat((x0[:, 0:3], x0[:, 7:11]), dim=1)
                    h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                    g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()
                else: # begin adaptive
                    x0 = torch.cat((target_hat[i - 1, :, 0:6], train_x_cat[i, :, 6:]), dim=1)
                    x_hat_only = None
                    for k in range(train_samples):
                        x0k = x0[k*N : (k+1)*N]
                        x0k_ct, x0k_fl = x0k[:(N - 1 - (i - horizon_adaptive)), :], x0k[N - 1 - (i - horizon_adaptive):, :]
                        x0k_ct_hat = model.predict(x=x0k_ct, num_steps=1, mode=1)  # prediction in contact
                        x0k_fl_hat = model.predict(x=x0k_fl, num_steps=1, mode=2)  # prediction in flight
                        x0k_ct_hat_only, x0k_fl_hat_only = x0k_ct_hat[-1, :, 0:6], x0k_fl_hat[-1, :, 0:6]
                        x0k_hat_only = torch.cat((x0k_ct_hat_only, x0k_fl_hat_only), dim=0)
                        if (k==0):
                            x_hat_only = x0k_hat_only
                        else:
                            x_hat_only = torch.cat((x_hat_only, x0k_hat_only), dim = 0)
                            
                        q_pf = torch.cat((x0k_ct[:, 0:3], x0k_ct[:, 7:11]), dim=1)
                        h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                        g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()
                        q_pf = torch.cat((x0k_fl[:, 0:3], x0k_fl[:, 7:11]), dim=1)
                        h2_net_l1_loss += h2_lambda * torch.abs(model.h2_net(q_pf)).mean()

                    target_hat = torch.cat((target_hat, torch.unsqueeze(x_hat_only, dim=0)), dim=0)

            regularization_loss =  h1_net_l1_loss + g1_net_l1_loss + h2_net_l1_loss

        target = train_x_cat[1:, :, 0:6]

        # Calculate loss
        train_loss = L2_loss(target, target_hat) + regularization_loss

        # Predict for test data
        target_hat = None
        for i in range(args.num_points - 1):
            if (i == 0):
                x0 = test_x_cat[i, :, :]
                x_hat = model.predict(x=x0, num_steps=1, mode=1)
                target_hat = torch.unsqueeze(x_hat[-1, :, 0:6], dim=0)
                q_pf = torch.cat((x0[:, 0:3], x0[:, 7:11]), dim=1)
                h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()
                
            else:
                if (i < horizon_adaptive):  # perform contact prediction
                    x0 = torch.cat((target_hat[i - 1, :, 0:6], test_x_cat[i, :, 6:]), dim=1)
                    x_hat = model.predict(x=x0, num_steps=1, mode=1)
                    target_hat = torch.cat((target_hat, torch.unsqueeze(x_hat[-1, :, 0:6], dim=0)), dim=0)

                    q_pf = torch.cat((x0[:, 0:3], x0[:, 7:11]), dim=1)
                    h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                    g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()

                else:  # begin adaptive
                    x0 = torch.cat((target_hat[i - 1, :, 0:6], test_x_cat[i, :, 6:]), dim=1)
                    x_hat_only = None
                    test_samples = num_traj - train_samples
                    if test_samples == 1:
                        x0k_ct, x0k_fl = x0[:(N - 1 - (i - horizon_adaptive)), :], x0[N - 1 - (i - horizon_adaptive):, :]
                        x0k_ct_hat = model.predict(x=x0k_ct, num_steps=1, mode=1)  # prediction in contact
                        x0k_fl_hat = model.predict(x=x0k_fl, num_steps=1, mode=2)  # prediction in flight
                        x0k_ct_hat_only, x0k_fl_hat_only = x0k_ct_hat[-1, :, 0:6], x0k_fl_hat[-1, :, 0:6]
                        x0k_hat_only = torch.cat((x0k_ct_hat_only, x0k_fl_hat_only), dim=0)
                        x_hat_only = x0k_hat_only

                        q_pf = torch.cat((x0k_ct[:, 0:3], x0k_ct[:, 7:11]), dim=1)
                        h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                        g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()
                        q_pf = torch.cat((x0k_fl[:, 0:3], x0k_fl[:, 7:11]), dim=1)
                        h2_net_l1_loss += h2_lambda * torch.abs(model.h2_net(q_pf)).mean()

                    else:
                        for k in range(test_samples):
                            x0k = x0[k * N: (k + 1) * N]
                            x0k_ct, x0k_fl = x0k[:(N - 1 - (i - horizon_adaptive)), :], x0k[N - 1 - (i - horizon_adaptive):, :]
                            x0k_ct_hat = model.predict(x=x0k_ct, num_steps=1, mode=1)  # prediction in contact
                            x0k_fl_hat = model.predict(x=x0k_fl, num_steps=1, mode=2)  # prediction in flight
                            x0k_ct_hat_only, x0k_fl_hat_only = x0k_ct_hat[-1, :, 0:6], x0k_fl_hat[-1, :, 0:6]
                            x0k_hat_only = torch.cat((x0k_ct_hat_only, x0k_fl_hat_only), dim=0)
                            if (k == 0):
                                x_hat_only = x0k_hat_only
                            else:
                                x_hat_only = torch.cat((x_hat_only, x0k_hat_only), dim=0)

                            q_pf = torch.cat((x0k_ct[:, 0:3], x0k_ct[:, 7:11]), dim=1)
                            h1_net_l1_loss += h1_lambda * torch.abs(model.h1_net(q_pf)).mean()
                            g1_net_l1_loss += g1_lambda * torch.abs(model.g1_net(q_pf)).mean()
                            q_pf = torch.cat((x0k_fl[:, 0:3], x0k_fl[:, 7:11]), dim=1)
                            h2_net_l1_loss += h2_lambda * torch.abs(model.h2_net(q_pf)).mean()

                    target_hat = torch.cat((target_hat, torch.unsqueeze(x_hat_only, dim=0)), dim=0)

            regularization_loss =  h1_net_l1_loss + g1_net_l1_loss + h2_net_l1_loss

        target = test_x_cat[1:, :, 0:6]
        
        # Calculate loss
        test_loss = L2_loss(target, target_hat) + regularization_loss

        if test_loss < best_loss:
            best_loss = test_loss
            best_module = copy.deepcopy(model.state_dict())

        # Gradient descent
        if step > 0:
            train_loss.backward()
            optim.step()
            optim.zero_grad()

        # Logging stats
        stats['train_loss'].append(train_loss.item())
        stats['test_loss'].append(test_loss.item())

        if step % args.print_every == 0:
            print(f'\n--------------------------------------------')
            print("step {}, train_loss {:.4e}, test_loss {:.4e}".format(step, train_loss.item(), test_loss.item()))

            # Uncomment this to save model every args.print_every steps
            os.makedirs(directory + '/trained_model/adaptive') if not os.path.exists(
                directory + '/trained_model/adaptive') else None
            label = '-full'
            path = '{}/{}{}-{}p-{}.tar'.format(directory + '/trained_model/adaptive', args.name, label, args.num_points,
                                               step)
            torch.save(model.state_dict(), path)

        # Estimate total time for num_epochs
        end_time = time.time()
        one_epoch_time = end_time - start_time
        total_time_estimate = one_epoch_time * (args.total_steps - step)
        print(f"\rTime taken for one epoch: {one_epoch_time:.2f} seconds", end='')
        print(
            f"\rEstimated time for the remaining {args.total_steps - step} epochs: {total_time_estimate / 60:.2f} minutes",
            end='')

    # save best model
    model.load_state_dict(best_module)
    os.makedirs(directory + '/trained_model/adaptive') if not os.path.exists(
        directory + '/trained_model/adaptive') else None
    label = '-full'
    path = '{}/{}{}-{}p-best.tar'.format(directory + '/trained_model/adaptive', args.name, label, args.num_points)
    torch.save(model.state_dict(), path)
    
    stats['train_x'] = train_x_cat.detach().cpu().numpy()
    stats['test_x'] = test_x_cat.detach().cpu().numpy()
    stats['dt'] = dt.detach().cpu().numpy()

    return model, stats


if __name__ == "__main__":
    args = get_args()
    model, stats = train(args)

    # Save model
    os.makedirs(directory + '/trained_model/adaptive') if not os.path.exists(
        directory + '/trained_model/adaptive') else None
    label = '-full'
    path = '{}/{}{}-{}p.tar'.format(directory + '/trained_model/adaptive', args.name, label, args.num_points)
    torch.save(model.state_dict(), path)
    path = '{}/{}{}-{}p-stats.pkl'.format(directory + '/trained_model/adaptive', args.name, label, args.num_points)
    print("Saved file: ", path)
    with open(path, 'wb') as file:
        pickle.dump(stats, file)
