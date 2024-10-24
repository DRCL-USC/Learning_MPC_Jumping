import torch

def L2_loss(q, q_hat, split=None):
    return (q-q_hat).pow(2).mean()