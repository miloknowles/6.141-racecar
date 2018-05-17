import numpy as np

def motion_model(proposal_dist, action, max_perturb=0.5, min_perturb=0):
    """
    applies the action and some random perturbation to the proposal dist
    """
    proposal_dist += action
    perturb = (max_perturb-min_perturb) \
        * np.random.random_sample(proposal_dist.shape) + min_perturb
    np.add(perturb, proposal_dist, perturb)
    return perturb


proposal_dist = np.array([[0,0,0],
                          [0,0,0],
                          [0,0,0]])

action = np.array([1,1,1])
motion_model(proposal_dist, action)
