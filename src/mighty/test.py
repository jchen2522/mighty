import sys
sys.path.append('/home/kkondo/code/mighty_ws/build/mighty')
import numpy as np
import py_mighty
# import yaml
import os
import importlib.util

# NN imports
import torch
from torch.utils.data import Dataset
import torch.nn as nn
from torch.utils.data import DataLoader
import torch.optim as optim

# global vars
par_ = py_mighty.parameters()
planner_params_ = py_mighty.planner_params_t()
lbfgs_params_ = py_mighty.lbfgs_parameter_t()

planner_params_.verbose = False                                 
planner_params_.V_max = 4                            
planner_params_.A_max = 10                            
planner_params_.J_max = 30                         
planner_params_.num_perturbation = 8
planner_params_.r_max = 1         
planner_params_.time_weight = 500        
planner_params_.pos_anchor_weight = 0 # 0
planner_params_.dyn_weight = 10
planner_params_.stat_weight = 1000 
planner_params_.jerk_weight = 0.1
planner_params_.dyn_constr_vel_weight = 1000
planner_params_.dyn_constr_acc_weight = 1000
planner_params_.dyn_constr_jerk_weight = 1000
planner_params_.dyn_constr_bodyrate_weight = 0
planner_params_.dyn_constr_tilt_weight = 0
planner_params_.dyn_constr_thrust_weight = 0
planner_params_.num_dyn_obst_samples = 10
planner_params_.Co = 0.2                       
planner_params_.Cw = 3                          
planner_params_.BIG = 1e8
planner_params_.dc = 0.01                                      
planner_params_.init_turn_bf = 80

lbfgs_params_.mem_size = 256
lbfgs_params_.min_step = 1.0e-32
lbfgs_params_.f_dec_coeff = 0.001
lbfgs_params_.cautious_factor = 1e-06
lbfgs_params_.past = 3
lbfgs_params_.max_linesearch = 32  # segfault if 32 was bc need 4 lc for safe corridor
lbfgs_params_.max_iterations = 1000
lbfgs_params_.g_epsilon = 1e-05
lbfgs_params_.delta = 1e-05

whole_traj_solver_ptr = py_mighty.SolverLBFGS()
whole_traj_solver_ptr.initializeSolver(planner_params_)

# need to fix the params for
A_time = 0.0
# global_path = [
#     (0.0, 0.0, 1.0),
#     (1.0, 0.0, 1.2),
#     (2.0, 1.0, 1.21),
#     (3.0, 1.2, 1.24),
#     (4.0, 1.3, 1.26),
# ]
lc1 = py_mighty.LinearConstraint3D()
lc2 = py_mighty.LinearConstraint3D()
lc3 = py_mighty.LinearConstraint3D()
lc4 = py_mighty.LinearConstraint3D()
safe_corridor_polytopes_whole_ = [lc1, lc2, lc3, lc4]

ob1 = py_mighty.dynTraj()
ob2 = py_mighty.dynTraj()
local_trajs = [] # empty in the OG mighty, ignore ob1, ob2

par_.use_multiple_initial_guesses = False


def generate_circle_waypoints(radius=None, center=None, num_points=None):
    # randomize if not provided
    if radius is None:
        radius = np.random.uniform(2.0, 3.0)  # adjust min/max radius
    if center is None:
        center = (np.random.uniform(-3,3), np.random.uniform(-3,3), np.random.uniform(1.0,2.0))
    if num_points is None:
        num_points = np.random.randint(15, 30)  # random number of points

    cx, cy, cz = center
    waypoints = []

    for i in range(num_points):
        theta = 2*np.pi * i / num_points
        x = cx + radius * np.cos(theta)
        y = cy + radius * np.sin(theta)
        z = cz
        waypoints.append((x,y,z))

    waypoints.append(waypoints[0])
    waypoints.append(waypoints[1])

    return waypoints


def compute_trajectory(num_circles=200):
    
    dataset = []
    traj_list = []

    local_A = py_mighty.state()
    local_A.setPos(np.array([0.0, 0.0, 0.0]))
    local_A.setVel(np.array([3.5, 0.9, 0.1]))
    local_A.setAccel(np.array([0.0, 0.0, 0.0])) # [0.2, 0.9, 0.1]
    local_A.setJerk(np.array([0.0, 0.0, 0.0]))

    local_E = py_mighty.state()
    local_E.setPos(np.array([10.0, 2.0, 1.3]))
    local_E.setVel(np.array([0.0, 0.0, 0.0]))
    local_E.setAccel(np.array([0.0, 0.0, 0.0]))
    local_E.setJerk(np.array([0.0, 0.0, 0.0]))

    for _ in range(num_circles):
        # generate a random circle
        circle_points = generate_circle_waypoints(2.5, (0.0, 0.0, 1.5), 25)

        # calculate speed tangent for each segment
        cx, cy, cz = circle_points[0][0], circle_points[0][1], circle_points[0][2]
        speed = 2.0
    
        for i in range(len(circle_points)-2):

            start = circle_points[i]
            med = circle_points[i+1]
            end = circle_points[i+2]

            global_path = [start, med, end]

            # compute tangent velocity
            segment_vec = np.array([med[0]-start[0], med[1]-start[1], 0.0])
            segment_norm = np.linalg.norm(segment_vec)
            if segment_norm < 1e-8:
                tangent = np.array([1.0,0.0,0.0])
            else:
                tangent = segment_vec / segment_norm

            local_A.setVel(speed * tangent)
            local_A.setPos(np.array(start))
            local_E.setPos(np.array(end))

            # print(global_path)

            whole_traj_solver_ptr = py_mighty.SolverLBFGS()
            whole_traj_solver_ptr.initializeSolver(planner_params_)

            initial_guess_computation_time = whole_traj_solver_ptr.prepareSolverForReplan(A_time, global_path, safe_corridor_polytopes_whole_, local_trajs, local_A, local_E, par_.use_multiple_initial_guesses)
            list_z0 = whole_traj_solver_ptr.getInitialGuesses()
            list__initial_guess_wps = whole_traj_solver_ptr.getInitialGuessWaypoints()

            status, zopt, fopt = whole_traj_solver_ptr.optimize(list_z0[0], lbfgs_params_)
            # print("zopt:", zopt)

            # skip invalid outputs
            if np.any(np.isnan(zopt)) or np.any(np.isinf(zopt)):
                continue
    
            
            traj = whole_traj_solver_ptr.reconstructPVATCPopt(zopt)
            traj = whole_traj_solver_ptr.getGoalSetpoints()
            traj_list.append(traj)

            z0 = list_z0[0]

            dataset.append((start, end, zopt))
        
        # whole_traj_solver_ptr.getGlobalPath(global_path)

    # dataset = array of multiple (start, end, zopt). traj_list parametrized traj for viz
    return dataset, traj_list

def resample_traj(traj, N=50):

    traj = np.array([s.pos for s in traj])

    t_old = np.linspace(0, 1, len(traj))
    t_new = np.linspace(0, 1, N)

    new_traj = np.zeros((N,3))

    for i in range(3):
        new_traj[:,i] = np.interp(t_new, t_old, traj[:,i])

    return new_traj

# FF Neural Network ------------------------------------------------------
class TrajDataset(Dataset):

    def __init__(self, dataset, traj_list):

        self.X = [] # start and end concat
        self.Y = [] # traj


        for (start, end, zopt), traj in zip(dataset, traj_list):

            x = list(start) + list(end)

            self.X.append(x)
            # zopt_np = np.array(zopt, dtype=np.float32).flatten()
            # self.Y.append(zopt_np)
            # traj_np = np.array(traj, dtype=np.float32).flatten()
            traj_np = resample_traj(traj, 50).astype(np.float32).flatten()            
            self.Y.append(traj_np)

        self.Y = np.array(self.Y, dtype=np.float32)  # shape: (num_samples, zopt_dim)
        self.X = np.array(self.X, dtype=np.float32)
        self.Y = torch.from_numpy(self.Y)
        self.X = torch.from_numpy(self.X)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]



class MightyNet(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()

        self.net = nn.Sequential(

            nn.Linear(input_dim, 128),
            nn.ReLU(),

            nn.Linear(128, 256),
            nn.ReLU(),

            nn.Linear(256, 256),
            nn.ReLU(),

            nn.Linear(256, 128),
            nn.ReLU(),

            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.net(x)

def model_out():
    # load
    raw_dataset, traj_list = compute_trajectory()
    print(len(raw_dataset))
    dataset = TrajDataset(raw_dataset, traj_list)
    loader = DataLoader(dataset, batch_size=64, shuffle=True)
    

    # X_mean = dataset.X.mean(axis=0)
    # X_std = dataset.X.std(axis=0) + 1e-6
    # Y_mean = dataset.Y.mean(axis=0)
    # Y_std = dataset.Y.std(axis=0) + 1e-6

    # validity of zopt
    # Y_max = dataset.Y.max(axis=0)
    # Y_min = dataset.Y.min(axis=0)
    # print("Y_max", Y_max)
    # print("Y_min", Y_min)

    # dataset.X = (dataset.X - X_mean) / X_std
    # dataset.Y = (dataset.Y - Y_mean) / Y_std

    input_dim = dataset.X.shape[1]
    output_dim = dataset.Y.shape[1]

    model = MightyNet(input_dim, output_dim)

    optimizer = optim.Adam(model.parameters(), lr=1e-4)
    criterion = torch.nn.MSELoss()

    # training loop
    for epoch in range(200):

        total_loss = 0

        for x, y in loader:

            pred = model(x)
            if torch.isnan(pred).any():
                print("Prediction became NaN")
                break

            loss = criterion(pred, y)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        print("epoch:", epoch, "loss:", total_loss)


    model.eval()
    # start, end, _ = raw_dataset[np.random.randint(len(raw_dataset))]  
    start, end, _ = raw_dataset[0] 
    input_vector = list(start) + list(end)
    x = torch.tensor(input_vector).float().unsqueeze(0)

    # convert to traj
    z_pred = model(x)
    # z_pred = z_pred * Y_std + Y_mean
    z_np = z_pred.detach().cpu().numpy().flatten()
    traj = z_np.reshape(-1, 3)  # or correct dimension

    # whole_traj_solver_ptr = py_mighty.SolverLBFGS()
    # whole_traj_solver_ptr.initializeSolver(planner_params_)

    # traj = whole_traj_solver_ptr.reconstructPVATCPopt(z_np)
    # traj = whole_traj_solver_ptr.getGoalSetpoints()

    return traj


if __name__ == '__main__':
    



    print("training done") # placeholder