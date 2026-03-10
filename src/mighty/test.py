import sys
sys.path.append('/home/kkondo/code/mighty_ws/build/mighty')
import numpy as np
import py_mighty
# import yaml
import os
import importlib.util

def generate_circle_waypoints(radius=5.0, center=(0,0,1.2), num_points=20):
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


def compute_trajectory():
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

    par_.use_multiple_initial_guesses = False

    # generate circle segments
    circle_points = generate_circle_waypoints(radius=5.0, num_points=20)
    dataset = []
    traj_list = []

    # for velocity calcs
    cx, cy, cz = 0.0, 0.0, 1.2
    speed = 2.0

    for i in range(len(circle_points)-2):

        start = circle_points[i]
        med = circle_points[i+1]
        end = circle_points[i+2]

        global_path = [start, med, end]

        # compute tangent velocity
        dx = start[0] - cx
        dy = start[1] - cy

        tangent = np.array([-dy, dx, 0.0])
        tangent = tangent / np.linalg.norm(tangent)

        local_A.setVel(speed * tangent)
        local_A.setPos(np.array(start))
        local_E.setPos(np.array(end))

        # print(global_path)

        initial_guess_computation_time = whole_traj_solver_ptr.prepareSolverForReplan(A_time, global_path, safe_corridor_polytopes_whole_, local_trajs, local_A, local_E, par_.use_multiple_initial_guesses)
        list_z0 = whole_traj_solver_ptr.getInitialGuesses()
        list__initial_guess_wps = whole_traj_solver_ptr.getInitialGuessWaypoints()

        status, zopt, fopt = whole_traj_solver_ptr.optimize(list_z0[0], lbfgs_params_)
        traj = whole_traj_solver_ptr.reconstructPVATCPopt(zopt)
        traj = whole_traj_solver_ptr.getGoalSetpoints()
        traj_list.append(traj)

        dataset.append((start, end, zopt))
    
    

    # whole_traj_solver_ptr.getGlobalPath(global_path)

    # dataset = array of multiple (start, end, zopt). traj_list parametrized traj for viz
    return dataset, traj_list

if __name__ == '__main__':
    # traj = compute_trajectory()[1] # wrong
    # for i, sp in enumerate(traj):
    #     print(f"Step {i}")
    #     print("t:", sp.t)
    #     print("pos:", sp.pos)
    #     print("vel:", sp.vel)
    #     print("acc:", sp.accel)
    #     print("----------------")
    print("hello") # placeholder