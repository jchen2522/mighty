import sys
sys.path.append('/home/kkondo/code/mighty_ws/build/mighty')
import numpy as np
# import py_mighty
# import yaml
import os
import importlib.util

so_file = '/home/kkondo/code/mighty_ws/build/mighty/libpy_mighty.so'
spec = importlib.util.spec_from_file_location("py_mighty", so_file)
py_mighty = importlib.util.module_from_spec(spec)
spec.loader.exec_module(py_mighty)


if __name__ == '__main__':
    par_ = py_mighty.parameters()
    planner_params_ = py_mighty.planner_params_t()
    lbfgs_params_ = py_mighty.lbfgs_parameter_t()

    planner_params_.verbose = False                                 
    planner_params_.V_max = par_.v_max                              
    planner_params_.A_max = par_.a_max                              
    planner_params_.J_max = par_.j_max                              
    planner_params_.num_perturbation = par_.num_perturbation_for_ig 
    planner_params_.r_max = par_.r_max_for_ig                       
    planner_params_.time_weight = par_.time_weight                  
    planner_params_.pos_anchor_weight = par_.pos_anchor_weight
    planner_params_.dyn_weight = par_.dynamic_weight
    planner_params_.stat_weight = par_.stat_weight
    planner_params_.jerk_weight = par_.jerk_weight
    planner_params_.dyn_constr_vel_weight = par_.dyn_constr_vel_weight
    planner_params_.dyn_constr_acc_weight = par_.dyn_constr_acc_weight
    planner_params_.dyn_constr_jerk_weight = par_.dyn_constr_jerk_weight
    planner_params_.dyn_constr_bodyrate_weight = par_.dyn_constr_bodyrate_weight
    planner_params_.dyn_constr_tilt_weight = par_.dyn_constr_tilt_weight
    planner_params_.dyn_constr_thrust_weight = par_.dyn_constr_thrust_weight
    planner_params_.num_dyn_obst_samples = par_.num_dyn_obst_samples
    planner_params_.Co = par_.planner_Co;                            
    planner_params_.Cw = par_.planner_Cw                             
    planner_params_.BIG = 1e8
    planner_params_.dc = par_.dc                                            
    planner_params_.init_turn_bf = par_.init_turn_bf

    lbfgs_params_.mem_size = 256
    lbfgs_params_.min_step = 1.0e-32
    lbfgs_params_.f_dec_coeff = par_.f_dec_coeff 
    lbfgs_params_.cautious_factor = par_.cautious_factor 
    lbfgs_params_.past = par_.past
    lbfgs_params_.max_linesearch = par_.max_linesearch 
    lbfgs_params_.max_iterations = par_.max_iterations
    lbfgs_params_.g_epsilon = par_.g_epsilon
    lbfgs_params_.delta = par_.delta


    whole_traj_solver_ptr = py_mighty.SolverLBFGS()
    whole_traj_solver_ptr.initializeSolver(planner_params_)

    # need to fix the params for
    A_time = 0.0
    global_path = [
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 1.2),
        (2.0, 1.0, 1.5),
    ]
    lc1 = py_mighty.LinearConstraint3D()
    lc2 = py_mighty.LinearConstraint3D()
    safe_corridor_polytopes_whole_ = [lc1, lc2]

    ob1 = py_mighty.dynTraj()
    ob2 = py_mighty.dynTraj()
    local_trajs = [ob1, ob2]
    local_A = py_mighty.state()
    local_E = py_mighty.state()
    par_.use_multiple_initial_guesses = False

    initial_guess_computation_time = whole_traj_solver_ptr.prepareSolverForReplan(A_time, global_path, safe_corridor_polytopes_whole_, local_trajs, local_A, local_E, par_.use_multiple_initial_guesses)
    # whole_traj_solver_ptr.getGlobalPath(global_path)

    list_z0 = whole_traj_solver_ptr.getInitialGuesses()
    list__initial_guess_wps = whole_traj_solver_ptr.getInitialGuessWaypoints()

    status, zopt, fopt = whole_traj_solver_ptr.optimize(list_z0[0], lbfgs_params_)
    print('zopt: ', zopt)