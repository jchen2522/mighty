#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "mighty/mighty.hpp"
#include "mighty/lbfgs_solver.hpp"
#include "mighty/lbfgs.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace py = pybind11;

PYBIND11_MODULE(py_mighty, m) {
    m.doc() = "Python bindings for MIGHTY planner";

    // ------------------------------
    // Bind PCL point minimally
    py::class_<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>::Ptr>(m, "PointCloudXYZ")
        .def(py::init<>())
        .def_readwrite("points", &pcl::PointCloud<pcl::PointXYZ>::points);


    // ------------------------------
    // Bind Polyhedon<3> minimally
    py::class_<Polyhedron<3>>(m, "Polyhedron3")
        .def(py::init<>());


    // ------------------------------
    // Bind PieceWisePol
    py::class_<PieceWisePol>(m, "PieceWisePol")
        .def(py::init<>())
        .def_readwrite("times", &PieceWisePol::times)
        .def_readwrite("coeff_x", &PieceWisePol::coeff_x)
        .def_readwrite("coeff_y", &PieceWisePol::coeff_y)
        .def_readwrite("coeff_z", &PieceWisePol::coeff_z)
        .def("clear", &PieceWisePol::clear)
        .def("getEndTime", &PieceWisePol::getEndTime)
        .def("eval", &PieceWisePol::eval)
        .def("velocity", &PieceWisePol::velocity)
        .def("acceleration", &PieceWisePol::acceleration)
        .def("print", &PieceWisePol::print)
        .def("__repr__", [](const PieceWisePol &a) { return "<py_mighty.PieceWisePol>"; });
    

    // ------------------------------
    // Bind lbfgs::lbfgs_parameter_t
    py::class_<lbfgs::lbfgs_parameter_t>(m, "lbfgs_parameter_t")
        .def(py::init<>())
        .def_readwrite("mem_size", &lbfgs::lbfgs_parameter_t::mem_size)
        .def_readwrite("min_step", &lbfgs::lbfgs_parameter_t::min_step)
        .def_readwrite("f_dec_coeff", &lbfgs::lbfgs_parameter_t::f_dec_coeff)
        .def_readwrite("cautious_factor", &lbfgs::lbfgs_parameter_t::cautious_factor)
        .def_readwrite("past", &lbfgs::lbfgs_parameter_t::past)
        .def_readwrite("max_linesearch", &lbfgs::lbfgs_parameter_t::max_linesearch)
        .def_readwrite("max_iterations", &lbfgs::lbfgs_parameter_t::max_iterations)
        .def_readwrite("g_epsilon", &lbfgs::lbfgs_parameter_t::g_epsilon)
        .def_readwrite("delta", &lbfgs::lbfgs_parameter_t::delta)
        .def("__repr__", [](const lbfgs::lbfgs_parameter_t &a) { return "<py_mighty.lbfgs_parameter_t>"; });


    // ------------------------------
    // Bind lbfgs::planner_params_t
    py::class_<lbfgs::planner_params_t>(m, "planner_params_t")
        .def(py::init<>())
        .def_readwrite("verbose", &lbfgs::planner_params_t::verbose)
        .def_readwrite("V_max", &lbfgs::planner_params_t::V_max)
        .def_readwrite("A_max", &lbfgs::planner_params_t::A_max)
        .def_readwrite("J_max", &lbfgs::planner_params_t::J_max)
        .def_readwrite("num_perturbation", &lbfgs::planner_params_t::num_perturbation)
        .def_readwrite("r_max", &lbfgs::planner_params_t::r_max)
        .def_readwrite("time_weight", &lbfgs::planner_params_t::time_weight)
        .def_readwrite("pos_anchor_weight", &lbfgs::planner_params_t::pos_anchor_weight)
        .def_readwrite("dyn_weight", &lbfgs::planner_params_t::dyn_weight)
        .def_readwrite("stat_weight", &lbfgs::planner_params_t::stat_weight)
        .def_readwrite("jerk_weight", &lbfgs::planner_params_t::jerk_weight)
        .def_readwrite("dyn_constr_bodyrate_weight", &lbfgs::planner_params_t::dyn_constr_bodyrate_weight)
        .def_readwrite("dyn_constr_tilt_weight", &lbfgs::planner_params_t::dyn_constr_tilt_weight)
        .def_readwrite("dyn_constr_thrust_weight", &lbfgs::planner_params_t::dyn_constr_thrust_weight)
        .def_readwrite("dyn_constr_vel_weight", &lbfgs::planner_params_t::dyn_constr_vel_weight)
        .def_readwrite("dyn_constr_acc_weight", &lbfgs::planner_params_t::dyn_constr_acc_weight)
        .def_readwrite("dyn_constr_jerk_weight", &lbfgs::planner_params_t::dyn_constr_jerk_weight)
        .def_readwrite("num_dyn_obst_samples", &lbfgs::planner_params_t::num_dyn_obst_samples)
        .def_readwrite("Co", &lbfgs::planner_params_t::Co)
        .def_readwrite("Cw", &lbfgs::planner_params_t::Cw)
        .def_readwrite("BIG", &lbfgs::planner_params_t::BIG)
        .def_readwrite("dc", &lbfgs::planner_params_t::dc)
        .def_readwrite("init_turn_bf", &lbfgs::planner_params_t::init_turn_bf)
        .def_readwrite("integral_resolution", &lbfgs::planner_params_t::integral_resolution)
        .def_readwrite("hinge_mu", &lbfgs::planner_params_t::hinge_mu)
        .def_readwrite("omega_max", &lbfgs::planner_params_t::omega_max)
        .def_readwrite("tilt_max_rad", &lbfgs::planner_params_t::tilt_max_rad)
        .def_readwrite("f_min", &lbfgs::planner_params_t::f_min)
        .def_readwrite("f_max", &lbfgs::planner_params_t::f_max)
        .def_readwrite("mass", &lbfgs::planner_params_t::mass)
        .def_readwrite("g", &lbfgs::planner_params_t::g)
        .def("__repr__", [](const lbfgs::planner_params_t &a) { return "<py_mighty.planner_params_t>"; });


    // ------------------------------
    // Bind lbfgs::SolverLBFGS
    py::class_<lbfgs::SolverLBFGS>(m, "SolverLBFGS")
        .def(py::init<>())
        // .def("pushWaypointsByStaticCorridor", lbfgs::SolverLBFGS::pushWaypointsByStaticCorridor)
        // .def("getGlobalPath", lbfgs::SolverLBFGS::getGlobalPath)
        // .def("buildInitialGuesses", lbfgs::SolverLBFGS::buildInitialGuesses)
        // .def("sample_systematic_perturbed_waypoints", lbfgs::SolverLBFGS::sample_systematic_perturbed_waypoints)
        // .def("evaluateObjective", lbfgs::SolverLBFGS::evaluateObjective)
        // .def("computeAnalyticalGrad", lbfgs::SolverLBFGS::computeAnalyticalGrad)
        // .def("evaluateObjectiveAndGradientFused", lbfgs::SolverLBFGS::evaluateObjectiveAndGradientFused)
        // .def("evaluateObjectiveAndGradient", lbfgs::SolverLBFGS::evaluateObjectiveAndGradient)
        // .def("progressCallback", lbfgs::SolverLBFGS::progressCallback)
        .def("optimize",
        [](const lbfgs::SolverLBFGS &self,
           const Eigen::VectorXd &z0,
           const lbfgs::lbfgs_parameter_t &param)
        {
            Eigen::VectorXd z_opt;
            double f_opt = 0.0;

            int status = self.optimize(z0, z_opt, f_opt, param);

            // Return everything Python cares about
            return py::make_tuple(status, z_opt, f_opt);
        },
        py::arg("z0"), py::arg("param"))
        // .def("reconstruct", lbfgs::SolverLBFGS::reconstruct)
        // .def("packDecisionVariables", lbfgs::SolverLBFGS::packDecisionVariables)
        // .def("setStaticConstraints", lbfgs::SolverLBFGS::setStaticConstraints)
        // .def("setStaticConstraintsForSafePath", lbfgs::SolverLBFGS::setStaticConstraintsForSafePath)
        // .def("sanityCheck", lbfgs::SolverLBFGS::sanityCheck)
        .def("reconstructPVATCPopt", &lbfgs::SolverLBFGS::reconstructPVATCPopt)
        .def("getGoalSetpoints", &lbfgs::SolverLBFGS::getGoalSetpoints)
        .def("getControlPoints", &lbfgs::SolverLBFGS::getControlPoints)
        .def("getInitialGuesses", &lbfgs::SolverLBFGS::getInitialGuesses)
        .def("getInitialGuessWaypoints", &lbfgs::SolverLBFGS::getInitialGuessWaypoints)
        .def("initializeSolver", &lbfgs::SolverLBFGS::initializeSolver)
        .def("prepareSolverForReplan",
        [](lbfgs::SolverLBFGS &self,
        double t0,
        const vec_Vec3f &global_wps,
        const std::vector<LinearConstraint3D> &safe_corridor,
        const std::vector<std::shared_ptr<dynTraj>> &obstacles,
        const state &initial_state,
        const state &goal_state,
        bool use_for_safe_path,
        bool use_multiple_initial_guesses)
        {
            double init_time = 0.0;
            self.prepareSolverForReplan(
                t0,
                global_wps,
                safe_corridor,
                obstacles,
                initial_state,
                goal_state,
                init_time,
                use_for_safe_path,
                use_multiple_initial_guesses);

            return init_time;
        },
        py::arg("t0"),
        py::arg("global_wps"),
        py::arg("safe_corridor"),
        py::arg("obstacles"),
        py::arg("initial_state"),
        py::arg("goal_state"),
        py::arg("use_for_safe_path") = false,
        py::arg("use_multiple_initial_guesses") = false)
        .def("__repr__", [](const lbfgs::SolverLBFGS &a) { return "<py_mighty.SolverLBFGS>"; });


    // ------------------------------
    // Bind dynTraj

    // dynTraj::Mode enum
    py::enum_<dynTraj::Mode>(m, "Mode")
        .value("Piecewise", dynTraj::Mode::Piecewise)
        .value("Quintic", dynTraj::Mode::Quintic)
        .value("Analytic", dynTraj::Mode::Analytic)
        .export_values();

    // dynTraj class
    py::class_<dynTraj, std::shared_ptr<dynTraj>>(m, "dynTraj")
        // default constructor
        .def(py::init<>())

        // quintic constructor
        .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&,
                      const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&,
                      double, double>(),
             py::arg("x0"), py::arg("v0"), py::arg("a0"),
             py::arg("xf"), py::arg("vf"), py::arg("af"),
             py::arg("poly_start_time"), py::arg("poly_end_time"))

        // member variables (public in struct, can expose directly)
        .def_readwrite("mode", &dynTraj::mode)
        .def_readwrite("pwp", &dynTraj::pwp)
        .def_readwrite("poly_start_time", &dynTraj::poly_start_time)
        .def_readwrite("poly_end_time", &dynTraj::poly_end_time)
        .def_readwrite("cx", &dynTraj::cx)
        .def_readwrite("cy", &dynTraj::cy)
        .def_readwrite("cz", &dynTraj::cz)
        .def_readwrite("traj_x", &dynTraj::traj_x)
        .def_readwrite("traj_y", &dynTraj::traj_y)
        .def_readwrite("traj_z", &dynTraj::traj_z)
        .def_readwrite("traj_vx", &dynTraj::traj_vx)
        .def_readwrite("traj_vy", &dynTraj::traj_vy)
        .def_readwrite("traj_vz", &dynTraj::traj_vz)
        .def_readwrite("t_var", &dynTraj::t_var)
        .def_readwrite("analytic_compiled", &dynTraj::analytic_compiled)
        .def_readwrite("ekf_cov_p", &dynTraj::ekf_cov_p)
        .def_readwrite("ekf_cov_q", &dynTraj::ekf_cov_q)
        .def_readwrite("poly_cov", &dynTraj::poly_cov)
        .def_readwrite("control_points", &dynTraj::control_points)
        .def_readwrite("bbox", &dynTraj::bbox)
        .def_readwrite("goal", &dynTraj::goal)
        .def_readwrite("is_agent", &dynTraj::is_agent)
        .def_readwrite("id", &dynTraj::id)
        .def_readwrite("time_received", &dynTraj::time_received)
        .def_readwrite("tracking_utility", &dynTraj::tracking_utility)
        .def_readwrite("communication_delay", &dynTraj::communication_delay)

        // member functions
        .def("setPiecewise", &dynTraj::setPiecewise)
        .def("compileAnalytic", &dynTraj::compileAnalytic)
        .def("eval", &dynTraj::eval)
        .def("velocity", &dynTraj::velocity)
        .def("accel", &dynTraj::accel)
        .def("evalQuinticPos", &dynTraj::evalQuinticPos)
        .def("evalAnalyticPos", &dynTraj::evalAnalyticPos)
        .def("velocityQuintic", &dynTraj::velocityQuintic)
        .def("velocityAnalytic", &dynTraj::velocityAnalytic)
        .def("accelQuintic", &dynTraj::accelQuintic)
        .def("accelAnalytic", &dynTraj::accelAnalytic)
        .def_static("modeName", &dynTraj::modeName)
        .def("print", &dynTraj::print)
        .def("__repr__", [](const dynTraj &a) { return "<py_mighty.dynTraj>"; });



    // ------------------------------
    // Bind state
    py::class_<state>(m, "state")
        .def(py::init<>())
        .def_readwrite("t", &state::t)
        .def_readwrite("pos", &state::pos)    // Eigen::Vector3d
        .def_readwrite("vel", &state::vel)    // Eigen::Vector3d
        .def_readwrite("accel", &state::accel)    // Eigen::Vector3d
        .def_readwrite("jerk", &state::jerk)    // Eigen::Vector3d
        .def_readwrite("yaw", &state::yaw)
        .def_readwrite("dyaw", &state::dyaw)
        .def_readwrite("use_tracking_yaw", &state::use_tracking_yaw)
        .def("setTimeStamp", &state::setTimeStamp)
        .def("setPos", py::overload_cast<const double, const double, const double>(&state::setPos))
        .def("setVel", py::overload_cast<const double, const double, const double>(&state::setVel))
        .def("setAccel", py::overload_cast<const double, const double, const double>(&state::setAccel))
        .def("setJerk", py::overload_cast<const double, const double, const double>(&state::setJerk))
        .def("setPos", py::overload_cast<const Eigen::Vector3d &>(&state::setPos))
        .def("setVel", py::overload_cast<const Eigen::Vector3d &>(&state::setVel))
        .def("setAccel", py::overload_cast<const Eigen::Vector3d &>(&state::setAccel))
        .def("setJerk", py::overload_cast<const Eigen::Vector3d &>(&state::setJerk))
        .def("setState", &state::setState)
        .def("setYaw", &state::setYaw)
        .def("setDYaw", &state::setDYaw)
        .def("setZero", &state::setZero)
        .def("printPos", &state::printPos)
        .def("print", &state::print)
        .def("printHorizontal", &state::printHorizontal)
        .def("__repr__", [](const state &a) { return "<py_mighty.state>"; });



    // ------------------------------
    // Bind parameters
    py::class_<parameters>(m, "parameters")
        .def(py::init<>())
        .def_readwrite("v_max", &parameters::v_max)
        .def_readwrite("a_max", &parameters::a_max)
        .def_readwrite("j_max", &parameters::j_max)
        .def_readwrite("num_perturbation_for_ig", &parameters::num_perturbation_for_ig)
        .def_readwrite("r_max_for_ig", &parameters::r_max_for_ig)
        .def_readwrite("time_weight", &parameters::time_weight)
        .def_readwrite("pos_anchor_weight", &parameters::pos_anchor_weight)
        .def_readwrite("dynamic_weight", &parameters::dynamic_weight)
        .def_readwrite("stat_weight", &parameters::stat_weight)
        .def_readwrite("jerk_weight", &parameters::jerk_weight)
        .def_readwrite("dyn_constr_vel_weight", &parameters::dyn_constr_vel_weight)
        .def_readwrite("dyn_constr_acc_weight", &parameters::dyn_constr_acc_weight)
        .def_readwrite("dyn_constr_jerk_weight", &parameters::dyn_constr_jerk_weight)
        .def_readwrite("dyn_constr_bodyrate_weight", &parameters::dyn_constr_bodyrate_weight)
        .def_readwrite("dyn_constr_tilt_weight", &parameters::dyn_constr_tilt_weight)
        .def_readwrite("dyn_constr_thrust_weight", &parameters::dyn_constr_thrust_weight)
        .def_readwrite("num_dyn_obst_samples", &parameters::num_dyn_obst_samples)
        .def_readwrite("planner_Co", &parameters::planner_Co)
        .def_readwrite("planner_Cw", &parameters::planner_Cw)
        .def_readwrite("dc", &parameters::dc)
        .def_readwrite("init_turn_bf", &parameters::init_turn_bf)
        .def_readwrite("f_dec_coeff", &parameters::f_dec_coeff)
        .def_readwrite("cautious_factor", &parameters::cautious_factor)
        .def_readwrite("past", &parameters::past)
        .def_readwrite("max_linesearch", &parameters::max_linesearch)
        .def_readwrite("max_iterations", &parameters::max_iterations)
        .def_readwrite("g_epsilon", &parameters::g_epsilon)
        .def_readwrite("delta", &parameters::delta)
        .def_readwrite("vehicle_type", &parameters::vehicle_type)
        .def_readwrite("provide_goal_in_global_frame", &parameters::provide_goal_in_global_frame)
        .def_readwrite("use_hardware", &parameters::use_hardware)
        .def_readwrite("flight_mode", &parameters::flight_mode)
        .def_readwrite("visual_level", &parameters::visual_level)
        .def_readwrite("global_planner", &parameters::global_planner)
        .def_readwrite("global_planner_verbose", &parameters::global_planner_verbose)
        .def_readwrite("global_planner_huristic_weight", &parameters::global_planner_huristic_weight)
        .def_readwrite("factor_dgp", &parameters::factor_dgp)
        .def_readwrite("inflation_dgp", &parameters::inflation_dgp)
        .def_readwrite("x_min", &parameters::x_min)
        .def_readwrite("x_max", &parameters::x_max)
        .def_readwrite("y_min", &parameters::y_min)
        .def_readwrite("y_max", &parameters::y_max)
        .def_readwrite("z_min", &parameters::z_min)
        .def_readwrite("z_max", &parameters::z_max)
        .def_readwrite("drone_radius", &parameters::drone_radius)
        .def_readwrite("dgp_timeout_duration_ms", &parameters::dgp_timeout_duration_ms)
        .def_readwrite("use_free_start", &parameters::use_free_start)
        .def_readwrite("free_start_factor", &parameters::free_start_factor)
        .def_readwrite("use_free_goal", &parameters::use_free_goal)
        .def_readwrite("free_goal_factor", &parameters::free_goal_factor)
        .def_readwrite("los_cells", &parameters::los_cells)
        .def_readwrite("min_len", &parameters::min_len)
        .def_readwrite("use_state_update", &parameters::use_state_update)
        .def_readwrite("use_random_color_for_global_path", &parameters::use_random_color_for_global_path)
        .def_readwrite("use_path_push_for_visualization", &parameters::use_path_push_for_visualization)
        .def_readwrite("local_box_size", &parameters::local_box_size)
        .def_readwrite("min_dist_from_agent_to_traj", &parameters::min_dist_from_agent_to_traj)
        .def_readwrite("use_shrinked_box", &parameters::use_shrinked_box)
        .def_readwrite("shrinked_box_size", &parameters::shrinked_box_size)
        .def_readwrite("map_buffer", &parameters::map_buffer)
        .def_readwrite("center_shift_factor", &parameters::center_shift_factor)
        .def_readwrite("min_wdx", &parameters::min_wdx)
        .def_readwrite("min_wdy", &parameters::min_wdy)
        .def_readwrite("min_wdz", &parameters::min_wdz)
        .def_readwrite("use_comm_delay_inflation", &parameters::use_comm_delay_inflation)
        .def_readwrite("comm_delay_inflation_alpha", &parameters::comm_delay_inflation_alpha)
        .def_readwrite("comm_delay_inflation_max", &parameters::comm_delay_inflation_max)
        .def_readwrite("comm_delay_filter_alpha", &parameters::comm_delay_filter_alpha)
        .def_readwrite("depth_camera_depth_max", &parameters::depth_camera_depth_max)
        .def_readwrite("fov_visual_depth", &parameters::fov_visual_depth)
        .def_readwrite("fov_visual_x_deg", &parameters::fov_visual_x_deg)
        .def_readwrite("fov_visual_y_deg", &parameters::fov_visual_y_deg)
        .def_readwrite("num_N", &parameters::num_N)
        .def_readwrite("max_dist_vertexes", &parameters::max_dist_vertexes)
        .def_readwrite("w_unknown", &parameters::w_unknown)
        .def_readwrite("w_align", &parameters::w_align)
        .def_readwrite("decay_len_cells", &parameters::decay_len_cells)
        .def_readwrite("w_side", &parameters::w_side)
        .def_readwrite("use_multiple_initial_guesses", &parameters::use_multiple_initial_guesses)
        .def_readwrite("horizon", &parameters::horizon)
        .def_readwrite("closed_form_traj_verbose", &parameters::closed_form_traj_verbose)
        .def_readwrite("drone_bbox", &parameters::drone_bbox)
        .def_readwrite("integral_resolution", &parameters::integral_resolution)
        .def_readwrite("hinge_mu", &parameters::hinge_mu)
        .def_readwrite("omega_max", &parameters::omega_max)
        .def_readwrite("tilt_max_rad", &parameters::tilt_max_rad)
        .def_readwrite("f_min", &parameters::f_min)
        .def_readwrite("f_max", &parameters::f_max)
        .def_readwrite("mass", &parameters::mass)
        .def_readwrite("g", &parameters::g)
        .def_readwrite("fopt_threshold", &parameters::fopt_threshold)
        .def_readwrite("traj_lifetime", &parameters::traj_lifetime)
        .def_readwrite("num_replanning_before_adapt", &parameters::num_replanning_before_adapt)
        .def_readwrite("alpha_filter_dyaw", &parameters::alpha_filter_dyaw)
        .def_readwrite("w_max", &parameters::w_max)
        .def_readwrite("yaw_spinning_threshold", &parameters::yaw_spinning_threshold)
        .def_readwrite("yaw_spinning_dyaw", &parameters::yaw_spinning_dyaw)
        .def_readwrite("force_goal_z", &parameters::force_goal_z)
        .def_readwrite("default_goal_z", &parameters::default_goal_z)
        .def_readwrite("debug_verbose", &parameters::debug_verbose)
        .def_readwrite("initial_wdx", &parameters::initial_wdx)
        .def_readwrite("initial_wdy", &parameters::initial_wdy)
        .def_readwrite("initial_wdz", &parameters::initial_wdz)
        .def_readwrite("res", &parameters::res)
        .def_readwrite("goal_radius", &parameters::goal_radius)
        .def_readwrite("goal_seen_radius", &parameters::goal_seen_radius)
        .def_readwrite("alpha_k_value_filtering", &parameters::alpha_k_value_filtering)
        .def_readwrite("k_value_factor", &parameters::k_value_factor)
        .def_readwrite("default_k_value", &parameters::default_k_value)
        .def("__repr__", [](const parameters &a) { return "<py_mighty.parameters>"; });



    // ------------------------------
    // Bind MIGHTY
    py::class_<MIGHTY>(m, "MIGHTY")
        .def(py::init<parameters>(), py::arg("par"))
        .def("startAdaptKValue", &MIGHTY::startAdaptKValue)
        .def("needReplan", &MIGHTY::needReplan, py::arg("localState"), py::arg("local_G_term"), py::arg("last_plan_state"))
        .def("findAandAtime", &MIGHTY::findAandAtime, py::arg("A"), py::arg("A_time"), py::arg("current_time"), py::arg("last_replaning_computation_time"))
        .def("computeMapSize", &MIGHTY::computeMapSize)
        .def("checkPointWithinMap", &MIGHTY::checkPointWithinMap)
        .def("getStaticPushPoints", &MIGHTY::getStaticPushPoints)
        .def("getLocalGlobalPath", &MIGHTY::getLocalGlobalPath)
        .def("getFreeGlobalPath", &MIGHTY::getFreeGlobalPath)
        .def("resetData", &MIGHTY::resetData)
        .def("retrieveData", &MIGHTY::retrieveData)
        .def("retrievePolytopes", &MIGHTY::retrievePolytopes)
        .def("retrieveGoalSetpoints", &MIGHTY::retrieveGoalSetpoints)
        .def("retrieveListSubOptGoalSetpoints", &MIGHTY::retrieveListSubOptGoalSetpoints)
        .def("retrieveCPs", &MIGHTY::retrieveCPs)
        .def("replan", &MIGHTY::replan, py::arg("last_replaning_computation_time"), py::arg("current_time"))
        .def("generateGlobalPath", &MIGHTY::generateGlobalPath)
        .def("planLocalTrajectory", &MIGHTY::planLocalTrajectory)
        .def("generateLocalTrajectory", &MIGHTY::generateLocalTrajectory)
        .def("appendToPlan", &MIGHTY::appendToPlan)
        .def("computeG", &MIGHTY::computeG, py::arg("A"), py::arg("G_term"), py::arg("horizon"))
        .def("checkIfPointFree", &MIGHTY::checkIfPointFree)
        .def("checkIfPointOccupied", &MIGHTY::checkIfPointOccupied)
        .def("getG", [](MIGHTY &self) {
            state g;
            self.getG(g);
            return g;
        })
        .def("setG", &MIGHTY::setG)
        .def("getA", [](MIGHTY &self) {
            state a;
            self.getA(a);
            return a;
        })
        .def("setA", &MIGHTY::setA)
        .def("getState", [](MIGHTY &self) {
            state s;
            self.getState(s);
            return s;
        })
        .def("getE", [](MIGHTY &self) {
            state e;
            self.getE(e);
            return e;
        })
        .def("getA_time", [](MIGHTY &self) {
            double A_time;
            self.getA_time(A_time);  // call original C++ function
            return A_time;           // return it to Python
        })
        .def("setA_time", &MIGHTY::setA_time)
        .def("getGterm", [](MIGHTY &self) {
            state gterm;
            self.getGterm(gterm);
            return gterm;
        })
        .def("setGterm", &MIGHTY::setGterm)
        .def("getGlobalPath", [](MIGHTY &self) {
            vec_Vecf<3> path;
            self.getGlobalPath(path);
            return path;
        })
        .def("getOriginalGlobalPath", [](MIGHTY &self) {
            vec_Vecf<3> path;
            self.getOriginalGlobalPath(path);
            return path;
        })
        .def("getSafeCorridor", &MIGHTY::getSafeCorridor)
        .def("getLastPlanState", [](MIGHTY &self){
            state s;
            self.getLastPlanState(s);
            return s;
        })
        .def("getTrajs", [](MIGHTY &self) {
            std::vector<std::shared_ptr<dynTraj>> out;
            self.getTrajs(out);  
            return out;         
        })
        .def("cleanUpOldTrajs", &MIGHTY::cleanUpOldTrajs)
        .def("addTraj", &MIGHTY::addTraj)
        .def("updateState", &MIGHTY::updateState)
        .def("getNextGoal", &MIGHTY::getNextGoal)
        .def("getDesiredYaw", [](MIGHTY &self, state next_goal) {
            self.getDesiredYaw(next_goal); // modifies next_goal internally
            return next_goal;              // return the updated state to Python
        }, py::arg("next_goal"))
        .def("yaw", [](MIGHTY &self, double diff, state next_goal) {
            self.yaw(diff, next_goal);   // modifies next_goal internally
            return next_goal;            // return the updated state
        }, py::arg("diff"), py::arg("next_goal"))
        .def("setTerminalGoal", &MIGHTY::setTerminalGoal)
        .def("changeDroneStatus", &MIGHTY::changeDroneStatus)
        .def("checkReadyToReplan", &MIGHTY::checkReadyToReplan)
        .def("updateMap", &MIGHTY::updateMap)
        .def("setInitialPose", &MIGHTY::setInitialPose)
        .def("applyInitiPoseTransform", &MIGHTY::applyInitiPoseTransform)
        .def("applyInitiPoseInverseTransform", &MIGHTY::applyInitiPoseInverseTransform)
        .def("goalReachedCheck", &MIGHTY::goalReachedCheck)
        .def("__repr__", [](const MIGHTY &a) { return "<py_mighty.MIGHTY>"; });
}
