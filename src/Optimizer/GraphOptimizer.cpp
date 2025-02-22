// Codes for pose graph optimization in LiDAR SLAM
// By Yue Pan et al.

#include "Optimizer/GraphOptimizer.h"
#include "Misc/Utility.hpp"

//eigen
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>

//select from the following three libs (you only need one to realize the non-linear optimization used in this project)
//ceres is recommended

#if G2O_ON
//g2o
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/slam3d/types_slam3d.h"
#endif

#if CERES_ON
//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "ceres/covariance.h"
#endif

#if GTSAM_ON
//gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>
#endif

#include <glog/logging.h>
#include <utility>


void GlobalOptimize::set_equal_weight(bool on_or_not)
{
    _param.use_equal_weight = on_or_not;
}

void GlobalOptimize::set_robust_function(bool robust_status)
{
    _param.robustify = robust_status;
}

void GlobalOptimize::set_diagonal_information_matrix(bool diagonal_information_matrix_on_or_not)
{
    _param.use_diagonal_information_matrix = diagonal_information_matrix_on_or_not;
}

void GlobalOptimize::set_max_iter_num(int max_iter)
{
    _param.num_iterations = max_iter;
}

void GlobalOptimize::set_rotation_limit(bool only_limit_translation_or_not)
{
    _param.only_limit_translation = only_limit_translation_or_not;
}

void GlobalOptimize::set_covariance_updating_ratio(float first_time_ratio, float life_long_ratio)
{
    _param.life_long_updating_ratio = life_long_ratio;
    _param.first_time_updating_ratio = first_time_ratio;
}

void GlobalOptimize::set_wrong_edge_check_threshold(float tran_thre, float rot_thre)
{
    _param.wrong_edge_translation_thre = tran_thre;
    _param.wrong_edge_rotation_thre = rot_thre;
}

void GlobalOptimize::set_free_node(bool on_or_not)
{
    _param.free_all_nodes = on_or_not;
}

void GlobalOptimize::set_problem_size(bool is_small_size_problem)
{
    _param.is_small_size_problem = is_small_size_problem;
}

bool GlobalOptimize::optimize_pose_graph_g2o(CloudBlockPtrs &all_blocks, constraints &all_cons, bool update_edge_or_not)
{
    //1.Create the _optimizer, determine the optimization method such as LM
    //2.Assign value to the nodes(initial value to unknown parameters), fixed the datum
    //3.Assign value to the edges(observation), assign the weight matrix and the robust kernel function such as Huber
    //4.Solving

    LOG(INFO) << "Start to optimize the pose graph using g2o";

    init();

    set_pgo_options_g2o();
    set_pgo_problem_g2o(all_blocks, all_cons);
    if (!solve_pgo_problem_g2o())
        return false;

    // assign the optimized value
    assign_pose_optimized_g2o(all_blocks);

    if (!update_edge_or_not)
    {
        LOG(INFO) << "Pose graph optimization done successfully";
        return true;
    }

    if (update_optimized_edges(all_cons))
    {
        LOG(INFO) << "Pose graph optimization done successfully";
        return true;
    }
    else //too much wrong edges, pgo may fail
        return false;
}

//G2O has some problem, use ceres instead
bool GlobalOptimize::set_pgo_options_g2o()
{
#if G2O_ON
    //g2o::SparseOptimizer *_optimizer;

	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
	typedef g2o::LinearSolver<BlockSolverType::PoseMatrixType> LinearSolverType;

	LinearSolverType *linearSolver = 0;
	//std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType());

	if (param_.linear_solver == "dense_schur")
		linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();
	else if (param_.linear_solver == "sparse_schur")
	{
		linearSolver = new g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>();
		dynamic_cast<g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType> *>(linearSolver)->setBlockOrdering(true);
	}
	else //default: dense schur
		linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();

	BlockSolverType *blockSolver;
	blockSolver = new BlockSolverType(linearSolver);
	//std::unique_ptr<BlockSolverType> blockSolver(new BlockSolverType(std::move(linearSolver))); //new version g2o

	g2o::OptimizationAlgorithmWithHessian *solver;
	//g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
	//g2o::OptimizationAlgorithmWithHessian *solver = new g2o::OptimizationAlgorithmWithHessian(std::move(blockSolver));

	if (param_.trust_region_strategy == "levenberg_marquardt")
		solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
	else if (param_.trust_region_strategy == "dogleg")
		solver = new g2o::OptimizationAlgorithmDogleg(blockSolver);
	else
		LOG(ERROR) << "Please check your trust_region_strategy, using default setting (L-M)";

	//g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

	_g2oOptimizer->setAlgorithm(solver);
	_g2oOptimizer->setVerbose(param_.verbose);

	LOG(INFO) << "G2O options setup done";

	return true;
#endif
    return false;
}

bool GlobalOptimize::set_pgo_problem_g2o(CloudBlockPtrs &all_blocks, constraints &all_cons)
{
#if G2O_ON
    //std::vector<g2o::VertexSE3Expmap*> pose_vertices;
	//std::vector<g2o::EdgeSE3Expmap*> edges_smo;
	//std::vector<g2o::EdgeSE3Expmap*> edges_reg;

	float quat_tran_ratio = 1.0;
	int min_index_for_loop = INT_MAX;
	bool with_reg_con = false;

	for (int j = 0; j < all_cons.size(); j++)
	{
		if (all_cons[j].con_type == REGISTRATION)
		{
			min_index_for_loop = min_(min_index_for_loop, all_cons[j].block1->_idInStrip);
			with_reg_con = true;
		}
	}

	//Set Nodes
	for (int i = 0; i < all_blocks.size(); i++)
	{
		g2o::VertexSE3Expmap *pose_v = new g2o::VertexSE3Expmap();
		pose_v->setId(all_blocks[i]->_idInStrip);

		if (all_blocks[i]->_poseFixed || (with_reg_con && i <= min_index_for_loop)) //set fixed nodes
			pose_v->setFixed(true);

		Eigen::Matrix4d node_pose = all_blocks[i]->_poseInit;
		pose_v->setEstimate(g2o::SE3Quat(node_pose.topLeftCorner<3, 3>(), node_pose.topRightCorner<3, 1>())); //Initial guess
		_g2oOptimizer->addVertex(pose_v);																	  //Add nodes;
																											  //pose_vertices.push_back(pose_v);
	}

	//Set Edges
	for (int j = 0; j < all_cons.size(); j++)
	{
		if (all_cons[j].con_type != NONE && all_cons[j].con_type != HISTORY)
		{
			// if (all_cons[j].con_type == ADJACENT)
			// 	all_cons[j].information_matrix = 4.0 * all_cons[0].information_matrix; //the frist one should be registration edge //TODO fix

			g2o::EdgeSE3Expmap *pose_e = new g2o::EdgeSE3Expmap();
			pose_e->setId(j);

			//very important!!!
			//g2o's edge is from vertex 1 to vertex 0
			//The transformation in constraint is from block 2 (v1) to block 1 (v0) (Trans1_2)
			//reference: https://github.com/koide3/hdl_graph_slam/blob/master/apps/hdl_graph_slam_nodelet.cpp
			int v0 = all_cons[j].block1->_idInStrip;
			int v1 = all_cons[j].block2->_idInStrip;
			pose_e->setVertex(0, _g2oOptimizer->vertices()[v0]); // block 1 -> v0 -> vertex 0
			pose_e->setVertex(1, _g2oOptimizer->vertices()[v1]); // block 2 -> v1 -> vertex 1

			Eigen::Matrix4d edge_pose = all_cons[j].Trans1_2;

			pose_e->setMeasurement(g2o::SE3Quat(edge_pose.topLeftCorner<3, 3>(), edge_pose.topRightCorner<3, 1>())); //Set the transformation measurement
																													 //float weight = determine_weight(all_cons[j].block1->data_type, all_cons[j].block2->data_type, all_cons[j].con_type);

			//read the paper about quaternion's covariance calculation
			Matrix6d info_mat;
			info_mat.setIdentity();
			if (param_.use_equal_weight)
			{
				info_mat(3, 3) = param_.quat_tran_ratio;
				info_mat(4, 4) = param_.quat_tran_ratio;
				info_mat(5, 5) = param_.quat_tran_ratio;
			}
			else //directly use information matrix
			{
				if (param_.use_diagonal_information_matrix)
				{
					for (int j = 0; j < 6; j++)
						info_mat(j, j) = all_cons[j].information_matrix(j, j);
				}
				else
					info_mat = all_cons[j].information_matrix;
			}

			pose_e->setInformation(info_mat);

			if (param_.robustify)
			{
				g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
				rk->setDelta(param_.robust_delta);
				pose_e->setRobustKernel(rk); //Set the robust kernel function
			}

			_g2oOptimizer->addEdge(pose_e);
			//if (all_cons[j].con_type == 1)      edges_smo.push_back(pose_e);  //Smooth Edges
			//else if (all_cons[j].con_type == 2) edges_reg.push_back(pose_e);  //Registration Edges

			// LOG(INFO) << "set con " << all_cons[j]._uniqueId << " : " << all_cons[j].block1->_uniqueId << " - " << all_cons[j].block2->_uniqueId << std::endl
			// 		  << all_cons[j].Trans1_2;
		}
	}

	LOG(INFO) << "G2O problem setup done";

	return true;
#endif
    return false;
}

bool GlobalOptimize::solve_pgo_problem_g2o()
{
#if G2O_ON
    //Optimize
	LOG(INFO) << "=============================================  Optimization Start  =============================================";
	_g2oOptimizer->initializeOptimization();
	_g2oOptimizer->optimize(param_.num_iterations); // Number of Iterations
	LOG(INFO) << "==============================================  Optimization END  ==============================================";
	return true;
#endif
    return false;
}

bool GlobalOptimize::assign_pose_optimized_g2o(CloudBlockPtrs &all_blocks)
{
#if G2O_ON
    float warning_tran_thre = 0.5;

	for (int i = 0; i < all_blocks.size(); i++)
	{
		g2o::VertexSE3Expmap *v_o = dynamic_cast<g2o::VertexSE3Expmap *>(_g2oOptimizer->vertices()[all_blocks[i]->_idInStrip]);
		Eigen::Isometry3d pose_o = v_o->estimate();
		all_blocks[i]->_poseOptimized = pose_o.matrix();

		Eigen::Matrix4d diff_mat = all_blocks[i]->_poseInit.inverse() * all_blocks[i]->_poseOptimized;

		if (diff_mat.block<3, 1>(0, 3).norm() > warning_tran_thre)
			LOG(WARNING) << "transformation of node [" << all_blocks[i]->_idInStrip << "], _uniqueId [" << all_blocks[i]->_uniqueId << "]:\n"
						 << diff_mat;
		else
			LOG(INFO) << "transformation of node [" << all_blocks[i]->_idInStrip << "], _uniqueId [" << all_blocks[i]->_uniqueId << "]:\n"
					  << diff_mat;
	}
	return true;
#endif
    return false;
}

//TODO:
bool GlobalOptimize::robust_verify_chi2_g2o()
{
    //To guarentee the robustness, chi2 test
    /*for (int i = 0; i < iterations.size(); ++i)
        {
        double last_active_chi2 = 0.0, epsilon = 0.000001;
        for (int j = 0; j < iterations[i]; ++j)
        {
        _optimizer.initializeOptimization(0);
        _optimizer.optimize(1);
        _optimizer.computeActiveErrors();

        double active_chi2 = _optimizer.activeRobustChi2();
        LOG(INFO) << j << "th active_chi2 is " << active_chi2;
        if (std::isnan(active_chi2))
        {
        LOG(WARNING) << "Essential Graph Optimization " << j << "th iteration chi2 is NAN!";
        bSuccess = false;
        bBreak = true;
        break;
        }
        if (std::isinf(active_chi2))
        {
        LOG(WARNING) << "Essential Graph Optimization " << j << "th iteration chi2 is INF!";
        bSuccess = false;
        bBreak = true;
        break;
        }

        double improvment = last_active_chi2 - active_chi2;
        if (j > 0 && improvment < epsilon) // negative/ not enough improvement
        {
        LOG(WARNING) << "Essential Graph Optimization negative/ not enough improvement(" << improvment << " < " << epsilon << ")";
        bBreak = true;
        break;
        }
        else if (i == 0 && active_chi2 < epsilon) // error is under epsilon
        {
        LOG(INFO) << "Essential Graph Optimization error(" << active_chi2 << ") is under epsilon(" << epsilon << ")";
        bBreak = true;
        break;
        }

        last_active_chi2 = active_chi2;
        }*/
    return true;
}

//TODO: add smooth edge (for some indoor or huge loop dataset)

bool GlobalOptimize::optimize_pose_graph_ceres(CloudBlockPtrs &all_blocks, constraints &all_cons,
                                               double moving_threshold_tran, double moving_threshold_rot, bool update_edge_or_not)
{
    LOG(INFO) << "Start to optimize the pose graph using ceres";

    //clear the problem, options, etc.
    init();

    set_pgo_options_ceres();

    if (!set_pgo_problem_ceres(all_blocks, all_cons, moving_threshold_tran, moving_threshold_rot))
        return false;
    if (!solve_pgo_problem_ceres())
        return false;

    // assign the optimized value
    assign_pose_optimized_ceres(all_blocks);

    if (!update_edge_or_not)
    {
        LOG(INFO) << "Pose graph optimization done successfully";
        return true;
    }

    if (update_optimized_edges(all_cons))
    {
        update_edge_covariance_ceres(all_cons);
        LOG(INFO) << "Pose graph optimization done successfully";
        return true;
    }
    else //too much wrong edges, pgo may fail
        return false;
}

bool GlobalOptimize::set_pgo_options_ceres()
{
#if CERES_ON
    //reference: http://ceres-solver.org/solving_faqs.html

    //When using the TRUST_REGION minimizer, the choice of linear solver is an important decision. It affects solution quality and runtime. Here is a simple way to reason about it.
    //1. For small (a few hundred parameters) or dense problems use DENSE_QR.
    //2. For general sparse problems (i.e., the Jacobian matrix has a substantial number of zeros) use SPARSE_NORMAL_CHOLESKY. This requires that you have SuiteSparse or CXSparse installed.
    //3. For bundle adjustment problems with up to a hundred or so cameras, use DENSE_SCHUR.
    //4. For larger bundle adjustment problems with sparse Schur Complement/Reduced camera matrices use SPARSE_SCHUR. This requires that you build Ceres with support for SuiteSparse, CXSparse or Eigen’s sparse linear algebra libraries.
    //5. If you do not have access to these libraries for whatever reason, ITERATIVE_SCHUR with SCHUR_JACOBI is an excellent alternative.
    //6. For large bundle adjustment problems (a few thousand cameras or more) use the ITERATIVE_SCHUR solver. There are a number of preconditioner choices here. SCHUR_JACOBI offers an excellent balance of speed and accuracy. This is also the recommended option if you are solving medium sized problems for which DENSE_SCHUR is too slow but SuiteSparse is not available.

    //
    // std::string trust_region_strategy = "levenberg_marquardt";
    // // std::string linear_solver = "dense_schur";
    //  std::string linear_solver = "sparse_schur";
    // std::string sparse_linear_algebra_library = "suite_sparse";
    // std::string dense_linear_algebra_library = "eigen";
    // std::string robust_kernel_strategy = "huber";

    _ceresOptions->max_num_iterations = _param.num_iterations;
    _ceresOptions->num_threads = _param.num_threads;
    _ceresOptions->minimizer_progress_to_stdout = true;

    //_ceresOptions->gradient_tolerance=1e-16;
    _ceresOptions->function_tolerance = 1e-16;

    //robust kernel function
    _ceresRobustKernelFunction = _param.robustify ? new ceres::HuberLoss(_param.robust_delta) : NULL;

    if (_param.is_small_size_problem)
    {
        //_ceresOptions->use_explicit_schur_complement = true;
        _param.linear_solver = "dense_schur";
    }
    else
    {
        _param.linear_solver = "sparse_schur";
    }

    //example : "dense_schur", "sparse_schur"
    CHECK(ceres::StringToLinearSolverType(_param.linear_solver,
                                          &_ceresOptions->linear_solver_type));

    //example : "suite_sparse"
    CHECK(ceres::StringToSparseLinearAlgebraLibraryType(_param.sparse_linear_algebra_library,
                                                        &_ceresOptions->sparse_linear_algebra_library_type));

    //example : "eigen"
    CHECK(ceres::StringToDenseLinearAlgebraLibraryType(_param.dense_linear_algebra_library,
                                                       &_ceresOptions->dense_linear_algebra_library_type));

    //example : "levenberg_marquardt"
    CHECK(ceres::StringToTrustRegionStrategyType(_param.trust_region_strategy,
                                                 &_ceresOptions->trust_region_strategy_type));

    return true;
#endif
    return false;
}

bool GlobalOptimize::set_pgo_problem_ceres(CloudBlockPtrs &all_blocks, constraints &all_cons,
                                           double stable_threshold_tran, double stable_threshold_rot)
{
#if CERES_ON

    //For reference (fixed) frame (this should be too small, such as 1e-14, which would cause some infeasible bound problem since it would be regarded as 0)
    double fixed_threshold = 1e-10;
    //float quat_tran_ratio = 1000.0;

    _numCeresObservations = 0;
    for (int i = 0; i < all_cons.size(); i++)
    {
        if (all_cons[i].con_type != NONE && all_cons[i].con_type != HISTORY)
            _numCeresObservations++;
    }

    _numCeresParameters = 7 * all_blocks.size(); //x,y,z,i,j,k,w
    _ceresParameters = new double[_numCeresParameters];
    //ceres_observations_ = new double[6 * _numCeresObservations];

    int edge_count = _numCeresObservations;
    int node_count = all_blocks.size();
    int fixed_node_count = 0;
    for (int i = 0; i < all_blocks.size(); i++)
    {
        //fix nodes or not
        if (all_blocks[i]->_poseFixed)
            fixed_node_count++;
    }

    LOG(INFO) << "PGO setup: [" << node_count << "] nodes, [" << fixed_node_count << "] fixed nodes, [" << edge_count << "] edges";

    if (node_count - fixed_node_count > edge_count)
    {
        LOG(ERROR) << "Edge number is not enough for the PGO";
        return false;
    }

    //Set initial guess of parameters for estimating (original pose)
    for (int i = 0; i < all_blocks.size(); i++)
    {
        pose_qua_t node_pose;
        node_pose.SetPose(all_blocks[i]->_poseInit);
        _ceresParameters[7 * i + 0] = node_pose.trans(0); //x
        _ceresParameters[7 * i + 1] = node_pose.trans(1); //y
        _ceresParameters[7 * i + 2] = node_pose.trans(2); //z
        _ceresParameters[7 * i + 3] = node_pose.quat.x(); //i
        _ceresParameters[7 * i + 4] = node_pose.quat.y(); //j
        _ceresParameters[7 * i + 5] = node_pose.quat.z(); //k
        _ceresParameters[7 * i + 6] = node_pose.quat.w(); //w
    }

    int min_index_for_loop = INT_MAX;
    bool with_reg_con = false;
    int stable_index = 0;

    //LOG(INFO) << "Begin to calculate the loss";
    for (int i = 0; i < all_cons.size(); i++)
    {
        if (all_cons[i].con_type != NONE && all_cons[i].con_type != HISTORY)
        {
            pose_qua_t edge_tran;
            edge_tran.SetPose(all_cons[i].Trans1_2); //set edge //test pass (the order is correct)

            //set information_matrix
            Matrix6d sqrt_info_mat;
            sqrt_info_mat.setIdentity();
            if (_param.use_equal_weight)
            {
                sqrt_info_mat(3, 3) = _param.quat_tran_ratio;
                sqrt_info_mat(4, 4) = _param.quat_tran_ratio;
                sqrt_info_mat(5, 5) = _param.quat_tran_ratio;
            }
            else //directly use information matrix
            {
                if (_param.use_diagonal_information_matrix) //only use the diagonal elements of the information matrix
                {
                    for (int j = 0; j < 6; j++)
                        sqrt_info_mat(j, j) = std::sqrt(all_cons[i].information_matrix(j, j));
                    // LOG(INFO) << "Sqaure root of information matrix for edge [" << i << "]:\n"
                    // 		  << sqrt_info_mat;
                }
                else
                {
                    //F1
                    sqrt_info_mat = all_cons[i].information_matrix.sqrt();
                    //F2
                    //sqrt_info_mat = all_cons[i].information_matrix.llt().matrixL(); //Cholesky (LLT) decomposition: A * A' = B ==> A = LLT(B)
                    //LOG(INFO) << info_mat;
                }
            }

            ceres::CostFunction *cost_function =
                    PoseGraph3dErrorTermQUAT::Create(edge_tran, sqrt_info_mat); //actually, info_mat is the squared root of diagonal information matrix

            _ceresProblem->AddResidualBlock(cost_function,
                                            _ceresRobustKernelFunction, // NULL /* squared loss */, /* new CauchyLoss(0.5) */
                                            mutable_pose_tran(all_cons[i].block1->_idInStrip),
                                            mutable_pose_quat(all_cons[i].block1->_idInStrip),
                                            mutable_pose_tran(all_cons[i].block2->_idInStrip),
                                            mutable_pose_quat(all_cons[i].block2->_idInStrip));
            if (all_cons[i].con_type == REGISTRATION)
            {
                min_index_for_loop = min_(min_index_for_loop, all_cons[i].block1->_idInStrip);
                with_reg_con = true;
                stable_index = min_index_for_loop;
            }
        }
    }
    //TODO: set fix for those pose before the earliest loop node
    std::cout<<"min_index_for_loop:"<<min_index_for_loop<<std::endl;

    //set node limit
    for (int i = 0; i < all_blocks.size(); i++)
    {
        _ceresProblem->SetParameterization(mutable_pose_quat(all_blocks[i]->_idInStrip), new ceres::EigenQuaternionParameterization);

        if (with_reg_con && i <= min_index_for_loop)
        {
//			fix_node_ceres(i, fixed_threshold, fixed_threshold, param_.only_limit_translation);
            _ceresProblem->SetParameterBlockConstant(mutable_pose_quat(all_blocks[i]->_idInStrip));
            _ceresProblem->SetParameterBlockConstant(mutable_pose_tran(all_blocks[i]->_idInStrip));
            LOG(INFO) << "Node [" << i << "] Fixed: (" << fixed_threshold << "," << fixed_threshold << ")";
        }
        else
        {
            //fix nodes or not
            if (all_blocks[i]->_poseFixed)
            {
                _ceresProblem->SetParameterBlockConstant(mutable_pose_quat(all_blocks[i]->_idInStrip));
                _ceresProblem->SetParameterBlockConstant(mutable_pose_tran(all_blocks[i]->_idInStrip));

                //fix_node_ceres(i, fixed_threshold, fixed_threshold, param_.only_limit_translation);
                LOG(INFO) << "Node [" << i << "] Fixed: (" << fixed_threshold << "," << fixed_threshold << ")";
            }
            else if (all_blocks[i]->_poseStable)
            {
                if (!_param.free_all_nodes)
                {
                    fix_node_ceres(i, stable_threshold_tran, stable_threshold_rot, _param.only_limit_translation);
                    stable_index = i;
                    LOG(INFO) << "Node [" << i << "] Stable: (" << stable_threshold_tran << "," << stable_threshold_rot << ")";
                }
            }
            else
            {
                if (!_param.free_all_nodes)
                {
                    double moving_threshold_tran = 1.0 * (i - stable_index) * stable_threshold_tran;
                    double moving_threshold_rot = 1.0 * (i - stable_index) * stable_threshold_rot;
                    fix_node_ceres(i, moving_threshold_tran, moving_threshold_rot, _param.only_limit_translation);
                    LOG(INFO) << "Node [" << i << "] Free: (" << moving_threshold_tran << "," << moving_threshold_rot << ")";
                }
            }
        }
    }
    //You must add the parameter block to the problem before you can set a lower bound on one of its components.

    //when using the diagnoal information matrix, the optimization may not be done (stuck)
    return true;

#endif
    return false;
}

bool GlobalOptimize::fix_node_ceres(int block_id, float translate_thre, float quat_thre, bool only_translation) // set the limit for the parameters for estimating
{
#if CERES_ON
    for (int j = 0; j < 3; j++)
    {
        _ceresProblem->SetParameterLowerBound(mutable_pose_tran(block_id), j, *(mutable_pose_tran(block_id) + j) - translate_thre);
        _ceresProblem->SetParameterUpperBound(mutable_pose_tran(block_id), j, *(mutable_pose_tran(block_id) + j) + translate_thre);
    }
    if (!only_translation)
    {
        for (int k = 0; k < 4; k++)
        {
            _ceresProblem->SetParameterLowerBound(mutable_pose_quat(block_id), k, *(mutable_pose_quat(block_id) + k) - quat_thre);
            _ceresProblem->SetParameterUpperBound(mutable_pose_quat(block_id), k, *(mutable_pose_quat(block_id) + k) + quat_thre);
        }
    }
    return true;
#endif
    return false;
}

bool GlobalOptimize::solve_pgo_problem_ceres()
{
#if CERES_ON

    ceres::Solve(*_ceresOptions, _ceresProblem, _ceresSummary);

    LOG(INFO) << "=============================================  Optimization Start  =============================================";
    if (_param.verbose)
        LOG(INFO) << _ceresSummary->FullReport();
    else
        LOG(INFO) << _ceresSummary->BriefReport();
    LOG(INFO) << "==============================================  Optimization END  ==============================================";

    if (!_ceresSummary->IsSolutionUsable())
        LOG(WARNING) << "Optimization meet some problem";

    return _ceresSummary->IsSolutionUsable();
#endif
    return false;
}

//TO CHECK: let the unique id equal to the saving sequence of blocks
bool GlobalOptimize::assign_pose_optimized_ceres(CloudBlockPtrs &all_blocks)
{
    float warning_tran_thre = 0.2;
    //update nodes
    for (int i = 0; i < all_blocks.size(); i++)
    {
        pose_qua_t temp_pose;

        temp_pose.trans << _ceresParameters[7 * i], _ceresParameters[7 * i + 1], _ceresParameters[7 * i + 2];
        temp_pose.quat.x() = _ceresParameters[7 * i + 3];
        temp_pose.quat.y() = _ceresParameters[7 * i + 4];
        temp_pose.quat.z() = _ceresParameters[7 * i + 5];
        temp_pose.quat.w() = _ceresParameters[7 * i + 6];

        temp_pose.quat.normalize();

        all_blocks[i]->_poseOptimized = temp_pose.GetMatrix();

        Eigen::Matrix4d diff_mat = all_blocks[i]->_poseInit.inverse() * all_blocks[i]->_poseOptimized;

        // if (diff_mat.block<3, 1>(0, 3).norm() > warning_tran_thre)
        // 	LOG(WARNING) << "transformation of node [" << all_blocks[i]->_idInStrip << "], _uniqueId [" << all_blocks[i]->_uniqueId << "]:\n"
        // 				 << diff_mat;
        // else
        // 	LOG(INFO) << "transformation of node [" << all_blocks[i]->_idInStrip << "], _uniqueId [" << all_blocks[i]->_uniqueId << "]:\n"
        // 			  << diff_mat;
    }

    return true;
}

bool GlobalOptimize::update_optimized_edges(constraints &all_cons, bool fixed_reg_edge)
{
    int wrong_edge_count = 0;
    int correct_reg_edge_count = 0;
    int used_edge_count = 0;
    for (int i = 0; i < all_cons.size(); i++)
    {
        //check if the change is too much (Trans1_2_optimized vs. Trans1_2)
        Eigen::Matrix4d Trans1_2_optimized = all_cons[i].block1->_poseOptimized.inverse() * all_cons[i].block2->_poseOptimized;
        Eigen::Matrix4d delta_Trans1_2 = Trans1_2_optimized.inverse() * all_cons[i].Trans1_2;

        if (all_cons[i].con_type == REGISTRATION || all_cons[i].con_type == ADJACENT)
        {
            used_edge_count++;
            if (check_wrong_edge(delta_Trans1_2, _param.wrong_edge_translation_thre, _param.wrong_edge_rotation_thre)) //wrong //TODO: double-check this function here
            {
                wrong_edge_count++;
                if (all_cons[i].con_type == REGISTRATION)
                {
                    std::cerr << "Find a possible wrong registration edge [" << i << "]: connecting [" << all_cons[i].block1->_idInStrip << "] and [" << all_cons[i].block2->_idInStrip << "]."<<std::endl;
                    all_cons[i].con_type = NONE;
                }
            }
            else //correct
            {
                if (all_cons[i].con_type == REGISTRATION)
                    correct_reg_edge_count++;
            }
            // LOG(INFO) << "transformation difference of the edge [" << i << "]:\n"
            // 		  << delta_Trans1_2;
        }
    }
    if (1.0 * wrong_edge_count / used_edge_count > _param.wrong_edge_ratio_thre || correct_reg_edge_count == 0) //too many candidate wrong edges or there's no correct reg edge
    {
        LOG(WARNING) << "Too many possible wrong edges, this optimization might encounter some problem";
        // for (int i = 0; i < all_cons.size(); i++)
        // {
        // 	if (all_cons[i].con_type == REGISTRATION) //if wrong ---> all the reg edges would be seem as none
        // 		all_cons[i].con_type = NONE;
        // }
        return false;
    }
    else //udpate edges' transformation and information matrix //!!! don't update it, its constraint should not change throughout the computation (issue fixed)
    {
        //don't update them
        for (int i = 0; i < all_cons.size(); i++)
        {
            if (fixed_reg_edge && all_cons[i].con_type == ADJACENT)
            {
                //all_cons[i].Trans1_2 = all_cons[i].block1->_poseOptimized.inverse() * all_cons[i].block2->_poseOptimized;
                //update infomation matrix
                if (!all_cons[i].cov_updated)
                {
                    all_cons[i].information_matrix = _param.first_time_updating_ratio * all_cons[i].information_matrix;
                    all_cons[i].cov_updated = true;
                }
                else
                    all_cons[i].information_matrix = _param.life_long_updating_ratio * all_cons[i].information_matrix;
            }
        }
    }

    return true;
}

bool GlobalOptimize::update_optimized_nodes(CloudBlockPtrs &all_blocks, bool update_init, bool update_bbx)
{
    CloudUtility<PointType> cu;

    for (int i = 0; i < all_blocks.size(); i++)
    {
        all_blocks[i]->_poseLo = all_blocks[i]->_poseOptimized;
        if (update_init)
            all_blocks[i]->_poseInit = all_blocks[i]->_poseLo;

        //update global bbx
        if (update_bbx)
        {
            PointCloudXYZI::Ptr globalPcDown(new PointCloudXYZI());
            pcl::transformPointCloud(*all_blocks[i]->_pcDown, *globalPcDown, all_blocks[i]->_poseLo);
            cu.get_cloud_bbx(globalPcDown, all_blocks[i]->_bound);
            //all_blocks[i]->freeRawCloud();
        }
    }
    return true;
}

bool GlobalOptimize::update_edge_covariance_ceres(constraints &all_cons)
{
#if CERES_ON
    // But we need to update the infor-mat of edges instead of nodes
    // // Covariance of poses
    // ceres::Covariance::Options cov_options;
    // cov_options.num_threads = param_.num_threads;
    // cov_options.algorithm_type = ceres::DENSE_SVD;
    // cov_options.min_reciprocal_condition_number = 1.95146e-300; // for deficient Jacobian matrix
    // cov_options.null_space_rank = 10;							// for deficient Jacobian matrix
    // ceres::Covariance covariance(cov_options);

    // std::vector<std::pair<const double *, const double *>> covariance_blocks;
    // covariance_blocks.push_back(std::make_pair(_ceresParameters, _ceresParameters));
    // // //        CHECK(covariance.Compute(covariance_blocks, &problem));
    // if (covariance.Compute(covariance_blocks, _ceresProblem))
    // {
    // 	double covariance_pose[_numCeresParameters * _numCeresParameters]; //row major matrix
    // 	covariance.GetCovarianceBlock(_ceresParameters, _ceresParameters, covariance_pose);
    // 	//be carful of the memory problem here, covariance_pose can be very huge
    // 	for (int i = 0; i < all_cons.size(); i++)
    // 	{
    // 		if (all_cons[i].con_type == ADJACENT)
    // 		{
    // 			LOG(INFO) << "Edge [" << i << "] 's information matrix before optimization: \n"
    // 					  << all_cons[i].information_matrix;

    // 			int index_1 = all_cons[i].block1->_idInStrip;
    // 			int index_2 = all_cons[i].block2->_idInStrip;
    // 			int row_begin = index_1 * 7;
    // 			int col_begin = index_2 * 7;
    // 			Matrix6d temp_cov_mat;
    // 			temp_cov_mat.setIdentity();
    // 			for (int j = 0; j < 6; j++)
    // 				temp_cov_mat(j, j) = 1.0 / covariance_pose[(row_begin + j) * _numCeresParameters + (col_begin + j)];

    // 			all_cons[i].information_matrix = temp_cov_mat;

    // 			LOG(INFO) << "Edge [" << i << "] 's information matrix after optimization: \n"
    // 					  << all_cons[i].information_matrix;
    // 		}
    // 	}
    // 	delete[] covariance_pose;
    // }

    // std::vector<std::pair<const double *, const double *>>().swap(covariance_blocks);
    //has some problem, the information matrix is a bit too big after the optimization
    return true;
#endif
    return false;
}

bool GlobalOptimize::optimize_pose_graph_gtsam(CloudBlockPtrs &allBlocks, constraints &allCons)
{
#if GTSAM_ON
    //init(); //clear the problem, options, etc
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2* isam = new gtsam::ISAM2(parameters);
    int edgeCount = allCons.size();
    int nodeCount = allBlocks.size();
    int fixedNodeCount = 0;
    gtsam::NonlinearFactorGraph gtsamFactorGraph;
    gtsam::Values initialEstimates;
    for (int i = 0; i < allBlocks.size(); i++)
    {
        //fix nodes or not
        if (allBlocks[i]->_poseFixed)
        {
            fixedNodeCount++;
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
                    gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8).finished());
			gtsamFactorGraph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(allBlocks[i]->_poseInit), priorNoise));

//            gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
//            gtsamFactorGraph.add(gtsam::PriorFactor<gtsam::Pose3>(i, gtsam::Pose3(allBlocks[i]->_poseInit), priorNoise));
        }
        initialEstimates.insert(i, gtsam::Pose3(allBlocks[i]->_poseInit));
    }

    LOG(INFO) << "PGO setup: [" << nodeCount << "] nodes, [" << fixedNodeCount << "] fixed nodes, [" << edgeCount << "] edges";

    if (nodeCount - fixedNodeCount > edgeCount)
    {
        LOG(ERROR) << "Edge number is not enough for the PGO";
        return false;
    }

    //For each edge
    gtsam::noiseModel::Diagonal::shared_ptr edgeNoise;
    for (int i = 0; i < allCons.size(); i++)
    {
        if (allCons[i].con_type != NONE && allCons[i].con_type != HISTORY)
        {
            if(allCons[i].con_type==ADJACENT)
                edgeNoise =gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2).finished());
            else if(allCons[i].con_type==REGISTRATION)
                edgeNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4).finished());
            //edge_noise = gtsam::noiseModel::Gaussian::Information(allCons[i].information_matrix);
            gtsamFactorGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(allCons[i].block1->_idInStrip,
                                                                    allCons[i].block2->_idInStrip,
                                                                    gtsam::Pose3(allCons[i].Trans1_2),
                                                                    edgeNoise));
        }
    }

    // print factor graph
    //gtsamFactorGraph.print("\nFactor Graph:\n");

    // print initial values
    //initialEstimates.print("\nInitial Values:\n");
    std::cout << "iSAM, PGO....\n";

    // update iSAM
    isam->update(gtsamFactorGraph, initialEstimates);
    isam->update();
//    isam->printStats();
    gtsamFactorGraph.resize(0);
    initialEstimates.clear();

// // Use Gauss-Newton method to optimize the initial values
    // gtsam::GaussNewtonParams parameters;

    // // print per iteration
    // parameters.setVerbosity("ERROR");

    // // optimize!
    // gtsam::GaussNewtonOptimizer _optimizer(gtsamFactorGraph, initialEstimates, parameters);
    // gtsam::Values results = _optimizer.optimize();

    // // print final values
    // results.print("Final Result:\n");

    // // Calculate marginal covariances for all poses
    // gtsam::Marginals marginals(graph, results);

    // // print marginal covariances
    // for (int i = 0; i < allBlocks.size(); i++)
    // {
    // 	LOG(INFO) << "node [" << i << "]'s marginal covariance:\n"
    // 			  << marginals.marginalCovariance(i);
    // }

    gtsam::Values isamCurrentEstimate = isam->calculateEstimate();
    size_t numPoses = isamCurrentEstimate.size();
    for (size_t kfIdx = 0; kfIdx < numPoses; kfIdx++)
    {
        double x = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).translation().x();
        double y = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).translation().y();
        double z = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).translation().z();
        double roll = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).rotation().roll();
        double pitch = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).rotation().pitch();
        double yaw = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).rotation().yaw();
        allBlocks[kfIdx]->_poseOptimized=pcl::getTransformation(x, y, z, roll, pitch, yaw).matrix().cast<double>();
    }
    delete isam;
    return true;
#endif
    return false;
}

bool GlobalOptimize::optimize_pose_graph_gtsam(CloudBlockPtrs &allBlocks)
{
#if GTSAM_ON
    std::cout << "iSAM, PGO incremental....\n";
    if(_gtsamFactorGraph.empty())
        return false;
    // update iSAM
    _isam->update(_gtsamFactorGraph, _initialEstimates);
    _isam->update();

//    _isam->update();
//    _isam->update();
//    _isam->update();
//    _isam->update();
//    _isam->update();

//    _isam->printStats();
    _gtsamFactorGraph.resize(0);
    _initialEstimates.clear();

    gtsam::Values isamCurrentEstimate = _isam->calculateEstimate();
    size_t numPoses = isamCurrentEstimate.size();
    std::cout<<"Optimized poses size:"<<numPoses<<std::endl;
//    for (size_t kfIdx = 0; kfIdx < numPoses; kfIdx++)
//    {
//        double x = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).translation().x();
//        double y = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).translation().y();
//        double z = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).translation().z();
//        double roll = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).rotation().roll();
//        double pitch = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).rotation().pitch();
//        double yaw = isamCurrentEstimate.at<gtsam::Pose3>(kfIdx).rotation().yaw();
//        allBlocks[kfIdx]->_poseOptimized=pcl::getTransformation(x, y, z, roll, pitch, yaw).matrix().cast<double>();
//    }
    int idx=0;
    for (const auto& estimate : isamCurrentEstimate)
    {
        double x = estimate.value.cast<gtsam::Pose3>().translation().x();
        double y = estimate.value.cast<gtsam::Pose3>().translation().y();
        double z = estimate.value.cast<gtsam::Pose3>().translation().z();
        double roll = estimate.value.cast<gtsam::Pose3>().rotation().roll();
        double pitch = estimate.value.cast<gtsam::Pose3>().rotation().pitch();
        double yaw = estimate.value.cast<gtsam::Pose3>().rotation().yaw();
        allBlocks[idx]->_poseOptimized=pcl::getTransformation(x, y, z, roll, pitch, yaw).matrix().cast<double>();
        idx++;
    }
    return true;
#endif
    return false;
}

bool GlobalOptimize::check_wrong_edge(Eigen::Matrix4d delta_tran_mat, float translation_thre, float rotation_thre_deg)
{
    Eigen::Vector3d ts(delta_tran_mat(0, 3), delta_tran_mat(1, 3), delta_tran_mat(2, 3));
    Eigen::AngleAxisd rs(delta_tran_mat.block<3, 3>(0, 0));

    if (ts.norm() > translation_thre || std::abs(rs.angle()) > rotation_thre_deg / 180.0 * M_PI)
        return true;
    else
        return false;
}

void GlobalOptimize::set_adjacent_edge_information_matrix(Matrix6d &vc_matrix)
{
    vc_matrix.setIdentity();

    vc_matrix(0, 0) = _param.tx_std * _param.tx_std;
    vc_matrix(1, 1) = _param.ty_std * _param.ty_std;
    vc_matrix(2, 2) = _param.tz_std * _param.tz_std;
    vc_matrix(3, 3) = (_param.roll_std * M_PI / 180.0) * (_param.roll_std * M_PI / 180.0);
    vc_matrix(4, 4) = (_param.pitch_std * M_PI / 180.0) * (_param.pitch_std * M_PI / 180.0);
    vc_matrix(5, 5) = (_param.yaw_std * M_PI / 180.0) * (_param.yaw_std * M_PI / 180.0);
}

bool GlobalOptimize::addEdgeConstraint(Constraint &conn, bool isAdjReliable)
{
#ifdef GTSAM_ON
    if(conn.con_type==ADJACENT)
        _initialEstimates.insert(conn.block2->_idInStrip, gtsam::Pose3(conn.block2->_poseInit));

    gtsam::noiseModel::Diagonal::shared_ptr edgeNoise;
    if(conn.con_type==ADJACENT)
        if(isAdjReliable)
            edgeNoise =gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2).finished());
        else
            edgeNoise =gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 5e-2, 5e-2, 5e-2, 5e-2, 5e-2, 5e-2).finished());
    else if(conn.con_type==REGISTRATION)
        edgeNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-5, 1e-5, 1e-5, 1e-4, 1e-4, 1e-4).finished());
    //edge_noise = gtsam::noiseModel::Gaussian::Information(allCons[i].information_matrix);
    _gtsamFactorGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(conn.block1->_idInStrip,
                                                             conn.block2->_idInStrip,
                                                             gtsam::Pose3(conn.Trans1_2),
                                                             edgeNoise));

    return true;
#endif
    return false;
}

bool GlobalOptimize::addFixedConstraint(CloudBlockPtr &block)
{
#ifdef GTSAM_ON
//
//    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
//            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8).finished());

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
    _gtsamFactorGraph.add(gtsam::PriorFactor<gtsam::Pose3>(block->_idInStrip, gtsam::Pose3(block->_poseLo), priorNoise));
    _initialEstimates.insert(block->_idInStrip, gtsam::Pose3(block->_poseLo));

    return true;
#endif
    return false;
}

bool GlobalOptimize::addGPSConstraint(SensorMsgs::GNSSData::ConstPtr& gpsData, const int& frameID, double maxGPSCovTh)
{
#ifdef GTSAM_ON
    double noiseX = gpsData->covariance[0];
    double noiseY = gpsData->covariance[7];
    double noiseZ = gpsData->covariance[14];
    if (noiseX > maxGPSCovTh || noiseY > maxGPSCovTh)
        return false;

    double gpsX = gpsData->pos.x();
    double gpsY = gpsData->pos.y();
    double gpsZ = gpsData->pos.z();

    // GNSS not properly initialized (0,0,0)
    if (abs(gpsX) < 1e-6 && abs(gpsY) < 1e-6)
        return false;

    // Add GNSS every a few meters
    PointType curGPSPoint;
    noiseX = std::max(noiseX, 1.0);
    noiseY = std::max(noiseY, 1.0);
    noiseZ = std::max(noiseZ, 1.0);

    gtsam::noiseModel::Diagonal::shared_ptr gpsNoise =
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) <<noiseX, noiseY,noiseZ).finished());
    gtsam::GPSFactor gpsFactor(frameID, gtsam::Point3(gpsX, gpsY, gpsZ), gpsNoise);
    _gtsamFactorGraph.add(gpsFactor);
    return true;
#endif
    return false;
}

#if 0
//registration blunder elimination (By Zhao Xing)
	bool GlobalOptimize::cal_con_number(constraints &all_cons, strip &all_blocks)
	{
		LOG(INFO) << "Begin calculating reg_source_con_number!";
		//std::cout << "Begin calculating reg_source_con_number!\n";
		for (int i = 0; i < all_cons.size(); i++)
		{
			if ((all_cons[i].con_type == REGISTRATION) && (all_blocks[all_cons[i].block2->_uniqueId].cal_regsource_con_number == false))
			{
				int temp_uniqueID = all_cons[i].block2->_uniqueId;
				all_blocks[temp_uniqueID].reg_source_con_number = 0;
				for (int j = i; j < all_cons.size(); j++)
				{
					if ((all_cons[j].con_type == REGISTRATION) && (all_cons[j].block2->_uniqueId == temp_uniqueID))
					{
						all_blocks[temp_uniqueID].reg_source_con_number++;
						all_blocks[temp_uniqueID].as_source_con_unique_id.push_back(all_cons[j]._uniqueId);

						LOG(INFO) << "all_cons[j]._uniqueId: " << all_cons[j]._uniqueId << "all_blocks[temp_uniqueID].as_source_con_unique_id[reg_source_con_number-1]: " << all_blocks[temp_uniqueID].as_source_con_unique_id[all_blocks[temp_uniqueID].reg_source_con_number - 1] << endl;
					}
				}
				all_blocks[temp_uniqueID].cal_regsource_con_number = true;
			}
		}
		return 1;
	}

	bool GlobalOptimize::eliminate_blunders(constraints &all_cons, strip &all_blocks)
	{
		LOG(INFO) << "Begin eliminating errors!";
		cout << "Begin eliminating errors!\n";
		int elimination_count = 0; //count the number of the edges that would been eliminate
		//Each REGISTRATION con
		for (int i = 0; i < all_cons.size(); i++)
		{
			if (all_cons[i].con_type == REGISTRATION)
			{
				int temp_uniqueID = all_cons[i].block2->_uniqueId;

				if (all_blocks[temp_uniqueID].reg_source_con_number <= 1 && (all_blocks[temp_uniqueID].elimination_flag == false))
				{
					/*all_cons[i].con_type = NONE;
					all_cons[i].Trans1_2.setIdentity();
					all_cons[i].information_matrix.setIdentity();*/
					all_blocks[temp_uniqueID].elimination_flag = true;
					//elimination_count++;
					LOG(WARNING) << "The registration edge " << all_cons[i]._uniqueId << " has been deleted because the source block just has one registration!";
					//cout << "The registration edge " << all_cons[i]._uniqueId << " has been deleted because the source block just has one registration!" << endl;
				}
				if (all_blocks[temp_uniqueID].reg_source_con_number == 2 && (all_blocks[temp_uniqueID].elimination_flag == false))
				{
					int con_number = all_blocks[temp_uniqueID].reg_source_con_number;
					//construct adjacent matrix of registration
					std::vector<std::vector<float>> RegMatrix(con_number, std::vector<float>(con_number, 0.0));
					//sum each line of RegMatrix
					//std::vector<float> temp(con_number, 0.0);
					//find con_unique_id
					//vector<constraint_t, Eigen::aligned_allocator<constraint_t>>  tempcon;
					std::vector<int> index;
					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < all_cons.size(); k++)
						{
							if (all_cons[k]._uniqueId == all_blocks[temp_uniqueID].as_source_con_unique_id[j])
							{
								index.push_back(k);
								//tempcon.push_back(all_cons[k]);
								break;
							}
						}
					}
					LOG(INFO) << "The source block " << all_blocks[temp_uniqueID]._uniqueId << "'s temp:\n";
					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number; k++)
						{
							LOG(INFO) << "RegMatrix[" << j << "][" << k << "]'sFnorm^2:"
									  << "( con" << all_cons[index[j]]._uniqueId << " with con" << all_cons[index[k]]._uniqueId << " )\n";
							RegMatrix[j][k] = cal_f_norm(all_cons[index[j]].Trans1_2, all_cons[index[k]].Trans1_2);
							LOG(INFO) << "RegMatrix[" << k << "][" << j << "]'sFnorm^2:"
									  << "( con" << all_cons[index[k]]._uniqueId << " with con" << all_cons[index[j]]._uniqueId << " )\n";
							RegMatrix[k][j] = cal_f_norm(all_cons[index[k]].Trans1_2, all_cons[index[j]].Trans1_2);
						}
					}

					cout << "all_blocks[temp_uniqueID]._uniqueId: " << all_blocks[temp_uniqueID]._uniqueId << "     temp_uniqueID: " << temp_uniqueID << endl;
					cout << "The source block " << all_blocks[temp_uniqueID]._uniqueId << "'s Fnorm:\n";

					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number - 1; k++)
							cout << RegMatrix[j][k] << "\t";
						cout << RegMatrix[j][con_number - 1] << "\n";
					}

					if (RegMatrix[0][1] > 1.0) //1.0 is the threshold
					{
						for (int j = 0; j < con_number; j++)
						{
							all_cons[index[j]].con_type = NONE;
							all_cons[index[j]].Trans1_2.setIdentity();
							all_cons[index[j]].information_matrix.setIdentity();
							elimination_count++;
							LOG(INFO) << "all_cons[index[j]]._uniqueId: " << all_cons[index[j]]._uniqueId << "     all_blocks[temp_uniqueID].as_source_con_unique_id[j]: " << all_blocks[temp_uniqueID].as_source_con_unique_id[j] << endl;
							LOG(WARNING) << "The registration edge " << all_cons[index[j]]._uniqueId << " has been deleted because the transformation matrix of source block is inconsistent with another one!";
							LOG(INFO) << "The registration edge " << all_cons[index[j]]._uniqueId << " has been deleted because the transformation matrix of source block is inconsistent with another one!" << endl;
						}
					}

					all_blocks[temp_uniqueID].elimination_flag = true;
				}
				if ((all_blocks[temp_uniqueID].reg_source_con_number > 2) && (all_blocks[temp_uniqueID].elimination_flag == false))
				{
					int con_number = all_blocks[temp_uniqueID].reg_source_con_number;

					//construct adjacent matrix of registration
					std::vector<std::vector<float>> RegMatrix(con_number, std::vector<float>(con_number, 0.0));
					//sum each line of RegMatrix
					std::vector<float> temp(con_number, 0.0);

					//find con_unique_id
					//vector<constraint_t, Eigen::aligned_allocator<constraint_t>>  tempcon;
					std::vector<int> index;
					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < all_cons.size(); k++)
						{
							if (all_cons[k]._uniqueId == all_blocks[temp_uniqueID].as_source_con_unique_id[j])
							{
								index.push_back(k);
								//tempcon.push_back(all_cons[k]);
								break;
							}
						}
					}

					float sum = 0.0;
					float average = 0.0;
					float sigama = 0.0;

					/*	for (int j = 0; j < RegMatrix.size() - 1; j++)
					{
						for (int k = j + 1; k < RegMatrix.size(); k++)
						{
							RegMatrix[j][k] = calFnorm(all_cons[all_blocks[all_cons[i].block2->_uniqueId].as_source_con_unique_id[j]].Trans1_2, all_cons[all_blocks[all_cons[i].block2->_uniqueId].as_source_con_unique_id[k]].Trans1_2);
							RegMatrix[k][j] = RegMatrix[j][k];
							temp[j] += RegMatrix[j][k];
							temp[k] += RegMatrix[k][j];
							sum += 2*RegMatrix[j][k];
						}

					}*/

					LOG(INFO) << "The source block " << all_blocks[temp_uniqueID]._uniqueId << "'s temp:\n";

					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number; k++)
						{
							LOG(INFO) << "RegMatrix[" << j << "][" << k << "]'sFnorm^2:"
									  << "( con" << all_cons[index[j]]._uniqueId << " with con" << all_cons[index[k]]._uniqueId << " )\n";
							RegMatrix[j][k] = cal_f_norm(all_cons[index[j]].Trans1_2, all_cons[index[k]].Trans1_2);
							LOG(INFO) << "RegMatrix[" << k << "][" << j << "]'sFnorm^2:"
									  << "( con" << all_cons[index[k]]._uniqueId << " with con" << all_cons[index[j]]._uniqueId << " )\n";
							RegMatrix[k][j] = cal_f_norm(all_cons[index[k]].Trans1_2, all_cons[index[j]].Trans1_2);
							temp[j] += RegMatrix[j][k];
							sum += RegMatrix[j][k];
						}
					}

					LOG(INFO) << "all_blocks[temp_uniqueID]._uniqueId: " << all_blocks[temp_uniqueID]._uniqueId << "     temp_uniqueID: " << temp_uniqueID << endl;
					LOG(INFO) << "The source block " << all_blocks[temp_uniqueID]._uniqueId << "'s Fnorm:\n";

					for (int j = 0; j < con_number; j++)
					{
						for (int k = 0; k < con_number - 1; k++)
							cout << RegMatrix[j][k] << "\t";
						cout << RegMatrix[j][con_number - 1] << "\n";
					}

					//find out error and eliminate
					average = 1.0 * sum / con_number;
					for (int j = 0; j < con_number; j++)
					{
						sigama += 1.0 * (temp[j] - average) * (temp[j] - average) / con_number;
					}
					sigama = sqrt(sigama);

					for (int j = 0; j < con_number; j++)
					{
						if (temp[j] - average > sigama) //is error
						{
							all_cons[index[j]].con_type = NONE;
							all_cons[index[j]].Trans1_2.setIdentity();
							all_cons[index[j]].information_matrix.setIdentity();
							elimination_count++;
							cout << "all_cons[index[j]]._uniqueId: " << all_cons[index[j]]._uniqueId << "     all_blocks[temp_uniqueID].as_source_con_unique_id[j]: " << all_blocks[temp_uniqueID].as_source_con_unique_id[j] << endl;
							LOG(WARNING) << "The registration edge " << all_cons[index[j]]._uniqueId << " has been deleted because the transformation matrix of source block is inconsistent with others!";
							cout << "The registration edge " << all_cons[index[j]]._uniqueId << " has been deleted because the transformation matrix of source block is inconsistent with others!" << endl;
						}
					}
					all_blocks[temp_uniqueID].elimination_flag = true;
				}
			}
		}

		LOG(INFO) << "!----------------------------------------------------------------------------!";
		LOG(INFO) << "Eliminate blunders done ";
		LOG(INFO) << "The number of eliminated registration constraints : " << elimination_count;
		LOG(INFO) << "!----------------------------------------------------------------------------!";

		return 1;
	}

	//Calculate Fnorm between Transformation martix1 and Transformtion martix2
	float GlobalOptimize::cal_f_norm(Eigen::Matrix4d &Trans1, Eigen::Matrix4d &Trans2)
	{
		float temp = 0.0;
		float Fnorm = 0.0;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if ((Trans1(i, j) - Trans2(i, j) < 0.001) && (Trans2(i, j) - Trans1(i, j) < 0.001))
				{
					temp += 0.0;
				}
				else
				{
					temp += (Trans1(i, j) - Trans2(i, j)) * (Trans1(i, j) - Trans2(i, j));
				}
			}
		}
		cout << "before:" << temp << "\t";
		if (temp < 0.000001)
		{
			temp = 0.0;
		}
		cout << "after:" << temp << endl;
		Fnorm = sqrt(temp);
		return Fnorm;
	}

#endif
