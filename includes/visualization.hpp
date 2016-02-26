#ifndef __VISUALIZATION_HPP__
#define __VISUALIZATION_HPP__

namespace vis
{
int cal_scale, cal_rotate, cal_uncertainty, cal_uncertainty_scale, cal_uncertainty_rotate, axis_group, collision_hit_group, collision_free_group, threshold_hit_group, threshold_free_group
	, start_node_group, goal_node_group, node_group, edge_group, paths_group, velocity_group, solution_group
	, solution_marker_group, simulation_belief_group, simulation_actual_group;
int robot_base, robot_group, robot_collision_object, robot_model, robot_model_object;

void clearSolution() {
	CAL_EmptyGroup(solution_group);
	CAL_EmptyGroup(solution_marker_group);
}

void clearEdges() {
	CAL_EmptyGroup(edge_group);
}

void clearNodes() {
	CAL_EmptyGroup(node_group);
}

void clearPaths() {
	CAL_EmptyGroup(paths_group);
}

void clearVelocities() {
	CAL_EmptyGroup(velocity_group);
}

void clearSimulation() {
	CAL_EmptyGroup(simulation_belief_group);
	CAL_EmptyGroup(simulation_actual_group);
}

void clearAll() {
	clearSolution();
	clearEdges();
	clearNodes();
	clearPaths();
	clearVelocities();
	clearSimulation();
}

void buildKeyframe(const double& t, const state& x, bool still = false, double alpha = 0.0, double offset = 0.0) {
	bool isQuat = true;
	double x_rot, y_rot;
	double x_pos = 0.0, y_pos = 0.0, z_pos = 0.0;

	robot->getPosition(x, &x_pos, &y_pos, &z_pos);
	float * o;
	robot->getRotation(x, &o, &isQuat);

	if (still) {
		Eigen::Matrix<double,3,1> current_pos;
		current_pos[0] = x_pos;
		current_pos[1] = y_pos;
		current_pos[2] = z_pos;

		int new_group;

#if (DYNAMICS == QUADROTOR)
		//CAL_CloneGroup(&new_group, robot_model, stills_group, false, "Stills subgroup");
		//CAL_SetGroupVisibility(new_group, 0, true, true);
		//CAL_SetGroupQuaternion(new_group,o[0],o[1],o[2],o[3]);
#elif (DYNAMICS == NONHOLONOMIC)
		CAL_CreateGroup(&new_group, stills_group, false, "Stills subgroup");

		CAL_CreateBox(new_group, 5, 3, 2.5, 2.5, 1.5, 0);
		CAL_SetGroupColor(new_group, 0, 0, 0);

		int inner;
		CAL_CreateBox(new_group, 4.5, 3.1, 2, 2.5, 1.5, 0, &inner);
		CAL_SetObjectColor(inner, 0.5 + alpha, 0.5 - alpha, 0.5 - alpha);
#else
		CAL_CreateGroup(&new_group, stills_group, false, "Stills subgroup");

		CAL_CreateCylinder(new_group, 5, 3, 0, 0, 0);
		CAL_SetGroupColor(new_group, 0, 0, 0);

		int inner;
		CAL_CreateCylinder(new_group, 4.75, 3.05, 0, 0, 0, &inner);
		CAL_SetObjectColor(inner, 0.5 + alpha, 0.5 - alpha, 0.5 - alpha);
#endif

		stills_groups.push_back(make_pair(new_group, current_pos));

		CAL_SetGroupPosition(new_group, x_pos, y_pos + offset, z_pos);

#if (DYNAMICS == NONHOLONOMIC)
		CAL_SetGroupOrientation(new_group, o[0], o[1], o[2]);
#endif

	} else {
		robot->addGroupKeyState(t, x);
		////cout << "Key frame (" << t << "): " << ~x << "\t" << x[6] << ": " << x_rot << endl;
		//float p[3] = {(float) x_pos, (float) y_pos, (float) z_pos};

		//// Daman
		//int result = CAL_AddGroupKeyState(robot_model, (float) t, p, o, CAL_NULL, isQuat);
		//if (CAL_SUCCESS != result) {
		//	cout << "Failed (" << result << ") to add key frame!" << endl;
		//}
	}

	//CAL_CreateSphere(solution_group, NODE_SIZE, x_pos, y_pos, z_pos);
}

void renderAxis() {
	double length = 10.0;
	double radius = 1.0;
	double position = 1.5;

	int origin_id = 0;
	int x_point_id = 0, y_point_id = 0, z_point_id = 0;
	CAL_CreateSphere(axis_group, 1, 0, 0, 0, &origin_id);
	CAL_SetObjectColor(origin_id, 0, 0, 0);

	CAL_CreateSphere(axis_group, 1, position, 0, 0, &x_point_id);
	CAL_SetObjectColor(x_point_id, 1, 0, 0);

	CAL_CreateSphere(axis_group, 1, 0, position, 0, &y_point_id);
	CAL_SetObjectColor(y_point_id, 0, 1, 0);

	CAL_CreateSphere(axis_group, 1, 0, 0, position, &z_point_id);
	CAL_SetObjectColor(z_point_id, 0, 0, 1);
}

void showAxis() {
	if (SHOW_AXIS) {
		SHOW_AXIS = false;
	} else {
		SHOW_AXIS = true;
	}
	CAL_SetGroupVisibility(axis_group, 0, SHOW_AXIS);
}

void initVisulization() {
	CAL_Initialisation(true, true, true);

	// ROTATE & SCALE
#if (SCALE_THEN_ROTATE == 1)
	CAL_CreateGroup(&cal_scale, 0, false, "Global Rotation Group");
	CAL_CreateGroup(&cal_rotate, cal_scale, false, "Global Scale Group");
#else
	CAL_CreateGroup(&cal_rotate, 0, false, "Global Rotation Group");
	CAL_CreateGroup(&cal_scale, cal_rotate, false, "Global Scale Group");
#endif

	CAL_CreateGroup(&cal_uncertainty, cal_uncertainty_rotate, false, "Uncertainty group");
	CAL_CreateGroup(&robot_base, 0, false, "Robot Base Group");
}

template <size_t _dim>
void RotateAndScaleGroup(const int rotate_group_id, const int scale_group_id, const Eigen::Matrix<double,_dim,_dim>& V, const Eigen::Matrix<double,_dim,_dim>& S, bool use_inverse_scale = false, bool use_inverse_rotate = false) {
	int result;
	
	Eigen::Matrix<double,_dim,_dim> V_inverse = V;
	if (use_inverse_rotate) {
		V_inverse = V.transpose();
	}
	Eigen::Quaternion<double> q(V_inverse);
	
	//Eigen::Quaternion<double> q(V);
	result = CAL_SetGroupQuaternion(rotate_group_id, (CAL_scalar)q.x(), (CAL_scalar)q.y(), (CAL_scalar)q.z(), (CAL_scalar)q.w());
	if (result != CAL_SUCCESS) {
		std::cout << "CAL_SetGroupQuaternion failed (" << result << ")." << std::endl;
		_getchar();
		exit(1);
	}

	if (use_inverse_scale) {
		result = CAL_SetGroupScaling(scale_group_id, 1.0/(S(0, 0)), 1.0/(S(1, 1)), 1.0/(S(2, 2)));
	} else {
		result = CAL_SetGroupScaling(scale_group_id, S(0, 0), S(1, 1), S(2, 2));
	}
	if (result != CAL_SUCCESS) {
		std::cout << "CAL_SetGroupScaling failed (" << result << ")." << std::endl;
		_getchar();
		exit(1);
	}

}

template <size_t _dim>
void ShowUncertainty(World * world, Robot * robot, const Eigen::Matrix<double,_dim,_dim>& V, const Eigen::Matrix<double,_dim,_dim>& S, double distance_threshold, double x = 0.0f, double y = 0.0f, double z = 0.0f) {
	std::cout << "V: " << V << std::endl;
	std::cout << "S: " << S << std::endl;

	World::points_t pts;
	float scaler = world->getScaler();
	world->getCorners(pts);

	int result;

	/*
	int temp;
	CAL_CreateGroup(&temp, 0, false, "Original Uncertainty");
	result = CAL_CreateSphere(temp, 1, 0.0f, 0.0f, 0.0f);
	CAL_SetGroupColor(temp, 0, 1, 0, 0.25);
	CAL_SetGroupVisibility(temp, 1, true);
	*/

	for (World::points_t::const_iterator pt = pts.cbegin(); pt != pts.cend(); ++pt) {
		int cal_uncertainty_scale, cal_uncertainty_rotate, cal_uncertainty;

		// ROTATE & SCALE
#if (SCALE_THEN_ROTATE == 0)
		CAL_CreateGroup(&cal_uncertainty_scale, 0, false);
		CAL_CreateGroup(&cal_uncertainty_rotate, cal_uncertainty_scale, false);
		CAL_CreateGroup(&cal_uncertainty, cal_uncertainty_rotate, false);
#else
		CAL_CreateGroup(&cal_uncertainty_rotate, 0, false);
		CAL_CreateGroup(&cal_uncertainty_scale, cal_uncertainty_rotate, false);
		CAL_CreateGroup(&cal_uncertainty, cal_uncertainty_scale, false);
#endif

		//result = CAL_CreateSphere(cal_uncertainty, scaler*distance_threshold+robot->getLargestRadius(), 0.0f, 0.0f, 0.0f);
		result = CAL_CreateSphere(cal_uncertainty, scaler*distance_threshold, 0.0f, 0.0f, 0.0f);
		//result = CAL_CreateSphere(cal_uncertainty, distance_threshold, 0.0f, 0.0f, 0.0f);

		CAL_SetGroupColor(cal_uncertainty, 0.95, 0.15, 0.15, 0.75);

		RotateAndScaleGroup<_dim>(cal_uncertainty_rotate, cal_uncertainty_scale, V, S, USE_INVERSE_SCALE, !USE_INVERSE_ROTATE);

#if (SCALE_THEN_ROTATE == 0)
		CAL_SetGroupPosition(cal_uncertainty_scale, pt->first, pt->second, 0.0f);
#else
		CAL_SetGroupPosition(cal_uncertainty_rotate, pt->first, pt->second, 0.0f);
#endif
	}
}

template <size_t _dim>
void WarpEnvironment(const Eigen::Matrix<double,_dim,_dim>& V, const Eigen::Matrix<double,_dim,_dim>& S) {
	std::cout << "V: " << V << std::endl;
	std::cout << "S: " << S << std::endl;

	//std::cout << R << std::endl;
	//Eigen::JacobiSVD< Eigen::Matrix<double,_dim,_dim> > svd(R);
	//const Eigen::JacobiSVD< Eigen::Matrix<double,_dim,_dim> >::MatrixUType& U = svd.matrixU();
	//const Eigen::JacobiSVD< Eigen::Matrix<double,_dim,_dim> >::SingularValuesType& S = svd.singularValues();

	RotateAndScaleGroup<_dim>(cal_rotate, cal_scale, V, S, !USE_INVERSE_SCALE, USE_INVERSE_ROTATE);

	/*
	q = Eigen::Quaternion<double>(V.transpose());
	result = CAL_SetGroupQuaternion(cal_rotate, (CAL_scalar)q.x(), (CAL_scalar)q.y(), (CAL_scalar)q.z(), (CAL_scalar)q.w());
	if (result != CAL_SUCCESS) {
		std::cout << "CAL_SetGroupQuaternion failed (" << result << ")." << std::endl;
		_getchar();
		exit(1);
	}
	*/
}

template <size_t _dim>
void RestoreEnvironment() {
	int result;

	result = CAL_SetGroupQuaternion(cal_rotate, (CAL_scalar)0, (CAL_scalar)0, (CAL_scalar)0, (CAL_scalar)1);
	if (result != CAL_SUCCESS) {
		std::cout << "CAL_SetGroupQuaternion failed (" << result << ")." << std::endl;
		_getchar();
		exit(1);
	}

	result = CAL_SetGroupScaling(cal_scale, 1, 1, 1);
	if (result != CAL_SUCCESS) {
		std::cout << "CAL_SetGroupScaling failed (" << result << ")." << std::endl;
		_getchar();
		exit(1);
	}
}

void setupVisualization(const state& x0, const state& x1, void (*buildEnvironment)(int)) {
#if (DYNAMICS != QUADROTOR)
	CAL_SetViewOptions(0, CAL_ORTHOPROJ);
#endif

#if defined(EXPERIMENT)
	CAL_SuspendVisualisation();
#endif
	CAL_CreateGroup(&axis_group, cal_rotate, false, "Axis group");
	CAL_CreateGroup(&collision_hit_group, 0, false, "Collision hit");
	CAL_CreateGroup(&collision_free_group, 0, false, "Collision free");
	CAL_CreateGroup(&threshold_hit_group, 0, false, "Collision hit");
	CAL_CreateGroup(&threshold_free_group, 0, false, "Collision free");
	CAL_CreateGroup(&solution_group, 0, false, "Solution");
	CAL_CreateGroup(&solution_marker_group, 0, false, "Solution Way Points");
	CAL_CreateGroup(&node_group, 0, false, "Nodes");
	CAL_CreateGroup(&edge_group, 0, false, "Edges");
	CAL_CreateGroup(&paths_group, 0, false, "Paths");
	CAL_CreateGroup(&velocity_group, 0, false, "Velocities");
	CAL_CreateGroup(&start_node_group, 0, false, "Start");
	CAL_CreateGroup(&goal_node_group, 0, false, "Goal");
	CAL_CreateGroup(&simulation_belief_group, 0, false, "Beliefs");
	CAL_CreateGroup(&simulation_actual_group, 0, false, "Beliefs");

	CAL_SetGroupVisibility(axis_group, 0, SHOW_AXIS, true);
	CAL_SetGroupVisibility(node_group, 0, SHOW_NODES, true);
	CAL_SetGroupVisibility(edge_group, 0, SHOW_TREE, true);
	CAL_SetGroupVisibility(paths_group, 0, SHOW_TREE_PATHS, true);
	CAL_SetGroupVisibility(velocity_group, 0, SHOW_VELOCITIES, true);
	CAL_SetGroupVisibility(solution_group, 0, true, true);
	CAL_SetGroupVisibility(collision_hit_group, 0, true, true);
	CAL_SetGroupVisibility(collision_free_group, 0, true, true);
	CAL_SetGroupVisibility(threshold_hit_group, 0, true, true);
	CAL_SetGroupVisibility(threshold_free_group, 0, true, true);
	CAL_SetGroupVisibility(simulation_belief_group, 0, true, true);
	CAL_SetGroupVisibility(simulation_actual_group, 0, true, true);
	CAL_SetGroupVisibility(cal_uncertainty, 0, true, true);

	CAL_SetGroupColor(collision_hit_group, 1, 0, 0);
	CAL_SetGroupColor(collision_free_group, 0, 1, 0);
	CAL_SetGroupColor(threshold_hit_group, 1, 0, 1);
	CAL_SetGroupColor(threshold_free_group, 0.5, 0.5, 0.5);
	//CAL_SetGroupColor(robot_group, 0, 0, 0);
	CAL_SetGroupColor(solution_group, 0, 0, 1);
	CAL_SetGroupColor(solution_marker_group, 0, 0, 1);
	CAL_SetGroupColor(velocity_group, 0, 0, 1);
	CAL_SetGroupColor(edge_group, 0, 0.5, 0);
	CAL_SetGroupColor(paths_group, 0, 0.75, 0);
	CAL_SetGroupColor(start_node_group, 0, 0, 1);
	CAL_SetGroupColor(goal_node_group, 0, 1, 0);
	CAL_SetGroupColor(simulation_belief_group, 1, 0, 0);
	CAL_SetGroupColor(simulation_actual_group, 0, 0, 1);

	renderAxis();

	// TODO select the world appropriately
	// ROTATE & SCALE
#if (SCALE_THEN_ROTATE)
	buildEnvironment(cal_rotate);
#else
	buildEnvironment(cal_scale);
#endif

	//setupRobot();

#if defined(MAKE_STILLS) && (DYNAMICS != QUADROTOR)
	world->set_obstacle_color(0.15, 0.15, 0.15, 1);
#else
	world->set_obstacle_color(0.1, 0.1, 0.1, 0.1);
#endif

#if (USE_OBSTACLES > 0)
	world->show_obstacles();
#else
	world->hide_obstacles();
#endif

	/*
	Eigen::Matrix<double,3,3> V = Eigen::Matrix<double,3,3>::Zero();
	V(0, 1) = 1;
	V(1, 0) = -1;
	V(2, 2) = 1;

	Eigen::Matrix<double,3,3> S = Eigen::Matrix<double,3,3>::Zero();
	S(0, 0) = 0.5;
	S(1, 1) = 1;
	S(2, 2) = 1;
	*/
	world->positionCamera(Rotation, Scale.inverse());

	//CAL_SetGroupScaling(cal_scale, 1.0/(S(0, 0)), 1.0/(S(1, 1)), 1.0);
	//Eigen::Quaternion<double> q(V);
	//CAL_SetGroupQuaternion(cal_rotate, (CAL_scalar)q.x(), (CAL_scalar)q.y(), (CAL_scalar)q.z(), (CAL_scalar)q.w());

	/*
	V(0, 1) = -1;
	V(1, 0) = 1;
	q = Eigen::Quaternion<double>(V);
	//CAL_SetGroupQuaternion(cal_rotate, (CAL_scalar)q.x(), (CAL_scalar)q.y(), (CAL_scalar)q.z(), (CAL_scalar)q.w());
	CAL_SetGroupQuaternion(cal_rotate, (CAL_scalar)0, (CAL_scalar)0, (CAL_scalar)0, (CAL_scalar)1);
	*/

	robot->position(x0, true);
	robot->rotate(x0, true);

	// Setup the start and goal nodes
	double start_x = x0[0]; double goal_x = x1[0];
	double start_y = x0[1]; double goal_y = x1[1];
#if POSITION_DIM == 3
	double start_z = x0[2]; double goal_z = x1[2];
#else
	double start_z = 0; double goal_z = 0;
#endif
	CAL_CreateSphere(start_node_group, 5*NODE_SIZE, start_x, start_y, start_z);
	CAL_CreateSphere(goal_node_group, 5*NODE_SIZE, goal_x, goal_y, goal_z);
}

void visualizePath(const tree_t& tree, bool thick_line=false, bool log = true, bool build_keyframe = true) {
	CAL_EmptyGroup(solution_group);
	CAL_EmptyGroup(solution_marker_group);
	CAL_SetGroupColor(solution_group, 0, 0, 1);
	CAL_SetGroupColor(solution_marker_group, 0, 0, 1);

	robot->clearGroupKeyStates(true);

	float x_pos, y_pos, z_pos;
	state_time_list_t path;
	control_time_list_t controls;
	createNominalTrajectory(tree, &path, &controls);
	size_t path_length = path.size();
	CAL_scalar * points = new CAL_scalar[3*path_length];
	for (int idx = 0; idx < path_length; ++idx) {
		if (build_keyframe) {
			buildKeyframe(path[idx].first, path[idx].second);
		}
		if (thick_line) {
			x_pos = path[idx].second[0];
			y_pos = path[idx].second[1];
#if POSITION_DIM == 3
			z_pos = path[idx].second[2];
#else
			z_pos = 0;
#endif
			CAL_CreateSphere(solution_group, NODE_SIZE, x_pos, y_pos, z_pos);
		} else {
			points[3*idx] = path[idx].second[0];
			points[3*idx+1] = path[idx].second[1];
#if POSITION_DIM == 3
			points[3*idx+2] = path[idx].second[2];
#else
			points[3*idx+2] = 0;
#endif
		}
		if (log) {
			fwrite((const void *)&(path[idx].first), sizeof(double), 1, path_log);
			double *current_elements = (path[idx].second.data());
			//fwrite((const void *)current_elements, sizeof(double), X_DIM, path_log);
			for (int i = 0; i < X_DIM; i++) {
				current_time = current_elements[i];
				fwrite((const void *)&current_time, sizeof(double), 1, path_log);
			}
		}
	}

	if (!thick_line) {
		CAL_CreatePolyline(vis::solution_group, 1, (int*)&path_length, points);
	}

	double sentinel = -1;
	if (log) {
		fwrite((const void *)&sentinel, sizeof(double), 1, path_log);
		fflush(path_log);
	}
}

void plotPath(const state& x0, const state& x1, const double radius) {
	state_time_list_t segment;
	double max_tau = 0.0;
	double current_time = 0.0;
	double cost = 0.0;
	double actual_deltaT = 0.0;

	connect(x0, x1, radius, cost, max_tau, &actual_deltaT, &segment, NULL);

	sort(segment.begin(), segment.end(), temporal_order<state_time_t>);
	for(state_time_list_t::iterator q = segment.begin(); q != segment.end(); q++) {
		current_time = (q->first) + max_tau;
		buildKeyframe(current_time, q->second);
		double *current_elements = (q->second.data());
		for (int i = 0; i < X_DIM; i++) {
			current_time = current_elements[i];
		}
	}
}

// The next two functions could be better generalized
void createSphere(CAL_scalar r, CAL_scalar x, CAL_scalar y, CAL_scalar z, int group = solution_group) {
	// TODO this needs error checking
	CAL_CreateSphere(group, r, x, y, z);
}

void markBelief(CAL_scalar x, CAL_scalar y, CAL_scalar z) {
	createSphere(1.5*NODE_SIZE, x, y, z, simulation_belief_group);
}

void markActual(CAL_scalar x, CAL_scalar y, CAL_scalar z) {
	createSphere(2*NODE_SIZE, x, y, z, simulation_actual_group);
}

/**
Draws a single node in the tree.
*/
void visualizeTreeNode(const tree_t& tree, node_id_t node_idx) {
	CAL_SuspendVisualisation();

	CAL_SetGroupVisibility(node_group, 0, true, true);

	//CAL_EmptyGroup(node_group);
	//CAL_EmptyGroup(node_group);
	//CAL_SetGroupColor(node_group, 1, 0, 0);
	CAL_SetGroupColor(node_group, 1, 0, 0);

#if POSITION_DIM == 3
	CAL_CreateSphere(node_group, 3*NODE_SIZE, tree[node_idx].x[0], tree[node_idx].x[1], tree[node_idx].x[2]);
#else
	CAL_CreateSphere(node_group, 3*NODE_SIZE, tree[node_idx].x[0], tree[node_idx].x[1], 0);
#endif

	CAL_ResumeVisualisation();
}

/**
Draws a single node in the tree.
*/
void visualizeTreePath(const tree_t& tree, const node_id_t start_node_idx, const node_id_t end_node_idx) {
	CAL_SuspendVisualisation();

	CAL_SetGroupVisibility(node_group, 0, true, true);

	CAL_EmptyGroup(node_group);
	CAL_SetGroupColor(node_group, 0, 1, 0);

	double cost, tau;
	double actual_deltaT = 0.0;
	state_time_list_t segment;
	segment.clear();
	connect(tree[start_node_idx].x, tree[end_node_idx].x, DBL_MAX, cost, tau, &actual_deltaT, &segment, NULL);
	sort(segment.begin(), segment.end(), temporal_order<state_time_t>);

	size_t segment_length = segment.size();
	size_t max_length = 500000; // TODO This needs to be a function of the distance between neighboring nodes in the tree and deltaT.
	CAL_scalar * points = new CAL_scalar[max_length];
	if (segment_length*3 > max_length) {
		max_length = segment_length;
		delete[] points;
		points = new CAL_scalar[max_length];
	}

	for (size_t idx = 0; idx < segment_length; ++idx) {
		float x_pos = segment[idx].second[0];
		float y_pos = segment[idx].second[1];
		float z_pos = 0;
#if POSITION_DIM == 3
		z_pos = segment[idx].second[2];
#endif


		CAL_CreateSphere(node_group, 0.1, x_pos, y_pos, z_pos);
	}

	CAL_ResumeVisualisation();
}

/**
Draws the tree as a set of curves representing the real paths between nodes.
*/
void visualizeTree(const tree_t& tree, bool prune = false, bool vertices_only = false) {
	CAL_SuspendVisualisation();

	CAL_SetGroupVisibility(paths_group, 0, true, true);

	CAL_EmptyGroup(solution_group);
	CAL_EmptyGroup(solution_marker_group);
	CAL_SetGroupColor(solution_group, 0, 0, 1);
	CAL_SetGroupColor(solution_marker_group, 0, 0, 1);

	//CAL_ClearGroupKeyStates(robot_model, true);
	robot->clearGroupKeyStates(true);
	
	std::queue<int> dfs;
	dfs.push(1);

	cout << "Rendering paths...";
	double cost, tau;
	state_time_list_t segment;
	double actual_deltaT = 0.0;
	int node_counter = 0, nodes_skipped = 0;
	size_t max_length = 500000; // TODO This needs to be a function of the distance between neighboring nodes in the tree and deltaT.
	CAL_scalar * points = new CAL_scalar[max_length];
	size_t segment_length = 0;
	for (tree_t::const_iterator parent = tree.begin(); parent != tree.end(); ++parent) {
		for (node_list_t::const_iterator child = parent->children.begin(); child != parent->children.end(); ++child) {
			std::cout << ++node_counter << '\r';

			segment.clear();
			connect(parent->x, tree[*child].x, DBL_MAX, cost, tau, &actual_deltaT, &segment, NULL);
			sort(segment.begin(), segment.end(), temporal_order<state_time_t>);

			segment_length = segment.size();
			if (segment_length > max_length) {
				max_length = segment_length;
				delete[] points;
				points = new CAL_scalar[max_length];
			}

			for (size_t idx = 0; idx < segment_length; ++idx) {
				float x_pos = segment[idx].second[0];
				float y_pos = segment[idx].second[1];
				float z_pos = 0;
#if POSITION_DIM == 3
				z_pos = segment[idx].second[2];
#endif

				points[3*idx] = x_pos;
				points[3*idx+1] = y_pos;
				points[3*idx+2] = z_pos;

				if (vertices_only == true) {
					CAL_CreateSphere(node_group, 3*NODE_SIZE, x_pos, y_pos, z_pos);
				}
			}
			if (vertices_only == false) {
				CAL_CreatePolyline(paths_group, 1, (int *)&segment_length, points);
			}
		}
	}

	std::cout << " done. " << std::endl << nodes_skipped << " nodes skipped because they had a cost greater than the cost to the goal." << std::endl;

	CAL_ResumeVisualisation();
}

/**
Draws the tree as a set of straight lines.
*/
void drawTree(const tree_t& tree, bool vertices_only = false) {
	CAL_SuspendVisualisation();

	CAL_SetGroupVisibility(edge_group, 0, true, true);
	CAL_SetGroupVisibility(node_group, 0, true, true);

	int np[1] = {2};
	float p[6] = {0, 0, 0, 0, 0, 0};

	Node current = tree[0];
	while (current.parent != NO_PARENT) {
#if (DYNAMICS == QUADROTOR)
		p[0] = current.x[0];
		p[1] = current.x[1];
		p[2] = current.x[2];
		p[3] = tree[current.parent].x[0];
		p[4] = tree[current.parent].x[1];
		p[5] = tree[current.parent].x[2];
#elif (DYNAMICS == NONHOLONOMIC)
		p[0] = current.x[0];
		p[1] = current.x[1];
		p[3] = tree[current.parent].x[0];
		p[4] = tree[current.parent].x[1];
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
		p[X_COORD] = current.x[0];
		p[Y_COORD] = current.x[1];
		p[XV_COORD] = tree[current.parent].x[0];
		p[YV_COORD] = tree[current.parent].x[1];
#else
		p[0] = current.x[0];
		p[2] = current.x[1];
		p[3] = tree[current.parent].x[0];
		p[5] = tree[current.parent].x[1];
#endif

		CAL_CreatePolyline(solution_group, 1, np, p);

		current = tree[current.parent];
	}

	double x = 0.0, y = 0.0, z = 0.0;
	double x_parent = 0.0, y_parent = 0.0, z_parent = 0.0;
	double v_x = 0.0, v_y = 0.0, v_z = 0.0;
	for (size_t i = 0; i < tree.size(); ++i) {
		x = y = z = x_parent = y_parent = z_parent = v_x = v_y = v_z = 0.0;

#if (DYNAMICS == QUADROTOR)
		x = tree[i].x[0];
		y = tree[i].x[1];
		z = tree[i].x[2];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			y_parent = tree[tree[i].parent].x[1];
			z_parent = tree[tree[i].parent].x[2];
		}
		v_x = x + tree[i].x[3];
		v_y = y + tree[i].x[4];
		v_z = z + tree[i].x[5];
#elif (DYNAMICS == NONHOLONOMIC)
		x = tree[i].x[0];
		y = tree[i].x[1];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			y_parent = tree[tree[i].parent].x[1];
		}
		// TODO DWEBB implement velocity drawing
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
		x = tree[i].x[0];
		y = tree[i].x[1];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			y_parent = tree[tree[i].parent].x[1];
		}
		v_x = x + tree[i].x[2];
		v_y = y + tree[i].x[3];
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
		x = tree[i].x[0];
		y = tree[i].x[1];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			y_parent = tree[tree[i].parent].x[1];
		}
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
		x = tree[i].x[0];
		z = tree[i].x[1];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			z_parent = tree[tree[i].parent].x[1];
		}

#else
		dynamicsError();
#endif
		//CAL_CreateSphere(node_group, 0.5, tree[i].x[0], tree[i].x[1], 0);
		CAL_CreateSphere(node_group, NODE_SIZE, x, y, z);
		if (tree[i].parent != NO_PARENT) {

#if (DYNAMICS == QUADROTOR)
			p[0] = x;
			p[1] = y;
			p[2] = z;
			p[3] = v_x;
			p[4] = v_y;
			p[5] = v_z;

			CAL_CreatePolyline(velocity_group, 1, np, p);
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
			p[X_COORD] = x;
			//p[2] = z;
			p[Y_COORD] = y;
			p[XV_COORD] = v_x;
			//p[5] = v_z;
			p[YV_COORD] = v_y;

			CAL_CreatePolyline(velocity_group, 1, np, p);
#endif
			p[0] = x;
			p[1] = y;
			p[2] = z;
			p[3] = x_parent;
			p[4] = y_parent;
			p[5] = z_parent;

			if (vertices_only == true) {
				CAL_CreateSphere(node_group, 3*NODE_SIZE, x, y, z);
			} else {
				CAL_CreatePolyline(edge_group, 1, np, p);
			}
		}
	}

	CAL_ResumeVisualisation();
}

void visualizeLog() {
	path_log = fopen("path_log.txt", "rb");

	//rewind(path_log);

	double t = 0.0, t2;
	state x;
	int path_count = 0;
	fread((void *)&t, sizeof(double), 1, path_log);
	while (true) {
		cout << path_count << '\r';
		if (t == -1) {
			path_count++;

			cout << endl << "Continue? ";
			char response = _getchar();
			if (!((response == 'y') || (response == 'Y'))) {
				exit(0);
			}

			CAL_EmptyGroup(solution_group);
			CAL_EmptyGroup(solution_marker_group);
			CAL_SetGroupColor(solution_group, 0, 0, 1);
			CAL_SetGroupColor(solution_marker_group, 0, 0, 1);

			//CAL_ClearGroupKeyStates(robot_model, true);
			robot->clearGroupKeyStates(true);
			fread((void *)&t, sizeof(double), 1, path_log);
		}
		for (int i = 0; i < X_DIM; i++) {
			fread((void *)&t2, sizeof(double), 1, path_log);
			x[i] = t2;
		}
		//fread((void*)x._elems, sizeof(double), x.numColumns()*x.numRows(), path_log);

		//cout << "t: " << t << "\tx: " << ~x << endl;
		//_getchar();

		buildKeyframe(t, x);

		fread((void *)&t, sizeof(double), 1, path_log);
	}

	_getchar();
	exit(0);
}

void visualizeFinalPath(const tree_t& tree, bool thick_line=false, bool log = true, bool build_keyframes = true) {
	CAL_SuspendVisualisation();
	if (tree[0].cost_from_start < DBL_MAX) {
		cout << setw(10) << 0 << " " << setw(11) << TARGET_NODES << "/" << TARGET_NODES << " cost: " << tree[0].cost_from_start << endl;

		// Build a visualization of the final path
		vis::visualizePath(tree, thick_line, log, build_keyframes);
	} else {
		cout << endl << "No path found" << endl;
	}
	CAL_ResumeVisualisation();
}

void makeStills(int path) {
	path_log = fopen("path_log.txt", "rb");

	CAL_CreateGroup(&stills_group, cal_rotate, false, "Stills Parent Group");

	double t = 0.0, t2;
	state x;
	int path_count = 0;
	int frame_count = 0;
	state_time_list_t states;

	// Get starting time
	fread((void *)&t, sizeof(double), 1, path_log);

	// Read data
	while (true) {
		// Finalize path
		if (t == -1) {
			path_count++;

			if (path_count >= path) {
				double current_r = 0.5, current_g = 0.5, current_b = 0.5;
				current_alpha = 0.5;
				double change = 0.5/states.size();
				float current_z = 0;
				reverse(states.begin(), states.end());
				for (state_time_list_t::iterator p = states.begin(); p != states.end(); p++) {
//
//				CAL_SetGroupColor(p->first, current_r, current_g, current_b);
//				current_r += alpha_change;
//				current_g -= alpha_change;
//				current_b -= alpha_change;
//				current_alpha += alpha_change;
//#if DYNAMICS != QUADROTOR
//				CAL_SetGroupPosition(p->first, (p->second)[0], current_z, (p->second)[2]);
//#endif
//				current_z += 0.05;
//
#if (DYNAMICS == QUADROTOR)
					buildKeyframe(p->first, p->second, true, current_r);
#else
					buildKeyframe(p->first, p->second, true, current_r, current_z);
#endif
#ifdef HUE_SHIFT
					current_r -= change;
#endif
					current_z -= 1;
				}

				cout << " Continue? ";
				char response = _getchar();
				if (!((response == 'y') || (response == 'Y'))) {
					exit(0);
				}
			}

			stills_groups.clear();
			states.clear();

			frame_count = 0;


			CAL_EmptyGroup(stills_group, true);
//
//			CAL_EmptyGroup(solution_group);
//			CAL_EmptyGroup(solution_marker_group);
//			CAL_SetGroupColor(solution_group, 0, 0, 1);
//			CAL_SetGroupColor(solution_marker_group, 0, 0, 1);
//
//			CAL_ClearGroupKeyStates(robot_model, true);
//
			// Get next starting time
			fread((void *)&t, sizeof(double), 1, path_log);
		}

		cout << '\r' << path_count;

		// Get state data
		for (int i = 0; i < X_DIM; i++) {
			fread((void *)&t2, sizeof(double), 1, path_log);
			x[i] = t2;
		}
		//fread((void*)x._elems, sizeof(double), x.numColumns()*x.numRows(), path_log);

		//cout << "t: " << t << "\tx: " << ~x << endl;
		//_getchar();

		// Only collect frames at still rate
		if ((frame_count % STILL_RATE) == 0) {
			states.push_back(make_pair(t, x));
		}
		frame_count++;

		// Read next time
		fread((void *)&t, sizeof(double), 1, path_log);
	}

	_getchar();
	exit(0);
}

void graphPath() {
	double radius = START_RADIUS; //DBL_MAX;

#if (DYNAMICS == QUADROTOR)
//	/*
//	x0.reset();
//	x0[3] = x0[5] = -2;

//	x1.reset();
//	x1[2] = 5;
//	x1[4] = x1[5] = 2;
//	
#elif (DYNAMICS == NONHOLONOMIC)
	//x_bounds[0].first = -DBL_MAX;
	//x_bounds[0].second = DBL_MAX;

	//x_bounds[1].first = -DBL_MAX;
	//x_bounds[1].second = DBL_MAX;

	//x0.reset();
	x0 = Eigen::Matrix<double,X_DIM,1>::Zero();
	x0[0] = x0[1] = 0;
	x0[2] = -1;
	x0[3] = 1;

	//x1.reset();
	x1 = Eigen::Matrix<double,X_DIM,1>::Zero();
	x1[0] = 10;
	x1[1] = -5;
	x1[2] = 0;
	x1[3] = 1;

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	x0[0] = 50; x0[1] = 50; x0[2] = -6; x0[3] = -4;
	x1[0] = 75; x1[1] = 30; x1[2] = 7; x1[3] = 1;

#else
	assert("Not configured");
#endif

	tree_t tree;
	double cost;
	double junk, junk2;
	if (connect(x0, x1, radius, cost, junk, &junk2, NULL, NULL)) {
		std::cout << "connected\t";

		Node start, end;

		end.parent = 1;
		end.x = x1;
		end.cost_from_start = cost;
		tree.push_back(end);

		start.parent = NO_PARENT;
		start.x = x0;
		start.cost_from_start = 0;
		start.children.push_back(0);
		tree.push_back(start);

		visualizePath(tree);
	}

	else {
		std::cout << "failed connect\t";
	}

	std::cout << "cost: " << cost << std::endl;

	CAL_ResumeVisualisation();

	_getchar();

	exit(0);
}

bool testCollisions_loop = true;
float testCollisions_x_delta = 0;
float testCollisions_y_delta = 0;
bool testCollisions_use_thresholds = USE_THRESHOLDS;
bool testCollisions_world_warped = false;
void testCollisionsKeypressCallback(char key, bool pressed) {
	if (pressed && key == '?') {
		std::cout << "u - up" << std::endl;
		std::cout << "j - down" << std::endl;
		std::cout << "h - left" << std::endl;
		std::cout << "k - right" << std::endl;
		std::cout << "c - switch collision check method" << std::endl;
		std::cout << "q - quit" << std::endl;
	}
	if (key == 'u') {
		if (pressed == 1)
			testCollisions_y_delta = 1;
		else
			testCollisions_y_delta = 0;
	}
	if (key == 'j') {
		if (pressed == 1)
			testCollisions_y_delta -= 1;
		else
			testCollisions_y_delta = 0;
	}
	if (key == 'h') {
		if (pressed == 1)
			testCollisions_x_delta = -1;
		else
			testCollisions_x_delta = 0;
	}
	if (key == 'k') {
		if (pressed == 1)
			testCollisions_x_delta = 1;
		else
			testCollisions_x_delta = 0;
	}
	if (key == 'q') {
		testCollisions_loop = false;
	}
	if (key == 's') {
		if (testCollisions_world_warped) {
			RestoreEnvironment<3>();
		} else {
			WarpEnvironment<3>(Rotation, Scale);
		}
	}
	if (pressed && key == 'c') {
		if (testCollisions_use_thresholds == 1) {
#if USE_THRESHOLDS
			testCollisions_use_thresholds = 0;
#elif USE_SET_CLEARANCE
			world->setClearance(0);
#endif
		} else {
#if USE_THRESHOLDS
			testCollisions_use_thresholds = 1;
#elif USE_SET_CLEARANCE
			world->setClearance(distance_thresholds[0]);
#endif
		}
		std::cout << "Set testCollisions_use_thresholds to " << testCollisions_use_thresholds << std::endl;
	}
}

void testCollisions(World * world, Robot * robot) {
	int collisions;
	bool collision;

	robot->hide_model();
	robot->show_collision_checker();
	state s = world->getStartState();

	CAL_SetKeypressCallback(testCollisionsKeypressCallback);

	while (testCollisions_loop) {
		s[0] += testCollisions_x_delta;
		s[1] += testCollisions_y_delta;
		robot->position(s);
		if (testCollisions_use_thresholds) {
			collision = world->checkDistance(robot, true);
			std::cout << "Colliding? " << collision << std::endl;
		} else {
			world->checkCollisions(robot, &collisions, true);
			std::cout << "Collisions? " << collisions << std::endl;
		}
		Sleep(50);
	}
}

}
#endif // __VISUALIZATION_HPP__