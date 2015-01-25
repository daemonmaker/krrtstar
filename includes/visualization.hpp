#ifndef __VISUALIZATION_HPP__
#define __VISUALIZATION_HPP__

namespace vis
{
int cal_scale, cal_rotate, axis_group, collision_hit_group, collision_free_group
	, start_node_group, goal_node_group, node_group, edge_group, velocity_group, solution_group
	, solution_marker_group;
int robot_group, robot_collision_object, robot_model, robot_model_object;

void buildKeyframe(const double& t, const state& x, bool still = false, double alpha = 0.0, double offset = 0.0) {
	bool isQuat = true;
	double x_rot, y_rot;
	double x_pos = 0.0, y_pos = 0.0, z_pos = 0.0;

//#if (DYNAMICS == QUADROTOR)
//	x_pos = x[0];
//	y_pos = x[1];
//	z_pos = x[2];
//#else
//	x_pos = x[0];
//	y_pos = x[1];
//#endif
//
//#if (DYNAMICS == QUADROTOR)
//	Eigen::Matrix<double,3,3> rot = Eigen::Matrix<double,3,3>::Zero();
//	rot(0,2) = x[7];
//	rot(1,2) = -x[6];
//	rot(2,0) = -x[7];
//	rot(2,1) = x[6];
//
//	Eigen::Matrix<double,3,3> R = rot.exp();
//	Eigen::Quaternion<double> q(R);
//	float o[4] = {(float)q.x(), (float)q.y(), (float)q.z(), (float)q.w()};
//#elif (DYNAMICS == NONHOLONOMIC)
//	isQuat = false;
//	double rot = x[2] - 0.5*M_PI;
//	while (rot < 0) rot += 2*M_PI;
//	float o[3] = {0, 0, rot};
//#else
//	float *o = CAL_NULL;
//#endif

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

#ifdef SHOW_PATHS
	CAL_CreateSphere(solution_group, 2*NODE_SIZE, x_pos, y_pos, z_pos);
#endif
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

void initVisulization() {
	CAL_Initialisation(true, true, true);
	CAL_CreateGroup(&cal_scale, 0, false, "Global Scale Group");
	CAL_CreateGroup(&cal_rotate, cal_scale, false, "Global Rotation Group");
}

void setupVisualization(const state& x0, const state& x1, void (*buildEnvironment)(int)) {
	// visualization
	//CAL_Initialisation(true, true, true);

#if (DYNAMICS != QUADROTOR)
	CAL_SetViewOptions(0, CAL_ORTHOPROJ);
#endif

#if defined(EXPERIMENT)
	CAL_SuspendVisualisation();
#endif
	CAL_CreateGroup(&axis_group, cal_rotate, false, "Axis group");

	CAL_CreateGroup(&collision_hit_group, cal_rotate, false, "Collision hit");
	CAL_CreateGroup(&collision_free_group, cal_rotate, false, "Collision free");
	CAL_CreateGroup(&solution_group, cal_rotate, false, "Solution");
	CAL_CreateGroup(&solution_marker_group, cal_rotate, false, "Solution Way Points");
	CAL_CreateGroup(&node_group, cal_rotate, false, "Nodes");
	CAL_CreateGroup(&edge_group, cal_rotate, false, "Edges");
	CAL_CreateGroup(&velocity_group, cal_rotate, false, "Velocities");

	CAL_SetGroupVisibility(axis_group, 0, SHOW_AXIS, true);
	
	CAL_SetGroupVisibility(node_group, 0, false, true);
	CAL_SetGroupVisibility(edge_group, 0, SHOW_TREE, true);
	CAL_SetGroupVisibility(velocity_group, 0, SHOW_TREE, true);
	CAL_SetGroupVisibility(solution_group, 0, true, true);

#if defined(SHOW_COLLISIONS)
	CAL_SetGroupVisibility(collision_hit_group, 0, true, true);
#else
	CAL_SetGroupVisibility(collision_hit_group, 0, false, true);
#endif
#if defined(SHOW_COLLISION_CHECKS)
	CAL_SetGroupVisibility(collision_free_group, 0, true, true);
#else
	CAL_SetGroupVisibility(collision_free_group, 0, false, true);
#endif

	CAL_SetGroupColor(collision_hit_group, 1, 0, 0);
	CAL_SetGroupColor(collision_free_group, 0, 1, 0);
	//CAL_SetGroupColor(robot_group, 0, 0, 0);
	CAL_SetGroupColor(solution_group, 0, 0, 1);
	CAL_SetGroupColor(solution_marker_group, 0, 0, 1);
	CAL_SetGroupColor(velocity_group, 0, 0, 1);
	CAL_SetGroupColor(edge_group, 0.65, 0.16, 0.16);

	renderAxis();

	// TODO select the world appropriately
	buildEnvironment(cal_rotate);
	//setupRobot();

#if defined(MAKE_STILLS) && (DYNAMICS != QUADROTOR)
	world->set_obstacle_color(0.1, 0.1, 0.1, 1);
#else
	world->set_obstacle_color(0.1, 0.1, 0.1, 0.1);
#endif

#if (USE_OBSTACLES > 0)
	world->show_obstacles();
#else
	world->hide_obstacles();
#endif


	// Position the camera based on the state bounds
	double eye_x = 0.0, eye_y = 0.0, eye_z = 0.0;
	double camera_x = 0.0, camera_y = 0.0, camera_z = 0.0;
	double up_x = 1.0, up_y = 0.0, up_z = 0.0;

	world->positionCamera();

	double start_x = 0.0, start_y = 0.0, start_z = 0.0;
	double goal_x = 0.0, goal_y = 0.0, goal_z = 0.0;

#if (DYNAMICS == QUADROTOR)
	start_x = x0[0]; goal_x = x1[0];
	start_y = x0[1]; goal_y = x1[1];
	start_z = x0[2]; goal_z = x1[2];

#else
	start_x = x0[0]; goal_x = x1[0];
	start_y = x0[1]; goal_y = x1[1];
#endif 

//	// Position the robot
//	CAL_SetGroupPosition(robot_model, start_x, start_y, start_z);
//
//#if (DYNAMICS == NONHOLONOMIC)
//	double rot = x0[2] - 0.5*M_PI;
//	while (rot < 0) rot += 2*M_PI;
//
//	CAL_SetGroupOrientation(robot_model, 0, 0, rot);
//	CAL_SetGroupOrientation(robot_group, 0, 0, rot);
//#endif

	robot->position(x0, true);
	robot->rotate(x0, true);

#ifdef SHOW_PATHS
	// Setup the start and goal nodes
	CAL_CreateGroup(&start_node_group, cal_rotate, false, "Start");
	CAL_CreateGroup(&goal_node_group, cal_rotate, false, "Goal");
	CAL_SetGroupColor(start_node_group, 1, 0, 0);
	CAL_SetGroupColor(goal_node_group, 0, 1, 0);
	CAL_CreateSphere(start_node_group, 5*NODE_SIZE, start_x, start_y, start_z);
	CAL_CreateSphere(goal_node_group, 5*NODE_SIZE, goal_x, goal_y, goal_z);
#endif
}

bool state_order(const state_time_t& a, const state_time_t& b) {
	return (a.first < b.first);
}

void visualize(const tree_t& tree) {
	CAL_EmptyGroup(solution_group);
	CAL_EmptyGroup(solution_marker_group);
	CAL_SetGroupColor(solution_group, 0, 0, 1);
	CAL_SetGroupColor(solution_marker_group, 0, 0, 1);

	//CAL_ClearGroupKeyStates(robot_model, true);
	robot->clearGroupKeyStates(true);
	
	Node current = tree[0];
	double cost, tau;
	state d_tau;
	vector<state, Eigen::aligned_allocator<state> > state_list;
	while (current.parent != NO_PARENT) {
		state_list.push_back(current.x);
		current = tree[current.parent];
	}
	
	state_list.push_back(current.x);
	reverse(state_list.begin(), state_list.end());
	
	state_time_list_t segment;
	double max_tau = 0.0;
	double current_time = 0.0;
	
	for(vector<state, Eigen::aligned_allocator<state> >::iterator p = state_list.begin(); (p + 1) != state_list.end(); p++) {
		segment.clear();
		//computeCost(*p, *(p + 1), DBL_MAX, cost, tau, d_tau);
		//checkPath(*p, *(p+1), tau, d_tau, false, &segment);

		connect(*p, *(p+1), DBL_MAX, cost, tau, &segment);

		sort(segment.begin(), segment.end(), state_order);
		for(state_time_list_t::iterator q = segment.begin(); q != segment.end(); q++) {
			current_time = (q->first) + max_tau;
			buildKeyframe(current_time, q->second);
			fwrite((const void *)&current_time, sizeof(double), 1, path_log);
			double *current_elements = (q->second.data());
			//fwrite((const void *)current_elements, sizeof(double), X_DIM, path_log);
			for (int i = 0; i < X_DIM; i++) {
				current_time = current_elements[i];
				fwrite((const void *)&current_time, sizeof(double), 1, path_log);
			}
		}
		max_tau += tau;
	}

	double sentinel = -1;
	fwrite((const void *)&sentinel, sizeof(double), 1, path_log);
	fflush(path_log);
}

void plotPath(const state& x0, const state& x1, const double radius) {
	state_time_list_t segment;
	double max_tau = 0.0;
	double current_time = 0.0;
	double cost = 0.0;

	connect(x0, x1, radius, cost, max_tau, &segment);

	sort(segment.begin(), segment.end(), state_order);
	for(state_time_list_t::iterator q = segment.begin(); q != segment.end(); q++) {
		current_time = (q->first) + max_tau;
		buildKeyframe(current_time, q->second);
		double *current_elements = (q->second.data());
		for (int i = 0; i < X_DIM; i++) {
			current_time = current_elements[i];
		}
	}
}

void createSphere(CAL_scalar r, CAL_scalar x, CAL_scalar y, CAL_scalar z) {
	// TODO this needs error checking
	CAL_CreateSphere(solution_group, r, x, y, z);
}

void drawTree(const tree_t& tree) {
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

			CAL_CreatePolyline(edge_group, 1, np, p);
		}
	}
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
	double junk;
	if (connect(x0, x1, radius, cost, junk, NULL)) {
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

		visualize(tree);
	}

	else {
		std::cout << "failed connect\t";
	}

	std::cout << "cost: " << cost << std::endl;

	CAL_ResumeVisualisation();

	_getchar();

	exit(0);
}

template <size_t _dim>
void TransformEnvironment(int cal_scale, int cal_rotate, const Eigen::Matrix<double,_dim,_dim>& R) {
	Eigen::JacobiSVD< Eigen::Matrix<double,_dim,_dim> > svd(R);
	Eigen::Quaternion<double> q(svd.matrixU());
	CAL_SetGroupQuaternion(cal_rotate, (CAL_scalar)q.x(), (CAL_scalar)q.y(), (CAL_scalar)q.z(), (CAL_scalar)q.w());
	CAL_SetGroupScaling(cal_scale, 1.0/svd.singularValues()[0], 1.0/svd.singularValues()[1], 1.0);
}

}
#endif // __VISUALIZATION_HPP__