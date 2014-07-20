#include "dynamicrrt.h"
#include <cmath>
#include <complex>

using namespace std;

void dynamicsError() {
	cout << "Invalid dynamics (" << DYNAMICS << ")" << endl;
	_getchar();
	exit(-1);
}

void setupParameters(void) {
	x_bounds.resize(X_DIM);
	u_bounds.resize(U_DIM);

#if (DYNAMICS == QUADROTOR)
	control_penalty = 0.1;

	// Position
	x_bounds[0] = std::make_pair(-2.5, 2.5);
	x_bounds[1] = std::make_pair(-2.5, 2.5);
	x_bounds[2] = std::make_pair(0, 5);

	// Velocity
	x_bounds[3] = std::make_pair(-5, 5);
	x_bounds[4] = std::make_pair(-5, 5);
	x_bounds[5] = std::make_pair(-5, 5);

	// Angular position
	x_bounds[6] = std::make_pair(-1000, 1000);
	x_bounds[7] = std::make_pair(-1000, 1000);

	// Angular velocity
	x_bounds[8] = std::make_pair(-5000, 5000);
	x_bounds[9] = std::make_pair(-5000, 5000);

	u_bounds[0] = std::make_pair(-gravity*mass, 2*gravity*mass);
	u_bounds[1] = std::make_pair(-1.5*gravity*mass, 1.5*gravity*mass);
	u_bounds[2] = std::make_pair(-1.5*gravity*mass, 1.5*gravity*mass);

	u_bounds[0] = std::make_pair(-4.545, 9.935);
	u_bounds[1] = std::make_pair(-3.62, 3.62);
	u_bounds[2] = std::make_pair(-3.62, 3.62);

	//A.reset();
	A = Eigen::Matrix<float,X_DIM,X_DIM>::Zero();
	//A.insert(0, 3, eye<3>());
	A.block<3,3>(0,3) = Eigen::Matrix<float,3,3>::Identity();
	//A.insert(6, 8, eye<2>());
	A.block<2,2>(6,8) = Eigen::Matrix<float,2,2>::Identity();
	A(3,7) = gravity;
	A(4,6) = -gravity;

	B(5,0) = 1/mass;
	B(8,1) = length/inertia;
	B(9,2) = length/inertia;

	//c = zeros<X_DIM>();
	c = state::Zero();

	//R = eye<U_DIM>();
	R = Eigen:Matrix<float,U_DIM,U_DIM>::Identity();
	R(0,0) = 0.25*control_penalty;
	R(1,1) = R(2,2) = 0.5*control_penalty;

	//x0.reset();
	x0 = state::Zero();
	x0[0] = 1;
	x0[2] = 4;

	//x1.reset();
	x1 = state::Zero();
	x1[0] = 4;
	x1[2] = 1;

#elif (DYNAMICS == NONHOLONOMIC)
	control_penalty = 1;
	control_penalty1 = 50;

	x_bounds[0] = std::make_pair(0,100);
	x_bounds[1] = std::make_pair(0,100);
	x_bounds[2] = std::make_pair(-M_PI,M_PI);
	x_bounds[3] = std::make_pair(0.01,10);
	x_bounds[4] = std::make_pair(-0.25,0.25);

	u_bounds[0] = std::make_pair(-DBL_MAX,DBL_MAX);
	u_bounds[1] = std::make_pair(-DBL_MAX,DBL_MAX);

	//B.reset();
	B = Eigen::Matrix<float,X_DIM,U_DIM>::Zero();
	B(3,0) = 1;
	B(4,1) = 1;

	//R.reset();
	R = Eigen::Matrix<float,U_DIM,U_DIM>::Zero();
	R(0,0) = control_penalty;
	R(1,1) = control_penalty1;

	//x0.reset();
	x0 = state::Zero();
	x0[2] = -M_PI/2;		
	x0[3] = 1;
	
	//x1.reset();
	x1 = state::Zero();
	x1[2] = -M_PI/2;
	x1[3] = 1;


#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	control_penalty = 0.25;

	x_bounds[0] = std::make_pair(0, 100);
	x_bounds[1] = std::make_pair(0, 100);
	x_bounds[2] = std::make_pair(-10, 10);
	x_bounds[3] = std::make_pair(-10, 10);

	u_bounds[0] = std::make_pair(-10, 10);
	u_bounds[1] = std::make_pair(-10, 10);

	//A.reset();
	A = Eigen::Matrix<float,X_DIM,X_DIM>::Zero();
	A(0,2) = 1;
	A(1,3) = 1;

	//B.reset();
	B = Eigen::Matrix<float,X_DIM,U_DIM>::Zero();
	B(2,0) = 1;
	B(3,1) = 1;

	//c = zeros<X_DIM>();
	c = state::Zero();

	//R = eye<U_DIM>();
	R = Eigen::Matrix<float,U_DIM,U_DIM>::Identity();

	/* 2.99034 */
	//x0.reset();
	x0 = state::Zero();

	//x1.reset();
	x1 = state::Zero();
	x1[0] = 0;
	x1[1] = 100;

	/* 2.95112
	//x0.reset();
	x0 = state::Zero();
	x0[0] = x0[1] = 1;
	x0[3] = 1;

	//x1.reset();
	x1 = state::Zero();
	x1[0] = x1[1] = 2;
	x1[2] = 1;
	*/

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
	x_bounds[0] = std::make_pair(0, 100);
	x_bounds[1] = std::make_pair(0, 100);
	//x_bounds[2] = std::make_pair(-10, 10);
	//x_bounds[3] = std::make_pair(-10, 10);

	u_bounds[0] = std::make_pair(-10, 10);
	u_bounds[1] = std::make_pair(-10, 10);

	//A.reset();
	A = Eigen::Matrix<float,X_DIM,X_DIM>::Zero();

	//B.reset();
	B = Eigen::Matrix<float,X_DIM,U_DIM>::Zero();
	B(0,1) = 1;
	B(1,1) = 1;

	//c = zeros<X_DIM>();
	c = state::Zero();

	//R = eye<U_DIM>();
	R = Eigen::Matrix<float,U_DIM,U_DIM>::Identity();

	//x0.reset();
	x0 = state::Zero();
	//x0[2] = 1;

	//x1.reset();
	x1 = state::Zero();
	x1[0] = 100;
	x1[1] = 100;
	//x1[3] = 0.5;

#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	x_bounds[0] = std::make_pair(0, 100);
	x_bounds[1] = std::make_pair(-10, 10);

	u_bounds[0] = std::make_pair(-10, 10);

	//A.reset();
	A = Eigen::Matrix<float,X_DIM,X_DIM>::Zero();
	A(0,1) = 1;

	//B.reset();
	B = Eigen::Matrix<float,X_DIM,U_DIM>::Zero();
	B(1,0) = 1;

	//c = zeros<X_DIM>();
	c = state::Zero();
	//R = eye<U_DIM>();
	R = Eigen::Matrix<float,U_DIM,U_DIM>::Zero();

	//x0.reset();
	x0 = state::Zero();
	//x1.reset();
	x1 = state::Zero();
	x1[0] = 100;

#else
    dynamicsError();
#endif
}

void setupRobot() {
	int obj;

	CAL_CreateGroup(&robot_model, 0, false, "Robot Model");
	CAL_CreateGroup(&robot_group, 0, true, "Robot Group");

#if DYNAMICS == QUADROTOR
	// Visualization parameters
	double beamWidth     = 0.015; // m
	double beamHeight    = 0.0065; // m
	double beamRadius    = 0.02; // m
	double motorRadius   = 0.015; // m
	double motorHeight   = 0.02; // m
	double rotorRadius   = 0.10; // m
	double rotorHeight   = 0.005; // m
	double centerSide    = 0.0889; // m
	double centerHeight  = 0.0365; // m
	double centerTopSide = 0.03; // m
	double flagLength    = 0.0508; // m
	double tileSize      = 1;  // m

	CAL_CreateCylinder(robot_group, length+rotorRadius, length/2.0, 0, 0, 0, &robot_object);
	CAL_SetGroupOrientation(robot_group, M_PI/2.0, 0, 0);

	// Quadrotor
	CAL_SetGroupColor(robot_model, 0.05, 0.05, 0.05);
	CAL_CreateBox(robot_model, 2*length, beamWidth, beamHeight, 0, 0, 0);
	CAL_CreateBox(robot_model, beamWidth, 2*length, beamHeight, 0, 0, 0);
	CAL_CreateCylinder(robot_model, motorRadius, motorHeight, length, 0, beamHeight / 2 + motorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, motorRadius, motorHeight, -length, 0, beamHeight / 2 + motorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, motorRadius, motorHeight, 0, length, beamHeight / 2 + motorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, motorRadius, motorHeight, 0, -length, beamHeight / 2 + motorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, beamRadius, beamHeight, length, 0, 0, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, beamRadius, beamHeight, -length, 0, 0, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, beamRadius, beamHeight, 0, length, 0, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, beamRadius, beamHeight, 0, -length, 0, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_CreateCylinder(robot_model, rotorRadius, rotorHeight, length, 0, beamHeight / 2 + motorHeight + rotorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_SetObjectColor(obj, 0, 0, 0, 0.1);
	CAL_CreateCylinder(robot_model, rotorRadius, rotorHeight, -length, 0, beamHeight / 2 + motorHeight + rotorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_SetObjectColor(obj, 0, 0, 0, 0.1);
	CAL_CreateCylinder(robot_model, rotorRadius, rotorHeight, 0, length, beamHeight / 2 + motorHeight + rotorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_SetObjectColor(obj, 0, 0, 0, 0.1);
	CAL_CreateCylinder(robot_model, rotorRadius, rotorHeight, 0, -length, beamHeight / 2 + motorHeight + rotorHeight / 2, &obj);
	CAL_SetObjectOrientation(obj, (float) (M_PI*0.5), 0, 0);
	CAL_SetObjectColor(obj, 0, 0, 0, 0.1);
	CAL_CreateBox(robot_model, centerSide, centerSide, beamHeight, 0, 0, 0, &obj);
	CAL_SetObjectOrientation(obj, 0, 0, (float) (M_PI*0.25));
	CAL_CreateBox(robot_model, flagLength, beamWidth + 0.001, beamHeight + 0.001, length / 1.65, 0, 0, &obj);
	CAL_SetObjectColor(obj, 1, 0.15, 0);

	float flagTriangle[18] = {length / 1.65 - flagLength / 2, 0, -beamHeight / 2,
		length / 1.65, 0, -beamHeight / 2 - flagLength / 2,
		length / 1.65 + flagLength / 2, 0, -beamHeight / 2,
		length / 1.65 + flagLength / 2, 0, -beamHeight / 2,
		length / 1.65, 0, -beamHeight / 2 - flagLength / 2,
		length / 1.65 - flagLength / 2, 0, -beamHeight / 2};
	CAL_CreateTriangles(robot_model, 2, flagTriangle, &obj);
	//CAL_SetObjectColor(obj, 1, 0.15, 0);
	CAL_SetObjectColor(obj, 1,1,1);

	float polygon1[18] = {-sqrt(2.0)*centerSide/2, 0, 0,
		-sqrt(2.0)*centerSide/2+centerHeight, 0, centerHeight,
		sqrt(2.0)*centerSide/2-centerHeight, 0, centerHeight,
		sqrt(2.0)*centerSide/2, 0, 0,
		sqrt(2.0)*centerSide/2-centerHeight, 0, -centerHeight,
		-sqrt(2.0)*centerSide/2+centerHeight, 0, -centerHeight};
	CAL_CreatePolygon(robot_model, 6, polygon1, &obj);
	CAL_SetObjectColor(obj, 0.15, 0.15, 0.15);
	float polygon2[18] = {-sqrt(2.0)*centerSide/2, 0, 0,
		-sqrt(2.0)*centerSide/2+centerHeight, 0, -centerHeight,
		sqrt(2.0)*centerSide/2-centerHeight, 0, -centerHeight,
		sqrt(2.0)*centerSide/2, 0, 0,
		sqrt(2.0)*centerSide/2-centerHeight, 0, centerHeight,
		-sqrt(2.0)*centerSide/2+centerHeight, 0, centerHeight};
	CAL_CreatePolygon(robot_model, 6, polygon2, &obj);
	//CAL_SetObjectColor(obj, 0.15, 0.15, 0.15);
	CAL_SetObjectColor(obj,1,0,0);
	float polygon3[18] = {0, -sqrt(2.0)*centerSide/2, 0,
		0, -sqrt(2.0)*centerSide/2+centerHeight, centerHeight,
		0, sqrt(2.0)*centerSide/2-centerHeight, centerHeight,
		0, sqrt(2.0)*centerSide/2, 0,
		0, sqrt(2.0)*centerSide/2-centerHeight, -centerHeight,
		0, -sqrt(2.0)*centerSide/2+centerHeight, -centerHeight};
	CAL_CreatePolygon(robot_model, 6, polygon3, &obj);
	CAL_SetObjectColor(obj, 0.15, 0.15, 0.15);
	float polygon4[18] = {0, -sqrt(2.0)*centerSide/2, 0,
		0, -sqrt(2.0)*centerSide/2+centerHeight, -centerHeight,
		0, sqrt(2.0)*centerSide/2-centerHeight, -centerHeight,
		0, sqrt(2.0)*centerSide/2, 0,
		0, sqrt(2.0)*centerSide/2-centerHeight, centerHeight,
		0, -sqrt(2.0)*centerSide/2+centerHeight, centerHeight};
	CAL_CreatePolygon(robot_model, 6, polygon4, &obj);
	CAL_SetObjectColor(obj, 0.15, 0.15, 0.15);
#elif (DYNAMICS == NONHOLONOMIC)
    CAL_CreateBox(robot_group, 5, 3, 2.5, 2.5, 1.5, 0);

	CAL_CreateBox(robot_model, 5, 3, 2.5, 2.5, 1.5, 0);
	CAL_SetGroupColor(robot_model, 0.05, 0.05, 0.05);
#else
	CAL_CreateCylinder(robot_group, 5, 3, 0, 0, 0, &robot_object);

	CAL_CreateCylinder(robot_model, 5, 3, 0, 0, 0);
	CAL_SetGroupColor(robot_model, 0.05, 0.05, 0.05);
#endif

	CAL_SetGroupVisibility(robot_group, 0, false, true);

#ifdef SHOW_ROBOT
	CAL_SetGroupVisibility(robot_model, 0, true, true);
#else
	CAL_SetGroupVisibility(robot_model, 0, false, false);
#endif
}

void makeWall(float x, float y, float z, int length, int height, bool vertical, bool reverse) {
#if (DYNAMICS == QUADROTOR)
	double width = (vertical ? length : 0);
	double depth = (vertical ? 0 : length);
	double x_pos = (reverse ? x - width/2 : x + width/2);
	double y_pos = (reverse ? y - depth/2 : y + depth/2);
	double z_pos = z + height/2.0;
	double xw = width;
	double yw = depth;
	double 	zw = height;
#else
	double width = (vertical ? length : 0.5);
	double depth = (vertical ? 0.5 : length);
	double x_pos = (reverse ? y - width/2 : y + width/2);
	double y_pos = z + height/2.0;
	double z_pos = (reverse ? x - depth/2 : x + depth/2);
	double xw = width;
	double yw = height;
	double zw = depth;
#endif

	CAL_CreateBox(obstacle_group, xw, yw, zw, x_pos, y_pos, z_pos);
}

void buildEnvironment() {
#if (DYNAMICS == QUADROTOR)
	CAL_CreateGroup(&border_group, 0, false, "Borders");
	CAL_SetGroupColor(border_group, 0.25, 0.25, 0.25);

	int nl = 1;
	int np[1] = {2};
	float left_x = x_bounds[0].first;
	float right_x = x_bounds[0].second;
	float back_y = x_bounds[1].first;
	float front_y = x_bounds[1].second;
	float bottom_z = x_bounds[2].first;
	float top_z = x_bounds[2].second;
	float p[6];

#define MAKE_LINE(x1, y1, z1, x2, y2, z2) \
	p[0] = x1; \
	p[1] = y1; \
	p[2] = z1; \
	p[3] = x2; \
	p[4] = y2; \
	p[5] = z2; \
	CAL_CreatePolyline(border_group, 1, np, p);

	MAKE_LINE(left_x, back_y, bottom_z, right_x, back_y, bottom_z);
	MAKE_LINE(right_x, back_y, bottom_z, right_x, back_y, top_z);
	MAKE_LINE(right_x, back_y, top_z, left_x, back_y, top_z);
	MAKE_LINE(left_x, back_y, top_z, left_x, back_y, bottom_z);
	
	MAKE_LINE(left_x, front_y, bottom_z, left_x, back_y, bottom_z);
	MAKE_LINE(right_x, front_y, bottom_z, right_x, back_y, bottom_z);
	MAKE_LINE(right_x, front_y, top_z, right_x, back_y, top_z);
	MAKE_LINE(left_x, front_y, top_z, left_x, back_y, top_z);
	
	MAKE_LINE(left_x, front_y, bottom_z, right_x, front_y, bottom_z);
	MAKE_LINE(right_x, front_y, bottom_z, right_x, front_y, top_z);
	MAKE_LINE(right_x, front_y, top_z, left_x, front_y, top_z);
	MAKE_LINE(left_x, front_y, top_z, left_x, front_y, bottom_z);
	
	// Make & reset matrices
	Matrix<6> bottom_segment, top_segment, left_segment, right_segment, window;
	bottom_segment.reset();
	top_segment.reset();
	left_segment.reset();
	right_segment.reset();
	window.reset();

	// Get environment stats
	double total_height = x_bounds[0].second - x_bounds[0].first;
	double total_depth = x_bounds[1].second - x_bounds[1].first;
	double window_height = 1.0, window_width = 2.0;

#define BUILD_SEGMENTS() \
	bottom_segment[3] = 0.1; \
	bottom_segment[4] = total_depth; \
	bottom_segment[5] = window[2] - window[5]/2.0 - x_bounds[2].first; \
	bottom_segment[0] = window[0]; \
	bottom_segment[2] = bottom_segment[5]/2.0; \
	\
	top_segment[3] = bottom_segment[3]; \
	top_segment[4] = total_depth; \
	top_segment[5] = x_bounds[2].second - (window[2] + window[5]/2.0); \
	top_segment[0] = window[0]; \
	top_segment[2] = top_segment[5]/2.0 + window[2] + window[5]/2.0; \
	\
	right_segment[3] = bottom_segment[3]; \
	right_segment[4] = abs(x_bounds[1].second - (window[1] + window[4]/2.0)); \
	right_segment[5] = total_height - top_segment[5] - bottom_segment[5]; \
	right_segment[0] = window[0]; \
	right_segment[1] = window[1] + window[4]/2.0 + right_segment[4]/2.0; \
	right_segment[2] = bottom_segment[5] + right_segment[5]/2.0; \
	\
	left_segment[3] = bottom_segment[3]; \
	left_segment[4] = total_depth - window[4] - right_segment[4]; \
	left_segment[5] = right_segment[5]; \
	left_segment[0] = window[0]; \
	left_segment[1] = window[1] - window[4]/2.0 - left_segment[4]/2.0; \
	left_segment[2] = right_segment[2]; \
	\
	CAL_CreateBox(obstacle_group, bottom_segment[3], bottom_segment[4], bottom_segment[5], bottom_segment[0], bottom_segment[1], bottom_segment[2]); \
	CAL_CreateBox(obstacle_group, top_segment[3], top_segment[4], top_segment[5], top_segment[0], top_segment[1], top_segment[2]); \
	CAL_CreateBox(obstacle_group, left_segment[3], left_segment[4], left_segment[5], left_segment[0], left_segment[1], left_segment[2]); \
	CAL_CreateBox(obstacle_group, right_segment[3], right_segment[4], right_segment[5], right_segment[0], right_segment[1], right_segment[2]);

#define CALCULATE_WINDOW_AREA(window, bounds) \
	bounds.resize(X_DIM); \
	bounds[0] = make_pair(window[0] - window[3]/2.0, window[0] + window[3]/2.0); \
	bounds[1] = make_pair(window[1] - window[4]/2.0, window[1] + window[4]/2.0); \
	bounds[2] = make_pair(window[2] - window[5]/2.0, window[2] + window[5]/2.0); \
	for (int CALCULATE_WINDOW_AREA_i = 3; CALCULATE_WINDOW_AREA_i < X_DIM; CALCULATE_WINDOW_AREA_i++) { \
	  bounds[CALCULATE_WINDOW_AREA_i] = x_bounds[CALCULATE_WINDOW_AREA_i]; \
	} /*\
	CAL_CreateBox(obstacle_group, window[3], window[4], window[5], window[0], window[1], window[2]);*/

#if USE_OBSTACLES == 5

	x0[0] = 2.5;
	x0[1] = -1;
	x0[2] = 4;
	x1[0] = -2.5;
	x1[1] = 1;
	x1[2] = 1;


	// Position window
	window[0] = 1.0; // x pos
	window[1] = 1.0; // y pos
	window[2] = 1.0; // z pos
	window[3] = 1.0; // width - this is really controlling the width of the sample area
	window[4] = 1.0; // depth
	window[5] = 0.5; // height

	CALCULATE_WINDOW_AREA(window, x_bounds_window_1);

	BUILD_SEGMENTS();

	// Position window
	window[0] = -1.0; // x pos
	window[1] = -1.0; // y pos
	window[2] = 4.0; // z pos

	CALCULATE_WINDOW_AREA(window, x_bounds_window_2);

	BUILD_SEGMENTS();

#elif USE_OBSTACLES == 4
	double wall_start_x = 0.0, wall_start_y = x_bounds[1].first, wall_start_z = x_bounds[2].first;
	double wall_length = (x_bounds[0].second - x_bounds[0].first), wall_height = (x_bounds[2].second - x_bounds[2].first)/3.0;
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height + 1, 0, 0, (wall_height + 1)/2.0);
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height, 0, 0, x_bounds[2].second - wall_height/2.0);
	double segment_height = (x_bounds[2].second - x_bounds[2].first) - (wall_height + 1) - wall_height;
 	CAL_CreateBox(obstacle_group, 0.1, wall_length/3.0, segment_height, 0, 2*x_bounds[1].second/3.0, wall_height + 1 + segment_height/2.0);
 	CAL_CreateBox(obstacle_group, 0.1, wall_length/3.0, segment_height, 0, 2*x_bounds[1].first/3.0, wall_height + 1 + segment_height/2.0);

#elif USE_OBSTACLES == 3
	double wall_start_x = 0.0, wall_start_y = x_bounds[1].first, wall_start_z = x_bounds[2].first;
	double wall_length = (x_bounds[0].second - x_bounds[0].first), wall_height = (x_bounds[2].second - x_bounds[2].first)/3.0;
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height + 1, 0, 0, (wall_height + 1)/2.0);
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height, 0, 0, x_bounds[2].second - wall_height/2.0);
	double segment_height = (x_bounds[2].second - x_bounds[2].first) - (wall_height + 1) - wall_height;
 	CAL_CreateBox(obstacle_group, 0.1, wall_length/4.0, segment_height, 0, 3*x_bounds[1].first/4.0, wall_height + 1 + segment_height/2.0);
 	CAL_CreateBox(obstacle_group, 0.1, wall_length/4.0, segment_height, 0, -3*x_bounds[1].first/4.0, wall_height + 1 + segment_height/2.0);

#elif USE_OBSTACLES == 2
	double wall_start_x = 0.0, wall_start_y = x_bounds[1].first, wall_start_z = x_bounds[2].first;
	double wall_length = (x_bounds[0].second - x_bounds[0].first), wall_height = (x_bounds[2].second - x_bounds[2].first)/3.0;
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height + 1, 0, 0, (wall_height + 1)/2.0);
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height, 0, 0, x_bounds[2].second - wall_height/2.0);
	double segment_height = (x_bounds[2].second - x_bounds[2].first) - (wall_height + 1) - wall_height;
	CAL_CreateBox(obstacle_group, 0.1, 3*wall_length/4.0, segment_height, 0, x_bounds[1].first/4.0, wall_height + 1 + segment_height/2.0);

#elif USE_OBSTACLES == 1
	double wall_start_x = 0.0, wall_start_y = x_bounds[1].first, wall_start_z = x_bounds[2].first;
	double wall_length = (x_bounds[0].second - x_bounds[0].first), wall_height = 3*(x_bounds[2].second - x_bounds[2].first)/4.0;
	CAL_CreateBox(obstacle_group, 0.1, wall_length, wall_height, 0, 0, wall_height/2.0);
#endif

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == NONHOLONOMIC)
#if USE_OBSTACLES == 6
#if (DYNAMICS == NONHOLONOMIC)
	x_bounds[0] = std::make_pair(0, 200);
	x_bounds[1] = std::make_pair(0, 100);

	x0[1] = 44;
	x0[0] = 10;
	x1[1] = 60;
	x1[0] = 10;

	double offset = 0;
	makeWall(-10, -10, offset, 120, 20, true, false);
	makeWall(-10, -10, offset, 220, 20, false, false);
	makeWall(210, -10, offset, 120, 20, true, false);
	makeWall(-10, 110, offset, 220, 20, false, false);

	CAL_CreateBox(obstacle_group, 50, 20, 50, 50, 10, 50);

	int temp;
	CAL_CreateGroup(&temp, 0, false, "Faux hole");
	CAL_SetGroupColor(temp, 1, 1, 1);
	CAL_SetGroupVisibility(temp, 0, true, true);
	CAL_CreateBox(temp, 49.25, 22, 49.25, 50, 10, 50);

	makeWall(-10, 50, offset, 135, 20, false, false);
	makeWall(210, 50, offset, 60, 20, false, true);

	makeWall(125, 25, offset, 50, 20, true, false);
	makeWall(125, 25, offset, 50, 20, false, false);
	makeWall(125, 75, offset, 50, 20, false, false);

	makeWall(100, -10, offset, 35, 20, true, false);
	makeWall(100, 110, offset, 35, 20, true, true);
#else
	x_bounds[0] = std::make_pair(0, 100);
	x_bounds[1] = std::make_pair(0, 200);

	x0[1] = 15;
	x0[0] = 50;
	x1[1] = 140;
	x1[0] = 50;
	double offset = 0;

    // Make bounding box	
	makeWall(-10, -10, offset, 120, 20, true, false);
	makeWall(-10, -10, offset, 220, 20, false, false);
	makeWall(210, -10, offset, 120, 20, true, false);
	makeWall(-10, 110, offset, 220, 20, false, false);
	/*
	CAL_CreateBox(obstacle_group, 50, 20, 50, 50, 10, 50);

	int temp;
	CAL_CreateGroup(&temp, 0, false, "Faux hole");
	CAL_SetGroupColor(temp, 1, 1, 1);
	CAL_SetGroupVisibility(temp, 0, true, true);
	CAL_CreateBox(temp, 49.25, 22, 49.25, 50, 10, 50);
	*/

    // Build long center dividing wall	
	makeWall(25, 50, offset, 100, 20, false, false);

	// Build top S-curve
	makeWall(25, 50, offset, 25, 20, true, false);
	makeWall(50, 110, offset, 35, 20, true, true);
	makeWall(75, 50, offset, 25, 20, true, false);
	makeWall(100, 110, offset, 35, 20, true, true);

    // Build bottom S-curve	
	makeWall(50, -10, offset, 35, 20, true, false);
	makeWall(75, 50, offset, 25, 20, true, true);
	makeWall(100, -10, offset, 35, 20, true, false);

	// Build box around goal on left	
	makeWall(125, 25, offset, 50, 20, true, false);
	makeWall(125, 25, offset, 50, 20, false, false);
	makeWall(125, 75, offset, 50, 20, false, false);
	makeWall(210, 50, offset, 60, 20, false, true);
#endif
#elif USE_OBSTACLES == 5

#if (DYNAMICS == NONHOLONOMIC)
	x_bounds[0] = std::make_pair(0, 200);
	x_bounds[1] = std::make_pair(0, 100);

	x0[1] = 44;
	x0[0] = 10;
	x1[1] = 56;
	x1[0] = 10;
#else
	x_bounds[0] = std::make_pair(0, 100);
	x_bounds[1] = std::make_pair(0, 200);

	x0[0] = 44;
	x0[1] = 10;
	x1[0] = 56;
	x1[1] = 10;
#endif

	int obstacle_height = 5;
	double offset = 0;
	makeWall(-10, -10, offset, 120, obstacle_height, true, false);
	makeWall(-10, -10, offset, 220, obstacle_height, false, false);
	makeWall(210, -10, offset, 120, obstacle_height, true, false);
	makeWall(-10, 110, offset, 220, obstacle_height, false, false);

	CAL_CreateBox(obstacle_group, 50, obstacle_height, 50, 50, 2.5, 50);
	CAL_CreateBox(obstacle_group, 50, obstacle_height, 50, 50, 2.5, 150);
	CAL_CreateBox(obstacle_group, 10, obstacle_height, 50, 70, 2.5, 100);

	makeWall(100, -10, offset, 50, obstacle_height, true, false);
	makeWall(-10, 50, offset, 40, obstacle_height, false, false);
	//makeWall(10, 10, offset, 80, obstacle_height, true, false);
	//makeWall(10, 10, offset, 30, obstacle_height, true, true);

#elif USE_OBSTACLES == 4
	makeWall(-20, -20, 0, 140, 20, true, false);
	makeWall(-20, -20, 0, 140, 20, false, false);
	makeWall(120, -20, 0, 140, 20, true, false);
	makeWall(-20, 120, 0, 140, 20, false, false);

	makeWall(25, -20, 0, 90, 20, true, false);
	makeWall(75, 120, 0, 90, 20, true, true);

	x1[0] = x1[1] = 100;

#elif USE_OBSTACLES == 3
	makeWall(-20, -20, 0, 140, 20, true, false);
	makeWall(-20, -20, 0, 140, 20, false, false);
	makeWall(120, -20, 0, 140, 20, true, false);
	makeWall(-20, 120, 0, 140, 20, false, false);

	makeWall(25, -20, 0, 70, 20, true, false);
	makeWall(75, 120, 0, 70, 20, true, true);

	x1[0] = x1[1] = 100;

#elif USE_OBSTACLES == 2
	makeWall(-20, -20, 0, 140, 20, true, false);
	makeWall(-20, -20, 0, 140, 20, false, false);
	makeWall(120, -20, 0, 140, 20, true, false);
	makeWall(-20, 120, 0, 140, 20, false, false);

	makeWall(50, -20, 0, 50, 20, true, false);
	makeWall(-20, 50, 0, 50, 20, false, false);
	makeWall(120, 50, 0, 50, 20, false, true);
	makeWall(50, 120, 0, 50, 20, true, true);
#elif USE_OBSTACLES == 1
	CAL_CreateCylinder(obstacle_group, 5, 10, 25, 5, 25);
	CAL_CreateCylinder(obstacle_group, 5, 10, 50, 5, 25);
	CAL_CreateCylinder(obstacle_group, 5, 10, 75, 5, 25);
	CAL_CreateCylinder(obstacle_group, 5, 10, 25, 5, 50);
	CAL_CreateCylinder(obstacle_group, 5, 10, 25, 5, 75);
	CAL_CreateCylinder(obstacle_group, 5, 10, 75, 5, 75);

	x1[0] = x1[1] = 100;
#endif
#endif
}

void setupVisualization(const state& x0, const state& x1) {
	// visualization
	CAL_Initialisation(true, true, true);

#if (DYNAMICS != QUADROTOR)
	CAL_SetViewOptions(0, CAL_ORTHOPROJ);
#endif

	/*
	int temp;
	CAL_CreateGroup(&temp, 0, false);
	CAL_SetGroupColor(temp, 1, 0, 0);
	CAL_CreateSphere(temp, 0.25, 0, 0, 0);
	int nl = 1;
	int np[1] = {2};
	CAL_scalar p[6] = {0, 0, 0, 0, 0, 10};
	CAL_CreatePolyline(temp, nl, np, p);
	p[3] = 10.0;
	p[5] = 0.0;
	CAL_CreatePolyline(temp, nl, np, p);
	*/

#if defined(EXPERIMENT)
	CAL_SuspendVisualisation();
#endif

	CAL_CreateGroup(&collision_hit_group, 0, false, "Collision hit");
	CAL_CreateGroup(&collision_free_group, 0, false, "Collision free");
	CAL_CreateGroup(&obstacle_group, 0, true, "Obstacle");
	CAL_CreateGroup(&solution_group, 0, false, "Solution");
	CAL_CreateGroup(&solution_marker_group, 0, false, "Solution Way Points");
	CAL_CreateGroup(&node_group, 0, false, "Nodes");
	CAL_CreateGroup(&edge_group, 0, false, "Edges");
	CAL_CreateGroup(&velocity_group, 0, false, "Velocities");
	
	CAL_SetGroupVisibility(node_group, 0, false, true);
	CAL_SetGroupVisibility(edge_group, 0, SHOW_TREE, true);
	CAL_SetGroupVisibility(velocity_group, 0, SHOW_TREE, true);
	CAL_SetGroupVisibility(solution_group, 0, true, true);
#if (USE_OBSTACLES > 0)
	CAL_SetGroupVisibility(obstacle_group, 0, true, true);
#else
	CAL_SetGroupVisibility(obstacle_group, 0, false, true);
#endif

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
	CAL_SetGroupColor(robot_group, 0, 0, 0);
#if defined(MAKE_STILLS) && (DYNAMICS != QUADROTOR)
	CAL_SetGroupColor(obstacle_group, 0.1, 0.1, 0.1, 1);
#else
	CAL_SetGroupColor(obstacle_group, 0.1, 0.1, 0.1, 0.1);
#endif
	CAL_SetGroupColor(solution_group, 0, 0, 1);
	CAL_SetGroupColor(solution_marker_group, 0, 0, 1);
	CAL_SetGroupColor(velocity_group, 0, 0, 1);
	CAL_SetGroupColor(edge_group, 0.65, 0.16, 0.16);

	buildEnvironment();

	setupRobot();

	// Position the camera based on the state bounds
	double eye_x = 0.0, eye_y = 0.0, eye_z = 0.0;
	double camera_x = 0.0, camera_y = 0.0, camera_z = 0.0;
	double up_x = 1.0, up_y = 0.0, up_z = 0.0;

#if (DYNAMICS == QUADROTOR)
	up_x = 0.0;
	up_z = 1.0;
/* isometric
	eye_x = -x_bounds[0].second;
	eye_y = 2*x_bounds[1].second;
	eye_z = 2*x_bounds[2].second;
	camera_x = x_bounds[0].second/2.0;
	camera_y = x_bounds[1].first/2.0;
*/

/* side
	eye_y = 3*x_bounds[1].second;
	eye_z = x_bounds[2].second/2.0;
	camera_z = x_bounds[2].second/2.0;
*/

/* above */
	eye_z = 3*x_bounds[2].second;
	up_y = 1;
	up_x = 0;
#elif (DYNAMICS == NONHOLONOMIC)
	camera_x = eye_x = x_bounds[1].second/2.0;
	eye_y = 150;
	camera_z = eye_z = x_bounds[0].second/2.0;
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	camera_x = eye_x = x_bounds[0].second/2.0;
	eye_y = 150;
	camera_z = eye_z = x_bounds[1].second/2.0;
#else
	camera_x = eye_x = x_bounds[0].second/2.0;
	eye_y = 125;
	camera_z = eye_z = x_bounds[1].second/2.0;
#endif

	CAL_SetViewParams(0, eye_x, eye_y, eye_z, camera_x, camera_y, camera_z, up_x, up_y, up_z);

	double start_x = 0.0, start_y = 0.0, start_z = 0.0;
	double goal_x = 0.0, goal_y = 0.0, goal_z = 0.0;

#if (DYNAMICS == QUADROTOR)
	start_x = x0[0]; goal_x = x1[0];
	start_y = x0[1]; goal_y = x1[1];
	start_z = x0[2]; goal_z = x1[2];
#elif (DYNAMICS == NONHOLONOMIC)
	start_x = x0[1]; goal_x = x1[1];
	start_z = x0[0]; goal_z = x1[0];
#else
	start_x = x0[0]; goal_x = x1[0];
	start_z = x0[1]; goal_z = x1[1];
#endif 

	// Position the robot
	CAL_SetGroupPosition(robot_model, start_x, start_y, start_z);

#if (DYNAMICS == NONHOLONOMIC)
	double rot = x0[2] - 0.5*M_PI;
	while (rot < 0) rot += 2*M_PI;

	CAL_SetGroupOrientation(robot_model, 0, rot, 0);
	CAL_SetGroupOrientation(robot_group, 0, rot, 0);
#endif

#ifdef SHOW_PATHS
	// Setup the start and goal nodes
	CAL_CreateGroup(&start_node_group, 0, false, "Start");
	CAL_CreateGroup(&goal_node_group, 0, false, "Goal");
	CAL_SetGroupColor(start_node_group, 1, 0, 0);
	CAL_SetGroupColor(goal_node_group, 0, 1, 0);
	CAL_CreateSphere(start_node_group, 5*NODE_SIZE, start_x, start_y, start_z);
	CAL_CreateSphere(goal_node_group, 5*NODE_SIZE, goal_x, goal_y, goal_z);
#endif
}
/*
inline Matrix<4,1> quatFromRot(const Matrix<3,3>& R) {
	Matrix<4,1> q;

	q[0] = 0.5*sqrt(max(1+R(0,0)-R(1,1)-R(2,2),0.0))*(R(2,1) - R(1,2) >= 0 ? 1 : -1);
	q[1] = 0.5*sqrt(max(1-R(0,0)+R(1,1)-R(2,2),0.0))*(R(0,2) - R(2,0) >= 0 ? 1 : -1);
	q[2] = 0.5*sqrt(max(1-R(0,0)-R(1,1)+R(2,2),0.0))*(R(1,0) - R(0,1) >= 0 ? 1 : -1);
	q[3] = 0.5*sqrt(max(1+R(0,0)+R(1,1)+R(2,2),0.0));

	return q;
}
*/
void buildKeyframe(const double& t, const state& x, bool still = false, double alpha = 0.0, double offset = 0.0) {
	bool isQuat = true;
	double x_rot, y_rot;
	double x_pos = 0.0, y_pos = 0.0, z_pos = 0.0;

#if (DYNAMICS == QUADROTOR)
	x_pos = x[0];
	y_pos = x[1];
	z_pos = x[2];
#elif (DYNAMICS == NONHOLONOMIC)
	x_pos = x[1];
	z_pos = x[0];
#else
	x_pos = x[0];
	z_pos = x[1];
#endif

#if (DYNAMICS == QUADROTOR)
	//Matrix<3, 3> rot;
	//rot.reset();
	Eigen::Matrix<float,3,3> rot = Eigen::Matrix<float,3,3>::Zero();
	rot(0,2) = x[7];
	rot(1,2) = -x[6];
	rot(2,0) = -x[7];
	rot(2,1) = x[6];

	//Matrix<4,1> q = quatFromRot(exp(rot));
	Eigen::Quaternion<float> q(rot.exp());
	//float o[4] = {(float) q[0], (float) q[1], (float) q[2], (float) q[3]};
	float o[4] = {q.x(), q.y(), q.z(), q.w()};
	//float o[4] = {(float) q[1], (float) q[2], (float) q[3], (float) q[0]};
#elif (DYNAMICS == NONHOLONOMIC)
	isQuat = false;
	double rot = x[2] - 0.5*M_PI;
	while (rot < 0) rot += 2*M_PI;
	float o[3] = {0, rot, 0};
#else
	float *o = CAL_NULL;
#endif

	//cout << "Key frame (" << t << "): " << ~x << "\t" << x[6] << ": " << x_rot << endl;
	float p[3] = {(float) x_pos, (float) y_pos, (float) z_pos};

	if (still) {
		//Matrix<3, 1> current_pos;
		Eigen::Matrix<float,3,1> current_pos;
		current_pos[0] = x_pos;
		current_pos[1] = y_pos;
		current_pos[2] = z_pos;

		int new_group;

#if (DYNAMICS == QUADROTOR)
		CAL_CloneGroup(&new_group, robot_model, stills_group, false, "Stills subgroup");
		CAL_SetGroupVisibility(new_group, 0, true, true);
		CAL_SetGroupQuaternion(new_group, q[0], q[1], q[2], q[3]);
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
		int result = CAL_AddGroupKeyState(robot_model, (float) t, p, o, CAL_NULL, isQuat);
		if (CAL_SUCCESS != result) {
			cout << "Failed (" << result << ") to add key frame!" << endl;
		}
	}

#ifdef SHOW_PATHS
	CAL_CreateSphere(solution_group, 2*NODE_SIZE, x_pos, y_pos, z_pos);
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

	CAL_ClearGroupKeyStates(robot_model, true);

	Node current = tree[0];
	double cost, tau;
	state d_tau;
	vector<state> state_list;
	while (current.parent != NO_PARENT) {
		state_list.push_back(current.x);
		current = tree[current.parent];
	}
	state_list.push_back(current.x);
	reverse(state_list.begin(), state_list.end());

	state_time_list_t segment;
	double max_tau = 0.0;
	double current_time = 0.0;
	for(vector<state>::iterator p = state_list.begin(); (p + 1) != state_list.end(); p++) {
		segment.clear();
		//computeCost(*p, *(p + 1), DBL_MAX, cost, tau, d_tau);
		//checkPath(*p, *(p+1), tau, d_tau, false, &segment);

		connect(*p, *(p+1), DBL_MAX, cost, tau, &segment);

		sort(segment.begin(), segment.end(), state_order);
		for(state_time_list_t::iterator q = segment.begin(); q != segment.end(); q++) {
			current_time = (q->first) + max_tau;
			buildKeyframe(current_time, q->second);
			fwrite((const void *)&current_time, sizeof(double), 1, path_log);
			double *current_elements = (q->second._elems);
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

template <size_t _numRows>
struct lyapunov {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	//Matrix<_numRows, _numRows> *A;
	Eigen::Matrix<float,_numRows,_numRows> *A;
	//Matrix<_numRows, _numRows> *BRiBt;
	Eigen::Matrix<float,_numRows,_numRows> *BRiBt;
	//inline Matrix<_numRows, _numRows> operator()(double t, const Matrix<_numRows, _numRows>& G) const {
	inline Eigen::Matrix<float,_numRows,_numRows> operator()(double t, const Eigen::Matrix<float,_numRows,_numRows>& G) const {
		//Matrix<_numRows,_numRows> AG = (*A)*G;  // ~AG = G*~A
		Eigen::Matrix<float,_NumRows,_numRows> AG = (*A)*G;
		//return AG + ~AG + (*BRiBt); // A*G + G*~A + B*~B
		return AG + AG.transpose() +(*BRiBt);
	}
};
#ifdef FRONT_LOAD_RK4
lyapunov<X_DIM> lyap;
#endif

template <size_t _numRows>
struct linear {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	//Matrix<_numRows, _numRows> *A;
	Eigen::Matrix<float,_numRows,_numRows> *A;
	//Matrix<_numRows> *c;
	Eigen::Matrix<float,_numRows,1> *c;
	//inline Matrix<_numRows> operator()(double t, const Matrix<_numRows>& xbar) const {
	inline Eigen::Matrix<float,_numRows,1> operator()(double t, const Eigen::Matrix<float,_numRows,1>& xbar) const {
		return (*A)*xbar + (*c);
	}
};
#ifdef FRONT_LOAD_RK4
linear<X_DIM, X_DIM> diff;
linear<2*X_DIM,1> back;
#endif

template <size_t _numRows, size_t _numColumns, class Function>
class rk {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//inline void rk4(Matrix<_numRows, _numColumns>& x_0, double t_0, double dt, const Function& f) {
	inline void rk4(Eigen::Matrix<float,_numRows,_numColumns>& x_0, double t_0, double dt, const Function& f) {
		double t_1 = t_0 + dt;

		k_1 = dt*f(t_0, x_0);
		k_2 = dt*f(t_0 + 0.5*dt, x_0 + 0.5*k_1);
		k_3 = dt*f(t_0 + 0.5*dt, x_0 + 0.5*k_2);
		k_4 = dt*f(t_0 + dt, x_0 + k_3);

		x_0 += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
	}

private:
	/*Matrix<_numRows, _numColumns> k_1;
	Matrix<_numRows, _numColumns> k_2;
	Matrix<_numRows, _numColumns> k_3;
	Matrix<_numRows, _numColumns> k_4;

	Matrix<_numRows, _numColumns> k_5;
	Matrix<_numRows, _numColumns> k_6;*/
	Eigen::Matrix<float,_numRows,_numColumns> k_1,k_2,k_3,k_4,k_5,k_6;
};
#ifdef FRONT_LOAD_RK4
rk<X_DIM, 1, linear<X_DIM, X_DIM> > linearrk4;
rk<X_DIM, X_DIM, lyapunov<X_DIM> > lyaprk4;
rk<2*X_DIM, 1, linear<2*X_DIM,1> > linearrk;
#endif
/*w
template <size_t _numRows, size_t _numColumns>
class lyaprk {
public:
inline void rk4(Matrix<_numRows, _numColumns>& G, double dt) {
AG = A*G;
k_1 = dt*(AG + ~AG + BRiBt);
AG = A*(G + 0.5*k_1);
k_2 = dt*(AG + ~AG + BRiBt);
AG = A*(G + 0.5*k_2);
k_3 = dt*(AG + ~AG + BRiBt);
AG = A*(G + k_3);
k_4 = dt*(AG + ~AG + BRiBt);

G += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
}

private:
Matrix<_numRows, _numColumns> AG;
Matrix<_numRows, _numColumns> k_1;
Matrix<_numRows, _numColumns> k_2;
Matrix<_numRows, _numColumns> k_3;
Matrix<_numRows, _numColumns> k_4;
};
*/
template <size_t _numRows>
//inline double cost(double tau, const Matrix<_numRows, _numRows>& G, const Matrix<_numRows>& xbar) {
inline double cost(double tau, const Eigen::Matrix<float<_numRows,_numRows>& G, const Eigen::Matrix<float,_numRows,1>& xbar) {
	//return tau + tr(~(x1 - xbar)*(G%(x1-xbar)));
	return tau + ((x1-xbar).transpose()*G.ldlt().solve(x1-xbar)).trace();
}

template <size_t _numRows>
//inline double dcost(double tau, const Matrix<_numRows, _numRows>& G, const Matrix<_numRows>& xbar) {
inline double dcost(double tau, const Eigen::Matrix<float,_numRows,_numRows>& G, const Eigen::Matrix<float,_numRows,1>& xbar) {
	//Matrix<_numRows> d = -(G%(x1-xbar));
	//return 1 + 2*tr(~(A*x1)*d) - tr(~d*BRiBt*d);
	Eigen::Matrix<float,_numRows,1> d = -G.ldlt().solve(x1-xbar);
	return 1 + 2*(A*x1).transpose()*d).trace() - (d.transpose()*BRiBt*d).trace();
}

template<typename vec>
bool checkBounds(const vec& v, const BOUNDS& v_bounds) {
//return true; // TODO REMOVE
	for (size_t i = 0; i < v.numRows(); i++) {
		if ((v[i] < v_bounds[i].first) || (v[i] > v_bounds[i].second)) {
			return false;
		}
	}
	return true;
}

inline bool collision_free(const state& x) {
#if (USE_OBSTACLES > 0)
	int result = 0;
	int collisions = 0;
	double x_pos = 0.0, y_pos = 0.0, z_pos = 0.0;

	// Rotate the robot
#if (DYNAMICS == QUADROTOR)
	//Matrix<3, 3> rot;
	//rot.reset();
	Eigen::Matrix<float,3,3> rot = Eigen::Matrix<float,3,3>::Zero();
	rot(0,2) = x[4];
	rot(1,2) = -x[3];
	rot(2,0) = -x[4];
	rot(2,1) = x[3];

	//Matrix<4,1> q = quatFromRot(exp(rot));
	Eigen::Quaternion<float> q(rot.exp());
	//result = CAL_SetGroupQuaternion(robot_group, q[0], q[1], q[2], q[3]);
	result = CAL_SetGroupQuaternion(robot_group,q.x(),q.y(),q.z(),q.w());
	//result = CAL_SetGroupQuaternion(robot_group, q[1], q[2], q[3], q[0]);
	if (CAL_SUCCESS != result) {
		cout << "CAL_SetGroupQuaternion failed (" << result << ")" << endl;
	}
#elif (DYNAMICS == NONHOLONOMIC)
	double rot = x[2] - 0.5*M_PI;
	while (rot < 0) rot += 2*M_PI;

	CAL_SetGroupOrientation(robot_group, 0, rot, 0);
#endif

	// Position the robot
#if (DYNAMICS ==  QUADROTOR)
	x_pos = x[0];
	y_pos = x[1];
	z_pos = x[2];
#elif (DYNAMICS == NONHOLONOMIC)
	x_pos = x[1];
	z_pos = x[0];
#else
	x_pos = x[0];
	z_pos = x[1];
#endif

	result = CAL_SetGroupPosition(robot_group, x_pos, y_pos, z_pos);
	if (result != CAL_SUCCESS) {
		cout << "CAL_SetObjectPosition failed (" << result << ")." << endl;
		_getchar();
		exit(1);
	}

	// Check for collisions
	result = CAL_CheckGroupCollision(robot_group, obstacle_group, false, &collisions);
	if (result != CAL_SUCCESS) {
		cout << "CAL_CheckGroupCollision failed (" << result << ")." << endl;
		_getchar();
		exit(1);
	}

#if defined(SHOW_COLLISION_CHECKS) || defined(SHOW_COLLISIONS)

	if (collisions > 0) {
#ifdef SHOW_COLLISIONS
		CAL_CreateSphere(collision_hit_group, 3*NODE_SIZE, x_pos, y_pos, z_pos);
#endif
	} else {
#ifdef SHOW_COLLISION_CHECKS
		CAL_CreateSphere(collision_free_group, 3*NODE_SIZE, x_pos, y_pos, z_pos);
#endif
	}
#endif
	return (collisions == 0 ? true : false);
#else
	return true;
#endif
}

#ifdef INLINE_LYAP
/*Matrix<X_DIM, X_DIM> k_1;
Matrix<X_DIM, X_DIM> k_2;
Matrix<X_DIM, X_DIM> k_3;
Matrix<X_DIM, X_DIM> k_4;
Matrix<X_DIM,X_DIM> AG;*/
Eigen::Matrix<float,X_DIM,X_DIM> k_1,k_2,k_3,k_4,AG;
#endif

inline double computeHeuristic(const double x0, const double x1, const pair<double, double>& bounds) {
	double tmp = x1 - x0;
	return max(tmp/bounds.first, tmp/bounds.second);
}

inline double applyHeuristics(const state& x0, const state& x1) {
#ifndef USE_HEURISTICS
  return 0.0;
#endif

	
  // Calculate the cost
  double cost = 0.0;
#if (DYNAMICS == QUADROTOR)
	double cost1 = max(computeHeuristic(x0[0], x1[0], x_bounds[3]), computeHeuristic(x0[1], x1[1], x_bounds[4]));
	double cost2 = max(computeHeuristic(x0[2], x1[2], x_bounds[5]), computeHeuristic(x0[6], x1[6], x_bounds[8]));

	cost = max(cost1, cost2);
	cost = max(cost, computeHeuristic(x0[7], x1[7], x_bounds[9]));
#elif (DYNAMICS == NONHOLONOMIC)
  cost = sqrt(pow(x1[0] - x0[0],2) + pow(x1[1] - x0[1],2))/x_bounds[3].second;
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	double cost1 = max(computeHeuristic(x0[0], x1[0], x_bounds[2]), computeHeuristic(x0[1], x1[1], x_bounds[3]));
	double cost2 = max(computeHeuristic(x0[2], x1[2], u_bounds[0]), computeHeuristic(x0[3], x1[3], u_bounds[1]));

	cost = max(cost1, cost2);
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
	cost = max(computeHeuristic(x0[0], x1[0], u_bounds[0]), computeHeuristic(x0[1], x1[1], u_bounds[1]));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	cost = max(computeHeuristic(x0[0], x1[0], x_bounds[1]), computeHeuristic(x0[1], x1[1], u_bounds[0]));
#else
  dynamicsError();
#endif

	return cost;
}

bool computeCostClosedForm(const state& x0, const state& x1, double radius, double& cost, double& tau, state& d_tau) {
	if (applyHeuristics(x0, x1) > radius) return false;

#ifdef _DEBUG_COMPUTE_COST
	cout << "~~~~~~~~~~~~~~~~~~~~~~~ computeCostClosedForm ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
#endif

	//Matrix<X_DIM> x1diffbarx0;
	//Matrix<X_DIM> d;
	Eigen::Matrix<float,X_DIM,1> x1diffbarx0,d;

	int degree = POLY_DEGREE;
	double p[POLY_DEGREE + 1];
	double zeror[POLY_DEGREE], zeroi[POLY_DEGREE];

#if (DYNAMICS == QUADROTOR)
	p[0] = pow(gravity, 0.2e1) * pow(length, 0.2e1);
	p[1] = 0;
	p[2] = (-pow(mass, 0.2e1) * control_penalty * x1[5] * pow(gravity, 0.2e1) * pow(length, 0.2e1) * x0[5] - 0.8e1 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(x0[9], 0.2e1) - 0.8e1 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(x0[8], 0.2e1) - pow(mass, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(length, 0.2e1) * pow(x0[5], 0.2e1) - 0.4e1 * pow(inertia, 0.2e1) * control_penalty * x1[8] * pow(gravity, 0.2e1) * x0[8] - 0.8e1 * pow(inertia, 0.2e1) * control_penalty * pow(x1[8], 0.2e1) * pow(gravity, 0.2e1) - pow(mass, 0.2e1) * control_penalty * pow(x1[5], 0.2e1) * pow(gravity, 0.2e1) * pow(length, 0.2e1) - 0.4e1 * pow(inertia, 0.2e1) * control_penalty * x1[9] * pow(gravity, 0.2e1) * x0[9] - 0.8e1 * pow(inertia, 0.2e1) * control_penalty * pow(x1[9], 0.2e1) * pow(gravity, 0.2e1));
	p[3] = (-0.6e1 * pow(mass, 0.2e1) * control_penalty * x1[5] * pow(gravity, 0.2e1) * pow(length, 0.2e1) * x0[2] + 0.240e3 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x1[7] * x1[9] + 0.120e3 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x1[7] * x0[9] + 0.120e3 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x1[6] * x0[8] - 0.6e1 * pow(mass, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(length, 0.2e1) * x0[2] * x0[5] - 0.120e3 * pow(inertia, 0.2e1) * control_penalty * x1[9] * pow(gravity, 0.2e1) * x0[7] + 0.6e1 * pow(mass, 0.2e1) * control_penalty * x1[5] * pow(gravity, 0.2e1) * pow(length, 0.2e1) * x1[2] - 0.240e3 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x0[7] * x0[9] + 0.6e1 * pow(mass, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(length, 0.2e1) * x1[2] * x0[5] + 0.240e3 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x1[6] * x1[8] - 0.240e3 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x0[6] * x0[8] - 0.120e3 * pow(inertia, 0.2e1) * control_penalty * x1[8] * pow(gravity, 0.2e1) * x0[6]);
	p[4] = (0.2520e4 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x1[7] * x0[7] - 0.1800e4 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(x0[6], 0.2e1) - 0.9e1 * pow(mass, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(length, 0.2e1) * pow(x1[2], 0.2e1) - 0.1800e4 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(x1[6], 0.2e1) - 0.1440e4 * pow(inertia, 0.2e1) * control_penalty * x0[3] * gravity * x0[9] - 0.9e1 * pow(mass, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(length, 0.2e1) * pow(x0[2], 0.2e1) + 0.1440e4 * pow(inertia, 0.2e1) * control_penalty * x0[4] * gravity * x0[8] + 0.18e2 * pow(mass, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(length, 0.2e1) * x1[2] * x0[2] + 0.1080e4 * pow(inertia, 0.2e1) * control_penalty * x1[8] * gravity * x0[4] - 0.1080e4 * pow(inertia, 0.2e1) * control_penalty * x1[3] * gravity * x0[9] + 0.1440e4 * pow(inertia, 0.2e1) * control_penalty * x1[4] * gravity * x1[8] + 0.1080e4 * pow(inertia, 0.2e1) * control_penalty * x1[4] * gravity * x0[8] - 0.1800e4 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(x0[7], 0.2e1) + 0.2520e4 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * x1[6] * x0[6] - 0.1440e4 * pow(inertia, 0.2e1) * control_penalty * x1[3] * gravity * x1[9] - 0.1080e4 * pow(inertia, 0.2e1) * control_penalty * x1[9] * gravity * x0[3] - 0.1800e4 * pow(inertia, 0.2e1) * control_penalty * pow(gravity, 0.2e1) * pow(x1[7], 0.2e1));
	p[5] = (0.18720e5 * pow(inertia, 0.2e1) * control_penalty * gravity * x1[7] * x0[3] - 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x0[0] * gravity * x0[9] + 0.18720e5 * pow(inertia, 0.2e1) * control_penalty * x1[4] * gravity * x0[6] + 0.21600e5 * pow(inertia, 0.2e1) * control_penalty * x0[4] * gravity * x0[6] - 0.18720e5 * pow(inertia, 0.2e1) * control_penalty * x1[3] * gravity * x0[7] - 0.18720e5 * pow(inertia, 0.2e1) * control_penalty * gravity * x1[6] * x0[4] + 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x1[9] * gravity * x1[0] + 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x1[8] * gravity * x0[1] - 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x1[8] * gravity * x1[1] + 0.21600e5 * pow(inertia, 0.2e1) * control_penalty * x1[3] * gravity * x1[7] - 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x1[1] * gravity * x0[8] - 0.21600e5 * pow(inertia, 0.2e1) * control_penalty * x1[4] * gravity * x1[6] + 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x0[1] * gravity * x0[8] - 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x1[9] * gravity * x0[0] - 0.21600e5 * pow(inertia, 0.2e1) * control_penalty * x0[3] * gravity * x0[7] + 0.3360e4 * pow(inertia, 0.2e1) * control_penalty * x1[0] * gravity * x0[9]);
	p[6] = (0.50400e5 * pow(inertia, 0.2e1) * control_penalty * gravity * x1[7] * x0[0] - 0.122400e6 * pow(inertia, 0.2e1) * control_penalty * x1[4] * x0[4] - 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * gravity * x1[6] * x0[1] - 0.122400e6 * pow(inertia, 0.2e1) * control_penalty * x1[3] * x0[3] - 0.64800e5 * pow(inertia, 0.2e1) * control_penalty * pow(x0[3], 0.2e1) - 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * x0[0] * gravity * x0[7] + 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * gravity * x1[6] * x1[1] + 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * x0[1] * gravity * x0[6] - 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * gravity * x1[7] * x1[0] - 0.64800e5 * pow(inertia, 0.2e1) * control_penalty * pow(x1[4], 0.2e1) - 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * x1[1] * gravity * x0[6] - 0.64800e5 * pow(inertia, 0.2e1) * control_penalty * pow(x0[4], 0.2e1) - 0.64800e5 * pow(inertia, 0.2e1) * control_penalty * pow(x1[3], 0.2e1) + 0.50400e5 * pow(inertia, 0.2e1) * control_penalty * x1[0] * gravity * x0[7]);
	p[7] = (-0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x0[0] * x0[3] + 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x1[4] * x1[1] - 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x1[3] * x0[0] + 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x1[1] * x0[4] - 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x1[4] * x0[1] + 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x1[0] * x0[3] + 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x1[3] * x1[0] - 0.302400e6 * pow(inertia, 0.2e1) * control_penalty * x0[1] * x0[4]);
	p[8] = - 0.352800e6 * pow(inertia, 0.2e1) * control_penalty * pow(x1[0], 0.2e1) - 0.352800e6 * pow(inertia, 0.2e1) * control_penalty * pow(x1[1], 0.2e1) - 0.352800e6 * pow(inertia, 0.2e1) * control_penalty * pow(x0[1], 0.2e1) + 0.705600e6 * pow(inertia, 0.2e1) * control_penalty * x1[0] * x0[0] + 0.705600e6 * pow(inertia, 0.2e1) * control_penalty * x1[1] * x0[1] - 0.352800e6 * pow(inertia, 0.2e1) * control_penalty * pow(x0[0], 0.2e1);

	/*
      MapleGenVar3 = g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*_Z*_Z+(-8.0*j*j*r*g*g*x0_8*x0_8-m~*m*r*g*g*l*l*x0_5*x0_5-4.0*j*j*r*x1_8*g*g*x0_8-m*m*r*x1_5*g*g*l*l*x0_5-8.0*j*j~*r*x1_8*x1_8*g*g-4.0*j*j*r*x1_9*g*g*x0_9-8.0*j*j*r*x1_9*x1_9*g*g-m*m*r*x1_5*x1_5~*g*g*l*l-8.0*j*j*r*g*g*x0_9*x0_9)*_Z*_Z*_Z*_Z*_Z*_Z+(-120.0*j*j*r*x1_8*g*g*x0_6+120.0*j*j*r*g*g*x1_7*x0_9+240.0*j*j*r*g*g*x1_7*x1_9-6.0*m*m*r*x1_5*g*g*l*l~*x0_2-6.0*m*m*r*g*g*l*l*x0_2*x0_5+240.0*j*j*r*g*g*x1_6*x1_8-120.0*j*j*r*x1_9*g~*g*x0_7+6.0*m*m*r*g*g*l*l*x1_2*x0_5+6.0*m*m*r*x1_5*g*g*l*l*x1_2-240.0*j*j*r*g*g*x0_6*x0_8+120.0*j*j*r*g*g*x1_6*x0_8-240.0*j*j*r*g*g*x0_7*x0_9)*_Z*_Z*_Z*_Z*_Z;
      MapleGenVar4 = MapleGenVar3+(2520.0*j*j*r*g*g*x1_7*x0_7+18.0*m*m*r*g*g*l*l~*x1_2*x0_2-1800.0*j*j*r*g*g*x1_7*x1_7-1440.0*j*j*r*x0_3*g*x0_9-1080.0*j*j*r*x1_3~*g*x0_9+1080.0*j*j*r*x1_8*g*x0_4+1440.0*j*j*r*x0_4*g*x0_8-9.0*m*m*r*g*g*l*l~*x0_2*x0_2-9.0*m*m*r*g*g*l*l*x1_2*x1_2-1440.0*j*j*r*x1_3*g*x1_9-1800.0*j*j*r*g~*g*x0_7*x0_7+1440.0*j*j*r*x1_4*g*x1_8+1080.0*j*j*r*x1_4*g*x0_8-1800.0*j*j*r*g*g*x0_6*x0_6-1080.0*j*j*r*x1_9*g*x0_3+2520.0*j*j*r*g*g*x1_6*x0_6-1800.0*j*j*r*g*g*x1_6*x1_6)*_Z*_Z*_Z*_Z;
      MapleGenVar5 = MapleGenVar4;
      MapleGenVar7 = (-18720.0*j*j*r*x1_3*g*x0_7-3360.0*j*j*r*x0_0*g*x0_9+18720.0*j*j*r*g*x1_7*x0_3+18720.0*j*j*r*x1_4*g*x0_6-3360.0*j*j*r*x1_8*g*x1_1+3360.0*j*j*r*x0_1*g*x0_8+3360.0*j*j*r*x1_8*g*x0_1+21600.0*j*j*r*x1_3*g*x1_7-21600.0*j*j*r*x1_4*g*x1_6+3360.0*j*j*r*x1_9*g*x1_0+21600.0*j*j*r*x0_4*g*x0_6-18720.0*j*j*r*g*x1_6*x0_4-3360.0*j*j*r*x1_1*g*x0_8+3360.0*j*j*r*x1_0*g*x0_9-3360.0*j*j*r*x1_9*g*x0_0-21600.0*j*j*r*x0_3*g*x0_7)*_Z*_Z*_Z;
      MapleGenVar8 = (-50400.0*j*j*r*g*x1_6*x0_1-122400.0*j*j*r*x1_3*x0_3-50400.0*j*j*r*x1_1*g*x0_6-64800.0*j*j*r*x1_3*x1_3-64800.0*j*j*r*x0_4*x0_4-64800.0*j*j*r*x0_3*x0_3-50400.0*j*j*r*x0_0*g*x0_7+50400.0*j*j*r*g*x1_6*x1_1-122400.0*j*j*r*x1_4*x0_4+50400.0*j*j*r*x0_1*g*x0_6+50400.0*j*j*r*g*x1_7*x0_0-64800.0*j*j*r*x1_4*x1_4-50400.0*j*j*r*g*x1_7*x1_0+50400.0*j*j*r*x1_0*g*x0_7)*_Z*_Z;
      MapleGenVar6 = MapleGenVar7+MapleGenVar8;
      MapleGenVar2 = MapleGenVar5+MapleGenVar6;
      MapleGenVar1 = MapleGenVar2+(-302400.0*j*j*r*x1_3*x0_0+302400.0*j*j*r*x1_4~*x1_1+302400.0*j*j*r*x1_0*x0_3-302400.0*j*j*r*x0_0*x0_3+302400.0*j*j*r*x1_1~*x0_4+302400.0*j*j*r*x1_3*x1_0-302400.0*j*j*r*x1_4*x0_1-302400.0*j*j*r*x0_1~*x0_4)*_Z-352800.0*j*j*r*x1_1*x1_1-352800.0*j*j*r*x0_1*x0_1+705600.0*j*j*r*x1_1*x0_1-352800.0*j*j*r*x1_0*x1_0-352800.0*j*j*r*x0_0*x0_0+705600.0*j*j*r*x1_0~*x0_0;
      t1 = 0.0;
	  */

#elif (DYNAMICS == NONHOLONOMIC)
	double px = x0[0];
	double py = x0[1];
	double th = x0[2];
	double v = x0[3];
	double ka = x0[4];
	double px2 = x1[0];
	double py2 = x1[1];
	double th2 = x1[2];
	double v2 = x1[3];
	double ka2 = x1[4];
	double rv = control_penalty;
	double rk = control_penalty1;
	double cth = cos(th);
	double c2th = cos(2*th);
	double sth = sin(th);
	double s2th = sin(2*th);

	p[6] = -1800*(pow(px - px2,2) + pow(py - py2,2))*rk*rv +
		1800*(px - px2 + py - py2)*(px - px2 - py + py2)*rk*rv*c2th +
		3600*(px - px2)*(py - py2)*rk*rv*s2th;
	p[5] = (2880*py*rk*rv*th*v*cth - 2880*py2*rk*rv*th*v*cth -
		2880*py*rk*rv*th2*v*cth + 2880*py2*rk*rv*th2*v*cth -
		2880*px*rk*rv*th*v*sth + 2880*px2*rk*rv*th*v*sth +
		2880*px*rk*rv*th2*v*sth - 2880*px2*rk*rv*th2*v*sth);
	p[4] = (-18*pow(ka,4)*(pow(px - px2,2) + pow(py - py2,2))*pow(rk,2) -
		36*pow(ka,2)*pow(rk,2)*pow(th,2) +
		72*pow(ka,2)*pow(rk,2)*th*th2 -
		36*pow(ka,2)*pow(rk,2)*pow(th2,2) -
		36*pow(ka,2)*(pow(px - px2,2) + pow(py - py2,2))*rk*rv*
		pow(v,2) - 576*rk*rv*pow(th,2)*pow(v,2) +
		1152*rk*rv*th*th2*pow(v,2) - 576*rk*rv*pow(th2,2)*pow(v,2) -
		18*(pow(px - px2,2) + pow(py - py2,2))*pow(rv,2)*pow(v,4) +
		72*ka*(px - px2)*rk*(th - th2)*(pow(ka,2)*rk + rv*pow(v,2))*
		cth - 360*py*rk*rv*v*(2*ka*v - ka2*v - ka*v2)*cth +
		360*py2*rk*rv*v*(2*ka*v - ka2*v - ka*v2)*cth -
		18*(px - px2 + py - py2)*(px - px2 - py + py2)*
		pow(pow(ka,2)*rk + rv*pow(v,2),2)*c2th +
		72*ka*(py - py2)*rk*(th - th2)*(pow(ka,2)*rk + rv*pow(v,2))*
		sth + 360*px*rk*rv*v*(2*ka*v - ka2*v - ka*v2)*sth -
		360*px2*rk*rv*v*(2*ka*v - ka2*v - ka*v2)*sth -
		36*(px - px2)*(py - py2)*pow(pow(ka,2)*rk + rv*pow(v,2),2)*
		s2th);
	p[3] = (24*pow(ka,3)*pow(rk,2)*th*v - 24*pow(ka,2)*ka2*pow(rk,2)*th*v -
		24*pow(ka,3)*pow(rk,2)*th2*v +
		24*pow(ka,2)*ka2*pow(rk,2)*th2*v - 144*ka2*rk*rv*th*pow(v,3) +
		144*ka2*rk*rv*th2*pow(v,3) +
		24*ka*rk*rv*(th - th2)*pow(v,2)*(11*v - 5*v2) -
		24*(px - px2)*v*(pow(ka,2)*rk + rv*pow(v,2))*
		(ka*(ka - ka2)*rk + rv*v*(v + v2))*cth -
		24*(py - py2)*v*(pow(ka,2)*rk + rv*pow(v,2))*
		(ka*(ka - ka2)*rk + rv*v*(v + v2))*sth);
	p[2] = (-4*pow(ka,4)*pow(rk,2)*pow(v,2) +
		8*pow(ka,3)*ka2*pow(rk,2)*pow(v,2) -
		4*pow(ka,2)*pow(ka2,2)*pow(rk,2)*pow(v,2) -
		9*pow(ka2,2)*rk*rv*pow(v,4) +
		2*ka*ka2*rk*rv*pow(v,3)*(14*v - 5*v2) +
		pow(ka,2)*rk*rv*pow(v,2)*
		(-28*pow(v,2) + 16*v*v2 - 9*pow(v2,2)) -
		4*pow(rv,2)*pow(v,4)*(pow(v,2) + v*v2 + pow(v2,2)));
	p[1] = 0;
	p[0] = (pow(ka,2)*rk*pow(v,2) + rv*pow(v,4));

#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	p[0] = 1;
	p[1] = 0;
	p[2] = -4.0*control_penalty*x1[3]*x1[3]-4.0*control_penalty*x0[3]*x0[3]-4.0*control_penalty*x1[2]*x1[2]-4.0*control_penalty*x1[3]*x0[3]-4.0*control_penalty*x1[2]*x0[2]-4.0*control_penalty*x0[2]*x0[2];
	p[3] = 24.0*control_penalty*x1[1]*x0[3]+24.0*control_penalty*x1[0]*x0[2]-24.0*control_penalty*x0[0]*x0[2]-24.0*control_penalty*x0[1]*x0[3]+24.0*control_penalty*x1[2]*x1[0]-24.0*control_penalty*x1[2]*x0[0]+24.0*control_penalty*x1[3]*x1[1]-24.0*control_penalty*x1[3]*x0[1];
	p[4] = -36.0*control_penalty*x1[0]*x1[0]-36.0*control_penalty*x0[0]*x0[0]-36.0*control_penalty*x1[1]*x1[1]-36.0*control_penalty*x0[1]*x0[1]+72.0*control_penalty*x1[0]*x0[0]+72.0*control_penalty*x1[1]*x0[1];
	
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
	double t1 = x1[0]*x1[0];
	double t6 = x0[0]*x0[0];
	double t8 = x1[1]*x1[1];
	double t13 = x0[1]*x0[1];
	p[0] = 1;
	p[1] = 0;
	p[2] = -(control_penalty*t1-2.0*control_penalty*x1[0]*x0[0]+control_penalty*t6+control_penalty*t8-2.0*control_penalty*x1[1]*x0[1]+control_penalty*t13);
	
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	p[0] = 1;
	p[1] = 0;
	p[2] = (-4.0*control_penalty*x1[1]*x1[1]-4.0*control_penalty*x1[1]*x0[1]-4.0*control_penalty*x0[1]*x0[1]);
	p[3] = (24.0*control_penalty*x1[0]*x0[1]+24.0*control_penalty*x1[1]*x1[0]-24.0*control_penalty*x1[1]*x0[0]-24.0*control_penalty*x0[0]*x0[1]);
	p[4] = -36.0*control_penalty*x1[0]*x1[0]+72.0*control_penalty*x1[0]*x0[0]-36.0*control_penalty*x0[0]*x0[0];

#else
	dynamicsError();
#endif

#ifdef _DEBUG_COMPUTE_COST
	cout << "polynomial: " << f << endl;
#endif

	// TODO DWEBB clean this up!
	std::vector<std::complex<double> > complexRoots;
	std::vector<double> realRoots;

	memset(zeror, 0, sizeof(double)*degree);
	memset(zeroi, 0, sizeof(double)*degree);
	int info[1000];
	int returned_roots = rpoly(p, degree, zeror, zeroi, info);

	realRoots.clear();
	for (int i = 0; i < returned_roots; i++) {
		if (zeroi[i] != 0) {
		} else if (zeror[i] >= 0) {
			realRoots.push_back(zeror[i]);
		}
	}

	bool result = false;
	double minTau = radius;
	double minCost = radius;
	state mind;
	//mind.reset();
	mind = state::Zero();

	for (size_t i = 0; i < realRoots.size();++i) {

		if (!((realRoots[i] > 0.0) && (realRoots[i] < minCost))) {
			continue;
		}

#ifdef _DEBUG_COMPUTE_COST
		cout << "LOOP ____________________________" << endl;
		cout << "\tx0: " << ~x0 << "\tx1: " << ~x1 << endl;
		cout << "\trealRoots[" << i << "]: " << setprecision(12) <<  realRoots[i] << "\tf(realRoots[" << i << "]: " << f(realRoots[i]) << endl;
#endif

#if (DYNAMICS == QUADROTOR)
		double j = inertia;
		double g = gravity;
		double m = mass;
		double r = control_penalty; // This is the r parameter to matrix R -- refer to maple
		double l = length;
		double t2 = realRoots[i]*realRoots[i];
		double t3 = g*t2;
		double t6 = t2*realRoots[i];
		double t7 = g*t6;
		double t19 = g*realRoots[i];
		double t35 = g*g;
		double t36 = 1/t35;
		double t37 = l*l;
		double t38 = 1/t37;
		double t40 = j*j;
		double t41 = t36*t38*t40;
		double t42 = t2*t2;
		double t45 = r/t42/t6;
		double t52 = 1/t42/t2*t36*t38;
		double t53 = t40*r;
		double t57 = 1/g;
		double t59 = 1/t42/realRoots[i];
		double t61 = t57*t59*t38;
		double t67 = t57/t42*t38;
		double t85 = m*m;
		double t86 = t85*r;
		double t87 = 1/t6;
		double t91 = 1/t2;
		double t92 = t91*t85;
		double t100 = r*t59;
		double t107 = t57*t87*t38;
		double t125 = 1/realRoots[i];
		double t133 = t38*t40;
		double t134 = r*t87;
		double t138 = t91*t38;
		double t158 = r*t125;

		x1diffbarx0[0] = x1[0]-x0[0]-realRoots[i]*x0[3]-t3*x0[7]/2.0-t7*x0[9]/6.0;
		x1diffbarx0[1] = x1[1]-x0[1]-realRoots[i]*x0[4]+t3*x0[6]/2.0+t7*x0[8]/6.0;
		x1diffbarx0[2] = x1[2]-x0[2]-realRoots[i]*x0[5];
		x1diffbarx0[3] = x1[3]-x0[3]-t19*x0[7]-t3*x0[9]/2.0;
		x1diffbarx0[4] = x1[4]-x0[4]+t19*x0[6]+t3*x0[8]/2.0;
		x1diffbarx0[5] = x1[5]-x0[5];
		x1diffbarx0[6] = x1[6]-x0[6]-realRoots[i]*x0[8];
		x1diffbarx0[7] = x1[7]-x0[7]-realRoots[i]*x0[9];
		x1diffbarx0[8] = x1[8]-x0[8];
		x1diffbarx0[9] = x1[9]-x0[9];

		double t54 = t53*x1diffbarx0[3];
		double t62 = t53*x1diffbarx0[7];
		double t68 = t53*x1diffbarx0[9];
		double t75 = t53*x1diffbarx0[4];
		double t78 = t53*x1diffbarx0[6];
		double t81 = t53*x1diffbarx0[8];
		double t97 = t53*x1diffbarx0[0];
		double t111 = t53*x1diffbarx0[1];

		d[0] = 50400.0*t41*t45*x1diffbarx0[0]-25200.0*t52*t54+5040.0*t61*t62-420.0*t67*t68;
		d[1] = 50400.0*t41*t45*x1diffbarx0[1]-25200.0*t52*t75-5040.0*t61*t78+420.0*t67*t81;
		d[2] = 3.0*t86*t87*x1diffbarx0[2]-3.0/2.0*t92*r*x1diffbarx0[5];
		d[3] = -25200.0*t52*t97+12960.0*t41*t100*x1diffbarx0[3]-2700.0*t67*t62+240.0*t107*t68;
		d[4] = -25200.0*t52*t111+12960.0*t41*t100*x1diffbarx0[4]+2700.0*t67*t78-240.0*t107*t81;
		d[5] = -3.0/2.0*t92*r*x1diffbarx0[2]+t86*t125*x1diffbarx0[5];
		d[6] = -5040.0*t61*t111+2700.0*t67*t75+600.0*t133*t134*x1diffbarx0[6]-60.0*t138*t81;
		d[7] = 5040.0*t61*t97-2700.0*t67*t54+600.0*t133*t134*x1diffbarx0[7]-60.0*t138*t68;
		d[8] = 420.0*t67*t111-240.0*t107*t75-60.0*t138*t78+8.0*t133*t158*x1diffbarx0[8];
		d[9] = -420.0*t67*t97+240.0*t107*t54-60.0*t138*t62+8.0*t133*t158*x1diffbarx0[9];

#elif (DYNAMICS == NONHOLONOMIC)
		double tau = realRoots[i];
		x1diffbarx0[0] = -px + px2 + (tau*v*(-2*cth + ka*tau*v*sth))/2.;
		x1diffbarx0[1] = -py + py2 - (tau*v*(ka*tau*v*cth + 2*sth))/2.;
		x1diffbarx0[2] = -th + th2 - ka*tau*v;
		x1diffbarx0[3] = -v + v2;
		x1diffbarx0[4] = -ka + ka2;

		d[0] = (-6*((px - px2)*(pow(ka,4)*pow(rk,2)*pow(tau,2) + pow(rv,2)*pow(tau,2)*pow(v,4) + 2*rk*rv*(30 + pow(ka,2)*pow(tau,2)*pow(v,2))) +
			pow(tau,2)*(pow(ka,2)*rk + rv*pow(v,2))*(pow(ka,2)*rk*tau*v - ka*rk*(2*th - 2*th2 + ka2*tau*v) + rv*tau*pow(v,2)*(v + v2))*cth +
			(px - px2)*(pow(ka,4)*pow(rk,2)*pow(tau,2) + pow(rv,2)*pow(tau,2)*pow(v,4) + 2*rk*rv*(-30 + pow(ka,2)*pow(tau,2)*pow(v,2)))*c2th +
			10*rk*rv*tau*v*(6*th - 6*th2 - 2*ka*tau*v + ka2*tau*v + ka*tau*v2)*sth +
			(py - py2)*(pow(ka,4)*pow(rk,2)*pow(tau,2) + pow(rv,2)*pow(tau,2)*pow(v,4) + 2*rk*rv*(-30 + pow(ka,2)*pow(tau,2)*pow(v,2)))*s2th))/
			(pow(tau,5)*pow(v,2)*(pow(ka,2)*rk + rv*pow(v,2)));

		d[1] = (-6*
			((py - py2)*(pow(ka,4)*pow(rk,2)*pow(tau,2) + pow(rv,2)*pow(tau,2)*pow(v,4) + 2*rk*rv*(30 + pow(ka,2)*pow(tau,2)*pow(v,2))) +
			10*rk*rv*tau*v*(-6*th + 6*th2 + 2*ka*tau*v - ka2*tau*v - ka*tau*v2)*cth -
			(py - py2)*(pow(ka,4)*pow(rk,2)*pow(tau,2) + pow(rv,2)*pow(tau,2)*pow(v,4) + 2*rk*rv*(-30 + pow(ka,2)*pow(tau,2)*pow(v,2)))*c2th +
			pow(tau,2)*(pow(ka,2)*rk + rv*pow(v,2))*(pow(ka,2)*rk*tau*v - ka*rk*(2*th - 2*th2 + ka2*tau*v) + rv*tau*pow(v,2)*(v + v2))*sth +
			(px - px2)*(pow(ka,4)*pow(rk,2)*pow(tau,2) + pow(rv,2)*pow(tau,2)*pow(v,4) + 2*rk*rv*(-30 + pow(ka,2)*pow(tau,2)*pow(v,2)))*s2th))/
			(pow(tau,5)*pow(v,2)*(pow(ka,2)*rk + rv*pow(v,2)));

		d[2] = (6*rk*
			(tau*(pow(ka,3)*rk*tau*v - pow(ka,2)*rk*(2*th - 2*th2 + ka2*tau*v) - 2*rv*pow(v,2)*(16*th - 16*th2 + 3*ka2*tau*v) +
			ka*rv*tau*pow(v,2)*(11*v - 5*v2)) + 2*(30*(py - py2)*rv*v + ka*(px - px2)*tau*(pow(ka,2)*rk + rv*pow(v,2)))*cth +
			2*(30*(-px + px2)*rv*v + ka*(py - py2)*tau*(pow(ka,2)*rk + rv*pow(v,2)))*sth))/(pow(tau,4)*pow(v,2)*(pow(ka,2)*rk + rv*pow(v,2)));
		
		d[3] = (rv*(tau*v*(5*ka*rk*(6*th - 6*th2 + ka2*tau*v) + 2*rv*tau*pow(v,2)*(v + 2*v2) + pow(ka,2)*rk*tau*(-8*v + 9*v2)) +
			6*(10*ka*(-py + py2)*rk + pow(ka,2)*(px - px2)*rk*tau*v + (px - px2)*rv*tau*pow(v,3))*cth +
			6*(10*ka*(px - px2)*rk + pow(ka,2)*(py - py2)*rk*tau*v + (py - py2)*rv*tau*pow(v,3))*sth))/(pow(tau,3)*(pow(ka,2)*rk*v + rv*pow(v,3)));

		d[4] = (rk*(tau*(-4*pow(ka,3)*rk*tau*v + 9*rv*pow(v,2)*(4*th - 4*th2 + ka2*tau*v) + 2*pow(ka,2)*rk*(3*th - 3*th2 + 2*ka2*tau*v) +
			ka*rv*tau*pow(v,2)*(-14*v + 5*v2)) + 6*(10*(-py + py2)*rv*v - ka*(px - px2)*tau*(pow(ka,2)*rk + rv*pow(v,2)))*cth +
			6*(10*(px - px2)*rv*v - ka*(py - py2)*tau*(pow(ka,2)*rk + rv*pow(v,2)))*sth))/(pow(tau,3)*(pow(ka,2)*rk*v + rv*pow(v,3)));

#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
		double t7 = realRoots[i]*realRoots[i];
		double t10 = control_penalty/t7/realRoots[i];
		double t14 = 1/t7*control_penalty;
		double t26 = control_penalty/realRoots[i];
		x1diffbarx0[0] = x1[0]-x0[0]-realRoots[i]*x0[2];
		x1diffbarx0[1] = x1[1]-x0[1]-realRoots[i]*x0[3];
		x1diffbarx0[2] = x1[2]-x0[2];
		x1diffbarx0[3] = x1[3]-x0[3];
		d[0] = 12.0*t10*x1diffbarx0[0]-6.0*t14*x1diffbarx0[2];
		d[1] = 12.0*t10*x1diffbarx0[1]-6.0*t14*x1diffbarx0[3];
		d[2] = -6.0*t14*x1diffbarx0[0]+4.0*t26*x1diffbarx0[2];
		d[3] = -6.0*t14*x1diffbarx0[1]+4.0*t26*x1diffbarx0[3];
		
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
		double t4 = control_penalty/realRoots[i];
		x1diffbarx0[0] = x1[0]-x0[0];
		x1diffbarx0[1] = x1[1]-x0[1];
		d[0] = t4*x1diffbarx0[0];
		d[1] = t4*x1diffbarx0[1];
		
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
		double t4 = realRoots[i]*realRoots[i];
		double t11 = control_penalty/t4;
		x1diffbarx0[0] = x1[0]-x0[0]-realRoots[i]*x0[1];
		x1diffbarx0[1] = x1[1]-x0[1];
		d[0] = 12.0*control_penalty/t4/realRoots[i]*x1diffbarx0[0]-6.0*t11*x1diffbarx0[1];
		d[1] = -6.0*t11*x1diffbarx0[0]+4.0*control_penalty/realRoots[i]*x1diffbarx0[1];
		
#else
        dynamicsError();
#endif

#ifdef _DEBUG_COMPUTE_COST
		cout << "\tradius: " << radius << endl;
		cout << "\tx1diffbarx0: " << ~x1diffbarx0;
		cout << "\td: " << ~d;
#endif

		double current = realRoots[i] + tr(~x1diffbarx0*d);
#ifdef _DEBUG_COMPUTE_COST
		std::cout << "\tcurrent: " << current << "\tminCost: " << minCost << std::endl;
#endif
		if ((realRoots[i] > 0.0) && (current < minCost)) {
			mind = d;
			minTau = realRoots[i];
			minCost = current;
			result = true;
		}
	}

	d_tau = mind;
	cost = minCost;
	tau = minTau;

	return result;
}

bool computeCostRK4(const state& x0, const state& x1, double radius, double& cost, double& tau, state& d_tau) {
	if (applyHeuristics(x0, x1) > radius) return false;

	//Matrix<X_DIM, X_DIM> G = zeros<X_DIM, X_DIM>();
	Eigen::Matrix<float,X_DIM,X_DIM> G = Eigen::Matrix<float,X_DIM,X_DIM>::Zero();
	state xbar = x0;
	cost = radius;
	double t = 0;

#ifndef FRONT_LOAD_RK4
	linear<X_DIM> diff;
	diff.A = &A;
	diff.c = &c;
	rk<X_DIM, X_DIM, lyapunov<X_DIM> > lyaprk4;

	lyapunov<X_DIM> lyap;
	lyap.A = &A;
	lyap.BRiBt = &BRiBt;
	rk<X_DIM, 1, linear<X_DIM> > linearrk4;

	//lyaprk<X_DIM, X_DIM> lyaprk4;
#endif

	while (t < cost) {
#ifdef INLINE_LYAP
		AG = A*G;
		//k_1 = deltaT*(AG + ~AG + BRiBt);
		k1 = deltaT*(AG + AG.transpose() + BRiBt);
		AG = A*(G + 0.5*k_1);
		//k_2 = deltaT*(AG + ~AG + BRiBt);
		k_2 = deltaT*(AG + AG.transpose() + BRiBt);
		AG = A*(G + 0.5*k_2);
		//k_3 = deltaT*(AG + ~AG + BRiBt);
		k_3 = deltaT*(AG + AG.transpose() + BRiBt);
		AG = A*(G + k_3);
		//k_4 = deltaT*(AG + ~AG + BRiBt);
		k_4 = deltaT*(AG + AG.transpose() + BRiBt);

		G += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
#else
		lyaprk4.rk4(G, 0, deltaT, lyap);
		//lyaprk4.rk4(G, deltaT);
#endif
		linearrk4.rk4(xbar, 0, deltaT, diff);

		t += deltaT;

		//state d = (G%(x1 - xbar));
		state d = G.ldlt().solve(x1-xbar);
		//double cost_t = t + tr(~(x1 - xbar)*d);
		double cost_t = t + ((x1-xbar).transpose()*d).trace();
		if (cost_t > t && cost_t < cost) {
			cost = cost_t;
			tau = t;
			d_tau = d;
		}
	}

	bool result = false;
	if (cost < radius) result = true;

#ifdef CHECK_CLOSED_FORM
	double cf_cost;
	double cf_tau;
	state cf_dtau;
	bool cf_result = computeCostClosedForm(x0, x1, radius, cf_cost, cf_tau, cf_dtau);

	bool resultMatch = (cf_result == result);
	bool costMatch = (cf_cost == cost);
	bool tauMatch = (cf_tau == tau);
	bool dtauMatch = (cf_dtau == d_tau);
	cout << "Summary\n________________________\n\tx0: " << setprecision(10) << ~x0 << "\tx1: " << ~x1 << "\n\tResults (" << result << ", " << cf_result << "): " << resultMatch << "\n\tCosts (" << cost << ", " << cf_cost << "): " << costMatch << "\n\tTaus (" << tau << ", " << cf_tau << "): " << tauMatch << "\n\tD_taus (" << ~d_tau << ", " << ~cf_dtau << "): " << dtauMatch << endl;

	if (!resultMatch || !costMatch || !tauMatch || !dtauMatch) {
		int k;
		cin >> k;
	}
#endif

	return result;
}

bool checkState(const state& x, const control& u) {
BOUNDS x_bounds_real = x_bounds;
#if (DYNAMICS == NONHOLONOMIC)
	x_bounds_real[2].first = -DBL_MAX;
	x_bounds_real[2].second = DBL_MAX;
#endif
	return checkBounds(x, x_bounds_real) && checkBounds(u, u_bounds) && collision_free(x);
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
		double *current_elements = (q->second._elems);
		for (int i = 0; i < X_DIM; i++) {
			current_time = current_elements[i];
		}
	}
}

bool checkPathClosedForm(const state& x0, const state& x1, const double tau, const state& d_tau, const bool plot, state_time_list_t* vis) {
	double t;
	double bound = (plot ? deltaT * 8 : deltaT);
	int numPoints = ceil(tau / bound);
	int step = 1;
	while (step < numPoints) step *= 2;

	for ( ; step > 1; step /= 2) {
		for (int i = step / 2; i < numPoints; i += step) {
			t = (tau*i)/numPoints;

#if (DYNAMICS == QUADROTOR)
			double j = inertia;
			double g = gravity;
			double m = mass;
			double r = control_penalty; // This is the r parameter to matrix R -- refer to maple
			double l = length;			
			double t1 = t-tau;
			double t3 = t*t;
			double t4 = g*t3;
			double t6 = g*t;
			double t8 = tau*tau;
			double t11 = t4/2.0-t6*tau+g*t8/2.0;
			double t13 = t3*t;
			double t20 = t8*tau;
			double t23 = g*t13/6.0-t4*tau/2.0+t6*t8/2.0-g*t20/6.0;
			double t25 = g*g;
			double t26 = l*l;
			double t27 = t25*t26;
			double t28 = t3*t3;
			double t30 = t28*t3;
			double t33 = t28*t;
			double t38 = t8*t8;
			double t41 = t38*tau;
			double t44 = t38*t8;
			double t49 = t27*(t28*t13-7.0*t30*tau+21.0*t33*t8-35.0*t28*t20+35.0*t13*t38-21.0*t3*t41+7.0*t*t44-t38*t20);
			double t50 = j*j;
			double t51 = 1/t50;
			double t52 = 1/r;
			double t53 = t51*t52;
			double t54 = t53*d_tau[0];
			double t68 = t27*(t30-6.0*t33*tau+15.0*t28*t8-20.0*t13*t20+15.0*t3*t38-6.0*t*t41+t44);
			double t69 = t53*d_tau[3];
			double t72 = g*t26;
			double t81 = t33-5.0*t28*tau+10.0*t13*t8-10.0*t3*t20+5.0*t*t38-t41;
			double t82 = t72*t81;
			double t83 = t53*d_tau[7];
			double t93 = t72*(t28-4.0*t13*tau+6.0*t3*t8-4.0*t*t20+t38);
			double t94 = t53*d_tau[9];
			double t101 = t53*d_tau[1];
			double t104 = t53*d_tau[4];
			double t107 = t53*d_tau[6];
			double t110 = t53*d_tau[8];
			double t119 = t13-3.0*t3*tau+3.0*t*t8-t20;
			double t120 = m*m;
			double t121 = 1/t120;
			double t123 = t52*d_tau[2];
			double t128 = t3-2.0*t*tau+t8;
			double t129 = t128*t121;
			double t135 = t6-g*tau;
			double t140 = t27*t81;
			double t145 = t72*t119;
			double t172 = t26*t119;
			double t175 = t26*t128;
			double t192 = t26*t51;
			double t193 = t52*t1;
			chi[0] = x1[0]+t1*x1[3]+t11*x1[7]+t23*x1[9]-t49*t54/2520.0+t68*t69/360.0-t82*t83/60.0+t93*t94/12.0;
			chi[1] = x1[1]+t1*x1[4]-t11*x1[6]-t23*x1[8]-t49*t101/2520.0+t68*t104/360.0+t82*t107/60.0-t93*t110/12.0;
			chi[2] = x1[2]+t1*x1[5]-2.0/3.0*t119*t121*t123+2.0*t129*t52*d_tau[5];
			chi[3] = x1[3]+t135*x1[7]+t11*x1[9]-t68*t54/360.0+t140*t69/60.0-t93*t83/12.0+t145*t94/3.0;
			chi[4] = x1[4]-t135*x1[6]-t11*x1[8]-t68*t101/360.0+t140*t104/60.0+t93*t107/12.0-t145*t110/3.0;
			chi[5] = x1[5]-2.0*t129*t123+4.0*t121*t52*t1*d_tau[5];
			chi[6] = x1[6]+t1*x1[8]+t82*t101/60.0-t93*t104/12.0-t172*t107/3.0+t175*t110;
			chi[7] = x1[7]+t1*x1[9]-t82*t54/60.0+t93*t69/12.0-t172*t83/3.0+t175*t94;
			chi[8] = x1[8]+t93*t101/12.0-t145*t104/3.0-t175*t107+2.0*t192*t193*d_tau[8];
			chi[9] = x1[9]-t93*t54/12.0+t145*t69/3.0-t175*t83+2.0*t192*t193*d_tau[9];
			chi[10] = d_tau[0];
			chi[11] = d_tau[1];
			chi[12] = d_tau[2];
			chi[13] = -t1*d_tau[0]+d_tau[3];
			chi[14] = -t1*d_tau[1]+d_tau[4];
			chi[15] = -t1*d_tau[2]+d_tau[5];
			chi[16] = -t11*d_tau[1]+t135*d_tau[4]+d_tau[6];
			chi[17] = t11*d_tau[0]-t135*d_tau[3]+d_tau[7];
			chi[18] = t23*d_tau[1]-t11*d_tau[4]-t1*d_tau[6]+d_tau[8];
			chi[19] = -t23*d_tau[0]+t11*d_tau[3]-t1*d_tau[7]+d_tau[9];

#elif (DYNAMICS == NONHOLONOMIC)
	double px = x0[0];
	double py = x0[1];
	double th = x0[2];
	double v = x0[3];
	double ka = x0[4];
	double px2 = x1[0];
	double py2 = x1[1];
	double th2 = x1[2];
	double v2 = x1[3];
	double ka2 = x1[4];
	double rv = control_penalty;
	double rk = control_penalty1;
	double cth = cos(th);
	double c2th = cos(2*th);
	double sth = sin(th);
	double s2th = sin(2*th);

	chi[0] = (240*px2*rk*rv + pow(t - tau,3)*
		(10*d_tau[1]*ka*rk*(t - tau)*v +
		d_tau[0]*(rv*pow(t - tau,2)*pow(v,4) +
		rk*(-20 + pow(ka,2)*pow(t - tau,2)*pow(v,2)))) +
		(t - tau)*(40*rk*((t - tau)*(3*d_tau[3] + d_tau[2]*ka*(-t + tau)) + 6*rv*v2)*
		cth - d_tau[0]*pow(t - tau,2)*
		(rv*pow(t - tau,2)*pow(v,4) +
		rk*(20 + pow(ka,2)*pow(t - tau,2)*pow(v,2)))*c2th -
		10*v*(4*d_tau[3]*ka*rk*pow(t - tau,2) -
		d_tau[2]*pow(t - tau,3)*(pow(ka,2)*rk + rv*pow(v,2)) +
		4*rv*(d_tau[4]*pow(t - tau,2)*v +
		3*rk*(-2*th + 2*th2 + (t - tau)*(ka2*v + ka*(-v + v2)))))*
		sth - d_tau[1]*pow(t - tau,2)*
		(rv*pow(t - tau,2)*pow(v,4) +
		rk*(20 + pow(ka,2)*pow(t - tau,2)*pow(v,2)))*s2th))/
		(240.*rk*rv);
	chi[1] = (240*py2*rk*rv +
		pow(t - tau,3)*(10*d_tau[0]*ka*rk*(-t + tau)*v +
		d_tau[1]*(rv*pow(t - tau,2)*pow(v,4) +
		rk*(-20 + pow(ka,2)*pow(t - tau,2)*pow(v,2)))) +
		(t - tau)*(10*v*(4*d_tau[3]*ka*rk*pow(t - tau,2) -
		d_tau[2]*pow(t - tau,3)*(pow(ka,2)*rk + rv*pow(v,2)) +
		4*rv*(d_tau[4]*pow(t - tau,2)*v +
		3*rk*(-2*th + 2*th2 + (t - tau)*(ka2*v + ka*(-v + v2)))))*
		cth + d_tau[1]*pow(t - tau,2)*
		(rv*pow(t - tau,2)*pow(v,4) +
		rk*(20 + pow(ka,2)*pow(t - tau,2)*pow(v,2)))*c2th +
		40*rk*((t - tau)*(3*d_tau[3] + d_tau[2]*ka*(-t + tau)) + 6*rv*v2)*
		sth - d_tau[0]*pow(t - tau,2)*
		(rv*pow(t - tau,2)*pow(v,4) +
		rk*(20 + pow(ka,2)*pow(t - tau,2)*pow(v,2)))*s2th))/
		(240.*rk*rv);
	chi[2] = (4*(3*d_tau[3]*ka*rk*pow(t - tau,2) -
		d_tau[2]*pow(t - tau,3)*(pow(ka,2)*rk + rv*pow(v,2)) +
		3*rv*(d_tau[4]*pow(t - tau,2)*v +
		2*rk*(th2 + (t - tau)*(ka2*v + ka*(-v + v2))))) +
		pow(t - tau,3)*((-4*d_tau[0]*ka*rk +
		d_tau[1]*(t - tau)*v*(pow(ka,2)*rk + rv*pow(v,2)))*cth +
		(-4*d_tau[1]*ka*rk - d_tau[0]*(t - tau)*v*
		(pow(ka,2)*rk + rv*pow(v,2)))*sth))/(24.*rk*rv);
	chi[3] = -(3*d_tau[2]*ka*pow(t - tau,2) + 6*d_tau[3]*(-t + tau) - 6*rv*v2 +
		pow(t - tau,2)*((3*d_tau[0] + d_tau[1]*ka*(-t + tau)*v)*cth +
		(3*d_tau[1] + d_tau[0]*ka*(t - tau)*v)*sth))/(6.*rv);
	chi[4] = (6*ka2*rk + 3*(t - tau)*(2*d_tau[4] + d_tau[2]*(-t + tau)*v) +
		pow(t - tau,3)*pow(v,2)*(d_tau[1]*cth - d_tau[0]*sth))/(6.*rk);
	chi[5] = d_tau[0];
	chi[6] = d_tau[1];
	chi[7] = d_tau[2] + (t - tau)*v*(-(d_tau[1]*cth) + d_tau[0]*sth);
	chi[8] = (2*(d_tau[3] + d_tau[2]*ka*(-t + tau)) -
		(t - tau)*((2*d_tau[0] + d_tau[1]*ka*(-t + tau)*v)*cth +
		(2*d_tau[1] + d_tau[0]*ka*(t - tau)*v)*sth))/2.;
	chi[9] = (2*(d_tau[4] + d_tau[2]*(-t + tau)*v) +
		pow(t - tau,2)*pow(v,2)*(d_tau[1]*cth - d_tau[0]*sth))/2.;
		
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
			double t1 = t-tau;
			double t3 = tau*tau;
			double t7 = t*t;
			double t12 = 1/control_penalty;
			double t13 = (t3*tau-3.0*t3*t+3.0*tau*t7-t7*t)*t12;
			double t19 = (t3-2.0*tau*t+t7)*t12;
			double t31 = -t1*t12;
			chi[0] = x1[0]+t1*x1[2]+t13*d_tau[0]/6.0+t19*d_tau[2]/2.0;
			chi[1] = x1[1]+t1*x1[3]+t13*d_tau[1]/6.0+t19*d_tau[3]/2.0;
			chi[2] = x1[2]-t19*d_tau[0]/2.0-t31*d_tau[2];
			chi[3] = x1[3]-t19*d_tau[1]/2.0-t31*d_tau[3];
			chi[4] = d_tau[0];
			chi[5] = d_tau[1];
			chi[6] = -t1*d_tau[0]+d_tau[2];
			chi[7] = -t1*d_tau[1]+d_tau[3];

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
			double t3 = (tau-t)/control_penalty;
			chi[0] = x1[0]-t3*d_tau[0];
			chi[1] = x1[1]-t3*d_tau[1];
			chi[2] = d_tau[0];
			chi[3] = d_tau[1];

#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
			double t1 = t-tau;
			double t3 = tau*tau;
			double t7 = t*t;
			double t12 = 1/control_penalty;
			double t19 = (t3-2.0*tau*t+t7)*t12;
			chi[0] = x1[0]+t1*x1[1]+(t3*tau-3.0*t3*t+3.0*tau*t7-t7*t)*t12*d_tau[0]/6.0+t19*d_tau[1]/2.0;
			chi[1] = x1[1]-t19*d_tau[0]/2.0+t1*t12*d_tau[1];
			chi[2] = d_tau[0];
			chi[3] = -t1*d_tau[0]+d_tau[1];

#else
			dynamicsError();
#endif

			//state x = chi.subMatrix<X_DIM,1>(0,0);
			state x = chi.block<X_DIM,1>(0,0);
			//control u = R%(~B*chi.subMatrix<X_DIM,1>(X_DIM,0));
			control u = R.ldlt().solve(B.transpose()*chi.block<X_DIM,1>(X_DIM,0));

			if (!(plot || vis) && !checkState(x, u)) {
				return false;
			}

			if (plot || vis) {
				double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;

#if (DYNAMICS == QUADROTOR)
				x_coord = x[0];
				y_coord = x[1];
				z_coord = x[2];
#else
				x_coord = x[0];
				y_coord = 0;
				z_coord = x[1];
#endif

				if (plot) {
					fwrite((const void *)&t, sizeof(double), 1, path_log);
					// FIX HERE STILL
					fwrite(x._elems, sizeof(double), (x.numRows())*(x.numColumns()), path_log);
					CAL_CreateSphere(solution_group, 2*NODE_SIZE, x_coord, y_coord, z_coord);
				}

				if (vis) {
					vis->push_back(make_pair(t, x));
				}
			}
		}
	}

	return true;
}

bool checkPathRK4(const state& x0, const state& x1, const double tau, const state& d_tau, const bool plot, state_time_list_t* vis) {
	Alpha = block<X_DIM,X_DIM,X_DIM,X_DIM>(A, -BRiBt, Eigen::Matrix<float,X_DIM,X_DIM>::Zero(), -A.transpose());

	//chi.insert(0,0,x1);
	//chi.insert(X_DIM,0,-d_tau);
	chi.block<X_DIM,1>(0,0) = x1;
	chi.block<X_DIM,1>(X_DIM,0) = -d_tau;

	//c0.insert(0,0,c);
	//c0.insert(X_DIM,0, zeros<X_DIM>());
	c0.block<X_DIM,1>(0,0) = c;
	c0.block<X_DIM,1>(X_DIM,0) = Eigen::Matrix<float,X_DIM,1>::Zero();

	int num_steps = (int) ceil(tau / deltaT);
	double new_deltaT = tau / num_steps;

#ifndef FRONT_LOAD_RK4
	linear<2*X_DIM> back;
	back.A = &Alpha;
	back.c = &c0;

	rk<2*X_DIM, 1, linear<2*X_DIM> > linearrk;
#endif

	for (int j = 0; j < num_steps; j++) {
		linearrk.rk4(chi, 0, -new_deltaT, back);
#if (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == NONHOLONOMIC)
		/*
		ostringstream os;
		os << num_steps << " " << j << "     " << chi[0] << " " << chi[1] << " " << chi[2] << endl;
		fputs(os.str().c_str(), path_log);
		fflush(path_log);
		*/
		//cout << num_steps << " " << j << "     " << chi[0] << " " << chi[1] << " " << chi[2] << endl;
#else
		//cout << chi[0] << " " << chi[1] << " " << chi[2] << " " << chi[6] << " " << chi[7] << endl;
#endif

		//state x = chi.subMatrix<X_DIM>(0,0);
		//control u = -R%(~B*chi.subMatrix<X_DIM>(X_DIM,0)); // This should be negative as it is
		state x = chi.block<X_DIM,1>(0,0);
		control u = -R.ldlt().solve(B.transpose()*chi.block<X_DIM,1>(X_DIM,0));

		//cout << "checkBounds(x, x_bounds): " << checkBounds(x, x_bounds) << "\tcheckBounds(u, u_bounds): " << checkBounds(u, u_bounds) << endl;

		if (!(plot || vis) && !checkState(x, u)) {
			return false;
		}

		if (plot || vis) {
			double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;

#if (DYNAMICS == QUADROTOR)
			x_coord = x[0];
			y_coord = x[1];
			z_coord = x[2];
#else
			x_coord = x[0];
			y_coord = 0;
			z_coord = x[1];
#endif

			if (plot) {
				double temp = deltaT*j;
				fwrite((const void *)&(temp), sizeof(double), 1, path_log);
				//FIX HERE
				fwrite(x._elems, sizeof(double), (x.numRows())*(x.numColumns()), path_log);
				CAL_CreateSphere(solution_group, 2*NODE_SIZE, x_coord, y_coord, z_coord);
			}

			if (vis) {
				vis->push_back(make_pair(deltaT*j, x));
			}
		}
	}
	return true;
}

inline bool connect(const state& x0, const state& x1, const double radius, double& cost, double& tau, state_time_list_t* vis) {
	state d_tau;

	state xend = x1;
#if (DYNAMICS == NONHOLONOMIC)
	double theta_diff = x1[2] - x0[2];
	while (theta_diff < -M_PI) {
		theta_diff += 2*M_PI;
	}
	while (theta_diff > M_PI) {
		theta_diff -= 2*M_PI;
	}

	if (abs(theta_diff) > 0.25*M_PI || pow(x1[0] - x0[0],2) + pow(x1[1] - x0[1],2) > 20*20) {
		return false;
	}

	xend[2] = x0[2] + theta_diff;
#endif

	if (computeCost(x0, xend, radius, cost, tau, d_tau)) {
		if (checkPath(x0, xend, tau, d_tau, false, vis)) {
			return true;
		} else {
			//cout << "Failed checkPath" << endl;
		}
	} else {
		//cout << "Failed computeCost" << endl;
	}

	return false;
}

inline double rand_value(double a, double b) {
	return ((double) (rand()*(RAND_MAX+1) + rand()) / (RAND_MAX*(RAND_MAX + 2))) * (b - a) + a;
}

template<typename vec, typename bounds>
inline void rand_vec(vec& v, const bounds& b) {
	int count = v.numRows();
#if (DYNAMICS == QUADROTOR)
	v.reset();
	count = 6;
#endif
	for (size_t i = 0; i < count; i++) {
		v[i] = rand_value(b[i].first, b[i].second);
	}
#if (DYNAMICS == NONHOLONOMIC)
	v[4] = 0;
#endif
}

/**
 * Calculates the volume of a unit sphere.
 *
 * Verified with table on http://en.wikipedia.org/wiki/Unit_sphere#General_area_and_volume_formulas
 * Reproduced here
 * n 	V_n (volume)
 * 0 	(1/0!)\pi^0 	1.000
 * 1 	(2^1/1!!)\pi^0 	2.000
 * 2 	(1/1!)\pi^1 = \pi 	3.142
 * 3 	(2^2/3!!)\pi^1 = (4/3)\pi 	4.189
 * 4 	(1/2!)\pi^2 = (1/2)\pi^2 	4.935
 * 5 	(2^3/5!!)\pi^2 = (8/15)\pi^2 	5.264
 * 6 	(1/3!)\pi^3 = (1/6)\pi^3 	5.168
 * 7 	(2^4/7!!) \pi^3 = (16/105)\pi^3 	4.725
 * 8 	(1/4!)\pi^4 = (1/24)\pi^4 	4.059
 * 9 	(2^5/9!!) \pi^4 = (32/945)\pi^4 	3.299
 * 10 	(1/5!)\pi^5 = (1/120)\pi^5 	2.550
 */
inline double volume() {
#if X_DIM % 2 == 0
	int i = 4;
	int den = 2;
#else
	int i = 5;
	int den = 3;
#endif

#if ((X_DIM % 2 == 0) && (X_DIM > 2)) || ((X_DIM % 2 != 0) && (X_DIM > 3))
	for(; i <= X_DIM; i+=2) {
		den *= i;
	}
#endif

	double num = (2*M_PI);
#if X_DIM % 2 == 0
	num = pow(num, (double)X_DIM/2.0);
#else
	num = 2.0*pow(num, (double)(X_DIM-1)/2.0);
#endif

	return (num/den);
}

inline void setRadius(const double& num_states, double& radius) {
	//double c = TWO_TO_X_DIM * (1 + X_DIM_INVERSE) * FREE_SS_VOLUME * REACHABILITY_CONSTANT; /* Of the order of (2^X_DIM) * (1 + 1/X_DIM) * (Volume of free state space) * (reachability constant (start out arbitrarily, we need to research it further)) */
	//double v = volume(); /* Volume of a ball of dimension X_DIM */
//#ifdef REDUCE_RADIUS
	//radius = min(radius, pow(c/v*log((double)n)/(double)n, X_DIM_INVERSE));
//#endif
#ifdef REDUCE_RADIUS

#if (DYNAMICS == NONHOLONOMIC)
	double test = 1;
	double test2 = 2;
#elif (DYNAMICS == QUADROTOR)
	double j = inertia;
	double g = gravity;
	double m = mass;			
	double r = control_penalty; // This is the r parameter to matrix R -- refer to maple
	double l = length;

	double t1 = pow(77.0,0.4347826086956522E-1);
	double t2 = pow(3587226750000.0,0.2173913043478261E-1);
	double t3 = t2*t2;
	double t4 = t3*t3;
	double t6 = t4*t4;
	double t8 = t6*t6;
	double t9 = t8*t8;
	double t18 = statespace_volume*statespace_volume;
	double t19 = log(num_states);
	double t20 = t19*t19;
	double t22 = j*j;
	double t23 = t22*t22;
	double t24 = t23*t23;
	double t25 = t24*t24;
	double t26 = r*r;
	double t27 = t26*t26;
	double t28 = t27*t27;
	double t32 = m*m;
	double t33 = t32*t32;
	double t34 = sphere_volume*sphere_volume;
	double t35 = t34*t34;
	double t36 = t35*t35;
	double t38 = t36*t36;
	double t39 = t38*t38;
	double t42 = g*g;
	double t43 = t42*t42;
	double t45 = t43*t43;
	double t46 = t45*t45;
	double t47 = t46*t46;
	double t49 = l*l;
	double t50 = t49*t49;
	double t52 = t50*t50;
	double t54 = t52*t52;
	double t57 = num_states*num_states;
	double t58 = t57*t57;
	double t59 = t58*t58;
	double t61 = t59*t59;
	double t62 = t61*t61;
	double t67 = pow(t18*t20*t25*t28*t26*t33*t39*t36*t35*t47*t43*t42*t54*t52*t50*t49*t62*t59*t58,0.2173913043478261E-1);
	double t71 = 23.0/10761680250000.0*t1*t9*t6*t4*t2/sphere_volume/g/l/num_states*t67;

	radius = t71*RADIUS_MULTIPLIER;
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	double t4 = sphere_volume*num_states;
	double t5 = pow(225.0,0.3333333333333333);
	double t6 = statespace_volume*statespace_volume;
	double t7 = log(num_states);
	double t8 = t7*t7;
	double t11 = pow(t6*t8*t4,0.3333333333333333);
	double t14 = sqrt(t4*t5*t11);
	double t16 = sqrt(t4*t14);
	double t18 = 3.0/sphere_volume/num_states*t16;
	radius = t18*RADIUS_MULTIPLIER;
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	// I don't know whether gamma is supposed to be strictly greater than 2... as stated in the paper or if equal to will suffice
	// This section is the strictly greater than version
	double t1 = sqrt(3.0);
	double t5 = statespace_volume*statespace_volume;
	double t6 = log(num_states);
	double t7 = t6*t6;
	double t9 = sphere_volume*sphere_volume;
	double t10 = t9*t9;
	double t11 = num_states*num_states;
	double t12 = t11*t11;
	double t15 = pow(t5*t7*t10*t12,0.1666666666666667);
	double t18 = 3.0*t1/sphere_volume/num_states*t15;
	radius = t18*RADIUS_MULTIPLIER;

	/*
    double t1 = sqrt(3.0);
    double t5 = statespace_volume*statespace_volume;
    double t6 = log(num_states);
    double t7 = t6*t6;
    double t9 = sphere_volume*sphere_volume;
    double t10 = t9*t9;
    double t11 = num_states*num_states;
    double t12 = t11*t11;
    double t15 = pow(t5*t7*t10*t12,0.1666666666666667);
    double t18 = 3.0*t1/sphere_volume/num_states*t15;
	radius = t18;
	*/
#else
	assert("setRadius undefined");
	return;
#endif
#else
	return;
#endif
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
		p[0] = current.x[1];
		p[2] = current.x[0];
		p[3] = tree[current.parent].x[1];
		p[5] = tree[current.parent].x[0];
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
		x = tree[i].x[1];
		z = tree[i].x[0];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[1];
			z_parent = tree[tree[i].parent].x[0];
		}
		// TODO DWEBB implement velocity drawing
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
		x = tree[i].x[0];
		z = tree[i].x[1];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			z_parent = tree[tree[i].parent].x[1];
		}
		v_x = x + tree[i].x[2];
		v_z = z + tree[i].x[3];
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
		x = tree[i].x[0];
		z = tree[i].x[1];
		if (tree[i].parent != NO_PARENT) {
			x_parent = tree[tree[i].parent].x[0];
			z_parent = tree[tree[i].parent].x[1];
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
			p[0] = x;
			p[2] = z;
			p[3] = v_x;
			p[5] = v_z;

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

#define summarize_path(os, time_diff, number_of_nodes, target_number_of_nodes, cost_from_start, radius, number_of_orphans) {\
	paths++; \
	cout << "                                                                              \r"; \
	cout << paths << " " << setw(9) << time_diff << " " << setw(11) << number_of_nodes << "/" << target_number_of_nodes << " cost: " << setw(9) << cost_from_start << " radius: " << setw(9) << radius << " orphans: " << number_of_orphans << endl; \
	ostringstream os;\
	os << time_diff << "\t" << number_of_nodes << "\t" << target_number_of_nodes << "\t" << cost_from_start << "\t" << radius << "\t" << number_of_orphans << endl;\
	fputs(os.str().c_str(), stats_log);\
	fflush(stats_log);\
}

/*
void connect_forward(tree_t& tree, k_d_tree_t& k_d_tree, const node_cost_pair_t& start, const state& x_rand, const node_id_t x_rand_node_id, const Node& x_rand_node, double radius, double time_diff) {
	ostringstream os;
	double cost;
	stack<node_cost_pair_t> s;
	s.push(start);
			while (!s.empty()) {
				node_id_t j = s.top().first;
				double decrease_cost = s.top().second;
				s.pop();
				for (node_list_t::iterator p = tree[j].children.begin(); p != tree[j].children.end(); ) {
					// If we can get to a node via the new node faster than via it's existing parent then change the parent
					double junk;
					if (connect(x_rand, tree[*p].x, min(radius, tree[*p].cost_from_start - decrease_cost - x_rand_node.cost_from_start), cost, junk, NULL)) {
						tree[*p].parent = x_rand_node_id;
						tree[x_rand_node_id].children.push_back(*p);
						s.push(make_pair(*p, tree[*p].cost_from_start - (x_rand_node.cost_from_start + cost)));
						tree[*p].cost_from_start = x_rand_node.cost_from_start + cost;

						if (*p == 0) { // i.e. if we're updating the cost to the goal							
							// TODO DWEBB -- FIX THIS
							//summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())

#ifndef EXPERIMENT
							draw_path = true;
#endif
						}

						p = tree[j].children.erase(p);
					} else {
						if (decrease_cost > 0) {
							tree[*p].cost_from_start -= decrease_cost;
							if (*p == 0) {
								summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())

#ifndef EXPERIMENT
								draw_path = true;
#endif
							}
						}
						s.push(make_pair(*p, decrease_cost));
						++p;
					}
				}
			}

			// check orphans
			for (size_t j = 0; j < orphans.size(); ) {
				double junk;
				if (connect(x_rand, tree[orphans[j]].x, radius, cost, junk, NULL)) {
					tree[orphans[j]].cost_from_start = x_rand_node.cost_from_start + cost;
					tree[orphans[j]].parent = x_rand_node_id;
					tree[x_rand_node_id].children.push_back(orphans[j]);

					if (orphans[j] == 0) {
#ifndef EXPERIMENT
						draw_path = true;
#endif

						summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())
					}

					orphans[j] = orphans.back(); // remove orphan
					orphans.pop_back();
				} else {
					++j;
				}
			}

			// update parent of new node
			tree[x_near_id].children.push_back(x_rand_node_id);
}
*/

complex<double> im(0,1);
void calc_backward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) {
	bounds.resize(X_DIM);

#if (DYNAMICS == NONHOLONOMIC)
#elif (DYNAMICS == QUADROTOR)
	double x0_0 = state[0];
	double x0_1 = state[1];
	double x0_2 = state[2];
	double x0_3 = state[3];
	double x0_4 = state[4];
	double x0_5 = state[5];
	double x0_6 = state[6];
	double x0_7 = state[7];
	double x0_8 = state[8];
	double x0_9 = state[9];

	double j = inertia;
	double g = gravity;
	double m = mass;
	double r = control_penalty; // This is the r parameter to matrix R -- refer to maple
	double l = length;

	int info[1000];

	for (int i = 0; i < X_DIM; i++) {
		bounds[i].first = DBL_MAX;
		bounds[i].second = DBL_MIN;
	}

	// Calculate x1 bounds
	{
		const int degree = 7;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 64.0*g*g*l*l;
		p[1] = 112.0*g*g*l*l*radius;
		p[2] = (126.0*j*j*r*g*g*x0_9*x0_9+49.0*radius*radius*g*g*l*l);
		p[3] = (126.0*radius*j*j*r*g*g*x0_9*x0_9+504.0*j*j*r*g*g*x0_7*x0_9);
		p[4] = (504.0*radius*j*j*r*g*g*x0_7*x0_9+504.0*j*j*r*g*g*x0_7*x0_7+504.0*j*j*r*x0_3*g*x0_9);
		p[5] = (504.0*radius*j*j*r*g*g*x0_7*x0_7+504.0*radius*j*j*r*x0_3*g*x0_9+1008.0*j*j*r*x0_3*g*x0_7);
		p[6] = (1008.0*radius*j*j*r*x0_3*g*x0_7+504.0*j*1*r*x0_3*x0_3);
		p[7] = 504.0*radius*j*j*r*x0_3*x0_3;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t3 = t1*t1;
			complex<double> t7 = t3*t1;
			complex<double> t11 = g*g;
			complex<double> t12 = l*l;
			complex<double> t14 = j*j;
			complex<double> t18 = t3*t3;
			complex<double> t25 = sqrt(-14.0*t11*t12/t14/r*t18*t7*(radius+t1));
			complex<double> t27_min = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0-t25/42.0;
			complex<double> t27_max = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0+t25/42.0;

			bounds[0].first = min(bounds[0].first, t27_min.real());
			bounds[0].second = max(bounds[0].second, t27_max.real());
		}
	}

	// Calculate x2 bounds
	{
		const int degree = 7;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 64.0*g*g*l*l;
		p[1] = 112.0*g*g*l*l*radius;
		p[2] = (126.0*j*j*r*g*g*x0_8*x0_8+49.0*radius*radius*g*g*l*l);
		p[3] = (126.0*radius*j*j*r*g*g*x0_8*x0_8+504.0*j*j*r*g*g*x0_6*x0_8);
		p[4] = (504.0*radius*j*j*r*g*g*x0_6*x0_8+504.0*j*j*r*g*g*x0_6*x0_6-504.0*j*j*r*x0_4*g*x0_8);
		p[5] = (504.0*radius*j*j*r*g*g*x0_6*x0_6-504.0*radius*j*j*r*x0_4*g*x0_8-1008.0*j*j*r*x0_4*g*x0_6);
		p[6] = (-1008.0*radius*j*j*r*x0_4*g*x0_6+504.0*j*j*r*x0_4*x0_4);
		p[7] = 504.0*radius*j*j*r*x0_4*x0_4;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t3 = t1*t1;
			complex<double> t7 = t3*t1;
			complex<double> t11 = g*g;
			complex<double> t12 = l*l;
			complex<double> t14 = j*j;
			complex<double> t18 = t3*t3;
			complex<double> t25 = sqrt(-14.0*t11*t12/t14/r*t18*t7*(radius+t1));
			complex<double> t27_min = x0_1+t1*x0_4-g*t3*x0_6/2.0-g*t7*x0_8/6.0-t25/42.0;
			complex<double> t27_max = x0_1+t1*x0_4-g*t3*x0_6/2.0-g*t7*x0_8/6.0+t25/42.0;

			bounds[1].first = min(bounds[1].first, t27_min.real());
			bounds[1].second = max(bounds[1].second, t27_max.real());
		}
	}

	// Calculate x3 bounds
	{
		complex<double> t1 = x0_5*x0_5;
		complex<double> t3 = m*m;
		complex<double> t4 = t3*r;
		complex<double> t7 = radius*radius;
		complex<double> t9 = r*r;
		complex<double> t11 = t3*t3;
		complex<double> t14 = t1*t1;
		complex<double> t21 = t7*t7;
		complex<double> t26 = sqrt(t9*r*t11*t3*t14*t1+6.0*t9*t11*t14*t7-3.0*t4*t1*t21);
		complex<double> t28 = pow(-3.0*radius*t1*t4+t7*radius+t26,0.3333333333333333);
		complex<double> t29 = t28/4.0;
		complex<double> t34 = (t4*t1-t7)/t28/4.0;
		complex<double> t35 = radius/2.0;
		complex<double> t36 = t29-t34-t35;
		complex<double> t41 = t36*t36;
		complex<double> t47 = sqrt(-3.0/t3/r*t41*t36*(t35+t29-t34));
		complex<double> t49_min = x0_2+t36*x0_5-2.0/3.0*t47;
		complex<double> t49_max = x0_2+t36*x0_5+2.0/3.0*t47;

		bounds[2].first = t49_min.real();
		bounds[2].second = t49_max.real();
	}

	{
		complex<double> t1 = x0_5*x0_5;
		complex<double> t3 = m*m;
		complex<double> t4 = t3*r;
		complex<double> t7 = radius*radius;
		complex<double> t9 = r*r;
		complex<double> t11 = t3*t3;
		complex<double> t14 = t1*t1;
		complex<double> t21 = t7*t7;
		complex<double> t26 = sqrt(t9*r*t11*t3*t14*t1+6.0*t9*t11*t14*t7-3.0*t4*t1*t21);
		complex<double> t28 = pow(-3.0*radius*t1*t4+t7*radius+t26,0.3333333333333333);
		complex<double> t29 = t28/8.0;
		complex<double> t33 = (t4*t1-t7)/t28/16.0;
		complex<double> t34 = 2.0*t33;
		complex<double> t35 = radius/2.0;
		complex<double> t36 = sqrt(3.0);
		complex<double> t41 = (1.0/2.0*im)*t36*(t28/4.0+4.0*t33);
		complex<double> t42 = -t29+t34-t35+t41;
		complex<double> t47 = t42*t42;
		complex<double> t53 = sqrt(-3.0/t3/r*t47*t42*(t35-t29+t34+t41));
		complex<double> t55_min = x0_2+t42*x0_5-2.0/3.0*t53;
		complex<double> t55_max = x0_2+t42*x0_5+2.0/3.0*t53;

		bounds[2].first = min(bounds[2].first, t55_min.real());
		bounds[2].second = max(bounds[2].second, t55_max.real());
	}

	{
		complex<double> t1 = x0_5*x0_5;
		complex<double> t3 = m*m;
		complex<double> t4 = t3*r;
		complex<double> t7 = radius*radius;
		complex<double> t9 = r*r;
		complex<double> t11 = t3*t3;
		complex<double> t14 = t1*t1;
		complex<double> t21 = t7*t7;
		complex<double> t26 = sqrt(t9*r*t11*t3*t14*t1+6.0*t9*t11*t14*t7-3.0*t4*t1*t21);
		complex<double> t28 = pow(-3.0*radius*t1*t4+t7*radius+t26,0.3333333333333333);
		complex<double> t29 = t28/8.0;
		complex<double> t33 = (t4*t1-t7)/t28/16.0;
		complex<double> t34 = 2.0*t33;
		complex<double> t35 = radius/2.0;
		complex<double> t36 = sqrt(3.0);
		complex<double> t41 = (-1.0/2.0*im)*t36*(t28/4.0+4.0*t33);
		complex<double> t42 = -t29+t34-t35+t41;
		complex<double> t47 = t42*t42;
		complex<double> t53 = sqrt(-3.0/t3/r*t47*t42*(t35-t29+t34+t41));
		complex<double> t55_min = x0_2+t42*x0_5-2.0/3.0*t53;
		complex<double> t55_max = x0_2+t42*x0_5+2.0/3.0*t53;

		bounds[2].first = min(bounds[2].first, t55_min.real());
		bounds[2].second = max(bounds[2].second, t55_max.real());
	}

	// Calculate x4 bounds
	{
		const int degree = 5;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 36.0*l*l;
		p[1] = 60.0*l*l*radius;
		p[2] = (25.0*radius*radius*l*l+40.0*j*j*r*x0_9*x0_9);
		p[3] = (40.0*radius*j*j*r*x0_9*x0_9+80.0*x0_9*x0_7*r*j*j);
		p[4] = (80.0*radius*j*j*r*x0_7*x0_9+40.0*x0_7*x0_7*r*j*j);
		p[5] = 40.0*radius*j*j*r*x0_7*x0_7;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t4 = t1*t1;
			complex<double> t8 = g*g;
			complex<double> t9 = t4*t4;
			complex<double> t12 = l*l;
			complex<double> t14 = j*j;
			complex<double> t22 = sqrt(-10.0*t8*t9*t1*t12/t14/r*(radius+t1));
			complex<double> t24_min = x0_3+g*t1*x0_7+g*t4*x0_9/2.0-t22/10.0;
			complex<double> t24_max = x0_3+g*t1*x0_7+g*t4*x0_9/2.0+t22/10.0;

			bounds[3].first = min(bounds[3].first, t24_min.real());
			bounds[3].second = max(bounds[3].second, t24_max.real());
		}
	}

	// Calculate x5 bounds
	{
		const int degree = 7;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 36.0*l*l;
		p[1] = 60.0*l*l*radius;
		p[2] = (25.0*radius*radius*l*l+40.0*j*j*r*x0_8*x0_8);
		p[3] = (40.0*radius*j*j*r*x0_8*x0_8+80.0*x0_8*x0_6*r*j*j);
		p[4] = (80.0*radius*j*j*r*x0_6*x0_8+40.0*x0_6*x0_6*r*j*j);
		p[5] = 40.0*radius*j*j*r*x0_6*x0_6;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t4 = t1*t1;
			complex<double> t8 = g*g;
			complex<double> t9 = t4*t4;
			complex<double> t12 = l*l;
			complex<double> t14 = j*j;
			complex<double> t22 = sqrt(-10.0*t8*t9*t1*t12/t14/r*(radius+t1));
			complex<double> t24_min = x0_4-g*t1*x0_6-g*t4*x0_8/2.0-t22/10.0;
			complex<double> t24_max = x0_4-g*t1*x0_6-g*t4*x0_8/2.0+t22/10.0;

			bounds[4].first = min(bounds[4].first, t24_min.real());
			bounds[4].second = max(bounds[4].second, t24_max.real());
		}
	}

	// Calculate x6 bounds
	{
		complex<double> t1 = sqrt(4.0);
		complex<double> t2 = m*m;
		complex<double> t6 = radius*radius;
		complex<double> t8 = sqrt(pow(t2,-1)/r*t6);
		complex<double> t11_min = x0_5-t1*t8/2.0;
		complex<double> t11_max = x0_5+t1*t8/2.0;

		bounds[5].first = t11_min.real();
		bounds[5].second = t11_max.real();
	}

	// Calculate x7 bounds
	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_8*x0_8;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t9*t9;
		complex<double> t27 = t11*t11;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t9*t11-3.0*t26*t27));
		complex<double> t36 = pow(-6.0*radius*l*t6+t9*radius*t11*l+t14*j*x0_8*t32, 0.3333333333333333);
		complex<double> t38 = t1*t36/4.0;
		complex<double> t45 = (2.0*t6-t9*t11)*t1/t36/4.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = t38-t45-t46;
		complex<double> t52 = t47*t47;
		complex<double> t59 = sqrt(-6.0*t11/t3/r*t52*t47*(t46+t38-t45));
		complex<double> t61_min = x0_6+t47*x0_8-t59/3.0;
		complex<double> t61_max = x0_6+t47*x0_8+t59/3.0;

		bounds[6].first = t61_min.real();
		bounds[6].second = t61_max.real();
	}
	
	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_8*x0_8;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t9*t9;
		complex<double> t27 = t11*t11;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t9*t11-3.0*t26*t27));
		complex<double> t36 = pow(-6.0*radius*l*t6+t9*radius*t11*l+t14*j*x0_8*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t9*t11)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t50 = (1.0/8.0*im)*t47*(t37+t44);
		complex<double> t51 = -t38+t45-t46+t50;
		complex<double> t56 = t51*t51;
		complex<double> t63 = sqrt(-6.0*t11/t3/r*t56*t51*(t46-t38+t45+t50));
		complex<double> t65_min = x0_6+t51*x0_8-t63/3.0;
		complex<double> t65_max = x0_6+t51*x0_8+t63/3.0;

		bounds[6].first = min(bounds[6].first, t65_min.real());
		bounds[6].second = max(bounds[6].second, t65_max.real());
	}

	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_8*x0_8;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t9*t9;
		complex<double> t27 = t11*t11;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t9*t11-3.0*t26*t27));
		complex<double> t36 = pow(-6.0*radius*l*t6+t9*radius*t11*l+t14*j*x0_8*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t9*t11)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t50 = (-1.0/8.0*im)*t47*(t37+t44);
		complex<double> t51 = -t38+t45-t46+t50;
		complex<double> t56 = t51*t51;
		complex<double> t63 = sqrt(-6.0*t11/t3/r*t56*t51*(t46-t38+t45+t50));
		complex<double> t65_min = x0_6+t51*x0_8-t63/3.0;
		complex<double> t65_max = x0_6+t51*x0_8+t63/3.0;

		bounds[6].first = min(bounds[6].first, t65_min.real());
		bounds[6].second = max(bounds[6].second, t65_max.real());
	}

	// Calculate x8 bounds
	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_9*x0_9;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t9*t9;
		complex<double> t27 = t11*t11;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t9*t11-3.0*t26*t27));
		complex<double> t36 = pow(-6.0*radius*l*t6+t9*radius*t11*l+t14*j*x0_9*t32, 0.3333333333333333);
		complex<double> t38 = t1*t36/4.0;
		complex<double> t45 = (2.0*t6-t9*t11)*t1/t36/4.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = t38-t45-t46;
		complex<double> t52 = t47*t47;
		complex<double> t59 = sqrt(-6.0*t11/t3/r*t52*t47*(t46+t38-t45));
		complex<double> t61_min = x0_7+t47*x0_9-t59/3.0;
		complex<double> t61_max = x0_7+t47*x0_9+t59/3.0;

		bounds[7].first = t61_min.real();
		bounds[7].second = t61_max.real();
	}
	
	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_9*x0_9;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t9*t9;
		complex<double> t27 = t11*t11;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t9*t11-3.0*t26*t27));
		complex<double> t36 = pow(-6.0*radius*l*t6+t9*radius*t11*l+t14*j*x0_9*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t9*t11)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t50 = (1.0/8.0*im)*t47*(t37+t44);
		complex<double> t51 = -t38+t45-t46+t50;
		complex<double> t56 = t51*t51;
		complex<double> t63 = sqrt(-6.0*t11/t3/r*t56*t51*(t46-t38+t45+t50));
		complex<double> t65_min = x0_7+t51*x0_9-t63/3.0;
		complex<double> t65_max = x0_7+t51*x0_9+t63/3.0;

		bounds[7].first = min(bounds[7].first, t65_min.real());
		bounds[7].second = max(bounds[7].second, t65_max.real());
	}

	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_9*x0_9;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t9*t9;
		complex<double> t27 = t11*t11;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t9*t11-3.0*t26*t27));
		complex<double> t36 = pow(-6.0*radius*l*t6+t9*radius*t11*l+t14*j*x0_9*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t9*t11)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t50 = (-1.0/8.0*im)*t47*(t37+t44);
		complex<double> t51 = -t38+t45-t46+t50;
		complex<double> t56 = t51*t51;
		complex<double> t63 = sqrt(-6.0*t11/t3/r*t56*t51*(t46-t38+t45+t50));
		complex<double> t65_min = x0_7+t51*x0_9-t63/3.0;
		complex<double> t65_max = x0_7+t51*x0_9+t63/3.0;

		bounds[7].first = min(bounds[7].first, t65_min.real());
		bounds[7].second = max(bounds[7].second, t65_max.real());
	}

	// Calculate x9 bounds
	{
		complex<double> t1 = sqrt(2.0);
		complex<double> t2 = l*l;
		complex<double> t3 = j*j;
		complex<double> t7 = radius*radius;
		complex<double> t10 = sqrt(t2/t3/r*t7);
		complex<double> t13_min = x0_8-t1*t10/2.0;
		complex<double> t13_max = x0_8+t1*t10/2.0;

		bounds[8].first = t13_min.real();
		bounds[8].second = t13_max.real();
	}

	// Calculate x10 bounds
	{
		complex<double> t1 = sqrt(2.0);
		complex<double> t2 = l*l;
		complex<double> t3 = j*j;
		complex<double> t7 = radius*radius;
		complex<double> t10 = sqrt(t2/t3/r*t7);
		complex<double> t13_min = x0_9-t1*t10/2.0;
		complex<double> t13_max = x0_9+t1*t10/2.0;

		bounds[9].first = t13_min.real();
		bounds[9].second = t13_max.real();
	}

#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	double x0_0 = state[0];
	double x0_1 = state[1];
	double x0_2 = state[2];
	double x0_3 = state[3];

	// Calculate x1 bounds
	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(-12.0*radius*t1+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25-t26;
		complex<double> t29 = t27*t27;
		complex<double> t34 = sqrt(-3.0*t29*t27*(t26+t19-t25));
		complex<double> t36_min = x0_0+t27*x0_2-t34/3.0;
		complex<double> t36_max = x0_0+t27*x0_2+t34/3.0;

		bounds[0].first = t36_min.real();
		bounds[0].second = t36_max.real();
	}

	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(-12.0*radius*t1+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t32 = (1.0/2.0*im)*t27*(t18/4.0+4.0*t24);
		complex<double> t33 = -t19+t25-t26+t32;
		complex<double> t35 = t33*t33;
		complex<double> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
		complex<double> t42_min = x0_0+t33*x0_2-t40/3.0;
		complex<double> t42_max = x0_0+t33*x0_2+t40/3.0;

		bounds[0].first = min(bounds[0].first, t42_min.real());
		bounds[0].second = max(bounds[0].second, t42_max.real());
	}

	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(-12.0*radius*t1+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t32 = (-1.0/2.0*im)*t27*(t18/4.0+4.0*t24);
		complex<double> t33 = -t19+t25-t26+t32;
		complex<double> t35 = t33*t33;
		complex<double> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
		complex<double> t42_min = x0_0+t33*x0_2-t40/3.0;
		complex<double> t42_max = x0_0+t33*x0_2+t40/3.0;

		bounds[0].first = min(bounds[0].first, t42_min.real());
		bounds[0].second = max(bounds[0].second, t42_max.real());
	}

	// Calculate x2 bounds
	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(-12.0*radius*t1+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25-t26;
		complex<double> t29 = t27*t27;
		complex<double> t34 = sqrt(-3.0*t29*t27*(t26+t19-t25));
		complex<double> t36_min = x0_1+t27*x0_3-t34/3.0;
		complex<double> t36_max = x0_1+t27*x0_3+t34/3.0;

		bounds[1].first = t36_min.real();
		bounds[1].second = t36_max.real();
	}

	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(-12.0*radius*t1+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t32 = (1.0/2.0*im)*t27*(t18/4.0+4.0*t24);
		complex<double> t33 = -t19+t25-t26+t32;
		complex<double> t35 = t33*t33;
		complex<double> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
		complex<double> t42_min = x0_1+t33*x0_3-t40/3.0;
		complex<double> t42_max = x0_1+t33*x0_3+t40/3.0;

		bounds[1].first = min(bounds[1].first, t42_min.real());
		bounds[1].second = max(bounds[1].second, t42_max.real());
	}

	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(-12.0*radius*t1+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t32 = (-1.0/2.0*im)*t27*(t18/4.0+4.0*t24);
		complex<double> t33 = -t19+t25-t26+t32;
		complex<double> t35 = t33*t33;
		complex<double> t40 = sqrt(-3.0*t35*t33*(t26-t19+t25+t32));
		complex<double> t42_min = x0_1+t33*x0_3-t40/3.0;
		complex<double> t42_max = x0_1+t33*x0_3+t40/3.0;

		bounds[1].first = min(bounds[1].first, t42_min.real());
		bounds[1].second = max(bounds[1].second, t42_max.real());
	}

	// Calculate x3 bounds
	{
		double t1 = sqrt(4.0);
		double t2 = radius*radius;
		double t3 = sqrt(t2);
		double t6_min = x0_2-t1*t3/4.0;
		double t6_max = x0_2+t1*t3/4.0;

		bounds[2].first = t6_min;
		bounds[2].second = t6_max;
	}

	// Calculate x4 bounds
	{
		double t1 = sqrt(4.0);
		double t2 = radius*radius;
		double t3 = sqrt(t2);
		double t6_min = x0_3-t1*t3/4.0;
		double t6_max = x0_3+t1*t3/4.0;

		bounds[3].first = t6_min;
		bounds[3].second = t6_max;
	}
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	double x0_0 = state[0];
	double x0_1 = state[1];

	// Calculate x1 bounds
	{
		complex<double> t1 = x0_1*x0_1;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t4*t6-3.0*t11*t1);
		complex<double> t18 = pow(-12.0*t1*radius+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0/t18*(t1/4.0-t4/16.0);
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25-t26;
		complex<double> t29 = t27*t27;
		complex<double> t34 = sqrt(-3.0*(t26+t19-t25)*t29*t27);
		complex<double> t36_min = x0_0+x0_1*t27-t34/3.0;
		complex<double> t36_max = x0_0+x0_1*t27+t34/3.0;

		bounds[0].first = t36_min.real();
		bounds[0].second = t36_max.real();
	}

	{
		complex<double> t1 = x0_1*x0_1;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t4*t6-3.0*t11*t1);
		complex<double> t18 = pow(-12.0*t1*radius+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = pow(t18,-1)*(t1/4.0-t4/16.0);
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t32 = (1.0/2.0*im)*(t18/4.0+4.0*t24)*t27;
		complex<double> t33 = -t19+t25-t26+t32;
		complex<double> t35 = t33*t33;
		complex<double> t40 = sqrt(-3.0*(t26-t19+t25+t32)*t35*t33);
		complex<double> t42_min = x0_0+x0_1*t33-t40/3.0;
		complex<double> t42_max = x0_0+x0_1*t33+t40/3.0;

		bounds[0].first = min(bounds[0].first, t42_min.real());
		bounds[0].second = max(bounds[0].second, t42_max.real());
	}

	{
		complex<double> t1 = x0_1*x0_1;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t4*t6-3.0*t11*t1);
		complex<double> t18 = pow(-12.0*t1*radius+t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = pow(t18,-1)*(t1/4.0-t4/16.0);
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t32 = (-1.0/2.0*im)*(t18/4.0+4.0*t24)*t27;
		complex<double> t33 = -t19+t25-t26+t32;
		complex<double> t35 = t33*t33;
		complex<double> t40 = sqrt(-3.0*(t26-t19+t25+t32)*t35*t33);
		complex<double> t42_min = x0_0+x0_1*t33-t40/3.0;
		complex<double> t42_max = x0_0+x0_1*t33+t40/3.0;

		bounds[0].first = min(bounds[0].first, t42_min.real());
		bounds[0].second = max(bounds[0].second, t42_max.real());
	}

	// Calculate x2 bounds
	{
		complex<double> t1 = sqrt(4.0);
		complex<double> t2 = radius*radius;
		complex<double> t3 = sqrt(t2);
		complex<double> t6_min = x0_1-t3*t1/4.0;
		complex<double> t6_max = x0_1+t3*t1/4.0;

		bounds[1].first = t6_min.real();
		bounds[1].second = t6_max.real();
	}
#endif
}

void calc_forward_reachable_bounds(const state& state, const double& radius, BOUNDS& bounds) {
	bounds.resize(X_DIM);

#if (DYNAMICS == QUADROTOR)
	double x0_0 = state[0];
	double x0_1 = state[1];
	double x0_2 = state[2];
	double x0_3 = state[3];
	double x0_4 = state[4];
	double x0_5 = state[5];
	double x0_6 = state[6];
	double x0_7 = state[7];
	double x0_8 = state[8];
	double x0_9 = state[9];
	
	double j = inertia;
	double g = gravity;
	double m = mass;
	double r = control_penalty; // This is the r parameter to matrix R -- refer to maple
	double l = length;

	int info[1000];

	for (int i = 0; i < X_DIM; i++) {
		bounds[i].first = DBL_MAX;
		bounds[i].second = DBL_MIN;
	}

	// Calculate x1 bounds
	{
		/* Before polynomial solver
		double t1 = RootOf(64.0*g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*_Z-112.0*g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*radius+(126.0*j*j*r*g*g*x0_9*x0_9+49.0*g*g*radius*radius*l*l)*_Z*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_7*x0_9-126.0*radius*j*j*r*g*g*x0_9*x0_9)*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_7*x0_7-504.0*radius*j*j*r*g*g*x0_7*x0_9+504.0*j*j*r*x0_3*g*x0_9~)*_Z*_Z*_Z+(-504.0*radius*j*j*r*g*g*x0_7*x0_7-504.0*radius*j*j*r*x0_3*g*x0_9~+1008.0*j*j*r*x0_3*g*x0_7)*_Z*_Z+(-1008.0*radius*j*j*r*x0_3*g*x0_7+504.0*j*j*r*x0_3*x0_3)*_Z-504.0*radius*j*j*r*x0_3*x0_3);
		double t3 = t1*t1;
		double t7 = t3*t1;
		double t11 = sqrt(14.0);
		double t12 = g*g;
		double t13 = l*l;
		double t15 = j*j;
		double t19 = t3*t3;
		double t25 = sqrt(t12*t13/t15/r*t19*t7*(radius-t1));
		double t28_min = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0-t11*t25/42.0;
		double t28_max = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0+t11*t25/42.0;

		bounds[0].first = t28_min;
		bounds[0].second = t28_max;
		*/

		const int degree = 7;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 64.0*g*g*l*l;
		p[1] = -112.0*g*g*l*l*radius;
		p[2] = (126.0*j*j*r*g*g*x0_9*x0_9+49.0*g*g*radius*radius*l*l);
		p[3] = (504.0*j*j*r*g*g*x0_7*x0_9-126.0*radius*j*j*r*g*g*x0_9*x0_9);
		p[4] = (504.0*j*j*r*g*g*x0_7*x0_7-504.0*radius*j*j*r*g*g*x0_7*x0_9+504.0*j*j*r*x0_3*g*x0_9);
		p[5] = (-504.0*radius*j*j*r*g*g*x0_7*x0_7-504.0*radius*j*j*r*x0_3*g*x0_9+1008.0*j*j*r*x0_3*g*x0_7);
		p[6] = (-1008.0*radius*j*j*r*x0_3*g*x0_7+504.0*j*j*r*x0_3*x0_3);
		p[7] = -504.0*radius*j*j*r*x0_3*x0_3;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t3 = t1*t1;
			complex<double> t7 = t3*t1;
			complex<double> t11 = sqrt(14.0);
			complex<double> t12 = g*g;
			complex<double> t13 = l*l;
			complex<double> t15 = j*j;
			complex<double> t19 = t3*t3;
			complex<double> t25 = sqrt(t12*t13/t15/r*t19*t7*(radius-t1));
			complex<double> t28_min = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0-t11*t25/42.0;
			complex<double> t28_max = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0+t11*t25/42.0;

			bounds[0].first = min(bounds[0].first, t28_min.real());
			bounds[0].second = max(bounds[0].second, t28_max.real());
		}
	}

	// Calculate x2 bounds
	{
		/* Before polynomial solver
		double t1 = RootOf(64.0*g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*_Z-112.0*g*g*l*l*_Z*_Z*_Z*_Z*Z*_Z*radius+(126.0*j*j*r*g*g*x0_8*x0_8+49.0*g*g*radius*radius*l*l)*_Z*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_6*x0_8-126.0*radius*j*j*r*g*g*x0_8*x0_8)*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_6*x0_6-504.0*radius*j*j*r*g*g*x0_6*x0_8-504.0*j*j*r*x0_4*g*x0_8~)*_Z*_Z*_Z+(-504.0*radius*j*j*r*g*g*x0_6*x0_6+504.0*radius*j*j*r*x0_4*g*x0_8~-1008.0*j*j*r*x0_4*g*x0_6)*_Z*_Z+(1008.0*radius*j*j*r*x0_4*g*x0_6+504.0*j*1*r*x0_4*x0_4)*_Z-504.0*radius*j*j*r*x0_4*x0_4);
		double t3 = t1*t1;
		double t7 = t3*t1;
		double t11 = sqrt(14.0);
		double t12 = g*g;
		double t13 = l*l;
		double t15 = j*j;
		double t19 = t3*t3;
		double t25 = sqrt(t12*t13/t15/r*t19*t7*(radius-t1));
		double t28_min = x0_1+t1*x0_4-g*t3*x0_6/2.0-g*t7*x0_8/6.0-t11*t25/42.0;
		double t28_max = x0_1+t1*x0_4-g*t3*x0_6/2.0+g*t7*x0_8/6.0-t11*t25/42.0;

		bounds[1].first = t28_min;
		bounds[1].second = t28_max;
		*/

		const int degree = 7;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 64.0*g*g*l*l;
		p[1] = -112.0*g*g*l*l*radius;
		p[2] = (126.0*j*j*r*g*g*x0_8*x0_8+49.0*g*g*radius*radius*l*l);
		p[3] = (504.0*j*j*r*g*g*x0_6*x0_8-126.0*radius*j*j*r*g*g*x0_8*x0_8);
		p[4] = (504.0*j*j*r*g*g*x0_6*x0_6-504.0*radius*j*j*r*g*g*x0_6*x0_8-504.0*j*j*r*x0_4*g*x0_8);
		p[5] = (-504.0*radius*j*j*r*g*g*x0_6*x0_6+504.0*radius*j*j*r*x0_4*g*x0_8-1008.0*j*j*r*x0_4*g*x0_6);
		p[6] = (1008.0*radius*j*j*r*x0_4*g*x0_6+504.0*j*1*r*x0_4*x0_4);
		p[7] = -504.0*radius*j*j*r*x0_4*x0_4;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t3 = t1*t1;
			complex<double> t7 = t3*t1;
			complex<double> t11 = sqrt(14.0);
			complex<double> t12 = g*g;
			complex<double> t13 = l*l;
			complex<double> t15 = j*j;
			complex<double> t19 = t3*t3;
			complex<double> t25 = sqrt(t12*t13/t15/r*t19*t7*(radius-t1));
			complex<double> t28_min = x0_1+t1*x0_4-g*t3*x0_6/2.0-g*t7*x0_8/6.0-t11*t25/42.0;
			complex<double> t28_max = x0_1+t1*x0_4-g*t3*x0_6/2.0+g*t7*x0_8/6.0-t11*t25/42.0;

			bounds[1].first = min(bounds[1].first, t28_min.real());
			bounds[1].second = max(bounds[1].second, t28_max.real());
		}
	}

	// Calculate x3 bounds
	{
		complex<double> t1 = x0_5*x0_5;
		complex<double> t3 = m*m;
		complex<double> t4 = t3*r;
		complex<double> t7 = radius*radius;
		complex<double> t9 = r*r;
		complex<double> t11 = t3*t3;
		complex<double> t14 = t1*t1;
		complex<double> t21 = t7*t7;
		complex<double> t26 = sqrt(t9*r*t11*t3*t14*t1+6.0*t9*t11*t14*t7-3.0*t4*t1*t21);
		complex<double> t28 = pow(3.0*radius*t1*t4-t7*radius+t26,0.3333333333333333);
		complex<double> t29 = t28/4.0;
		complex<double> t34 = (t4*t1-t7)/t28/4.0;
		complex<double> t35 = radius/2.0;
		complex<double> t36 = t29-t34+t35;
		complex<double> t38 = sqrt(3.0);
		complex<double> t43 = t36*t36;
		complex<double> t47 = sqrt((t35-t29+t34)/t3/r*t43*t36);
		complex<double> t50_min = x0_2+t36*x0_5-2.0/3.0*t38*t47;
		complex<double> t50_max = x0_2+t36*x0_5+2.0/3.0*t38*t47;

		bounds[2].first = t50_min.real();
		bounds[2].second = t50_max.real();
	}

	{
		complex<double> t1 = x0_5*x0_5;
		complex<double> t3 = m*m;
		complex<double> t4 = t3*r;
		complex<double> t7 = radius*radius;
		complex<double> t9 = r*r;
		complex<double> t11 = t3*t3;
		complex<double> t14 = t1*t1;
		complex<double> t21 = t7*t7;
		complex<double> t26 = sqrt(t9*r*t11*t3*t14*t1+6.0*t9*t11*t14*t7-3.0*t4*t1*t21);
		complex<double> t28 = pow(3.0*radius*t1*t4-t7*radius+t26,0.3333333333333333);
		complex<double> t29 = t28/8.0;
		complex<double> t33 = (t4*t1-t7)/t28/16.0;
		complex<double> t34 = 2.0*t33;
		complex<double> t35 = radius/2.0;
		complex<double> t36 = sqrt(3.0);
		complex<double> t40 = t28/4.0+4.0*t33;
		complex<double> t42 = -t29+t34+t35+(1.0/2.0*im)*t36*t40;
		complex<double> t50 = t42*t42;
		complex<double> t54 = sqrt((t35+t29-t34+(-1.0/2.0*im)*t36*t40)/t3/r*t50*t42);
		complex<double> t57_min = x0_2+t42*x0_5-2.0/3.0*t36*t54;
		complex<double> t57_max = x0_2+t42*x0_5+2.0/3.0*t36*t54;

		bounds[2].first = min(bounds[2].first, t57_min.real());
		bounds[2].second = max(bounds[2].second, t57_max.real());
	}

	{
		complex<double> t1 = x0_5*x0_5;
		complex<double> t3 = m*m;
		complex<double> t4 = t3*r;
		complex<double> t7 = radius*radius;
		complex<double> t9 = r*r;
		complex<double> t11 = t3*t3;
		complex<double> t14 = t1*t1;
		complex<double> t21 = t7*t7;
		complex<double> t26 = sqrt(t9*r*t11*t3*t14*t1+6.0*t9*t11*t14*t7-3.0*t4*t1*t21);
		complex<double> t28 = pow(3.0*radius*t1*t4-t7*radius+t26,0.3333333333333333);
		complex<double> t29 = t28/8.0;
		complex<double> t33 = (t4*t1-t7)/t28/16.0;
		complex<double> t34 = 2.0*t33;
		complex<double> t35 = radius/2.0;
		complex<double> t36 = sqrt(3.0);
		complex<double> t40 = t28/4.0+4.0*t33;
		complex<double> t42 = -t29+t34+t35+(-1.0/2.0*im)*t36*t40;
		complex<double> t50 = t42*t42;
		complex<double> t54 = sqrt((t35+t29-t34+(1.0/2.0*im)*t36*t40)/t3/r*t50*t42);
		complex<double> t57_min = x0_2+t42*x0_5-2.0/3.0*t36*t54;
		complex<double> t57_max = x0_2+t42*x0_5+2.0/3.0*t36*t54;

		bounds[2].first = min(bounds[2].first, t57_min.real());
		bounds[2].second = max(bounds[2].second, t57_max.real());
	}

	// Calculate x4 bounds
	{
		const int degree = 5;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 36.0*l*l;
		p[1] = -60.0*l*l*radius;
		p[2] = (25.0*l*l*radius*radius+40.0*j*j*r*x0_9*x0_9);
		p[3] = (-40.0*radius*j*j*r*x0_9*x0_9+80.0*x0_9*x0_7*r*j*j);
		p[4] = (-80.0*radius*j*j*r*x0_7*x0_9+40.0*x0_7*x0_7*r*j*j);
		p[5] = -40.0*radius*j*j*r*x0_7*x0_7;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[i]);
			complex<double> t4 = t1*t1;
			complex<double> t8 = sqrt(10.0);
			complex<double> t9 = g*g;
			complex<double> t10 = t4*t4;
			complex<double> t13 = l*l;
			complex<double> t15 = j*j;
			complex<double> t22 = sqrt(t9*t10*t1*t13/t15/r*(radius-t1));
			complex<double> t25_min = x0_3+g*t1*x0_7+g*t4*x0_9/2.0-t8*t22/10.0;
			complex<double> t25_max = x0_3+g*t1*x0_7+g*t4*x0_9/2.0+t8*t22/10.0;

			bounds[3].first = min(bounds[3].first, t25_min.real());
			bounds[3].second = max(bounds[3].second, t25_max.real());
		}

		/* Before polynomial solver
		double t1 = RootOf(36.0*_Z*_Z*_Z*_Z*_Z*l*l-60.0*_Z*_Z*_Z*_Z*l*l*radius+(25.0*l*l*radius*radius+40.0*j*j*r*x0_9*x0_9)*_Z*_Z*_Z+(-40.0*radius*j*j*r*x0_9*x0_9+80.0*x0_9*x0_7*r*j*j)*_Z*_Z+(-80.0*radius*j*j*r*x0_7*x0_9+40.0*x0_7*x0_7*r*j*j)*_Z-40.0*radius*j*j*r*x0_7*x0_7);
		double t4 = t1*t1;
		double t8 = sqrt(10.0);
		double t9 = g*g;
		double t10 = t4*t4;
		double t13 = l*l;
		double t15 = j*j;
		double t22 = sqrt(t9*t10*t1*t13/t15/r*(radius-t1));
		double t25_min = x0_3+g*t1*x0_7+g*t4*x0_9/2.0-t8*t22/10.0;
		double t25_max = x0_3+g*t1*x0_7+g*t4*x0_9/2.0+t8*t22/10.0;

		bounds[3].first = t25_min;
		bounds[3].second = t25_max;
		*/
	}

	// Calculate x5 bounds
	{
		const int degree = 5;
		double p[degree + 1];
		double zeror[degree], zeroi[degree];
		memset(zeror, 0, sizeof(double)*degree);
		memset(zeroi, 0, sizeof(double)*degree);

		p[0] = 36.0*l*l;
		p[1] = -60.0*l*l*radius;
		p[2] = (25.0*l*l*radius*radius+40.0*j*j*r*x0_8*x0_8);
		p[3] = (-40.0*radius*j*j*r*x0_8*x0_8+80.0*x0_8*x0_6*r*j*j);
		p[4] = (-80.0*radius*j*j*r*x0_6*x0_8+40.0*x0_6*x0_6*r*j*j);
		p[5] = -40.0*radius*j*j*r*x0_6*x0_6;

		int returned_roots = rpoly(p, degree, zeror, zeroi, info);

		for (int i = 0; i < returned_roots; i++) {
			complex<double> t1(zeror[i], zeroi[0]);
			complex<double> t4 = t1*t1;
			complex<double> t8 = sqrt(10.0);
			complex<double> t9 = g*g;
			complex<double> t10 = t4*t4;
			complex<double> t13 = l*l;
			complex<double> t15 = j*j;
			complex<double> t22 = sqrt(t9*t10*t1*t13/t15/r*(radius-t1));
			complex<double> t25_min = x0_4-g*t1*x0_6-g*t4*x0_8/2.0-t8*t22/10.0;
			complex<double> t25_max = x0_4-g*t1*x0_6-g*t4*x0_8/2.0+t8*t22/10.0;

			bounds[4].first = min(bounds[4].first, t25_min.real());
			bounds[4].second = max(bounds[4].second, t25_max.real());
		}

		/* Before polynomial solver
		double t1 = RootOf(36.0*_Z*_Z*_Z*_Z*_Z*l*l-60.0*_Z*_Z*_Z*_Z*l*l*radius+(25.0*l*l*radius*radius+40.0*j*j*r*x0_8*x0_8)*_Z*_Z*_Z+(-40.0*radius*j*j*r*x0_8*x0_8+80.0*x0_8*x0_6*r*j*j)*_Z*_Z+(-80.0*radius*j*j*r*x0_6*x0_8+40.0*x0_6*x0_6*r*j*j)*_Z-40.0*radius*j*j*r*x0_6*x0_6);
		double t4 = t1*t1;
		double t8 = sqrt(10.0);
		double t9 = g*g;
		double t10 = t4*t4;
		double t13 = l*l;
		double t15 = j*j;
		double t22 = sqrt(t9*t10*t1*t13/t15/r*(radius-t1));
		double t25_min = x0_4-g*t1*x0_6-g*t4*x0_8/2.0-t8*t22/10.0;
		double t25_max = x0_4-g*t1*x0_6-g*t4*x0_8/2.0+t8*t22/10.0;

		bounds[4].first = t25_min;
		bounds[4].second = t25_max;
		*/
	}

	// Calculate x6 bounds
	{
		complex<double> t1 = sqrt(4.0);
		complex<double> t2 = radius*radius;
		complex<double> t3 = m*m;
		complex<double> t8 = sqrt(t2/t3/r);
		complex<double> t11_min = x0_5-t1*t8/2.0;
		complex<double> t11_max = x0_5+t1*t8/2.0;

		bounds[5].first = t11_min.real();
		bounds[5].second = t11_max.real();
	}

	// Calculate x7 bounds
	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_8*x0_8;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t11*t11;
		complex<double> t27 = t9*t9;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t11*t9-3.0*t26*t27));
		complex<double> t36 = pow(6.0*radius*l*t6-t9*radius*t11*l+t14*j*x0_8*t32,0.3333333333333333);
		complex<double> t38 = t1*t36/4.0;
		complex<double> t45 = (2.0*t6-t11*t9)*t1/t36/4.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = t38-t45+t46;
		complex<double> t49 = sqrt(6.0);
		complex<double> t53 = t47*t47;
		complex<double> t59 = sqrt(t11/t3/r*t53*t47*(t46-t38+t45));
		complex<double> t62_min = x0_6+t47*x0_8-t49*t59/3.0;
		complex<double> t62_max = x0_6+t47*x0_8-t49*t59/3.0;

		bounds[6].first = t62_min.real();
		bounds[6].second = t62_max.real();
	}

	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_8*x0_8;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t11*t11;
		complex<double> t27 = t9*t9;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t11*t9-3.0*t26*t27));
		complex<double> t36 = pow(6.0*radius*l*t6-t9*radius*t11*l+t14*j*x0_8*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t11*t9)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t49 = t37+t44;
		complex<double> t51 = -t38+t45+t46+(1.0/8.0*im)*t47*t49;
		complex<double> t53 = sqrt(6.0);
		complex<double> t57 = t51*t51;
		complex<double> t65 = sqrt(t11/t3/r*t51*t57*(t46+t38-t45+(-1.0/8.0*im)*t47*t49));
		complex<double> t68_min = x0_6+t51*x0_8-t53*t65/3.0;
		complex<double> t68_max = x0_6+t51*x0_8+t53*t65/3.0;

		bounds[6].first = min(bounds[6].first, t68_min.real());
		bounds[6].second = max(bounds[6].second, t68_max.real());
	}

	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_8*x0_8;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t11*t11;
		complex<double> t27 = t9*t9;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t11*t9-3.0*t26*t27));
		complex<double> t36 = pow(6.0*radius*l*t6-t9*radius*t11*l+t14*j*x0_8*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t11*t9)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t49 = t37+t44;
		complex<double> t51 = -t38+t45+t46+(-1.0/8.0*im)*t47*t49;
		complex<double> t53 = sqrt(6.0);
		complex<double> t57 = t51*t51;
		complex<double> t65 = sqrt(t11/t3/r*t51*t57*(t46+t38-t45+(1.0/8.0*im)*t47*t49));
		complex<double> t68_min = x0_6+t51*x0_8-t53*t65/3.0;
		complex<double> t68_max = x0_6+t51*x0_8+t53*t65/3.0;

		bounds[6].first = min(bounds[6].first, t68_min.real());
		bounds[6].second = max(bounds[6].second, t68_max.real());
	}

	// Calculate x8 bounds
	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_9*x0_9;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t11*t11;
		complex<double> t27 = t9*t9;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t11*t9-3.0*t26*t27));
		complex<double> t36 = pow(6.0*radius*l*t6-t9*radius*t11*l+t14*j*x0_9*t32, 0.3333333333333333);
		complex<double> t38 = t1*t36/4.0;
		complex<double> t45 = (2.0*t6-t11*t9)*t1/t36/4.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = t38-t45+t46;
		complex<double> t49 = sqrt(6.0);
		complex<double> t53 = t47*t47;
		complex<double> t59 = sqrt(t11/t3/r*t53*t47*(t46-t38+t45));
		complex<double> t62_min = x0_7+t47*x0_9-t49*t59/3.0;
		complex<double> t62_max = x0_7+t47*x0_9+t49*t59/3.0;

		bounds[7].first = t62_min.real();
		bounds[7].second = t62_max.real();
	}

	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_9*x0_9;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t11*t11;
		complex<double> t27 = t9*t9;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t11*t9-3.0*t26*t27));
		complex<double> t36 = pow(6.0*radius*l*t6-t9*radius*t11*l+t14*j*x0_9*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t11*t9)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t49 = t37+t44;
		complex<double> t51 = -t38+t45+t46+(1.0/8.0*im)*t47*t49;
		complex<double> t53 = sqrt(6.0);
		complex<double> t57 = t51*t51;
		complex<double> t65 = sqrt(t11/t3/r*t51*t57*(t46+t38-t45+(-1.0/8.0*im)*t47*t49));
		complex<double> t68_min = x0_7+t51*x0_9-t53*t65/3.0;
		complex<double> t68_max = x0_7+t51*x0_9+t53*t65/3.0;

		bounds[7].first = min(bounds[7].first, t68_min.real());
		bounds[7].second = max(bounds[7].second, t68_max.real());
	}

	{
		complex<double> t1 = 1/l;
		complex<double> t3 = j*j;
		complex<double> t4 = t3*r;
		complex<double> t5 = x0_9*x0_9;
		complex<double> t6 = t4*t5;
		complex<double> t9 = radius*radius;
		complex<double> t11 = l*l;
		complex<double> t14 = sqrt(2.0);
		complex<double> t16 = t3*t3;
		complex<double> t17 = r*r;
		complex<double> t19 = t5*t5;
		complex<double> t26 = t11*t11;
		complex<double> t27 = t9*t9;
		complex<double> t32 = sqrt(r*(4.0*t16*t17*t19+12.0*t4*t5*t11*t9-3.0*t26*t27));
		complex<double> t36 = pow(6.0*radius*l*t6-t9*radius*t11*l+t14*j*x0_9*t32, 0.3333333333333333);
		complex<double> t37 = t1*t36;
		complex<double> t38 = t37/8.0;
		complex<double> t44 = (2.0*t6-t11*t9)*t1/t36;
		complex<double> t45 = t44/8.0;
		complex<double> t46 = radius/2.0;
		complex<double> t47 = sqrt(3.0);
		complex<double> t49 = t37+t44;
		complex<double> t51 = -t38+t45+t46+(-1.0/8.0*im)*t47*t49;
		complex<double> t53 = sqrt(6.0);
		complex<double> t57 = t51*t51;
		complex<double> t65 = sqrt(t11/t3/r*t51*t57*(t46+t38-t45+(1.0/8.0*im)*t47*t49));
		complex<double> t68_min = x0_7+t51*x0_9-t53*t65/3.0;
		complex<double> t68_max = x0_7+t51*x0_9-t53*t65/3.0;

		bounds[7].first = min(bounds[7].first, t68_min.real());
		bounds[7].second = max(bounds[7].second, t68_max.real());
	}

	// Calculate x9 bounds
	{
		complex<double> t1 = sqrt(2.0);
		complex<double> t2 = sqrt(4.0);
		complex<double> t4 = l*l;
		complex<double> t5 = j*j;
		complex<double> t9 = radius*radius;
		complex<double> t12 = sqrt(t4/t5/r*t9);
		complex<double> t15_min = x0_8-t1*t2*t12/4.0;
		complex<double> t15_max = x0_8+t1*t2*t12/4.0;

		bounds[8].first = t15_min.real();
		bounds[8].second = t15_max.real();
	}

	// Calculate x10 bounds
	{
		complex<double> t1 = sqrt(2.0);
		complex<double> t2 = sqrt(4.0);
		complex<double> t4 = l*l;
		complex<double> t5 = j*j;
		complex<double> t9 = radius*radius;
		complex<double> t12 = sqrt(t4/t5/r*t9);
		complex<double> t15_min = x0_9-t1*t2*t12/4.0;
		complex<double> t15_max = x0_9+t1*t2*t12/4.0;

		bounds[9].first = t15_min.real();
		bounds[9].second = t15_max.real();
	}
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	double x0_0 = state[0];
	double x0_1 = state[1];
	double x0_2 = state[2];
	double x0_3 = state[3];

	// Calculate x1 min
	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25+t26;
		complex<double> t29 = sqrt(3.0);
		complex<double> t31 = t27*t27;
		complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
		complex<double> t37 = x0_0+t27*x0_2-t29*t34/3.0;

		bounds[0].first = t37.real();
	}

	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_0+t33*x0_2-t27*t41/3.0;

		bounds[0].first = min(bounds[0].first, t44.real());
	}

	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(-1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_0+t33*x0_2-t27*t41/3.0;

		bounds[0].first = min(bounds[0].first, t44.real());
	}

	// Calculate x1 max
	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25+t26;
		complex<double> t29 = sqrt(3.0);
		complex<double> t31 = t27*t27;
		complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
		complex<double> t37 = x0_0+t27*x0_2+t29*t34/3.0;

		bounds[0].second = t37.real();
	}

	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_0+t33*x0_2+t27*t41/3.0;

		bounds[0].second = max(bounds[0].second, t44.real());
	}

	{
		complex<double> t1 = x0_2*x0_2;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(-1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_0+t33*x0_2+t27*t41/3.0;

		bounds[0].second = max(bounds[0].second, t44.real());
	}

	// Calculate x2 min
	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25+t26;
		complex<double> t29 = sqrt(3.0);
		complex<double> t31 = t27*t27;
		complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
		complex<double> t37 = x0_1+t27*x0_3-t29*t34/3.0;

		bounds[1].first = t37.real();
	}

	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_1+t33*x0_3-t27*t41/3.0;

		bounds[1].first = min(bounds[1].first, t44.real());
	}

	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(-1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_1+t33*x0_3-t27*t41/3.0;

		bounds[1].first = min(bounds[1].first, t44.real());
	}

	// Calculate x2 max
	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/4.0;
		complex<double> t25 = 4.0*(t1/4.0-t4/16.0)/t18;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = t19-t25+t26;
		complex<double> t29 = sqrt(3.0);
		complex<double> t31 = t27*t27;
		complex<double> t34 = sqrt((t26-t19+t25)*t31*t27);
		complex<double> t37 = x0_1+t27*x0_3+t29*t34/3.0;

		bounds[1].second = t37.real();
	}

	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(-1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_1+t33*x0_3+t27*t41/3.0;

		bounds[1].second = max(bounds[1].second, t44.real());
	}

	{
		complex<double> t1 = x0_3*x0_3;
		complex<double> t4 = radius*radius;
		complex<double> t6 = t1*t1;
		complex<double> t11 = t4*t4;
		complex<double> t15 = sqrt(16.0*t6*t1+24.0*t6*t4-3.0*t1*t11);
		complex<double> t18 = pow(12.0*radius*t1-t4*radius+2.0*t15,0.3333333333333333);
		complex<double> t19 = t18/8.0;
		complex<double> t24 = (t1/4.0-t4/16.0)/t18;
		complex<double> t25 = 2.0*t24;
		complex<double> t26 = radius/2.0;
		complex<double> t27 = sqrt(3.0);
		complex<double> t31 = t18/4.0+4.0*t24;
		complex<double> t33 = -t19+t25+t26+(-1.0/2.0*im)*t27*t31;
		complex<double> t38 = t33*t33;
		complex<double> t41 = sqrt((t26+t19-t25+(1.0/2.0*im)*t27*t31)*t38*t33);
		complex<double> t44 = x0_1+t33*x0_3+t27*t41/3.0;

		bounds[1].second = max(bounds[1].second, t44.real());
	}

	// Calculate min/max bounds for x3
	{
		double t1 = sqrt(4.0);
		double t2 = radius*radius;
		double t3 = sqrt(t2);

		bounds[2].first = x0_2-t1*t3/4.0;
		bounds[2].second = x0_2+t1*t3/4.0;
	}

	// Calculate min/max bounds for x4
	{
		double t1 = sqrt(4.0);
		double t2 = radius*radius;
		double t3 = sqrt(t2);

		bounds[3].first = x0_3-t1*t3/4.0;
		bounds[3].second = x0_3+t1*t3/4.0;
	}

#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	double x0_0 = state[0];
	double x0_1 = state[1];

	// Calculate x1 min
	{
		complex<double> t1 = radius*radius;
		complex<double> t2 = t1*radius;
		complex<double> t3 = x0_1*x0_1;
		complex<double> t5 = 12.0*radius*t3;
		complex<double> t6 = t3*t3;
		complex<double> t10 = t1*t1;
		complex<double> t13 = sqrt(16.0*t6+24.0*t3*t1-3.0*t10);
		complex<double> t14 = fabs(x0_1);
		complex<double> t18 = pow(-t2+t5+2.0*t13*t14,0.3333333333333333);
		complex<double> t19 = t18*t18;
		complex<double> t28 = sqrt(3.0);
		complex<double> t36 = sqrt(16.0*t6*t3+24.0*t6*t1-3.0*t3*t10);
		complex<double> t39 = pow(t5-t2+2.0*t36,0.3333333333333333);
		complex<double> t40 = t39/4.0;
		complex<double> t46 = 4.0*(t3/4.0-t1/16.0)/t39;
		complex<double> t47 = radius/2.0;
		complex<double> t48 = t40-t46+t47;
		complex<double> t49 = t48*t48;
		complex<double> t53 = sqrt(t49*t48*(t47-t40+t46));
		complex<double> t56 = x0_0+(t19-4.0*t3+t1+2.0*radius*t18)/t18*x0_1/4.0-t28*t53/3.0;

		bounds[0].first = t56.real();
	}

	{
		complex<double> t1 = radius*radius;
		complex<double> t2 = t1*radius;
		complex<double> t3 = x0_1*x0_1;
		complex<double> t5 = 12.0*radius*t3;
		complex<double> t6 = t3*t3;
		complex<double> t10 = t1*t1;
		complex<double> t13 = sqrt(16.0*t6+24.0*t3*t1-3.0*t10);
		complex<double> t14 = fabs(x0_1);
		complex<double> t18 = pow(-t2+t5+2.0*t13*t14,0.3333333333333333);
		complex<double> t19 = t18*t18;
		complex<double> t26 = pow(t18,-1);
		complex<double> t28 = sqrt(3.0);
		complex<double> t33 = t28*t1;
		complex<double> t39 = sqrt(6.0);
		complex<double> t47 = sqrt(16.0*t6*t3+24.0*t6*t1-3.0*t3*t10);
		complex<double> t49 = t5-t2+2.0*t47;
		complex<double> t50 = pow(t49,0.3333333333333333);
		complex<double> t51 = t50*t50;
		complex<double> t59 = t51-4.0*t3+t1-4.0*radius*t50+(-im)*t28*t51+(-4.0*im)*t28*t3+(im)*t33;
		complex<double> t60 = t59*t59;
		complex<double> t70 = (t3/4.0-t1/16.0)/t50;
		complex<double> t79 = sqrt(-t60*t59/t49*(radius/2.0+t50/8.0-2.0*t70+(-1.0/2.0*im)*t28*(t50/4.0+4.0*t70)));
		complex<double> t82 = x0_0+((-t19/8.0+t3/2.0-t1/8.0+radius*t18/2.0)*t26+(im)*(t28*t19/8.0+t28*t3/2.0-t33/8.0)*t26)*x0_1-t39*t79/96.0;

		bounds[0].first = min(bounds[0].first, t82.real());
	}

	{
		complex<double> t1 = radius*radius;
		complex<double> t2 = t1*radius;
		complex<double> t3 = x0_1*x0_1;
		complex<double> t5 = 12.0*radius*t3;
		complex<double> t6 = t3*t3;
		complex<double> t10 = t1*t1;
		complex<double> t13 = sqrt(16.0*t6+24.0*t3*t1-3.0*t10);
		complex<double> t14 = fabs(x0_1);
		complex<double> t18 = pow(-t2+t5+2.0*t13*t14,0.3333333333333333);
		complex<double> t19 = t18*t18;
		complex<double> t26 = pow(t18,-1);
		complex<double> t28 = sqrt(3.0);
		complex<double> t33 = t28*t1;
		complex<double> t46 = sqrt(16.0*t6*t3+24.0*t6*t1-3.0*t3*t10);
		complex<double> t48 = t5-t2+2.0*t46;
		complex<double> t49 = pow(t48,0.3333333333333333);
		complex<double> t50 = t49*t49;
		complex<double> t58 = -t50+4.0*t3-t1+4.0*radius*t49+(-im)*t28*t50+(-4.0*im)*t28*t3+(im)*t33;
		complex<double> t59 = t58*t58;
		complex<double> t69 = (t3/4.0-t1/16.0)/t49;
		complex<double> t79 = sqrt(6.0)*sqrt(t59*t58/t48*(radius/2.0+t49/8.0-2.0*t69+(1.0/2.0*im)*t28*(t49/4.0+4.0*t69)));
		complex<double> t81 = x0_0+((-t19/8.0+t3/2.0-t1/8.0+radius*t18/2.0)*t26+(im)*(-t28*t19/8.0-t28*t3/2.0+t33/8.0)*t26)*x0_1-t79/96.0;

		bounds[0].first = min(bounds[0].first, t81.real());
	}

	// Calculate x1 max
	{
		complex<double> t1 = radius*radius;
		complex<double> t2 = t1*radius;
		complex<double> t3 = x0_1*x0_1;
		complex<double> t5 = 12.0*radius*t3;
		complex<double> t6 = t3*t3;
		complex<double> t10 = t1*t1;
		complex<double> t13 = sqrt(16.0*t6+24.0*t3*t1-3.0*t10);
		complex<double> t14 = fabs(x0_1);
		complex<double> t18 = pow(-t2+t5+2.0*t13*t14,0.3333333333333333);
		complex<double> t19 = t18*t18;
		complex<double> t28 = sqrt(3.0);
		complex<double> t36 = sqrt(16.0*t6*t3+24.0*t6*t1-3.0*t3*t10);
		complex<double> t39 = pow(t5-t2+2.0*t36,0.3333333333333333);
		complex<double> t40 = t39/4.0;
		complex<double> t46 = 4.0*(t3/4.0-t1/16.0)/t39;
		complex<double> t47 = radius/2.0;
		complex<double> t48 = t40-t46+t47;
		complex<double> t49 = t48*t48;
		complex<double> t53 = sqrt(t49*t48*(t47-t40+t46));
		complex<double> t56 = x0_0+(t19-4.0*t3+t1+2.0*radius*t18)/t18*x0_1/4.0+t28*t53/3.0;

		bounds[0].second = t56.real();
	}

	{
		complex<double> t1 = radius*radius;
		complex<double> t2 = t1*radius;
		complex<double> t3 = x0_1*x0_1;
		complex<double> t5 = 12.0*radius*t3;
		complex<double> t6 = t3*t3;
		complex<double> t10 = t1*t1;
		complex<double> t13 = sqrt(16.0*t6+24.0*t3*t1-3.0*t10);
		complex<double> t14 = fabs(x0_1);
		complex<double> t18 = pow(-t2+t5+2.0*t13*t14,0.3333333333333333);
		complex<double> t19 = t18*t18;
		complex<double> t26 = pow(t18,-1);
		complex<double> t28 = sqrt(3.0);
		complex<double> t46 = sqrt(16.0*t6*t3+24.0*t6*t1-3.0*t3*t10);
		complex<double> t48 = t5-t2+2.0*t46;
		complex<double> t49 = pow(t48,0.3333333333333333);
		complex<double> t50 = t49*t49;
		complex<double> t59 = -t50+4.0*t3-t1+4.0*radius*t49+(im)*t28*t50+(4.0*im)*t28*t3+(-im)*t28*t1;
		complex<double> t60 = t59*t59;
		complex<double> t70 = (t3/4.0-t1/16.0)/t49;
		complex<double> t80 = sqrt(6.0)*sqrt(t60*t59/t48*(radius/2.0+t49/8.0-2.0*t70+(-1.0/2.0*im)*t28*(t49/4.0+4.0*t70)));
		complex<double> t82 = x0_0+((-t19/8.0+t3/2.0-t1/8.0+radius*t18/2.0)*t26+(im)*(t28*t19/8.0+t28*t3/2.0-t28*t1/8.0)*t26)*x0_1+t80/96.0;

		bounds[0].second = max(bounds[0].second, t82.real());
	}

	{
		complex<double> t1 = radius*radius;
		complex<double> t2 = t1*radius;
		complex<double> t3 = x0_1*x0_1;
		complex<double> t5 = 12.0*radius*t3;
		complex<double> t6 = t3*t3;
		complex<double> t10 = t1*t1;
		complex<double> t13 = sqrt(16.0*t6+24.0*t3*t1-3.0*t10);
		complex<double> t14 = fabs(x0_1);
		complex<double> t18 = pow(-t2+t5+2.0*t13*t14,0.3333333333333333);
		complex<double> t19 = t18*t18;
		complex<double> t26 = pow(t18,-1);
		complex<double> t28 = sqrt(3.0);
		complex<double> t39 = sqrt(6.0);
		complex<double> t47 = sqrt(16.0*t6*t3+24.0*t6*t1-3.0*t3*t10);
		complex<double> t49 = t5-t2+2.0*t47;
		complex<double> t50 = pow(t49,0.3333333333333333);
		complex<double> t51 = t50*t50;
		complex<double> t60 = t51-4.0*t3+t1-4.0*radius*t50+(im)*t28*t51+(4.0*im)*t28*t3+(-im)*t28*t1;
		complex<double> t61 = t60*t60;
		complex<double> t71 = (t3/4.0-t1/16.0)/t50;
		complex<double> t80 = sqrt(-t61*t60/t49*(radius/2.0+t50/8.0-2.0*t71+(1.0/2.0*im)*t28*(t50/4.0+4.0*t71)));
		complex<double> t83 = x0_0+((-t19/8.0+t3/2.0-t1/8.0+radius*t18/2.0)*t26+(im)*(-t28*t19/8.0-t28*t3/2.0+t28*t1/8.0)*t26)*x0_1+t39*t80/96.0;

		bounds[0].second = max(bounds[0].second, t83.real());
	}

	// Calculate x2 min & max
	complex<double> t1 = radius*radius;
	complex<double> t2 = sqrt(t1);
	complex<double> t4 = x0_1+t2/2.0;
	double min_value = (x0_1-t2/2.0).real();
	double max_value = (x0_1+t2/2.0).real();
	bounds[1].first = min_value;
	bounds[1].second = max_value;
#endif
}

void rrtstar(const state& x_init, const state& x_final, int n, double radius, tree_t& tree) {
	// local variables
	ostringstream os;
	node_ids_t k_d_results;
	int xdims = X_DIM;
	int expected_nodes = 2*TARGET_NODES;
	KD_Tree k_d_tree(tree, xdims, x_bounds, expected_nodes);
	BOUNDS k_d_query;

	// Construct start and goal nodes
	Node finalNode;
	finalNode.parent = NO_PARENT;
	finalNode.x = x_final;
	finalNode.cost_from_start = DBL_MAX;

	Node startNode;
	startNode.parent = NO_PARENT;
	startNode.x = x_init;
	startNode.cost_from_start = 0;

	// Add start and goal nodes to RRT and k-d tree
	tree.push_back(finalNode);
	tree.push_back(startNode);

	k_d_tree.add(0);
	k_d_tree.add(1);

	std::vector<node_id_t> orphans;
	orphans.push_back(0); // push goal on orphans list

#ifndef EXPERIMENT
	bool draw_path = false; // Determines whether a short path was found with the addition of any given node.
#endif

	// Create n nodes -- counter variable only incremented when a valid new node is found
	for (int i = 1; i <= n;) {
#ifndef EXPERIMENT
		draw_path = false;
#endif

		// Generate a random configuration
		state x_rand;
#if (DYNAMICS == QUADROTOR) && (USE_OBSTACLES == 5)
		double area = rand_value(0, 1);

		if (area < 0.1) {
			rand_vec(x_rand, x_bounds_window_1);
		}

		else if (area < 0.2) {
			rand_vec(x_rand, x_bounds_window_2);
		}

		else {
			rand_vec(x_rand, x_bounds);
		}

#else
		rand_vec(x_rand, x_bounds);
#endif

		// Only continue if the configuration is not in collision
		if (!collision_free(x_rand)) {
			continue;
		}

#if (DYNAMICS == NONHOLONOMIC)
		double cth = cos(x_rand[4]);
		double sth = sin(x_rand[4]);

		A.reset();
		A(0,2) = -x_rand[3]*sth;
		A(0,3) = cth;
		A(1,2) = x_rand[3]*cth;
		A(1,3) = sth;
		A(2,3) = x_rand[4];
		A(2,4) = x_rand[3];

		Matrix<X_DIM> f;
		f.reset();
		f[0] = A(1,2);
		f[1] = -A(0,2);
		f[2] = x_rand[3]*x_rand[4];

		c = f - A*x_rand;
#endif

		// TODO DWEBB Loop through all nodes in the tree within radius according heuristic, sort them by the heuristic value then try to connect them in that order up to radius
		// Find a configuration in the graph near the random configuration
		double min_dist = DBL_MAX;
		node_id_t x_near_id = NO_PARENT;
		double cost;
		double tau;

		priority_queue<node_cost_pair_t, vector<node_cost_pair_t >, greater<node_cost_pair_t > > Q;

#ifdef K_D_TREE_BACKWARD
		k_d_query.clear();
		BOUNDS q_bounds;
		calc_backward_reachable_bounds(x_rand, radius, k_d_query);

		// Perform query
		k_d_results.clear();
		k_d_tree.range_query(k_d_query, k_d_results);

		for (node_ids_t::iterator n = k_d_results.begin(); n != k_d_results.end(); n++) {
			Q.push(make_pair(applyHeuristics(tree[*n].x, x_rand), *n));
		}

		while (!Q.empty() && Q.top().first < min_dist) {
			node_id_t j = Q.top().second;
			Q.pop();

			if (connect(tree[j].x, x_rand, min(radius, min_dist - tree[j].cost_from_start), cost, tau, NULL)) {
				min_dist = tree[j].cost_from_start + cost;
				x_near_id = j;
			}

		}
#else
		Q.push(make_pair(applyHeuristics(tree[1].x, x_rand), 1)); // push start onto queue
		while (!Q.empty() && Q.top().first < min_dist) {
			node_id_t j = Q.top().second;
			Q.pop();

			if (connect(tree[j].x, x_rand, min(radius, min_dist - tree[j].cost_from_start), cost, tau, NULL)) {
				min_dist = tree[j].cost_from_start + cost;
				x_near_id = j;
			}

			// cost of node is lower bound of cost of its children, if connection succeeded, no child will do better
			else {
				for (list<node_id_t>::iterator p = tree[j].children.begin(); p != tree[j].children.end(); ++p) {
					Q.push(make_pair(tree[*p].cost_from_start + applyHeuristics(tree[*p].x, x_rand), *p));
				}
			}
		}
#endif
		if (x_near_id == NO_PARENT) {
#ifndef ALLOW_ORPHANS
			continue;
#else
			orphans.push_back(tree.size());
#endif
		}

		double time_diff = (double)(clock() - start_time)/(double)CLOCKS_PER_SEC;
		cout << "                                                                              \r";
		cout << setw(9) << time_diff << " " << setw(11) << i << "/" << n << " tree: " << setw(9) << tree.size() << " radius: " << setw(9) << radius << " orphans: " << orphans.size() << "\r";

		// Update the tree
		node_id_t x_rand_node_id = tree.size();

		// Create a node for the new configuration
		Node x_rand_node;
		x_rand_node.parent = x_near_id;
		x_rand_node.x = x_rand;
		x_rand_node.cost_from_start = min_dist;

		// Add it to the tree and k-d tree
		tree.push_back(x_rand_node);
		k_d_tree.add(x_rand_node_id);

		if ((i % 1000) == 0) {
			os.str("");
			os << i << "\t" << time_diff << endl;
			fputs(os.str().c_str(), time_log);
			fflush(time_log);
		}

		i++;

		if (x_near_id != NO_PARENT) {

			// This starts from the first. Instead it should grab a region in space that the new state (x_rand)
			// can reach. It should then search those for the cheaper connections and rewire as appropriate.
#ifdef K_D_TREE_FORWARD
			// These variables simplify the conversion from Maple to C/C++
			double j = inertia;
			double g = gravity;
			double m = mass;
			double r = control_penalty; // This is the r parameter to matrix R -- refer to maple
			double l = length;

			// Build the query for the k-d tree
			k_d_query.clear();
			double temp_sqrt;
			pair<double, double> temp_bounds;
#define CALC_BOUNDS(x_rand, val)										\
	temp_sqrt = sqrt((double)val);												\
	temp_bounds = make_pair(x_rand-temp_sqrt, x_rand+temp_sqrt);

//#define CALC_BOUNDS(x_rand, min_val, max_val)										\
//	temp_bounds = make_pair(min_val, max_val);
	//std::cout << "\n\rfirst: " << temp_bounds.first << "\tsecond: " << temp_bounds.second;

	double min_value = DBL_MAX;
	double max_value = DBL_MIN;

#if (DYNAMICS == DOUBLE_INTEGRATOR_1D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == QUADROTOR) || (DYNAMICS == NONHOLONOMIC)
	calc_forward_reachable_bounds(x_rand_node.x, radius, k_d_query);

#else
			k_d_query = x_bounds;
#endif

			// Query k-d tree for nodes that could be reached by new node
			k_d_results.clear();
			k_d_tree.range_query(k_d_query, k_d_results);

			// Sort them heuristically
			Q = priority_queue<node_cost_pair_t, vector<node_cost_pair_t >, greater<node_cost_pair_t > >();
			for (node_ids_t::iterator n = k_d_results.begin(); n != k_d_results.end(); n++) {
				Q.push(make_pair(applyHeuristics(x_rand, tree[*n].x), *n));
			}

			// For each potentially reachable node attempt to connect.
			// Upon connecting update it's cost and the cost to each of
			// it's children.
			stack<pair<node_id_t, double> > s;
			while(!Q.empty() && Q.top().first < min_dist) {
				// Start the stack with the first thing on the queue
				s = stack<pair<node_id_t, double> >();
				s.push(make_pair(Q.top().second, 0));
				Q.pop();

#else
			// Search for forward shortcuts
			stack<pair<node_id_t, double> > s;
			s.push(make_pair(1, 0)); // push start on stack
#endif
			while (!s.empty()) {
				node_id_t j = s.top().first;
				double decrease_cost = s.top().second;
				s.pop();
				for (node_list_t::iterator p = tree[j].children.begin(); p != tree[j].children.end(); ) {
					// If we can get to a node via the new node faster than via it's existing parent then change the parent
					double junk;
					if (connect(x_rand, tree[*p].x, min(radius, tree[*p].cost_from_start - decrease_cost - x_rand_node.cost_from_start), cost, junk, NULL)) {
						tree[*p].parent = x_rand_node_id;
						tree[x_rand_node_id].children.push_back(*p);
						s.push(make_pair(*p, tree[*p].cost_from_start - (x_rand_node.cost_from_start + cost)));
						tree[*p].cost_from_start = x_rand_node.cost_from_start + cost;

						if (*p == 0) { // i.e. if we're updating the cost to the goal
							//std::cout << "\n\rConnect x_rand: " << x_rand << "\n\rto " << tree[*p].x << "\n\r";
							summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())

#ifndef EXPERIMENT
							draw_path = true;
#endif
						}

						p = tree[j].children.erase(p);
					} else {
						if (decrease_cost > 0) {
							tree[*p].cost_from_start -= decrease_cost;
							if (*p == 0) {
								//std::cout << "\n\rConnect child x_rand: " << x_rand << "\n\rto " << tree[*p].x << "\n\r";
								summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())

#ifndef EXPERIMENT
								draw_path = true;
#endif
							}
						}
						s.push(make_pair(*p, decrease_cost));
						++p;
					}
				}
			}

#ifdef K_D_TREE_FORWARD
		    }
#endif

			// check orphans
			for (size_t j = 0; j < orphans.size(); ) {
				double junk;
				if (connect(x_rand, tree[orphans[j]].x, radius, cost, junk, NULL)) {
					tree[orphans[j]].cost_from_start = x_rand_node.cost_from_start + cost;
					tree[orphans[j]].parent = x_rand_node_id;
					tree[x_rand_node_id].children.push_back(orphans[j]);

					if (orphans[j] == 0) {
#ifndef EXPERIMENT
						draw_path = true;
#endif

						//std::cout << "\n\rConnect orhpan x_rand: " << x_rand << "\n\rto " << tree[orphans[j]].x << "\n\r";
						summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())
					}

					orphans[j] = orphans.back(); // remove orphan
					orphans.pop_back();
				} else {
					++j;
				}
			}

			// update parent of new node
			tree[x_near_id].children.push_back(x_rand_node_id);
		}

#ifndef EXPERIMENT
		if (draw_path) {
			visualize(tree);
		}
#endif

		setRadius(tree.size(), radius);
	}
}

void init() {
	srand(time(NULL));

	TWO_TO_X_DIM = pow(2.0, X_DIM);
	statespace_volume = 1;
	for (BOUNDS::iterator p = x_bounds.begin(); p != x_bounds.end(); p++) {
		statespace_volume *= (p->second - p->first);
	}

	sphere_volume = volume();

#if defined(CLOSED_FORM) || defined(CLOSED_FORM_FORWARD)
	cout << "Using forward closed form." << endl;
	computeCost = &computeCostClosedForm;
#else
	cout << "Using forward RK4." << endl;
	computeCost = &computeCostRK4;
#endif

#if defined(CLOSED_FORM) || defined(CLOSED_FORM_BACKWARD)
	cout << "Using backward closed form." << endl;
	checkPath = &checkPathClosedForm;
#else
	cout << "Using backward RK4." << endl;
	checkPath = &checkPathRK4;
#endif

	BRiBt = B*(R%~B);
	BRiBt = 0.5*BRiBt + 0.5*~BRiBt;

#ifdef FRONT_LOAD_RK4
	diff.A = &A;
	diff.c = &c;

	lyap.A = &A;
	lyap.BRiBt = &BRiBt;

	back.A = &Alpha;
	back.c = &c0;
#endif
}

char _getchar() {
	char k;
	cin >> k;
	return k;
}

void graphPath() {
	double radius = START_RADIUS; //DBL_MAX;

#if (DYNAMICS == QUADROTOR)
	/*
	x0.reset();
	x0[3] = x0[5] = -2;

	x1.reset();
	x1[2] = 5;
	x1[4] = x1[5] = 2;
	*/
#elif (DYNAMICS == NONHOLONOMIC)
	x_bounds[0].first = -DBL_MAX;
	x_bounds[0].second = DBL_MAX;

	x_bounds[1].first = -DBL_MAX;
	x_bounds[1].second = DBL_MAX;

	x0.reset();
	x0[0] = x0[1] = 0;
	x0[2] = -1;
	x0[3] = 1;

	x1.reset();
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

			CAL_ClearGroupKeyStates(robot_model, true);
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

	CAL_CreateGroup(&stills_group, 0, false, "Stills Parent Group");

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
/*
				CAL_SetGroupColor(p->first, current_r, current_g, current_b);
				current_r += alpha_change;
				current_g -= alpha_change;
				current_b -= alpha_change;
				current_alpha += alpha_change;
#if DYNAMICS != QUADROTOR
				CAL_SetGroupPosition(p->first, (p->second)[0], current_z, (p->second)[2]);
#endif
				current_z += 0.05;
*/
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
/*
			CAL_EmptyGroup(solution_group);
			CAL_EmptyGroup(solution_marker_group);
			CAL_SetGroupColor(solution_group, 0, 0, 1);
			CAL_SetGroupColor(solution_marker_group, 0, 0, 1);

			CAL_ClearGroupKeyStates(robot_model, true);
*/
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

void convertTreeToPoints(const tree_t& tree, double *points) {
	int i = 0;
	for (tree_t::const_iterator p = tree.begin(); p != tree.end(); p++) {
		for (int j = 0; j < p->x.numRows(); j++, i++) {
			points[i] = p->x[j];
		}
	}
}

void testReduceRadius() {
	double radius;

	BOUNDS bounds;
	state x;

	x[0] = 0.12684194382917316;
	x[1] = -6.1335395985595316;
	setRadius(3, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 0.89226644569287672;
	x[1] = 0.63328319288164536;
	setRadius(4, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 60.176413469180844;
	x[1] = -6.6750103576807405;
	setRadius(5, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 45.078589250406800;
	x[1] = -8.8588820387226370;
	setRadius(6, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 78.331917131628757;
	x[1] = 0.39752316698201184;
	setRadius(7, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 87.596822145950810;
	x[1] = 9.1179966545831377;
	setRadius(8, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 53.934150239391386;
	x[1] = -0.75852335501324752;
	setRadius(9, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 86.221953189207198;
	x[1] = 5.5931662913348212;
	setRadius(10, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 99.679564498066497;
	x[1] = 2.2298537867421775;
	setRadius(11, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	x[0] = 26.621390624550536;
	x[1] = 6.8023826710902000;
	setRadius(12, radius);
	calc_forward_reachable_bounds(x, radius, bounds);

	_getchar();
	exit(0);
}

int _tmain(int argc, _TCHAR* argv[])
{
	setupParameters();
	init();

	double radius = START_RADIUS;
	double temp = DBL_MAX;
	setRadius(2, radius);

	setupVisualization(x0, x1);

	//testReduceRadius();

#ifdef PLOT_PATH
#if (DYNAMICS == QUADROTOR) // Quadrotor
x0[0] = 0;
x0[1] = 0;
x0[2] = 0;
x0[3] = 1;
x0[4] = 1;
x0[5] = 0;

x1[0] = 0;
x1[1] = 0;
x1[2] = 3;
x1[3] = 1;
x1[4] = -2;
x1[5] = 0;


#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == NONHOLONOMIC)
	x0[2] = 15;
	x0[3] = -10;
	x1[2] = 15;
	x1[3] = -10;
#endif

	cout << "x0: " << x0 << "\tx1: " << x1 << endl;
	plotPath(x0, x1, temp);
	_getchar();
#endif

#ifdef VISUALIZE_LOG
	visualizeLog();
#endif

#if MAKE_STILLS > -1
	makeStills(MAKE_STILLS);
#endif

	time_log = fopen("time_log.txt", "w");
	stats_log = fopen("stats_log.txt", "w");
	path_log = fopen("path_log.txt", "wb");

	ostringstream os;
	os << "time\tnumber_of_nodes\ttarget_number_of_nodes\tcost_from_start\tradius\tnumber_of_orphans" << endl;
	fputs(os.str().c_str(), stats_log);
	fflush(stats_log);

	os.str("");
	os << "number_of_nodes\ttime" << endl;
	fputs(os.str().c_str(), time_log);
	fflush(time_log);

#ifdef GRAPH_PATH
	graphPath();
#endif

	start_time = clock();
	tree_t tree;

	//KD_Tree::testKDTree(tree);
	
	rrtstar(x0, x1, TARGET_NODES, radius, tree);
	end_time = clock();

	cout << "Runtime: " << (double)(end_time - start_time)/(double)CLOCKS_PER_SEC << endl;

	if (tree[0].cost_from_start < DBL_MAX) {
		cout << setw(10) << 0 << " " << setw(11) << TARGET_NODES << "/" << TARGET_NODES << " cost: " << tree[0].cost_from_start << endl;
	} else {
		cout << endl << "No path found" << endl;
		drawTree(tree);
		_getchar();
		exit(0);
	}

	// Build a visualization of the final path
	visualize(tree);

	// Draw the tree
	drawTree(tree);

	CAL_ResumeVisualisation();

	_getchar();

	return 0;
}
