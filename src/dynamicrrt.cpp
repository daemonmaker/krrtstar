#include "utils.hpp"
#include "dynamicrrt.h"
#include <cmath>
#include <complex>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <cassert>
#include <string>
#include <algorithm>
#include <list>

using namespace std;

char user_response;

World * world = NULL;
StateSpace * state_space = NULL;
Robot * robot = NULL;

#define WRITE_HEADER(os, header, log_fh) \
	os.clear();	\
	os.str(""); \
	os << header << endl;	\
	fputs(os.str().c_str(), log_fh);	\
	fflush(stats_log);

#define WRITE_RECORD(os, label, value)	\
	os.clear();	\
	os.str(""); \
	os << label << value << std::endl;	\
	fputs(os.str().c_str(), experiment_log); \
	std::cout << label << value << std::endl;

#define WRITE_MATRIX(os, label, matrix)	\
	os.clear(); \
	os.str(""); \
	os << label << ": " << std::endl; \
	for (int idx = 0; idx < matrix.rows(); ++idx) { \
		for (int jdx = 0; jdx < matrix.cols(); ++jdx) { \
			os << matrix(idx, jdx) << ' '; \
		} \
		os << std::endl; \
	} \
	fputs(os.str().c_str(), experiment_log); \
	std::cout << label << ": " << std::endl << matrix << std::endl;

void dynamicsError() {
	cout << "Invalid dynamics \"" << dynamics_type_to_name(DYNAMICS) << "\" (" << DYNAMICS << ")" << endl;
	_getchar();
	exit(-1);
}

void open_logs(const string &experiment_name) {
	time_log = fopen(make_log_file_name(experiment_name, "time", "txt").c_str(), "w");
	stats_log = fopen(make_log_file_name(experiment_name, "stats", "txt").c_str(), "w");
	path_log = fopen(make_log_file_name(experiment_name, "path", "path").c_str(), "wb");
	experiment_log = fopen(make_log_file_name(experiment_name, "exp", "txt").c_str(), "w");

	ostringstream os;

	WRITE_HEADER(os, "time\tnumber_of_nodes\ttarget_number_of_nodes\tcost_from_start\tradius\tnumber_of_orphans", stats_log);
	WRITE_HEADER(os, "number_of_nodes\ttime", time_log);
}

void close_log(FILE * fh) {
	fflush(fh);
	fclose(fh);
}

void close_logs() {
	close_log(time_log);
	close_log(stats_log);
	close_log(path_log);
	close_log(experiment_log);
}

void write_state(const state& s, FILE * fh) {
	fwrite(s.data(), sizeof(double), (s.rows())*(s.cols()), fh);
}

void read_state(state * s, FILE * fh) {
	double t;
	for (size_t idx = 0; idx < X_DIM; ++idx) {
		fread((void *)&t, sizeof(double), 1, fh);
		(*s)[idx] = t;
	}
}

void write_node_list(const node_list_t& nodes, FILE* fh) {
	size_t num_nodes = nodes.size();
	fwrite((const void*)&num_nodes, sizeof(size_t), 1, fh);
	for (node_list_t::const_iterator nid = nodes.cbegin(); nid != nodes.cend(); ++nid) {
		fwrite((const void*)&(*nid), sizeof(node_id_t), 1, fh);
	}
}

void read_node_list(node_list_t * nodes, FILE * fh) {
	size_t num_nodes = 0;
	node_id_t nid;
	fread((void*)&num_nodes, sizeof(size_t), 1, fh);
	for (size_t idx = 0; idx < num_nodes; ++idx) {
		fread((void*)&nid, sizeof(node_id_t), 1, fh);
		nodes->push_back(nid);
	}
}

void save_tree(const string log_file, const tree_t& tree) {
	FILE* tree_log_fh = fopen(log_file.c_str(), "wb");

	size_t num_nodes = tree.size();
	fwrite((const void*)&num_nodes, sizeof(size_t), 1, tree_log_fh);
	for (tree_t::const_iterator node = tree.cbegin(); node != tree.cend(); ++node) {
		fwrite((const void *)&(node->parent), sizeof(node_id_t), 1, tree_log_fh);
		write_state(node->x, tree_log_fh);
		fwrite((const void *)&(node->cost_from_start), sizeof(cost_t), 1, tree_log_fh);
		write_node_list(node->children, tree_log_fh);
	}

	fflush(tree_log_fh);
	fclose(tree_log_fh);
}

void read_tree(const string log_file, tree_t * tree) {
	FILE* tree_log_fh = fopen(log_file.c_str(), "rb");

	size_t num_nodes = 0;
	fread((void *)&num_nodes, sizeof(size_t), 1, tree_log_fh);
	for (size_t idx = 0; idx < num_nodes; ++idx) {
		Node node;
		fread((void *)&(node.parent), sizeof(node_id_t), 1, tree_log_fh);
		read_state(&(node.x), tree_log_fh);
		fread((void *)&(node.cost_from_start), sizeof(cost_t), 1, tree_log_fh);
		read_node_list(&(node.children), tree_log_fh);
		tree->push_back(node);
	}

	fclose(tree_log_fh);
}

void save_experiment(FILE * experiment_log) {
	ostringstream record;

	WRITE_RECORD(record, "DYNAMICS: ", dynamics_type_to_name(DYNAMICS));
	WRITE_RECORD(record, "ROBOT: ", typeid(ROBOT).name());
	WRITE_RECORD(record, "STATE_SPACE: ", typeid(STATE_SPACE).name());
	WRITE_RECORD(record, "WORLD: ", typeid(WORLD).name());
	WRITE_RECORD(record, "USE_SET_CLEARANCE: ", USE_SET_CLEARANCE);
	WRITE_RECORD(record, "USE_THRESHOLDS: ", USE_THRESHOLDS);
	WRITE_RECORD(record, "distance_threshold: ", world->getDistanceThreshold());
	WRITE_RECORD(record, "EPSILON: ", EPSILON);
	WRITE_RECORD(record, "USE_OBSTACLES: ", USE_OBSTACLES);
	bool reduce_radius = false;
#if defined(REDUCE_RADIUS)
	reduce_radius = true;
#endif
	WRITE_RECORD(record, "REDUCE_RADIUS: ", reduce_radius);
	WRITE_RECORD(record, "TARGET_NODES: ", TARGET_NODES);
	WRITE_RECORD(record, "START_RADIUS: ", START_RADIUS);
	WRITE_RECORD(record, "RADIUS_MULTIPLIER: ", RADIUS_MULTIPLIER);
	WRITE_RECORD(record, "deltaT: ", deltaT);
	WRITE_RECORD(record, "control_penalty: ", control_penalty);
	WRITE_RECORD(record, "control_penalty1: ", control_penalty1);
	WRITE_RECORD(record, "REACHABILITY_CONSTANT: ", REACHABILITY_CONSTANT);
	WRITE_RECORD(record, "sphere_volume: ", sphere_volume);
	WRITE_RECORD(record, "statespace_volume: ", statespace_volume);
	WRITE_RECORD(record, "NOISE_FREE: ", NOISE_FREE);

	WRITE_MATRIX(record, "A", A);
	WRITE_MATRIX(record, "B", B);
	WRITE_MATRIX(record, "C", C);
	WRITE_MATRIX(record, "MotionNoiseCovariance", MotionNoiseCovariance);
	WRITE_MATRIX(record, "ObservationNoiseCovariance", ObservationNoiseCovariance);
	WRITE_MATRIX(record, "K", K);
	WRITE_MATRIX(record, "L", L);
	WRITE_MATRIX(record, "R", R);
	WRITE_MATRIX(record, "Rotation", Rotation);
	WRITE_MATRIX(record, "Scale", Scale);

	generic_matrix_t matrix = -B*L;
	WRITE_MATRIX(record, "-BL", matrix);

	matrix = K*C;
	WRITE_MATRIX(record, "KC", matrix);

	matrix = A - B*L - K*C;
	WRITE_MATRIX(record, "A - BL - KC", matrix);

	WRITE_MATRIX(record, "F", F);
	WRITE_MATRIX(record, "G", G);
	WRITE_MATRIX(record, "X", X);
	WRITE_MATRIX(record, "Xhat", Xhat);
	WRITE_MATRIX(record, "U", U);

	fflush(experiment_log);
}

#include "visualization.hpp"
#include "simulation.hpp"

template<class T>
void readMatlabArray(MATFile *pmat, const string &element, T &matrix) {
	mxArray *aPtr = matGetVariable(pmat, element.c_str());
    if (aPtr == NULL) {
		std::cout << "mxArray not found: " << element.c_str() << std::endl;
		_getchar();
        exit(-1);
    }

	const mwSize * dims = mxGetDimensions(aPtr);
	size_t num_rows = mxGetM(aPtr);
	size_t num_cols = mxGetN(aPtr);
	double *data = mxGetPr(aPtr);
	for (size_t idx = 0; idx < num_rows; ++idx) {
		for (size_t jdx = 0; jdx < num_cols; ++jdx) {
			matrix(idx, jdx) = data[jdx*num_rows + idx];
		}
	}
}

void setupParameters(string parameters_file) {
	vis::initVisulization();

	//x_bounds.resize(X_DIM);
	u_bounds.resize(U_DIM);

#if (DYNAMICS == QUADROTOR)
	robot = new ROBOT(vis::cal_rotate);

	control_penalty = 0.1;

	u_bounds[0] = std::make_pair(-gravity*mass, 2*gravity*mass);
	u_bounds[1] = std::make_pair(-1.5*gravity*mass, 1.5*gravity*mass);
	u_bounds[2] = std::make_pair(-1.5*gravity*mass, 1.5*gravity*mass);

	u_bounds[0] = std::make_pair(-4.545, 9.935);
	u_bounds[1] = std::make_pair(-3.62, 3.62);
	u_bounds[2] = std::make_pair(-3.62, 3.62);

	A.block<3,3>(0,3) = Eigen::Matrix<double,3,3>::Identity();
	A.block<2,2>(6,8) = Eigen::Matrix<double,2,2>::Identity();
	A(3,7) = gravity;
	A(4,6) = -gravity;

	B(5,0) = 1/mass;
	B(8,1) = length/inertia;
	B(9,2) = length/inertia;

	R(0,0) = 0.25*control_penalty;
	R(1,1) = R(2,2) = 0.5*control_penalty;

	//x0[0] = 1;
	//x0[2] = 4;

	//x1[0] = 4;
	//x1[2] = 1;
#elif (DYNAMICS == NONHOLONOMIC)
	robot = new ROBOT(vis::cal_rotate);

	control_penalty = 1;
	control_penalty1 = 50;

	u_bounds[0] = std::make_pair(-DBL_MAX,DBL_MAX);
	u_bounds[1] = std::make_pair(-DBL_MAX,DBL_MAX);

	B(3,0) = 1;
	B(4,1) = 1;

	R(0,0) = control_penalty;
	R(1,1) = control_penalty1;

	x0[2] = -M_PI/2;		
	x0[3] = 1;
	
	x1[2] = -M_PI/2;
	x1[3] = 1;


#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	/*
	K(0,0) = 0.9102;
	K(0,2) = 0.4142;
	K(1,1) = 0.9102;
	K(1,3) = 0.4142;
	K(2,0) = 0.4142;
	K(2,2) = 1.2872;
	K(3,1) = 0.4142;
	K(3,3) = 1.2872;

	L(0,0) = 1;
	L(1,1) = 1;
	L(0,2) = 1.7321;
	L(1,3) = 1.7321;
	
	MotionNoiseCovariance = motion_noise_covariance_t::Identity()*0.0000001;
	ObservationNoiseCovariance = observation_noise_covariance_t::Identity()*0.0000001;
	*/
	/*
	float rotation_angle = M_PI/8.0;
	Rotation(0, 0) = cos(rotation_angle);
	Rotation(0, 1) = -sin(rotation_angle);
	Rotation(1, 0) = sin(rotation_angle);
	Rotation(1, 1) = cos(rotation_angle);
	*/

	Scale(0, 0) = 0.1;
	Scale(1, 1) = 1;
	Scale(2, 2) = 1;

	/*
	Rotation(0, 0) = -0.1701;
	Rotation(0, 2) = 0.9854;
	Rotation(1, 1) = 0.1701;
	
	//Rotation(1, 3) = -0.9854;
	//Rotation(2, 0) = -0.9854;
	//Rotation(2, 2) = -0.1701;
	//Rotation(3, 1) = 0.9854;
	//Rotation(3, 3) = 0.1701;
	

	Scale(0, 0) = 7863600;
	Scale(1, 1) = 7863600;
	//Scale(2, 2) = 4662000;
	//Scale(2, 2) = 4662000;
	*/
	/*
	MotionNoiseCovariance(0,0) = 0.01;
	MotionNoiseCovariance(1,1) = 0.001;
	MotionNoiseCovariance(2,2) = 0.0001;
	MotionNoiseCovariance(3,3) = 0.0001;
	ObservationNoiseCovariance(0,0) = 0.0001;
	ObservationNoiseCovariance(1,1) = 0.0001;
	ObservationNoiseCovariance(2,2) = 0.0001;
	ObservationNoiseCovariance(3,3) = 0.0001;
	*/
	robot = new ROBOT(vis::cal_rotate);

	control_penalty = 1;

	u_bounds[0] = std::make_pair(-10, 10);
	u_bounds[1] = std::make_pair(-10, 10);

	A(0,2) = 1;
	A(1,3) = 1;

	B(2,0) = 1;
	B(3,1) = 1;

	//R(0,0) = control_penalty;
	//R(1,1) = control_penalty;

	c = state::Zero();

	C = observation_t::Identity();

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)

	/*
	float rotation_angle = M_PI/8.0;
	Rotation(0, 0) = cos(rotation_angle);
	Rotation(0, 1) = -sin(rotation_angle);
	Rotation(1, 0) = sin(rotation_angle);
	Rotation(1, 1) = cos(rotation_angle);

	Scale(0, 0) = 0.25;
	Scale(1, 1) = 1;
	Scale(2, 2) = 1;
	*/
	C(0,0) = 1;
	C(1,1) = 1;
	/*
	MotionNoiseCovariance(0,0) = 0.1;
	MotionNoiseCovariance(0,1) = 1.009;
	MotionNoiseCovariance(1,0) = 0;
	MotionNoiseCovariance(1,1) = 0.09;
	ObservationNoiseCovariance(0,0) = 0.1;
	ObservationNoiseCovariance(1,1) = 0.1;
	K(0,0) = 0.1851;
	K(0,1) = 0.9827;
	K(1,0) = 0.9827;
	K(1,1) = 10.0823;
	L(0,0) = 1;
	L(1,1) = 1;
	R(0,0) = 0.2213;
	//R(0,1) = 0.1667;
	//R(1,0) = 0.1667;
	R(1,1) = 0.2213;
	Scale(0, 0) = 1.2702;
	Scale(1, 1) = 32.9106;
	Rotation(0, 0) = 0.0979;
	Rotation(1, 0) = 0.9952;
	Rotation(0, 1) = -0.9952;
	Rotation(1, 1) = 0.0979;
	*/

	robot = new ROBOT(vis::cal_rotate);

	u_bounds[0] = std::make_pair(-10, 10);
	u_bounds[1] = std::make_pair(-10, 10);

	B(0,0) = 1;
	B(1,1) = 1;
	
	c = state::Zero();

#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	robot = new ROBOT(vis::cal_rotate);

	u_bounds[0] = std::make_pair(-10, 10);

	A(0,1) = 1;

	B(1,0) = 1;

	R = Eigen::Matrix<double,U_DIM,U_DIM>::Zero();

	x1[0] = 100;

#else
    dynamicsError();
#endif

	std::cout << "Loading parameters " << parameters_file << std::endl;
	MATFile *pmat = matOpen(parameters_file.c_str(), "r");
	if (pmat == NULL) {
		std::cout << "Could not load parameters" << std::endl;
		_getchar();
		exit(-1);
	}
	/*
	int ndir;
	const char **dir = (const char **)matGetDir(pmat, &ndir);
	if (dir == NULL) {
		std::cout << "Error reading directory of file " << parameters_file << std::endl;
		_getchar();
		exit(-1);
	} else {
		std::cout << "Directory of " << parameters_file << std::endl;
		for	(size_t i=0; i < ndir; i++) {
			std::cout << dir[i] << std::endl;
		}
		std::cout << std::endl;
	}
	*/
	readMatlabArray(pmat, "A", A);
	readMatlabArray(pmat, "B", B);
	readMatlabArray(pmat, "C", C);
	readMatlabArray(pmat, "M", MotionNoiseCovariance);
	readMatlabArray(pmat, "N", ObservationNoiseCovariance);
	readMatlabArray(pmat, "K", K);
	readMatlabArray(pmat, "L", L);
	readMatlabArray(pmat, "R_tilde", R);
	readMatlabArray(pmat, "S", Scale);
	readMatlabArray(pmat, "V", Rotation);

#if SHOW_ROBOT
	robot->show_model();
#else
	robot->hide_model();
#endif

#if SHOW_ROBOT_COLLISION_CHECKER
	robot->show_collision_checker();
#else
	robot->hide_collision_checker();
#endif
}

void buildEnvironment(int base_group) {
#if (DYNAMICS == QUADROTOR)
//#define STATE_SPACE QuadrotorStateSpace
//#define WORLD TwoWalls
	world = new WORLD(base_group);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x0[2] = world->getStartState()[2];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];
	x1[2] = world->getFinalState()[2];

	world->setBound(3, make_pair(-5, 5));
	world->setBound(4, make_pair(-5, 5));
	world->setBound(5, make_pair(-5, 5));
	world->setBound(6, make_pair(-1000, 1000));
	world->setBound(7, make_pair(-1000, 1000));
	world->setBound(8, make_pair(-5000, 5000));
	world->setBound(9, make_pair(-5000, 5000));

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == NONHOLONOMIC)
#if USE_OBSTACLES == 6
#if (DYNAMICS == NONHOLONOMIC)
//#define STATE_SPACE NonholonomicStateSpace
//#define WORLD SymmetricRaceTrackMaze
	world = new WORLD(base_group);
	world->setBound(2, make_pair(-M_PI,M_PI));
	world->setBound(3, make_pair(0.01,10));
	world->setBound(4, make_pair(-0.25,0.25));

	x0[1] = 44;
	x0[0] = 10;
	x1[1] = 60;
	x1[0] = 10;
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
//#define STATE_SPACE StateSpace
//#define WORLD TwoPathMaze
//#define WORLD worlds::LudersBoxes
//#define WORLD worlds::VanDenBergPassages
	world = new WORLD(base_group);
	world->setBound(2, make_pair(-10.0, 10.0));
	world->setBound(3, make_pair(-10.0, 10.0));

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];
#else
//#define STATE_SPACE StateSpace
//#define WORLD TwoPathMaze
//#define WORLD worlds::LudersBoxes
//#define WORLD worlds::VanDenBergPassages
	world = new WORLD(base_group);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];
#endif
#elif USE_OBSTACLES == 5
//#define WORLD SimpleRaceTrack
	world = new WORLD(base_group);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];

#if (DYNAMICS == NONHOLONOMIC)
	world->setBound(2, make_pair(-M_PI,M_PI));
	world->setBound(3, make_pair(0.01,10));
	world->setBound(4, make_pair(-0.25,0.25));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	world->setBound(2, make_pair(-10.0, 10.0));
	world->setBound(3, make_pair(-10.0, 10.0));
#endif
#elif USE_OBSTACLES == 4
#define WORLD HardSMaze
	world = new WORLD(base_group);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];

#if (DYNAMICS == NONHOLONOMIC)
	world->setBound(2, make_pair(-M_PI,M_PI));
	world->setBound(3, make_pair(0.01,10));
	world->setBound(4, make_pair(-0.25,0.25));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	world->setBound(2, make_pair(-10.0, 10.0));
	world->setBound(3, make_pair(-10.0, 10.0));
#endif
#elif USE_OBSTACLES == 3
	x1[0] = x1[1] = 100;

	world = new WORLD(vis::cal_rotate);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];

#if (DYNAMICS == NONHOLONOMIC)
	world->setBound(2, make_pair(-M_PI,M_PI));
	world->setBound(3, make_pair(0.01,10));
	world->setBound(4, make_pair(-0.25,0.25));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	world->setBound(2, make_pair(-10.0, 10.0));
	world->setBound(3, make_pair(-10.0, 10.0));
#endif

#elif USE_OBSTACLES == 2
//#define WORLD FourRooms
	world = new WORLD(base_group);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];

#if (DYNAMICS == NONHOLONOMIC)
	world->setBound(2, make_pair(-M_PI,M_PI));
	world->setBound(3, make_pair(0.01,10));
	world->setBound(4, make_pair(-0.25,0.25));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	world->setBound(2, make_pair(-10.0, 10.0));
	world->setBound(3, make_pair(-10.0, 10.0));
#endif
#elif USE_OBSTACLES == 1
//#define WORLD Cylinders
	world = new WORLD(base_group);

	x0[0] = world->getStartState()[0];
	x0[1] = world->getStartState()[1];
	x1[0] = world->getFinalState()[0];
	x1[1] = world->getFinalState()[1];

#if (DYNAMICS == NONHOLONOMIC)
	world->setBound(2, make_pair(-M_PI,M_PI));
	world->setBound(3, make_pair(0.01,10));
	world->setBound(4, make_pair(-0.25,0.25));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	world->setBound(2, make_pair(-10.0, 10.0));
	world->setBound(3, make_pair(-10.0, 10.0));
#endif
#endif
#else
#define STATE_SPACE StateSpace
#define WORLD EmptyWorld
	world = new WORLD(base_group);
	world->setBound(1, make_pair(-10.0, 10.0));
#endif

	boost::function<void (state&)> position_generator(boost::bind(&WORLD::randPosition, (WORLD*)world, _1));
	state_space = new STATE_SPACE(position_generator, world->getBounds());

	world->buildEnvironment();
}

template <size_t _numRows>
struct lyapunov {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Eigen::Matrix<double,_numRows,_numRows> *A;
	Eigen::Matrix<double,_numRows,_numRows> *BRiBt;
	inline Eigen::Matrix<double,_numRows,_numRows> operator()(double t, const Eigen::Matrix<double,_numRows,_numRows>& G) const {
		Eigen::Matrix<double,_numRows,_numRows> AG = (*A)*G;
		return AG + AG.transpose() +(*BRiBt);
	}
};
#ifdef FRONT_LOAD_RK4
lyapunov<X_DIM> lyap;
#endif

template <size_t _numRows>
struct linear {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Eigen::Matrix<double,_numRows,_numRows> *A;
	Eigen::Matrix<double,_numRows,1> *c;
	inline Eigen::Matrix<double,_numRows,1> operator()(double t, const Eigen::Matrix<double,_numRows,1>& xbar) const {
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
	inline void rk4(Eigen::Matrix<double,_numRows,_numColumns>& x_0, double t_0, double dt, const Function& f) {
		double t_1 = t_0 + dt;

		k_1 = dt*f(t_0, x_0);
		k_2 = dt*f(t_0 + 0.5*dt, x_0 + 0.5*k_1);
		k_3 = dt*f(t_0 + 0.5*dt, x_0 + 0.5*k_2);
		k_4 = dt*f(t_0 + dt, x_0 + k_3);

		x_0 += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
	}

private:
	Eigen::Matrix<double,_numRows,_numColumns> k_1,k_2,k_3,k_4,k_5,k_6;
};
#ifdef FRONT_LOAD_RK4
rk<X_DIM, 1, linear<X_DIM, X_DIM> > linearrk4;
rk<X_DIM, X_DIM, lyapunov<X_DIM> > lyaprk4;
rk<2*X_DIM, 1, linear<2*X_DIM,1> > linearrk;
#endif
///*w
//template <size_t _numRows, size_t _numColumns>
//class lyaprk {
//public:
//inline void rk4(Matrix<_numRows, _numColumns>& G, double dt) {
//AG = A*G;
//k_1 = dt*(AG + ~AG + BRiBt);
//AG = A*(G + 0.5*k_1);
//k_2 = dt*(AG + ~AG + BRiBt);
//AG = A*(G + 0.5*k_2);
//k_3 = dt*(AG + ~AG + BRiBt);
//AG = A*(G + k_3);
//k_4 = dt*(AG + ~AG + BRiBt);
//
//G += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
//}
//
//private:
//Matrix<_numRows, _numColumns> AG;
//Matrix<_numRows, _numColumns> k_1;
//Matrix<_numRows, _numColumns> k_2;
//Matrix<_numRows, _numColumns> k_3;
//Matrix<_numRows, _numColumns> k_4;
//};
//
template <size_t _numRows>
inline double cost(double tau, const Eigen::Matrix<double,_numRows,_numRows>& G, const Eigen::Matrix<double,_numRows,1>& xbar) {
	return tau + ((x1-xbar).transpose()*G.ldlt().solve(x1-xbar)).trace();
}

template <size_t _numRows>
inline double dcost(double tau, const Eigen::Matrix<double,_numRows,_numRows>& G, const Eigen::Matrix<double,_numRows,1>& xbar) {
	Eigen::Matrix<double,_numRows,1> d = -G.ldlt().solve(x1-xbar);
	return 1 + 2*((A*x1).transpose()*d).trace() - (d.transpose()*BRiBt*d).trace();
}


inline bool collision_free(const state& s, bool distance_check = USE_THRESHOLDS, bool model = false) {
#if (USE_OBSTACLES > 0)
	int result = 0;
	int collisions = 0;

	robot->rotate(s, model);
	robot->position(s, model);

	// Sloppy! Refactor this to properly handle the two different types of possible collision checks
	if (distance_check) {
		bool below_threshold = world->checkDistance(robot);
#if defined(SHOW_THRESHOLD_CHECKS)
		world->showCollisionCheck(s, below_threshold, vis::threshold_hit_group, vis::threshold_free_group);
#endif
		return !below_threshold;
	}

	world->checkCollisions(robot, &collisions);

#if defined(SHOW_COLLISION_CHECKS) || defined(SHOW_COLLISIONS)
	world->showCollisionCheck(s, collisions > 0, vis::collision_hit_group, vis::collision_free_group);
#endif

	return (collisions == 0 ? true : false);

#else
	return true;
#endif
}

#ifdef INLINE_LYAP
Eigen::Matrix<double,X_DIM,X_DIM> k_1,k_2,k_3,k_4,AG;
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
  const BOUNDS& temp_x_bounds = world->getBounds();

#if (DYNAMICS == QUADROTOR)
	double cost1 = max(computeHeuristic(x0[0], x1[0], temp_x_bounds[3]), computeHeuristic(x0[1], x1[1], temp_x_bounds[4]));
	double cost2 = max(computeHeuristic(x0[2], x1[2], temp_x_bounds[5]), computeHeuristic(x0[6], x1[6], temp_x_bounds[8]));

	cost = max(cost1, cost2);
	cost = max(cost, computeHeuristic(x0[7], x1[7], temp_x_bounds[9]));
#elif (DYNAMICS == NONHOLONOMIC)
	cost = sqrt(pow(x1[0] - x0[0],2) + pow(x1[1] - x0[1],2))/temp_x_bounds[3].second;
#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
	double cost1 = max(computeHeuristic(x0[0], x1[0], temp_x_bounds[2]), computeHeuristic(x0[1], x1[1], temp_x_bounds[3]));
	double cost2 = max(computeHeuristic(x0[2], x1[2], u_bounds[0]), computeHeuristic(x0[3], x1[3], u_bounds[1]));

	cost = max(cost1, cost2);
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D)
	cost = max(computeHeuristic(x0[0], x1[0], u_bounds[0]), computeHeuristic(x0[1], x1[1], u_bounds[1]));
#elif (DYNAMICS == DOUBLE_INTEGRATOR_1D)
	cost = max(computeHeuristic(x0[0], x1[0], temp_x_bounds[1]), computeHeuristic(x0[1], x1[1], u_bounds[0]));
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

	Eigen::Matrix<double,X_DIM,1> x1diffbarx0,d;

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
	///*
      //MapleGenVar3 = g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*_Z*_Z+(-8.0*j*j*r*g*g*x0_8*x0_8-m~*m*r*g*g*l*l*x0_5*x0_5-4.0*j*j*r*x1_8*g*g*x0_8-m*m*r*x1_5*g*g*l*l*x0_5-8.0*j*j~*r*x1_8*x1_8*g*g-4.0*j*j*r*x1_9*g*g*x0_9-8.0*j*j*r*x1_9*x1_9*g*g-m*m*r*x1_5*x1_5~*g*g*l*l-8.0*j*j*r*g*g*x0_9*x0_9)*_Z*_Z*_Z*_Z*_Z*_Z+(-120.0*j*j*r*x1_8*g*g*x0_6+120.0*j*j*r*g*g*x1_7*x0_9+240.0*j*j*r*g*g*x1_7*x1_9-6.0*m*m*r*x1_5*g*g*l*l~*x0_2-6.0*m*m*r*g*g*l*l*x0_2*x0_5+240.0*j*j*r*g*g*x1_6*x1_8-120.0*j*j*r*x1_9*g~*g*x0_7+6.0*m*m*r*g*g*l*l*x1_2*x0_5+6.0*m*m*r*x1_5*g*g*l*l*x1_2-240.0*j*j*r*g*g*x0_6*x0_8+120.0*j*j*r*g*g*x1_6*x0_8-240.0*j*j*r*g*g*x0_7*x0_9)*_Z*_Z*_Z*_Z*_Z;
      //MapleGenVar4 = MapleGenVar3+(2520.0*j*j*r*g*g*x1_7*x0_7+18.0*m*m*r*g*g*l*l~*x1_2*x0_2-1800.0*j*j*r*g*g*x1_7*x1_7-1440.0*j*j*r*x0_3*g*x0_9-1080.0*j*j*r*x1_3~*g*x0_9+1080.0*j*j*r*x1_8*g*x0_4+1440.0*j*j*r*x0_4*g*x0_8-9.0*m*m*r*g*g*l*l~*x0_2*x0_2-9.0*m*m*r*g*g*l*l*x1_2*x1_2-1440.0*j*j*r*x1_3*g*x1_9-1800.0*j*j*r*g~*g*x0_7*x0_7+1440.0*j*j*r*x1_4*g*x1_8+1080.0*j*j*r*x1_4*g*x0_8-1800.0*j*j*r*g*g*x0_6*x0_6-1080.0*j*j*r*x1_9*g*x0_3+2520.0*j*j*r*g*g*x1_6*x0_6-1800.0*j*j*r*g*g*x1_6*x1_6)*_Z*_Z*_Z*_Z;
      //MapleGenVar5 = MapleGenVar4;
      //MapleGenVar7 = (-18720.0*j*j*r*x1_3*g*x0_7-3360.0*j*j*r*x0_0*g*x0_9+18720.0*j*j*r*g*x1_7*x0_3+18720.0*j*j*r*x1_4*g*x0_6-3360.0*j*j*r*x1_8*g*x1_1+3360.0*j*j*r*x0_1*g*x0_8+3360.0*j*j*r*x1_8*g*x0_1+21600.0*j*j*r*x1_3*g*x1_7-21600.0*j*j*r*x1_4*g*x1_6+3360.0*j*j*r*x1_9*g*x1_0+21600.0*j*j*r*x0_4*g*x0_6-18720.0*j*j*r*g*x1_6*x0_4-3360.0*j*j*r*x1_1*g*x0_8+3360.0*j*j*r*x1_0*g*x0_9-3360.0*j*j*r*x1_9*g*x0_0-21600.0*j*j*r*x0_3*g*x0_7)*_Z*_Z*_Z;
      //MapleGenVar8 = (-50400.0*j*j*r*g*x1_6*x0_1-122400.0*j*j*r*x1_3*x0_3-50400.0*j*j*r*x1_1*g*x0_6-64800.0*j*j*r*x1_3*x1_3-64800.0*j*j*r*x0_4*x0_4-64800.0*j*j*r*x0_3*x0_3-50400.0*j*j*r*x0_0*g*x0_7+50400.0*j*j*r*g*x1_6*x1_1-122400.0*j*j*r*x1_4*x0_4+50400.0*j*j*r*x0_1*g*x0_6+50400.0*j*j*r*g*x1_7*x0_0-64800.0*j*j*r*x1_4*x1_4-50400.0*j*j*r*g*x1_7*x1_0+50400.0*j*j*r*x1_0*g*x0_7)*_Z*_Z;
      //MapleGenVar6 = MapleGenVar7+MapleGenVar8;
      //MapleGenVar2 = MapleGenVar5+MapleGenVar6;
      //MapleGenVar1 = MapleGenVar2+(-302400.0*j*j*r*x1_3*x0_0+302400.0*j*j*r*x1_4~*x1_1+302400.0*j*j*r*x1_0*x0_3-302400.0*j*j*r*x0_0*x0_3+302400.0*j*j*r*x1_1~*x0_4+302400.0*j*j*r*x1_3*x1_0-302400.0*j*j*r*x1_4*x0_1-302400.0*j*j*r*x0_1~*x0_4)*_Z-352800.0*j*j*r*x1_1*x1_1-352800.0*j*j*r*x0_1*x0_1+705600.0*j*j*r*x1_1*x0_1-352800.0*j*j*r*x1_0*x1_0-352800.0*j*j*r*x0_0*x0_0+705600.0*j*j*r*x1_0~*x0_0;
      //t1 = 0.0;
	  //

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

		//double current = realRoots[i] + tr(~x1diffbarx0*d);
		double current = realRoots[i] + (x1diffbarx0.transpose()*d).trace();
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

	Eigen::Matrix<double,X_DIM,X_DIM> G = Eigen::Matrix<double,X_DIM,X_DIM>::Zero();
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
		k1 = deltaT*(AG + AG.transpose() + BRiBt);
		AG = A*(G + 0.5*k_1);
		k_2 = deltaT*(AG + AG.transpose() + BRiBt);
		AG = A*(G + 0.5*k_2);
		k_3 = deltaT*(AG + AG.transpose() + BRiBt);
		AG = A*(G + k_3);
		k_4 = deltaT*(AG + AG.transpose() + BRiBt);

		G += ((k_1 + k_4) + 2.0*(k_2 + k_3))/6.0;
#else
		lyaprk4.rk4(G, 0, deltaT, lyap);
		//lyaprk4.rk4(G, deltaT);
#endif
		linearrk4.rk4(xbar, 0, deltaT, diff);

		t += deltaT;

		state d = G.ldlt().solve(x1-xbar);
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

bool validateStateAndControl(const state& x, const control& u) {
	BOUNDS x_bounds_real = world->getBounds();

#if (DYNAMICS == NONHOLONOMIC)
	x_bounds_real[2].first = -DBL_MAX;
	x_bounds_real[2].second = DBL_MAX;
#endif
	bool goodState = world->validateState(x);
	bool goodControl = checkBounds(u, u_bounds);
	return goodState && goodControl && collision_free(x);
}

void calculateStateAndControl(const state& x0, const state& x1, const double tau, const state& d_tau, double t, state * x, control * u) {
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
			(*x) = chi.block<X_DIM,1>(0,0);
			(*u) = R.ldlt().solve(B.transpose()*chi.block<X_DIM,1>(X_DIM,0));
}

void recordTrajectory(double t, const state &x, const control &u, state_time_list_t* vis, control_time_list_t* con) {
	if (vis) {
		vis->push_back(make_pair(t, x));
	}

	if (con) {
		con->push_back(make_pair(t, u));
	}
}

bool checkPathClosedForm(const state& x0, const state& x1, const double tau, const state& d_tau, double * actual_deltaT, state_time_list_t* vis, control_time_list_t* con) {
	double t;
	double bound = deltaT;
	size_t numPoints = ceil(tau / bound);
	size_t step = 1;
	while (step < numPoints) step *= 2;
	(*actual_deltaT) = tau/numPoints;

	for ( ; step > 1; step /= 2) {
		for (int i = step / 2; i < numPoints; i += step) {
			t = (*actual_deltaT)*i;

			state x;
			control u;
			calculateStateAndControl(x0, x1, tau, d_tau, t, &x, &u);

			if (!(vis || con) && !validateStateAndControl(x, u)) {
				return false;
			}

			recordTrajectory(t, x, u, vis, con);
		}
	}

	state x;
	control u;
	calculateStateAndControl(x0, x1, tau, d_tau, 0, &x, &u);

	if (!(vis || con) && !validateStateAndControl(x, u)) {
		return false;
	}

	recordTrajectory(0, x, u, vis, con);

	return true;
}

bool checkPathRK4(const state& x0, const state& x1, const double tau, const state& d_tau, double * actual_deltaT, state_time_list_t* vis, control_time_list_t* con) {
	(*actual_deltaT) = deltaT;

	Alpha = block<X_DIM,X_DIM,X_DIM,X_DIM>(A, -BRiBt, Eigen::Matrix<double,X_DIM,X_DIM>::Zero(), -A.transpose());

	chi.block<X_DIM,1>(0,0) = x1;
	chi.block<X_DIM,1>(X_DIM,0) = -d_tau;

	c0.block<X_DIM,1>(0,0) = c;
	c0.block<X_DIM,1>(X_DIM,0) = Eigen::Matrix<double,X_DIM,1>::Zero();

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
		///*
		//ostringstream os;
		//os << num_steps << " " << j << "     " << chi[0] << " " << chi[1] << " " << chi[2] << endl;
		//fputs(os.str().c_str(), path_log);
		//fflush(path_log);
		//
		//cout << num_steps << " " << j << "     " << chi[0] << " " << chi[1] << " " << chi[2] << endl;
#else
		//cout << chi[0] << " " << chi[1] << " " << chi[2] << " " << chi[6] << " " << chi[7] << endl;
#endif

		state x = chi.block<X_DIM,1>(0,0);
		control u = -R.ldlt().solve(B.transpose()*chi.block<X_DIM,1>(X_DIM,0));

		//cout << "checkBounds(x, x_bounds): " << checkBounds(x, x_bounds) << "\tcheckBounds(u, u_bounds): " << checkBounds(u, u_bounds) << endl;

		if (!(vis || con) && !validateStateAndControl(x, u)) {
			return false;
		}

		recordTrajectory(deltaT*j, x, u, vis, con);
	}

	return true;
}

inline bool connect(const state& x0, const state& x1, const double radius, double& cost, double& tau, double * actual_deltaT, state_time_list_t* vis, control_time_list_t * cons) {
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
		if (checkPath(x0, xend, tau, d_tau, actual_deltaT, vis, cons)) {
			return true;
		} else {
			//cout << "Failed checkPath" << endl;
		}
	} else {
		//cout << "Failed computeCost" << endl;
	}

	return false;
}

///**
 //* Calculates the volume of a unit sphere.
 //*
 //* Verified with table on http://en.wikipedia.org/wiki/Unit_sphere#General_area_and_volume_formulas
 //* Reproduced here
 //* n 	V_n (volume)
 //* 0 	(1/0!)\pi^0 	1.000
 //* 1 	(2^1/1!!)\pi^0 	2.000
 //* 2 	(1/1!)\pi^1 = \pi 	3.142
 //* 3 	(2^2/3!!)\pi^1 = (4/3)\pi 	4.189
 //* 4 	(1/2!)\pi^2 = (1/2)\pi^2 	4.935
 //* 5 	(2^3/5!!)\pi^2 = (8/15)\pi^2 	5.264
 //* 6 	(1/3!)\pi^3 = (1/6)\pi^3 	5.168
 //* 7 	(2^4/7!!) \pi^3 = (16/105)\pi^3 	4.725
 //* 8 	(1/4!)\pi^4 = (1/24)\pi^4 	4.059
 //* 9 	(2^5/9!!) \pi^4 = (32/945)\pi^4 	3.299
 //* 10 	(1/5!)\pi^5 = (1/120)\pi^5 	2.550
 //
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
	//double c = TWO_TO_X_DIM * (1 + X_DIM_INVERSE) * FREE_SS_VOLUME * REACHABILITY_CONSTANT; // Of the order of (2^X_DIM) * (1 + 1/X_DIM) * (Volume of free state space) * (reachability constant (start out arbitrarily, we need to research it further)) 
	//double v = volume(); /* Volume of a ball of dimension X_DIM 
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

	///*
    //double t1 = sqrt(3.0);
    //double t5 = statespace_volume*statespace_volume;
    //double t6 = log(num_states);
    //double t7 = t6*t6;
    //double t9 = sphere_volume*sphere_volume;
    //double t10 = t9*t9;
    //double t11 = num_states*num_states;
    //double t12 = t11*t11;
    //double t15 = pow(t5*t7*t10*t12,0.1666666666666667);
    //double t18 = 3.0*t1/sphere_volume/num_states*t15;
	//radius = t18;
	//
#else
	assert("setRadius undefined");
	return;
#endif
#else
	return;
#endif
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

///*
//void connect_forward(tree_t& tree, k_d_tree_t& k_d_tree, const node_cost_pair_t& start, const state& x_rand, const node_id_t x_rand_node_id, const Node& x_rand_node, double radius, double time_diff) {
//	ostringstream os;
//	double cost;
//	stack<node_cost_pair_t> s;
//	s.push(start);
//			while (!s.empty()) {
//				node_id_t j = s.top().first;
//				double decrease_cost = s.top().second;
//				s.pop();
//				for (node_list_t::iterator p = tree[j].children.begin(); p != tree[j].children.end(); ) {
//					// If we can get to a node via the new node faster than via it's existing parent then change the parent
//					double junk, junk2;
//					if (connect(x_rand, tree[*p].x, min(radius, tree[*p].cost_from_start - decrease_cost - x_rand_node.cost_from_start), cost, junk, &junk2, NULL, NULL)) {
//						tree[*p].parent = x_rand_node_id;
//						tree[x_rand_node_id].children.push_back(*p);
//						s.push(make_pair(*p, tree[*p].cost_from_start - (x_rand_node.cost_from_start + cost)));
//						tree[*p].cost_from_start = x_rand_node.cost_from_start + cost;
//
//						if (*p == 0) { // i.e. if we're updating the cost to the goal							
//							// TODO DWEBB -- FIX THIS
//							//summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())
//
//#ifndef EXPERIMENT
//							draw_path = true;
//#endif
//						}
//
//						p = tree[j].children.erase(p);
//					} else {
//						if (decrease_cost > 0) {
//							tree[*p].cost_from_start -= decrease_cost;
//							if (*p == 0) {
//								summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())
//
//#ifndef EXPERIMENT
//								draw_path = true;
//#endif
//							}
//						}
//						s.push(make_pair(*p, decrease_cost));
//						++p;
//					}
//				}
//			}
//
//			// check orphans
//			for (size_t j = 0; j < orphans.size(); ) {
//				double junk, junk2;
//				if (connect(x_rand, tree[orphans[j]].x, radius, cost, junk, &junk2, NULL, NULL)) {
//					tree[orphans[j]].cost_from_start = x_rand_node.cost_from_start + cost;
//					tree[orphans[j]].parent = x_rand_node_id;
//					tree[x_rand_node_id].children.push_back(orphans[j]);
//
//					if (orphans[j] == 0) {
//#ifndef EXPERIMENT
//						draw_path = true;
//#endif
//
//						summarize_path(os, time_diff, i, n, tree[0].cost_from_start, radius, orphans.size())
//					}
//
//					orphans[j] = orphans.back(); // remove orphan
//					orphans.pop_back();
//				} else {
//					++j;
//				}
//			}
//
//			// update parent of new node
//			tree[x_near_id].children.push_back(x_rand_node_id);
//}
//

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
#else
	assert(1==0);
	exit(-1);
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
		///* Before polynomial solver
		//double t1 = RootOf(64.0*g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*_Z-112.0*g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*radius+(126.0*j*j*r*g*g*x0_9*x0_9+49.0*g*g*radius*radius*l*l)*_Z*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_7*x0_9-126.0*radius*j*j*r*g*g*x0_9*x0_9)*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_7*x0_7-504.0*radius*j*j*r*g*g*x0_7*x0_9+504.0*j*j*r*x0_3*g*x0_9~)*_Z*_Z*_Z+(-504.0*radius*j*j*r*g*g*x0_7*x0_7-504.0*radius*j*j*r*x0_3*g*x0_9~+1008.0*j*j*r*x0_3*g*x0_7)*_Z*_Z+(-1008.0*radius*j*j*r*x0_3*g*x0_7+504.0*j*j*r*x0_3*x0_3)*_Z-504.0*radius*j*j*r*x0_3*x0_3);
		//double t3 = t1*t1;
		//double t7 = t3*t1;
		//double t11 = sqrt(14.0);
		//double t12 = g*g;
		//double t13 = l*l;
		//double t15 = j*j;
		//double t19 = t3*t3;
		//double t25 = sqrt(t12*t13/t15/r*t19*t7*(radius-t1));
		//double t28_min = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0-t11*t25/42.0;
		//double t28_max = x0_0+t1*x0_3+g*t3*x0_7/2.0+g*t7*x0_9/6.0+t11*t25/42.0;

		//bounds[0].first = t28_min;
		//bounds[0].second = t28_max;
		//

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
		///* Before polynomial solver
		//double t1 = RootOf(64.0*g*g*l*l*_Z*_Z*_Z*_Z*_Z*_Z*_Z-112.0*g*g*l*l*_Z*_Z*_Z*_Z*Z*_Z*radius+(126.0*j*j*r*g*g*x0_8*x0_8+49.0*g*g*radius*radius*l*l)*_Z*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_6*x0_8-126.0*radius*j*j*r*g*g*x0_8*x0_8)*_Z*_Z*_Z*_Z+(504.0*j*j*r*g*g*x0_6*x0_6-504.0*radius*j*j*r*g*g*x0_6*x0_8-504.0*j*j*r*x0_4*g*x0_8~)*_Z*_Z*_Z+(-504.0*radius*j*j*r*g*g*x0_6*x0_6+504.0*radius*j*j*r*x0_4*g*x0_8~-1008.0*j*j*r*x0_4*g*x0_6)*_Z*_Z+(1008.0*radius*j*j*r*x0_4*g*x0_6+504.0*j*1*r*x0_4*x0_4)*_Z-504.0*radius*j*j*r*x0_4*x0_4);
		//double t3 = t1*t1;
		//double t7 = t3*t1;
		//double t11 = sqrt(14.0);
		//double t12 = g*g;
		//double t13 = l*l;
		//double t15 = j*j;
		//double t19 = t3*t3;
		//double t25 = sqrt(t12*t13/t15/r*t19*t7*(radius-t1));
		//double t28_min = x0_1+t1*x0_4-g*t3*x0_6/2.0-g*t7*x0_8/6.0-t11*t25/42.0;
		//double t28_max = x0_1+t1*x0_4-g*t3*x0_6/2.0+g*t7*x0_8/6.0-t11*t25/42.0;

		//bounds[1].first = t28_min;
		//bounds[1].second = t28_max;
		//

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

		///* Before polynomial solver
		//double t1 = RootOf(36.0*_Z*_Z*_Z*_Z*_Z*l*l-60.0*_Z*_Z*_Z*_Z*l*l*radius+(25.0*l*l*radius*radius+40.0*j*j*r*x0_9*x0_9)*_Z*_Z*_Z+(-40.0*radius*j*j*r*x0_9*x0_9+80.0*x0_9*x0_7*r*j*j)*_Z*_Z+(-80.0*radius*j*j*r*x0_7*x0_9+40.0*x0_7*x0_7*r*j*j)*_Z-40.0*radius*j*j*r*x0_7*x0_7);
		//double t4 = t1*t1;
		//double t8 = sqrt(10.0);
		//double t9 = g*g;
		//double t10 = t4*t4;
		//double t13 = l*l;
		//double t15 = j*j;
		//double t22 = sqrt(t9*t10*t1*t13/t15/r*(radius-t1));
		//double t25_min = x0_3+g*t1*x0_7+g*t4*x0_9/2.0-t8*t22/10.0;
		//double t25_max = x0_3+g*t1*x0_7+g*t4*x0_9/2.0+t8*t22/10.0;

		//bounds[3].first = t25_min;
		//bounds[3].second = t25_max;
		//
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

		///* Before polynomial solver
		//double t1 = RootOf(36.0*_Z*_Z*_Z*_Z*_Z*l*l-60.0*_Z*_Z*_Z*_Z*l*l*radius+(25.0*l*l*radius*radius+40.0*j*j*r*x0_8*x0_8)*_Z*_Z*_Z+(-40.0*radius*j*j*r*x0_8*x0_8+80.0*x0_8*x0_6*r*j*j)*_Z*_Z+(-80.0*radius*j*j*r*x0_6*x0_8+40.0*x0_6*x0_6*r*j*j)*_Z-40.0*radius*j*j*r*x0_6*x0_6);
		//double t4 = t1*t1;
		//double t8 = sqrt(10.0);
		//double t9 = g*g;
		//double t10 = t4*t4;
		//double t13 = l*l;
		//double t15 = j*j;
		//double t22 = sqrt(t9*t10*t1*t13/t15/r*(radius-t1));
		//double t25_min = x0_4-g*t1*x0_6-g*t4*x0_8/2.0-t8*t22/10.0;
		//double t25_max = x0_4-g*t1*x0_6-g*t4*x0_8/2.0+t8*t22/10.0;

		//bounds[4].first = t25_min;
		//bounds[4].second = t25_max;
		//
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

void rrtstar(const state& x_init, const state& x_final, int n, double radius, tree_t& tree, const bool rewire, const float epsilon, bool terminate_after_first = false) {
	// local variables
	ostringstream os;
	node_ids_t k_d_results;
	int xdims = X_DIM;
	int expected_nodes = 2*TARGET_NODES;
	KD_Tree k_d_tree(tree, xdims, world->getBounds(), expected_nodes);
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
	if (terminate_after_first) {
		n = 100000; // HAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACK!
	}
	for (int idx = 1; idx <= n;) {
#if defined(SHOW_COLLISION_CHECKS) || defined(SHOW_COLLISIONS) || defined(SHOW_THRESHOLD_CHECKS)
		Sleep(SHOW_COLLISION_SLEEP_TIME);
#endif
#ifndef EXPERIMENT
		draw_path = false;
#endif

		// Sample configuration
		state x_rand;
		bool check_goal = false;
		// Occasionally check the goal
		if ((float)rand()/(float)RAND_MAX < epsilon) {
			check_goal = true;
			x_rand = tree[0].x;
		}
		// Most the time generate a random configuration
		else {
			state_space->randState(x_rand);
		}

		// Only continue if the configuration is not in collision
		if (!collision_free(x_rand) || SKIP_CONNECTIONS) {
			continue;
		}
		
		// Re-linearize about the sample
#if (DYNAMICS == NONHOLONOMIC)
		double cth = cos(x_rand[4]);
		double sth = sin(x_rand[4]);

		A = Eigen::Matrix<double,X_DIM,X_DIM>::Zero();
		A(0,2) = -x_rand[3]*sth;
		A(0,3) = cth;
		A(1,2) = x_rand[3]*cth;
		A(1,3) = sth;
		A(2,3) = x_rand[4];
		A(2,4) = x_rand[3];

		Eigen::Matrix<double,X_DIM,1> f = Eigen::Matrix<double,X_DIM,1>::Zero();
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
		double actual_deltaT;

		priority_queue<node_cost_pair_t, vector<node_cost_pair_t >, greater<node_cost_pair_t > > Q;

#ifdef K_D_TREE_BACKWARD // Use kd-tree for fast search
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
			if (j == GOAL_NODE_ID) { // The goal node should not be a parent.
				continue;
			}
			if (connect(tree[j].x, x_rand, min(radius, min_dist - tree[j].cost_from_start), cost, tau, &actual_deltaT, NULL, NULL)) {
				min_dist = tree[j].cost_from_start + cost;
				x_near_id = j;
			}

		}
#else // Use linear search through all nodes
		Q.push(make_pair(applyHeuristics(tree[1].x, x_rand), 1)); // push start onto queue
		while (!Q.empty() && Q.top().first < min_dist) {
			node_id_t j = Q.top().second;
			Q.pop();

			if (connect(tree[j].x, x_rand, min(radius, min_dist - tree[j].cost_from_start), cost, tau, &actual_deltaT, NULL, NULL)) {
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

		// Move to the next sample if we could not find a nearby configuration
		if (x_near_id == NO_PARENT) {
#ifndef ALLOW_ORPHANS
			continue;
#else
			orphans.push_back(tree.size());
#endif
		}

		double time_diff = (double)(clock() - start_time)/(double)CLOCKS_PER_SEC;
		cout << "                                                                              \r";
		cout << setw(9) << time_diff << " " << setw(11) << idx << "/" << n << " tree: " << setw(9) << tree.size() << " radius: " << setw(9) << radius << " orphans: " << orphans.size() << "\r";

		// Update the tree
		node_id_t x_rand_node_id;
		Node x_rand_node;
		if (check_goal) {
			// Update goal if new path is shorter
			if ((tree[0].parent != x_near_id) && min_dist < tree[0].cost_from_start) {
				tree[0].parent = x_near_id;
				tree[0].cost_from_start = min_dist;

				x_rand_node_id = 0;
				x_rand_node = tree[0];

#ifndef EXPERIMENT
				draw_path = true;
#endif
			} else {
				continue;
			}
		}

		// Add a new node
		else {
			x_rand_node_id = tree.size();

			// Create a node for the new configuration
			x_rand_node.parent = x_near_id;
			x_rand_node.x = x_rand;
			x_rand_node.cost_from_start = min_dist;

			// Add it to the tree and k-d tree
			tree.push_back(x_rand_node);
			k_d_tree.add(x_rand_node_id);

			if ((idx % int(TIMING_FREQUENCY*TARGET_NODES)) == 0) {
				os.str("");
				os << idx << "\t" << time_diff << endl;
				fputs(os.str().c_str(), time_log);
				fflush(time_log);
			}

			++idx;
		}

		if (rewire && !check_goal && x_near_id != NO_PARENT) {

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
			//k_d_query = x_bounds;
			k_d_query = world->getBounds();
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
					double junk, junk2;
					if (connect(x_rand, tree[*p].x, min(radius, tree[*p].cost_from_start - decrease_cost - x_rand_node.cost_from_start), cost, junk, &junk2, NULL, NULL)) {
						tree[*p].parent = x_rand_node_id;
						tree[x_rand_node_id].children.push_back(*p);
						s.push(make_pair(*p, tree[*p].cost_from_start - (x_rand_node.cost_from_start + cost)));
						tree[*p].cost_from_start = x_rand_node.cost_from_start + cost;

						if (*p == 0) { // i.e. if we're updating the cost to the goal
							//std::cout << "\n\rConnect x_rand: " << x_rand << "\n\rto " << tree[*p].x << "\n\r";
							summarize_path(os, time_diff, idx, n, tree[0].cost_from_start, radius, orphans.size())
								
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
								summarize_path(os, time_diff, idx, n, tree[0].cost_from_start, radius, orphans.size())

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
		}

		// check orphans
		if (x_near_id != NO_PARENT) {
			for (size_t j = 0; j < orphans.size(); ) {
				double junk, junk2;
				if (connect(x_rand, tree[orphans[j]].x, radius, cost, junk, &junk2, NULL, NULL)) {
					tree[orphans[j]].cost_from_start = x_rand_node.cost_from_start + cost;
					tree[orphans[j]].parent = x_rand_node_id;
					tree[x_rand_node_id].children.push_back(orphans[j]);

					if (orphans[j] == 0) {
#ifndef EXPERIMENT
						draw_path = true;
#endif

						//std::cout << "\n\rConnect orhpan x_rand: " << x_rand << "\n\rto " << tree[orphans[j]].x << "\n\r";
						summarize_path(os, time_diff, idx, n, tree[0].cost_from_start, radius, orphans.size())
						if (terminate_after_first) {
							idx = n;
						}
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
			vis::visualizePath(tree, false, true, false);
		}
#endif

		setRadius(tree.size(), radius);
	}
}

void init() {
	srand(time(NULL));
	
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
	
	BRiBt = B*R.ldlt().solve(B.transpose());
	BRiBt = 0.5*BRiBt + 0.5*BRiBt.transpose().eval();
#ifdef FRONT_LOAD_RK4
	diff.A = &A;
	diff.c = &c;

	lyap.A = &A;
	lyap.BRiBt = &BRiBt;

	back.A = &Alpha;
	back.c = &c0;
#endif
}

void convertTreeToPoints(const tree_t& tree, double *points) {
	int i = 0;
	for (tree_t::const_iterator p = tree.begin(); p != tree.end(); p++) {
		for (int j = 0; j < p->x.rows(); j++, i++) {
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

void extractPath(const tree_t &tree, path_t * path) {
	// Walk the tree backwards from the goal and store indices of the nodes traversed.
	node_id_t current_idx = 0;
	while (current_idx != NO_PARENT) {
		path->push_back(current_idx);
		current_idx = tree[current_idx].parent;
	}

	// The above yields a path that is backwards so reverse it.
	reverse(path->begin(), path->end());
}

void createNominalTrajectory(const tree_t & tree, nominal_trajectory_t & traj) {
	createNominalTrajectory(tree, &(traj.path), &(traj.controls));
}

void createNominalTrajectory(const tree_t & tree, state_time_list_t * path, control_time_list_t * controls) {
	path_t key_points;
	extractPath(tree, &key_points);

	double cost, tau;
	double actual_deltaT = 0, max_tau = 0;
	size_t segment_length = 0;
	size_t numPoints = 0;
	state_time_list_t segment_states;
	control_time_list_t segment_controls;
	for (size_t idx = 0; idx < key_points.size() - 1; ++idx) {
		segment_states.clear();
		segment_controls.clear();
		connect(tree[key_points[idx]].x, tree[key_points[idx+1]].x, DBL_MAX, cost, tau, &actual_deltaT, &segment_states, &segment_controls);
		if (idx > 0) {
			max_tau += actual_deltaT;
		}
		sort(segment_states.begin(), segment_states.end(), temporal_order<state_time_t>);
		sort(segment_controls.begin(), segment_controls.end(), temporal_order<control_time_t>);
		segment_length = segment_states.size();
		for (size_t jdx = 0; jdx < segment_length; ++jdx) {
			segment_states[jdx].first += max_tau;
			segment_controls[jdx].first += max_tau;
			path->push_back(segment_states[jdx]);
			controls->push_back(segment_controls[jdx]);
		}
		max_tau = segment_states[segment_length-1].first;
	}

	/*
	for (int current_step = 0; current_step < path.size() - 1; ++current_step) {
		double x_pos = path[current_step].second[0];
		double y_pos = path[current_step].second[1];
#if POSITION_DIM == 3
		double z_pos = path[current_step].second[2];
#else
		double z_pos = 0;
#endif
		vis::markBelief(x_pos, y_pos, z_pos);
		std::cout << x_pos << ", " << y_pos << std::endl;
	}
	_getchar();
	*/
}

bool planTrajectory(const string &experiment_name, tree_t & tree, double radius, double clearance, bool terminate_after_first = false) {
	tree.clear();

	vis::WarpEnvironment<3>(Rotation, Scale);
	world->positionCamera(Rotation, Scale.inverse());

#if USE_THRESHOLDS
	world->setDistanceThreshold(clearance);
#elif USE_SET_CLEARANCE
	world->setClearance(clearance);
#endif

	open_logs(experiment_name);
	save_experiment(experiment_log);

	start_time = clock();

	//KD_Tree::testKDTree(tree);
	
	rrtstar(x0, x1, TARGET_NODES, radius, tree, true, EPSILON, terminate_after_first);
	end_time = clock();

	double runtime = (double)(end_time - start_time)/(double)CLOCKS_PER_SEC;
	cout << "Runtime: " << runtime << endl;

	ostringstream os;
	os.clear();
	os.str("");
	os << "Runtime: " << runtime << std::endl;
	fputs(os.str().c_str(), experiment_log);

	bool result = false;
	os.clear();
	os.str("");
	os << "Solution found: ";
	if (tree[0].cost_from_start < DBL_MAX) {
		result = true;
		os << "Yes" << std::endl << "Cost to goal: " << tree[0].cost_from_start;
	} else {
		result = false;
		os << "No";
	}
	std::cout << os.str() << std::endl;
	fputs(os.str().c_str(), experiment_log);

	std::cout << "Saving tree...";
	save_tree(make_log_file_name(experiment_name, "tree", "rrt"), tree);
	std::cout << " done." << std::endl;

#if USE_SET_CLEARANCE
	world->setClearance(0);
#endif

	vis::RestoreEnvironment<3>();
	world->positionCamera();

	close_logs();

	return result;
}

size_t testSolution(const dynamics_t &dynamics, tree_t &tree, int num_sims, float &average_probability_of_collision, bool visualize_simulation = VISUALIZE_SIMULATION) {
	// Attempt to follow the trajectory the specified number of times.
	size_t good_runs = 0;
	int collision_step = -1;
	std::map<int, int> collision_stats;
	nominal_trajectory_t nominal_trajectory;
	createNominalTrajectory(tree, nominal_trajectory);
	for (int count = 1; count <= num_sims; ++count) {
		collision_step = -1;
		if (simulate(dynamics, tree, nominal_trajectory, collision_step, visualize_simulation, robot)) {
			++good_runs;
		} else {
			++collision_stats[collision_step];
		}
		std::cout << count << '/' << num_sims << " with " << good_runs << " successfully completed." << '\r';
	}
	std::cout << std::endl;

	float total_collisions = 0;
	std::cout << "Collision summary:";
	for (std::map<int, int>::const_iterator iter = collision_stats.cbegin(); iter != collision_stats.cend(); ++iter) {
		std::cout << "\tStep: " << iter->first << "\tCollisions: " << iter->second << std::endl;
		total_collisions += iter->second;
	}
	total_collisions /= 100.0f;
	average_probability_of_collision = total_collisions / (100.0*(nominal_trajectory.path.size()));
	std::cout << "Average probability of collision per time step: " << average_probability_of_collision << std::endl;
	std::cout << "Average probability of success per time step: " << (1.0f - average_probability_of_collision) << std::endl;

	// Return the count of successful runs.
	return good_runs;
}

int countPaths(const string & path_log_file) {
	path_log = fopen(path_log_file.c_str(), "rb");

	double t = 0.0, t2;
	int path_count = 0;
	fread((void *)&t, sizeof(double), 1, path_log);
	while (true) {
		if (t == -1) {
			path_count++;

			fread((void *)&t, sizeof(double), 1, path_log);
		}
		for (int i = 0; i < X_DIM; i++) {
			fread((void *)&t2, sizeof(double), 1, path_log);
		}

		fread((void *)&t, sizeof(double), 1, path_log);
	}

	return path_count;
}

void convertPathToTree(state_list_t state_list, tree_t & tree) {
	Node goal_node;
	goal_node.x = state_list.back();
	goal_node.parent = state_list.size() - 1;
	tree.push_back(goal_node);

	Node start_node;
	start_node.parent = NO_PARENT;
	start_node.x = state_list.front();
	if (state_list.size() != 2) { // There exist intermediary nodes
		start_node.children.push_back(2);
	} else { // There only exists a start and goal node
		start_node.children.push_back(GOAL_NODE_ID);
	}
	tree.push_back(start_node);

	// Remove the start and goal nodes so they are not added to the tree again
	state_list.pop_front();
	state_list.pop_back();

	// Add the intermediary nodes
	int parent_idx = START_NODE_ID;
	int child_idx = parent_idx + 2; // Wrong in case of the node immediately before the goal but it will be rewired below
	for (state_list_t::const_iterator state = state_list.cbegin(); state != state_list.cend(); ++state) {
		Node new_node;
		new_node.x = *state;
		new_node.parent = parent_idx;
		new_node.children.push_back(parent_idx+2);
		tree.push_back(new_node);

		++parent_idx;
		if (parent_idx == 0) {
			++parent_idx;
		}
	}

	// Rewire the last node in the tree to the goal
	tree.back().children.pop_back();
	tree.back().children.push_back(GOAL_NODE_ID);
}

void loadPathsAsTrees(const string & path_log_file, state_lists_t & state_lists) {
	path_log = fopen(path_log_file.c_str(), "rb");
	state x;
	state_list_t state_list;
	double t = 0.0, t2;
	int path_count = 0;

	std::cout << "Paths read: " << 0 << '\r';

	int fread_result = fread((void *)&t, sizeof(double), 1, path_log);
	while (true) {
		if (t == -1) { // Reached end of a path
			state_lists.push_back(state_list);

			state_list.clear();

			fread_result = fread((void *)&t, sizeof(double), 1, path_log);

			std::cout << "Paths read: " << state_lists.size() << '\r';
			
			if (fread_result == 0) {
				break;
			}

			++path_count;
		}

		// Read next state
		for (int i = 0; i < X_DIM; i++) {
			fread((void *)&t2, sizeof(double), 1, path_log);
			x[i] = t2;
		}

		// Add state to list
		state_list.push_back(x);

		// Read time associated with next state -- used to identify the end of a path at the top of the loop
		fread((void *)&t, sizeof(double), 1, path_log);
	}

	std::cout << std::endl;
}

int _tmain(int argc, _TCHAR* argv[])
{
	string parameters_file = "C:\\Users\\Dustin\\Documents\\GitHub\\krrtstar\\krrtstar\\krrtstar\\parameters.txt";
	setupParameters(parameters_file);
	init();

	vis::setupVisualization(x0, x1, buildEnvironment);

	sphere_volume = volume();
	TWO_TO_X_DIM = pow(2.0, X_DIM);
	statespace_volume = 1;
	const BOUNDS x_bounds = world->getBounds();
	for (BOUNDS::const_iterator p = x_bounds.begin(); p != x_bounds.end(); p++) {
		statespace_volume *= (p->second - p->first);
	}

	double radius = START_RADIUS;
	double temp = DBL_MAX;
	setRadius(2, radius);

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
	vis::visualizeLog();
#endif

#if MAKE_STILLS > -1
	makeStills(MAKE_STILLS);
#endif

#ifdef GRAPH_PATH
	vis::graphPath();
#endif

#ifdef RUN_COLLISION_TESTER
	vis::testCollisions(world, robot);
	
	_getchar();

	return 0;
#endif

	F.block<X_DIM,X_DIM>(0, 0) = A;
	F.block<X_DIM,X_DIM>(0, X_DIM) = -(B*L);
	F.block<X_DIM,X_DIM>(X_DIM, 0) = K*C;
	F.block<X_DIM,X_DIM>(X_DIM, X_DIM) = A - B*L - K*C;

	G.block<X_DIM, X_DIM>(0, 0) = MotionNoiseCovariance;
	G.block<X_DIM, Z_DIM>(X_DIM, Z_DIM) = K*ObservationNoiseCovariance;

	X.block<X_DIM,X_DIM>(0, 0) = natural_dynamics_t::Identity();

	Xhat.block<X_DIM,X_DIM>(0, X_DIM) = natural_dynamics_t::Identity();

	U.block<U_DIM, X_DIM>(0, X_DIM) = -L;

	dynamics.A = A;
	dynamics.B = B;
	dynamics.c = c;
	dynamics.MotionNoiseCovariance = MotionNoiseCovariance;
	dynamics.C = C;
	dynamics.ObservationNoiseCovariance = ObservationNoiseCovariance;
	dynamics.d = d;
	dynamics.K = K;
	dynamics.L = L;
	dynamics.F = F;
	dynamics.G = G;
	dynamics.X = X;
	dynamics.Xhat = Xhat;
	dynamics.U = U;

	tree_t tree;

	char key;
	bool tree_loaded = false;
	ostringstream experiment_name_os;
	experiment_name_os << dynamics_type_to_name(DYNAMICS) << "_M_" << MotionNoiseCovariance(0,0) << '_' << MotionNoiseCovariance(0,1) << '_' << MotionNoiseCovariance(1,1) << "_N_" << ObservationNoiseCovariance(0,0) << '_' << ObservationNoiseCovariance(0,1) << '_' << ObservationNoiseCovariance(1,1);
	string experiment_name = experiment_name_os.str();
	std::cout << "Experiment name: " << experiment_name << std::endl;
	ostringstream experiment_base_name;

	//string experiment_name = "test";
	do {
		std::cout << ">> ";
		key = _getchar();

		if (key == 'h') {
			std::cout << "h - This help menu." << std::endl;
			std::cout << "l - Load tree from file." << std::endl;
			std::cout << "r - Run kRRT*." << std::endl;
			std::cout << "v - Visualize." << std::endl;
			std::cout << "c - Clear visualization." << std::endl;
			std::cout << "t - Test solution." << std::endl;
			std::cout << "e - Run multiple experiments. Requires EXPERIMENT to be defined at compile time." << std::endl;
			std::cout << "a - Analyze a set of experiments." << std::endl;
			std::cout << "p - Analyze all paths from the experiments." << std::endl;
		}

		if (key == 'r') {
			planTrajectory(experiment_name, tree, radius, distance_thresholds[0], FIND_FIRST_PATH_ONLY);
		}

		if (key == 'l') {
			int trajectory_id = -1;
			int threshold = -1;
			experiment_base_name.clear();
			experiment_base_name.str("");
			experiment_base_name << experiment_name;

			std::cout << "Trajectory id? ";
			std::cin >> trajectory_id;
			if (trajectory_id >= 0) {
				experiment_base_name << "_trajectory_" << trajectory_id;
				while ((threshold < 0) || (threshold > all_distance_threshold_count-1)) {
					std::cout << "Thresholds: " << std::endl;
					for (int threshold_idx = 0; threshold_idx < all_distance_threshold_count; ++threshold_idx) {
						std::cout << '\t' << threshold_idx << " - " << all_distance_thresholds[threshold_idx] << std::endl;
					}
					std::cout << "? ";
					std::cin >> threshold;
				}
				experiment_base_name << "_threshold_" << all_distance_thresholds[threshold];
			}
			read_tree(make_log_file_name(experiment_base_name.str(), "tree", "rrt"), &tree);
			std::cout << "Number of nodes: " << tree.size() << std::endl;
			std::cout << "Solution found? ";
			if (tree[0].cost_from_start < DBL_MAX) {
				std::cout << "Yes\tCost to goal: " << tree[0].cost_from_start;
			} else {
				std::cout << "No";
			}
			std::cout << std::endl;

			vis::RestoreEnvironment<3>();
			world->positionCamera();

			vis::visualizeFinalPath(tree, false, false, true);

			tree_loaded = true;
		}

		if (key == 'v') {
			std::cout << "Select one:" << std::endl;
			std::cout << "\tt - Visualize tree." << std::endl;
			std::cout << "\tp - Visualize paths." << std::endl;
			std::cout << "\ts - Visualize solution." << std::endl;
			std::cout << "\tw - Warp environment." << std::endl;
			std::cout << "\tw - Restore environment." << std::endl;
			std::cout << "\tq - Cancel." << std::endl;
			std::cout << ">> ";

			key = _getchar();

			if (key == 't') {
				vis::drawTree(tree);
			}

			if (key == 's') {
				vis::visualizeFinalPath(tree, true, false);
			}

			if (key == 'p') {
				vis::visualizeTree(tree);
			}

			if (key == 'w') {
				vis::WarpEnvironment<3>(Rotation, Scale);
			}

			if (key == 'r') {
				vis::RestoreEnvironment<3>();
			}

			key = NULL;
		}

		if (key == 'c') {
			vis::clearAll();
		}

		if (key == 's') {
			int collision_step = -1;
			robot->hide_model();
			robot->show_collision_checker();
			nominal_trajectory_t nominal_trajectory;
			createNominalTrajectory(tree, nominal_trajectory);
			simulate(dynamics, tree, nominal_trajectory, collision_step, true, robot);
		}

		if (key == 't') {
			float average_probability_of_collision = 0.0f;
			testSolution(dynamics, tree, NUM_SIMS, average_probability_of_collision, false);
		}

		if (key == 'e') {
			float average_probability_of_collision = 0.0f;

			ostringstream current_experiment_name;
			ostringstream simulation_record;

			FILE * simulation_log = fopen(make_log_file_name(experiment_name, "simulations", "txt").c_str(), "w");
			WRITE_HEADER(simulation_record, "experiment\tsimulations\tsuccessful\ttrajectory\tthreshold", simulation_log);

			// Plan multiple trajectories
			for (size_t trajectory_count = 0; trajectory_count < TRAJECTORY_COUNT; ++trajectory_count) {
				experiment_base_name.clear();
				experiment_base_name.str("");
				experiment_base_name << experiment_name << "_trajectory_" << trajectory_count;

				// Iterate over thresholds
				for (size_t threshold_idx = 0; threshold_idx < distance_threshold_count; ++threshold_idx) {
					paths = 0;

					vis::clearPaths();
					vis::clearSimulation();

					world->setDistanceThreshold(distance_thresholds[threshold_idx]);
					current_experiment_name.clear();
					current_experiment_name.str("");
					current_experiment_name << experiment_base_name.str() << "_threshold_" << world->getDistanceThreshold();

					std::cout << "Planning for " << current_experiment_name.str() << "... ";

					if (planTrajectory(current_experiment_name.str(), tree, radius, world->getDistanceThreshold(), FIND_FIRST_PATH_ONLY)) {
						std::cout << "success." << std::endl << "Simulating... " << std::endl;

						size_t good_runs = testSolution(dynamics, tree, NUM_SIMS, average_probability_of_collision, false);

						simulation_record.clear();
						simulation_record.str("");
						simulation_record << current_experiment_name.str() << '\t' << NUM_SIMS << '\t' << good_runs << '\t' << trajectory_count << '\t' << world->getDistanceThreshold() << std::endl;
						fputs(simulation_record.str().c_str(), simulation_log);
					} else {
						std::cout << "failed" << std::endl;
					}

					fflush(simulation_log);
				}
			}

			fclose(simulation_log);
		}

		if (key == 'a') {
			ostringstream current_experiment_name;
			ostringstream collision_probabilities_record;

			FILE * collision_probabilities_log = fopen(make_log_file_name(experiment_name, "collision_probabilities", "txt").c_str(), "w");
			WRITE_HEADER(collision_probabilities_record, "experiment\tthreshold\tmean average probability of collision\tmean average probability of success", collision_probabilities_log);

			int trajectories_found = 0;

			// Iterate over thresholds
			for (size_t threshold_idx = 0; threshold_idx < distance_threshold_count; ++threshold_idx) {
				float average_probability_of_collision = 0.0f;
				float mean_average_probability_of_collision = 0.0f;

				// Load each trajectory
				for (size_t trajectory_count = 0; trajectory_count < TRAJECTORY_COUNT; ++trajectory_count) {
					paths = 0;

					vis::clearPaths();
					vis::clearSimulation();

					world->setDistanceThreshold(distance_thresholds[threshold_idx]);
					current_experiment_name.clear();
					current_experiment_name.str("");
					current_experiment_name << experiment_name << "_trajectory_" << trajectory_count << "_threshold_" << world->getDistanceThreshold();

					string tree_file_name = make_log_file_name(current_experiment_name.str(), "tree", "rrt");
					std::cout << "Reading " << current_experiment_name.str() << " tree from (" << tree_file_name << ")." << std::endl;
					read_tree(tree_file_name, &tree);

					// If no trajectory was found move on
					if (!(tree[0].cost_from_start < DBL_MAX)) {
						std::cout << "No trajectory found... skipping.";
						continue;
					} else {
						++trajectories_found;
					}

					average_probability_of_collision = 0.0f;
					size_t good_runs = testSolution(dynamics, tree, NUM_SIMS, average_probability_of_collision, false);
					mean_average_probability_of_collision += average_probability_of_collision;

					fflush(collision_probabilities_log);
				}

				mean_average_probability_of_collision /= float(TRAJECTORY_COUNT);

				std::cout << "Mean average probability of collision: " << std::fixed << std::setw(10) << std::setprecision(10) << mean_average_probability_of_collision << std::endl;
				std::cout << "Mean average probability of success: " << (1.0f - mean_average_probability_of_collision) << std::endl;

				collision_probabilities_record.clear();
				collision_probabilities_record.str("");
				collision_probabilities_record << std::fixed << std::setw(10) << std::setprecision(10) << experiment_name << '\t' << world->getDistanceThreshold()  << '\t' << mean_average_probability_of_collision << '\t' << (1.0f - mean_average_probability_of_collision) << std::endl;
				fputs(collision_probabilities_record.str().c_str(), collision_probabilities_log);
			}

			fclose(collision_probabilities_log);
		}

		if (key == 'p') {
			ostringstream current_experiment_name;
			ostringstream simulation_record;

			FILE * simulation_log = fopen(make_log_file_name(experiment_name, "path_simulations", "txt").c_str(), "w");
			WRITE_HEADER(simulation_record, "experiment\tthreshold\ttrajectory_id\tnum_paths\tpath_id\tpath_length\tsimulations\tsuccessful", simulation_log);

#define WRITE_SIMULATION_RECORD(threshold, trajectory_idx, num_paths, path_idx, path_length, simulation, successful)	\
	simulation_record.clear();	\
	simulation_record.str("");	\
	simulation_record << experiment_name << '\t' << threshold << '\t' << trajectory_idx << '\t' << num_paths << '\t' << path_idx << '\t' << path_length << '\t' << simulation << '\t' << successful << std::endl;	\
	fputs(simulation_record.str().c_str(), simulation_log);

			const int path_count = 2;
			const int path_idxs[path_count] = {0, -1}; // Simulate the first and last path
			int current_path_idx = path_idxs[0];

			float current_threshold = 0.0f;

			for (size_t threshold_idx = 0; threshold_idx < all_distance_threshold_count; ++threshold_idx) {
				current_threshold = all_distance_thresholds[threshold_idx];

				for (size_t trajectory_idx = 0; trajectory_idx < TRAJECTORY_COUNT; ++trajectory_idx) {
					current_experiment_name.clear();
					current_experiment_name.str("");
					current_experiment_name << experiment_name << "_trajectory_" << trajectory_idx << "_threshold_" << current_threshold;

					string path_log_file_name = make_log_file_name(current_experiment_name.str(), "path", "path");
					std::cout << "Reading paths from " << path_log_file_name << "." << std::endl;

					state_lists_t state_lists;
					loadPathsAsTrees(path_log_file_name, state_lists);

					// Determine whether any paths were found in the file
					if (state_lists.size() == 0) {
						std::cout << "No paths found... skipping.";

						WRITE_SIMULATION_RECORD(current_threshold, trajectory_idx, 0, 0, 0, 0, 0);

						continue;
					}

					for (int path_idx = 0; path_idx < path_count; ++path_idx) {
						std::cout << "path_idx: " << path_idxs[path_idx] << std::endl;
						current_path_idx = path_idxs[path_idx];
						if (current_path_idx < 0) {
							current_path_idx = state_lists.size() + current_path_idx;
						}
						tree_t tree;
						std::cout << "current_path_idx: " << current_path_idx << "\tstate_lists.size(): " << state_lists[current_path_idx].size() << std::endl;
						convertPathToTree(state_lists[current_path_idx], tree);

						vis::clearPaths();
						vis::clearSimulation();

						world->setDistanceThreshold(current_threshold);

						nominal_trajectory_t nominal_trajectory;
						createNominalTrajectory(tree, nominal_trajectory);

						float average_probability_of_collision = 0.0f;
						size_t good_runs = testSolution(dynamics, tree, NUM_SIMS, average_probability_of_collision, false);

						WRITE_SIMULATION_RECORD(current_threshold, trajectory_idx, state_lists.size(), current_path_idx, tree.size(), NUM_SIMS, good_runs);

						fflush(simulation_log);
					}
				}
			}

			fclose(simulation_log);
		}
	} while (key != 'q');

	std::cout << "Quitting." << std::endl;
	exit(-1);

	return 0;
}
