#define _USE_MATH_DEFINES

/*
TODO
- Add asserts to verify that certain values exist like sphere_volume
- Abstract robot
-- World::checkCollisions currently rotates and positions the robot, the robot or the dynamics should do it
-- Abstract control space
- Separate state space from world
- Re-organize initialization
- Abstract dynamics
*/

#include <SDKDDKVer.h>
#include <stdio.h>
#include <tchar.h>
#include <string>
#include <time.h>
//#include <utility>
#include <vector>
#include <stack>
#include <queue>
#include <list>
#include <map>
#include <utility>
#ifdef _MSC_VER
#include <Windows.h>
#else
#include <sys/utime.h>
#endif
//#include <omp.h>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

//#include "matrix.h"
#include "callisto.h"
#include "callistoTypes.h"
//#define _POLY_DEBUG
#include "polynomial.h"
#include "rpoly.h"

//#define _DEBUG_COMPUTE_COST
#define SHOW_AXIS false

// Optimization flags
//#define FRONT_LOAD_RK4 // Forces the RK4 stuff to be loaded globally and initialized in the main as opposed to in the function calls where it's used
//#define USE_PARTIAL_PIVOTING // Forces the Matrix::operator% to use partial pivoting in lieu of complete pivoting
//#define SPECIALIZE_2X2

// Control over whether closed form solutions are used
#define CLOSED_FORM
//#define CLOSED_FORM_FORWARD
//#define CLOSED_FORM_BACKWARD
//#define CHECK_CLOSED_FORM

// Ensure we're not using the closed form function if we're attempting to validate it
#ifdef CHECK_CLOSED_FORM
#undef CLOSED_FORM
#undef CLOSED_FORM_FORWARD
#undef CLOSED_FORM_BACKWARD
#endif

#define EXPERIMENT_NAME "test"

// Manages which dynamics are used
#define SINGLE_INTEGRATOR_2D 1  // KD tree is not defined for this -- more specifically calc_forward_reachable_set is not defined.
#define DOUBLE_INTEGRATOR_1D 2
#define DOUBLE_INTEGRATOR_2D 3
#define QUADROTOR 4
#define NONHOLONOMIC 5
#define DYNAMICS SINGLE_INTEGRATOR_2D

std::string dynamics_type_to_name(const int id) {
	std::string name = "UNKNOWN";

	switch(id) {
	case SINGLE_INTEGRATOR_2D:
		name = "SINGLE_INTEGRATOR_2D";
		break;
	case DOUBLE_INTEGRATOR_1D:
		name = "DOUBLE_INTEGRATOR_1D";
		break;
	case DOUBLE_INTEGRATOR_2D:
		name = "DOUBLE_INTEGRATOR_2D";
		break;
	case QUADROTOR:
		name = "QUADROTOR";
		break;
	case NONHOLONOMIC:
		name = "NONHOLONOMIC";
		break;
	}

	return name;
}

#define POSITION_DIM 2

// Flags to control various features of the program
//#define EXPERIMENT
#define USE_THRESHOLDS 1
#define DISTANCE_THRESHOLD 0
#define REDUCE_RADIUS 0 // Determines whether the radius should be reduced as the tree grows - This misses solutions and saves very little time. Observe the 1D double integrator. -- to be used in conjunction with USE_RANGE
//#define SHOW_COLLISION_CHECKS // Determines whether to show the collision checks
//#define SHOW_COLLISIONS // Determines whether to show the collisions
//#define SHOW_THRESHOLD_CHECKS // Determines whether to show the states that were distance checked
//#define RUN_COLLISION_TESTER // Runs a utility to allow for manual checking of collision functionality
#define SHOW_ROBOT 0 // Determines whether the robot model should be shown
#define SHOW_ROBOT_COLLISION_CHECKER 0 // Determines whether the robot collision checker should be displayed
#define SHOW_COLLISION_SLEEP_TIME 10
#define USE_OBSTACLES 6 // Determines whether to create obstacles
#define VISUALIZE_TREE
#define VISUALIZE_PATHS
#define SHOW_TREE false // Determines whether to render the tree by default -- set to true or false
#define SHOW_NODES false // Determines whether to render the nodes of the tree by default -- set to true or false
#define SHOW_TREE_PATHS false // Determines whether to render the paths represeented by the tree by default -- set to true or false
#define SHOW_VELOCITIES false // Determines whether to render the velocities at each state in the tree by default -- set to true or false
#define USE_HEURISTICS // Determines whether heuristics should be used in evaluating sample points
#define NODE_SIZE 0.25 // Determines the base size of noedes use for displaying things like the path, milestones, etc.
//#define ALLOW_ORPHANS // Determines whether orphans are permitted
#define SHOW_PATHS // Determines whether the paths should be displayed
//#define VISUALIZE_LOG // Determines whether paths from the log should be played back
#define MAKE_STILLS -1 // Determines whether stills should be made -- set to -1 to disable
#define STILL_RATE 25 // Determines how frequently stills should be made
#define HUE_SHIFT
//#define GRAPH_PATH
//#define PLOT_PATH // Displays just a single spline
#define EPSILON 0.1 // Probabitlity of checking the goal for a connection
#define TIMING_FREQUENCY 0.1 // How often to record the time taken to expand nodes
//#define WORLD EmptyWorld // Which world to use

#if (defined(REDUCE_RADIUS) && DYNAMICS == SINGLE_INTEGRATOR_2D)
#undef REDUCE_RADIUS
#endif

#if (defined(REDUCE_RADIUS) && DYNAMICS == SINGLE_INTEGRATOR_2D)
#error Reduce radius does not work with this system.
#endif

#ifdef REDUCE_RADIUS
#define K_D_TREE_BACKWARD
#define K_D_TREE_FORWARD
#endif

#if MAKE_STILLS > -1
#undef SHOW_PATHS
#define SHOW_ROBOT_COLLISION_CHECKER false
#endif

const double deltaT = 0.03125;
const double gravity = 9.81;
const double mass = 0.5;
const double inertia = 0.1;
const double length = (0.3429/2);
double control_penalty = 1;
double control_penalty1 = 0;
double sphere_volume;

#if (DYNAMICS == QUADROTOR) // Quadrotor
#define ROBOT Quadrotor
#define POSITION_DIM 3

#define STILL_RATE 5
#define USE_OBSTACLES 5 // Determines whether to create obstacles
#define TARGET_NODES 10000 // Determines how many 2000s the tree should have
#define START_RADIUS 100 // Determines the starting radius - Ignored if REDUCE_RADIUS is set.
#define RADIUS_MULTIPLIER 10

#define POLY_DEGREE 8
#define DIM 3
#define X_DIM 10
#define Z_DIM 10
#define U_DIM 3
#define NODE_SIZE 0.01

#elif (DYNAMICS == NONHOLONOMIC) // Car
#define ROBOT Nonholonomic
#define TARGET_NODES 25000
#define START_RADIUS 20
#define RADIUS_MULTIPLIER 1

#define POLY_DEGREE 6
#define DIM 2
#define X_DIM 5
#define Z_DIM 5
#define U_DIM 2

#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D) // 2D double integrator
#define ROBOT Puck
#define DISTANCE_THRESHOLD 5
#define TARGET_NODES 2000 // Determines how many nodes the tree should have
#define START_RADIUS 10 // Determines the starting radius - Ignored if REDUCE_RADIUS is set.
#define RADIUS_MULTIPLIER 1.01

#define POLY_DEGREE 4
#define DIM 2
#define X_DIM 4
#define Z_DIM 4
#define U_DIM 2

#define X_COORD 0
#define Y_COORD 1
#define XV_COORD 3
#define YV_COORD 4

#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) // 2D single integrator
#define ROBOT Puck
#define DISTANCE_THRESHOLD 10
#define TARGET_NODES 500
#define START_RADIUS 50
#define RADIUS_MULTIPLIER 1

#define POLY_DEGREE 2
#define DIM 2
#define X_DIM 2
#define Z_DIM 2
#define U_DIM 2

#else // 1D double integrator
#define ROBOT Puck
#define TARGET_NODES 10000 // Determines how many nodes the tree should have
#define START_RADIUS 100 // Determines the starting radius - Ignored if REDUCE_RADIUS is set.
#define RADIUS_MULTIPLIER 2.5

#define POLY_DEGREE 4
#define DIM 1
#define X_DIM 2
#define Z_DIM 2
#define U_DIM 1

#endif

#if (DYNAMICS == QUADROTOR)
	#define STATE_SPACE QuadrotorStateSpace
	#define WORLD TwoWalls
#elif (DYNAMICS == SINGLE_INTEGRATOR_2D) || (DYNAMICS == DOUBLE_INTEGRATOR_2D) || (DYNAMICS == NONHOLONOMIC)
	#if USE_OBSTACLES == 6
		#if (DYNAMICS == NONHOLONOMIC)
			#define STATE_SPACE NonholonomicStateSpace
			#define WORLD SymmetricRaceTrackMaze
		#elif (DYNAMICS == DOUBLE_INTEGRATOR_2D)
			#define STATE_SPACE StateSpace
			//#define WORLD TwoPathMaze
			//#define WORLD worlds::LudersBoxes
			#define WORLD worlds::VanDenBergPassages
		#else
			#define STATE_SPACE StateSpace
			//#define WORLD TwoPathMaze
			//#define WORLD worlds::LudersBoxes
			#define WORLD worlds::VanDenBergPassages
		#endif
	#elif USE_OBSTACLES == 5
		#define WORLD SimpleRaceTrack
	#elif USE_OBSTACLES == 4
		#define WORLD HardSMaze
	#elif USE_OBSTACLES == 3
		#define WORLD EasySMaze
	#elif USE_OBSTACLES == 2
		#define WORLD FourRooms
	#elif USE_OBSTACLES == 1
		#define WORLD Cylinders
	#endif
#else
	#define STATE_SPACE StateSpace
	#define WORLD EmptyWorld
#endif

#include "bounds.hpp"
#include "state.hpp"
#include "robots.hpp"
#include "worlds.hpp"

#define REACHABILITY_CONSTANT 1.5
unsigned int TWO_TO_X_DIM;
double statespace_volume;
const double X_DIM_INVERSE = 1.0/X_DIM;

FILE* time_log;
FILE* stats_log;
FILE* path_log;
FILE* experiment_log;

const int NO_PARENT = -1;
typedef int node_id_t;
typedef double cost_t;
typedef double location_t;
typedef std::list<node_id_t> node_list_t;
typedef std::pair<double, node_id_t> node_cost_pair_t; // The first element is the cost to the node specified by the second element
typedef std::pair<double, state> state_time_t;
typedef std::vector< state_time_t, Eigen::aligned_allocator<state_time_t> > state_time_list_t;
typedef std::vector< node_id_t > path_t;

//BOUNDS x_bounds;
BOUNDS u_bounds;
#if (DYNAMICS == QUADROTOR)
#if (USE_OBSTACLES == 5)
//BOUNDS x_bounds_window_1;
//BOUNDS x_bounds_window_2;
#endif
#endif

typedef Eigen::Matrix<double,U_DIM,1> control_t;
typedef Eigen::Matrix<double,3,3> generic_3d_matrix_t;
typedef Eigen::Matrix<double,X_DIM,X_DIM> natural_dynamics_t;
typedef Eigen::Matrix<double,X_DIM,U_DIM> control_dynamics_t;
typedef Eigen::Matrix<double,X_DIM,X_DIM> motion_noise_covariance_t;
typedef Eigen::Matrix<double,Z_DIM,X_DIM> observation_t;
typedef Eigen::Matrix<double,Z_DIM,Z_DIM> observation_noise_covariance_t;
typedef Eigen::Matrix<double,Z_DIM,1> observation_noise_t;
typedef state motion_noise_t;
typedef state motion_bias_t;
typedef Eigen::Matrix<double,Z_DIM,1> observation_bias_t;
typedef Eigen::Matrix<double,U_DIM,U_DIM> control_penalty_t;

natural_dynamics_t A = natural_dynamics_t::Zero();
control_dynamics_t B = control_dynamics_t::Zero();
motion_bias_t c = motion_bias_t::Zero();
control_penalty_t R = control_penalty_t::Zero();
Eigen::Matrix<double,X_DIM,X_DIM> BRiBt;
state x0 = state::Zero();
state x1 = state::Zero();
Eigen::Matrix<double,2*X_DIM,2*X_DIM> Alpha;
Eigen::Matrix<double,2*X_DIM,1> chi;
Eigen::Matrix<double,2*X_DIM,1> c0;
motion_noise_covariance_t M = motion_noise_covariance_t::Zero();
state v = state::Zero();
observation_t C = observation_t::Zero();
observation_bias_t d = Eigen::Matrix<double,Z_DIM,1>::Zero();
observation_noise_covariance_t N = observation_noise_covariance_t::Zero();
Eigen::Matrix<double,Z_DIM,1> w = Eigen::Matrix<double,Z_DIM,1>::Zero();
generic_3d_matrix_t Rotation = generic_3d_matrix_t::Identity();
generic_3d_matrix_t Scale = generic_3d_matrix_t::Identity();

struct dynamics_t {
	natural_dynamics_t A;
	control_dynamics_t B;
	motion_bias_t c;
	motion_noise_covariance_t M;
	observation_t C;
	observation_noise_covariance_t N;
	observation_bias_t d;
};

struct Node {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	node_id_t   parent;
	state       x;
	cost_t      cost_from_start;
	node_list_t children;
};

typedef std::vector<Node, Eigen::aligned_allocator<Node> > tree_t;

struct KD_Node {
	node_id_t  parent_id;
	node_id_t  node_id;
	location_t location;
	node_id_t  left_id;
	node_id_t  right_id;
};

typedef std::pair<int, node_id_t > splitting_dim_node_id_t;
typedef std::vector<splitting_dim_node_id_t > k_d_stack_t;
typedef std::vector<node_id_t > node_ids_t;

/*int axis_group, collision_hit_group, collision_free_group, robot_group, robot_object
	, start_node_group, goal_node_group, node_group, edge_group, velocity_group, solution_group
	, solution_marker_group, robot_model*/;

int stills_group;
std::vector< std::pair<int, Eigen::Matrix<double,3,1> > > stills_groups;
double current_alpha = 0.5;
double alpha_change = 0.0;

double *points;
double *current_point;
int *idx;

int paths; // Counts the number of paths

double start_time, current_time, end_time;
#define TimeToInt(t) ((t.wHour*3600 + t.wMinute*60 + t.wSecond)*1000 + t.wMilliseconds)

bool (*computeCost)(const state& x0, const state& x1, double radius, double& cost, double& tau, state& d_tau) = NULL;
bool __cdecl computeCostClosedForm(const state& x0, const state& x1, double radius, double& cost, double& tau, state& d_tau);
bool __cdecl computeCostRK4(const state& x0, const state& x1, double radius, double& cost, double& tau, state& d_tau);

bool (*checkPath)(const state& x0, const state& x1, const double tau, const state& d_tau, const bool plot, state_time_list_t* vis) = NULL;
bool __cdecl checkPathClosedForm(const state& x0, const state& x1, const double tau, const state& d_tau, const bool plot, state_time_list_t* vis = NULL);
bool __cdecl checkPathRK4(const state& x0, const state& x1, const double tau, const state& d_tau, const bool plot, state_time_list_t* vis = NULL);

char _getchar();

bool connect(const state& x0, const state& x1, const double radius, double& cost, double& tau, state_time_list_t* vis);

void convertTreeToPoints(const tree_t& tree, double *points);
	
template<typename vec, typename bounds>
inline void rand_vec(vec& v, const bounds& b);

template <size_t numRows1, size_t numRows2, size_t numCols1, size_t numCols2>
inline Eigen::Matrix<double,numRows1+numRows2, numCols1+numCols2> block(const Eigen::Matrix<double,numRows1,numCols1> A, const Eigen::Matrix<double,numRows1,numCols2> B,
																			const Eigen::Matrix<double,numRows2, numCols1> C, const Eigen::Matrix<double,numRows2,numCols2> D) 
{
	Eigen::Matrix<double,numRows1+numRows2,numCols1+numCols2> m;
	m.block<numRows1,numCols1>(0,0) = A;		m.block<numRows1,numCols2>(0,numCols1) = B;
	m.block<numRows2,numCols1>(numRows1,0) = C;	m.block<numRows2,numCols2>(numRows1,numCols1) = D;
	return m;
}

/**
 * The idea for the structure of this k-d tree is to use a structure for each node. The structure indicates whether the node
 * is a leaf, in which case it has a list of children from the RRT, or not, in which case it has at least one child to the
 * left or right. The list of children from the RRT are kept in a separate list to limit memory usage since only the leaf
 * nodes will have lists of children from the RRT.
 */
class KD_Tree {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	const int NO_LIST; // Indicates whether a node has a list of children
	const int NO_CHILD; // Indicates whether there is a node as child k-d tree node to the left/right of a given nod
	const int NO_SPLIT; // Indicates whether a node is split

	typedef int list_id_t;
	typedef std::list<node_id_t> child_list_t;
	typedef std::pair<double, double> bounds_t;

	// This is the core structure of the tree. It describes each node in the tree.
    // e.g. what region it represents, whether the node is a leaf or a list of children, etc..
	struct node_t {
		int depth; // The depth of this node in the tree, this will be used to determine whether we must calculate the bounds of new nodes
                   // based on the world bounds or the bounds of one of this nodes ancestors. We need to use the world bounds if there are
                   // less than n layers in the tree. Otherwise we go back n layers and split the region defined by that node.
		int split_dimension; // Which dimension this node splits on
		double split_value; // If this is a parent node then the split value indicates when to go left vs. right
		bounds_t bounds; // first => left bounds, second => right bounds
		node_id_t parent_id; // The id of the parent node
		std::pair< node_id_t, node_id_t > children; // If this node is not a leaf this structure indicates who its children are
		list_id_t child_list_id; // If this node is a leaf this is the id of its list children
                                 // these children are kept in a separate list to reduce the amount
                                 // of memory each node requires (NOTE: the number of lists required
                                 // is less than the number of nodes that will end up in the k-d tree).
	};

	typedef std::vector<node_t> nodes_t;

private:
	const tree_t& rrt_tree;
	int dimensions;
	BOUNDS world_bounds; // The bounds of the world in case the tree is not deep enough to 
	int split_threshhold; // The max number of RRT nodes per region
	nodes_t tree; // This contains all the nodes in the tree
	std::vector< child_list_t > child_lists; // Each node will have children to its left & right in the k-d tree or a list of children from the RRT.
                                                       // This list contains the list of children from the RRT for any given node in the k-d tree.
                                                       // Once a list is created it will never need to be deleted.
                                                       // If a parent is split then one of the children can take over the list
	                                                   // (it will just need to be split between the children)
	node_id_t next_new_tree_id; // This is the index of the next position in the tree where a node will be inserted
	list_id_t next_new_list; // This indicates the id of the next empty list that can be used for new nodes when a split occurs

public:

	/**
	* \brief Initialize the tree.
    *
	* bounds - Left/Right bounds of region covered by current.
	* expected_nodes - Number of nodes expected, this will prevent allocation of space whenever a new node is added.
	* splitting_dimension - The dimension on which to start splitting.
	* splitting_threshhold - The maximum number of nodes which a bucket can hold before being split. Defaults to 10.
	*/
	KD_Tree(const tree_t& rrt, const int dims, const BOUNDS& bounds, int expected_nodes, int splitting_dimension = 0, int split_threshhold_ = 10)
	: NO_LIST(-1), NO_CHILD(-1), NO_SPLIT(-1), rrt_tree(rrt), dimensions(dims), world_bounds(bounds), split_threshhold(split_threshhold_), next_new_tree_id(0), next_new_list(0)
	{
		// Initialize the vector of lists of children - this is all about front loading memory allocation
		child_lists.resize(expected_nodes); // At most there will be expected_nodes number of leafs, 
		                                    // at that point they'll have one element each, 
 		                                    // before that they will have up to 10 elements
		/*
		for (int i = 0; i < expected_nodes; i++) { // Resize each child list to prevent them from being resized later (i.e. this is front loading the memory allocation)
			child_lists[i].resize(split_threshhold, NO_CHILD);
		}
		*/

		// Initialize the tree
		tree.resize(expected_nodes); // Allocate the memory we expect to need

		node_t node; // Create & populate the first node
		node.depth = 0;
		node.split_dimension = splitting_dimension;
		node.bounds = bounds[node.split_dimension];
		node.split_value = node.bounds.first + (node.bounds.second - node.bounds.first)/2.0;
		node.parent_id = NO_PARENT;
		node.child_list_id = next_new_list++; // A node starts with a list of children from the RRT until it reaches the splitting threshold
		node.children.first = NO_CHILD;
		node.children.second = NO_CHILD;

		//tree.push_back(node); // Set the first node
		tree[next_new_tree_id++] = node;
	}

	/**
	* \brief Add a new node to the tree
	*
	* One weakness of the this method is that it uses a naive split. It splits when a node will have split_threshhold number of elements.
	* However there is no guarantee that the list has elements from both sides of the split_value because the split value is selected as
	* the geometric center and not the median as prescribed by the k-d tree algorithm. As such it's conceivable that all the elements will
	* end up on one side of the split.
	*
	* node_id - the RRT node_id of the node to be added
	*/
	void add(const node_id_t& rrt_node_id) {
		// Locate the k-d tree node to which the RRT node_id belongs
		//node_id_t kd_node_id = find_parent(rrt_node_id);

		node_id_t kd_node_id = 0;

		// Search for the leaf to which this new point belongs
		std::stack< node_id_t > node_id_stack;
		node_id_stack.push(0); // Seed the stack with the root of the tree
		while(!node_id_stack.empty()) {
			// Retrieve the top of the stack
			kd_node_id = node_id_stack.top();
			node_id_stack.pop();
			node_t& kd_node = this->tree[kd_node_id];

			// Determine whether to go left or right
			// Split or end
			if (kd_node.children.first == NO_CHILD) { // If the current node is a leaf
				
				// Split if necessary
				child_list_t& child_list = child_lists[kd_node.child_list_id];
				if ((child_list.size() + 1) > split_threshhold) {
					// Create the left and right nodes
					node_t right_node, left_node;
					right_node.parent_id = left_node.parent_id = kd_node_id; // Set the parent id of the new nodes
					right_node.depth = left_node.depth = kd_node.depth + 1; // Calculate the depth of the new nodes
					right_node.split_dimension = left_node.split_dimension = (kd_node.split_dimension + 1) % dimensions; // Calculate the splitting dimension of the new nodes
					right_node.children.first = right_node.children.second = left_node.children.first = left_node.children.second = NO_CHILD; // Note that the new nodes have no children -- they start with a list until there split_threshhold number of elements in the list

					// Create the child lists of the new nodes. This is done by allocating a new list for the right node 
					// and associating the current node's child list with the lft noede. We do the latter because the current
					// node no longer needs a child list and to reduce processing time due to memory management.
					right_node.child_list_id = next_new_list++;
					left_node.child_list_id = kd_node.child_list_id;
					kd_node.child_list_id = NO_LIST;

					// Separate the child list into left and right
					child_list_t::iterator m;
					child_list_t& right_list = child_lists[right_node.child_list_id];
					for (child_list_t::iterator n = child_list.begin(); n != child_list.end();) {
						// Determine whether the value of the current splitting dimension element of the state is greater than the splitting value
						if (rrt_tree[*n].x[kd_node.split_dimension] >= kd_node.split_value) {
							right_list.push_back(*n); // Add the node to the right list
							m=n;
							n++;
							child_list.erase(m); // Rmove the node from the left list
							continue;
						}
						n++;
					}

					// Set the children of the current node

					// Add the nodes to the tree and update parent node with child id
					kd_node.children.first = next_new_tree_id;
					tree[next_new_tree_id++] = left_node;

					kd_node.children.second = next_new_tree_id;
					tree[next_new_tree_id++] = right_node;

					// Set the bounds on the new nodes
					this->calculate_bound(kd_node.children.first, this->tree[kd_node.children.first].bounds);
					this->tree[kd_node.children.second].bounds = this->tree[kd_node.children.first].bounds;
					this->tree[kd_node.children.first].split_value = this->tree[kd_node.children.second].split_value = this->tree[kd_node.children.first].bounds.first + (this->tree[kd_node.children.first].bounds.second - this->tree[kd_node.children.first].bounds.first)/2.0;

					// Grow the tree if we have run out of space - note that we need room for 2 new elements at any given time
					if (next_new_tree_id == (this->tree.size() - 1)) {
						this->tree.resize(2*this->tree.size());
					}

				}

				// Otherwise add the node to the list
				else {
					child_list.push_back(rrt_node_id);
					return;
				}
			} // End If the current node is a leaf

			// Go left
			if (this->rrt_tree[rrt_node_id].x[this->tree[kd_node_id].split_dimension] < this->tree[kd_node_id].split_value) {
				node_id_stack.push(this->tree[kd_node_id].children.first);
			}

			// Go right
			else {
				node_id_stack.push(this->tree[kd_node_id].children.second);
			}
		} // End while stack not empty
	}

	/**
	 * \brief Locates all points withing the specified axis-aligned hyper-rectangle
	 *
	 * query - Bounds of hyper-rectangle
	 * results - A list to be populated by this routine of RRT node ids that are within the query hyper-rectangle
	 */
	void range_query(const BOUNDS& query, node_ids_t& results) {
		// Local variables
		bool in = true; // Indicates whether a given state is within the query bounds
		node_id_t kd_node_id = 0; // Current k-d tree node id

		// Setup stack
		std::stack<node_id_t> nodes;
		nodes.push(kd_node_id); // Seed the stack with the root node

		// Search stack
		while (!nodes.empty()) {
			// Grab the node on the top of the stack
			kd_node_id = nodes.top();
			nodes.pop();
			node_t& kd_node = this->tree[kd_node_id];

			// Determine whether node is a leaf. If it is then determine which nodes in the list are within
			// the query bounds.
			if (kd_node.children.first == NO_CHILD) {
				for (child_list_t::iterator n = this->child_lists[kd_node.child_list_id].begin(); n != this->child_lists[kd_node.child_list_id].end(); n++) {
					in = true; // Assume the node is within the query bounds
					for (int i = 0; i < this->dimensions; i++) { // Check the bounds for each dimension
						// Determine whether the state is within the query bounds
						if ((this->rrt_tree[*n].x[i] < query[i].first) || (this->rrt_tree[*n].x[i] > query[i].second)) {
							in = false; // The state is not within the query bounds for dimension i so set this to false and break the loop
							break;
						}
					}

					// If the state is within the query bounds add it to the results list
					if (in) {
						results.push_back(*n);
					}
				}
			}

			// If splitting plain intersects query region search both sides
			else if ((kd_node.split_value >= query[kd_node.split_dimension].first) && (kd_node.split_value <= query[kd_node.split_dimension].second)) {
				nodes.push(kd_node.children.first);
				nodes.push(kd_node.children.second);
			}

			// If splitting plain is to the left of the query bounds, search right only
			else if (kd_node.split_value <= query[kd_node.split_dimension].first) {
				nodes.push(kd_node.children.second);
			}

			// finally, splitting plain is to the right of query bounds so search left only
			else {
				nodes.push(kd_node.children.first);
			}
		} // End while nodes not empty
	}

	void summarize() {
		std::cout << "Nodes in tree: " << tree.size() << std::endl;
	}

private:
	/**
	* \brief Determine the tightest bounds for the specificied node using its depth and splitting dimension
	*
	* node_id - id of the node for which bounds are desired.
	* bounds_t - bounds structure via which the results will be returned
	*/
	/*
	void calculate_bound(node_id_t node_id, bounds_t& bounds) {
		node_t& current_node = this->tree[node_id];

		// Determine whether the tree is deep enough for tighter bounds, as compared to the world bounds, to exist
		if (current_node.depth >= this->dimensions) {
			// Walk back up the tree
			for (int i = this->dimensions; i > 0; i--) {
				node_id = this->tree[node_id].parent_id;
			}

			// Grab the bounds of the node this->dimension layers above the current node
			bounds = this->tree[node_id].bounds;
		}

		// Default to the world bounds
		else {
			bounds = world_bounds[current_node.split_dimension];
		}
	}
	*/

	/**
	* \brief Determine the tightest bounds for the specificied node using its depth and splitting dimension
	*
	* node_id - id of the node for which bounds are desired.
	* bounds_t - bounds structure via which the results will be returned
	*/
	void calculate_bound(node_id_t node_id, bounds_t& bounds) {
		node_t& current_node = this->tree[node_id];

		// Determine whether the tree is deep enough for tighter bounds, as compared to the world bounds, to exist
		if (current_node.depth >= this->dimensions) {
			// Walk back up the tree
			int prev_node_id = node_id;
			for (int i = this->dimensions; i > 0; i--) {
				prev_node_id = node_id;
				node_id = this->tree[node_id].parent_id;
			}

			// Grab the bounds of the node this->dimension layers above the current node
			bounds = this->tree[node_id].bounds;

			if (this->tree[node_id].children.first == prev_node_id) {
				// If child to left of parent then reduce the right bound
				bounds.second = bounds.second - (bounds.second - bounds.first)/2.0;
			} else {
				// If child to right of parent then increase left bound
				bounds.first = bounds.first + (bounds.second - bounds.first)/2.0;
			}

			//left_bounds.first = bounds.first;
			//left_bounds.second = right_bounds.first = bounds.first + (bounds.second - bounds.first)/2.0;
			//right_bounds.second = bounds.second;
		}

		// Default to the world bounds
		else {
			bounds = world_bounds[current_node.split_dimension];
		}
	}

public:
	static void testKDTree(tree_t& rrtree) {
		// Locals
		clock_t start, end, kd_time, linear_time;
		int parent = -1;
		int total_nodes = 100000;

		// Setup bounds
		BOUNDS x_bounds;
		x_bounds.resize(2);
		x_bounds[0].first = x_bounds[1].first = -10;
		x_bounds[0].second = x_bounds[1].second = 10;
	
		// Build the trees
		KD_Tree k_d_tree(rrtree, X_DIM, x_bounds, 2*total_nodes, 0, 100);
		bool randomize = true;
		if (randomize) {
			for (int i = 0; i < total_nodes; i++) {
				Node n;
				n.parent = parent;
				rand_vec(n.x, x_bounds);
				rrtree.push_back(n);
				int nid = rrtree.size() - 1;

				std::cout << "nid: " << nid << "\r";
				k_d_tree.add(nid);
				
				// Randomly assign parent
				//if (nid > 0) {
				//	while (parent == 0) {
				//		int temp = rand();
				//		parent = temp % nid;
				//	}
				//} else {
				//	parent = 0;
				//}
			}
		} else {
			Node n;
			n.parent = parent;

			for (int i = -10; i <= 10; i+=2) {
				for (int j = 0; j <= 10; j+=2) {
					n.x[0] = i;
					n.x[1] = j;
					rrtree.push_back(n);
					k_d_tree.add(rrtree.size()-1);
				}
			}
		}
	
		// Setup & execute k-d tree query
		BOUNDS q_bounds;
		q_bounds.resize(X_DIM);
		q_bounds[0].first = 0;
		q_bounds[0].second = 10;
		q_bounds[1].first = 0;
		q_bounds[1].second = 10;
		node_ids_t results;

		start = clock();
		k_d_tree.range_query(q_bounds, results);
		end = clock();
		kd_time = end - start;

		//  Create list of ndoes
		tree_t result_nodes;
		result_nodes.resize(results.size());

		//std::cout << results.size() << " ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
		for (int i = 0; i < results.size(); i++) {
			//std::cout << "i: " << i << "\t" << results[i] << std::endl;
			result_nodes[i] = rrtree[results[i]];
		}
		std::cout << "k-d tree query time: " << kd_time << std::endl;

		if (randomize) {
			node_ids_t results_check;
			tree_t result_nodes_check;
			
			start = clock();
			// For each node
			for (int i = 0; i != k_d_tree.tree.size(); i++) {
				node_t& current_node = k_d_tree.tree[i];

				// Only operate on leaves
				if (current_node.children.first != k_d_tree.NO_CHILD) {
					continue; // Not a leaf
				}

				// Iterate over children
				for (child_list_t::iterator child = k_d_tree.child_lists[current_node.child_list_id].begin(); child != k_d_tree.child_lists[current_node.child_list_id].end(); child++) {
					// Iterate over dimensions
					bool in = true;
					for (int k = 0; k < k_d_tree.dimensions; k++) {
						// Determine whether value within range
					
						if ((k_d_tree.rrt_tree[*child].x[k] < q_bounds[k].first) || (k_d_tree.rrt_tree[*child].x[k] > q_bounds[k].second)) {
							in = false; // It's not, node doesn't belong in result
							break;
						}
					}

					if (in) {
						results_check.push_back(*child);
					}
				}

			}
			end = clock();
			linear_time = end - start;

			std::cout << "Linear query time: " << linear_time << std::endl;

			// Resize nodes vector
			result_nodes_check.resize(results_check.size());
			for (int i = 0; i < results_check.size(); i++) {
				result_nodes_check[i] = k_d_tree.rrt_tree[results_check[i]];
			}

			// Confirm results vectors are equal in size
			if (results.size() == results_check.size()) {
				// Confirm results vectors contain same nodes
				for (tree_t::iterator result_node = result_nodes.begin(); result_node != result_nodes.end(); result_node++) {
					bool found = false;
					for (tree_t::iterator result_node_check = result_nodes_check.begin(); result_node_check != result_nodes_check.end(); result_node_check++) {
						bool match = true;
						for (int k = 0; k < k_d_tree.dimensions; k++) {
							if (result_node->x[k] != result_node_check->x[k]) {
								match = false;
								break;
							}
						}
						if (match) {
							result_nodes_check.erase(result_node_check);
							found = true;
							break;
						}
					}
					if (!found) {
						std::cout << "Missing node" << std::endl;
					}
				}
			} else {
				std::cout << "Error, results don't match." << std::endl << "\tresults.size(): " << results.size() << "\tresults_check.size(): " << results_check.size() << "\tMatch? " << (results.size() == results_check.size()) << std::endl;
			}

			_getchar();
			exit(0);
		}
    }
};
