#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

// Optimization flags
//#define USE_PARTIAL_PIVOTING // Forces the Matrix::operator% to use partial pivoting in lieu of complete pivoting
//#define SPECIALIZE_2X2
//#define FRONT_LOAD_RK4 // Forces the RK4 stuff to be loaded globally and initialized in the main as opposed to in the function calls where it's used

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

#if defined(CLOSED_FORM) || defined(CLOSED_FORM_FORWARD)
#define compute_cost compute_cost_closed_form
#else
#define compute_cost compute_cost_RK4
#endif

#if defined(CLOSED_FORM) || defined(CLOSED_FORM_BACKWARD)
#define check_path check_path_closed_form
#else
#define check_path check_path_RK4
#endif

// Manages which dynamics are used
#define SINGLE_INTEGRATOR_2D 1
#define DOUBLE_INTEGRATOR_1D 2
#define DOUBLE_INTEGRATOR_2D 3
#define QUADROTOR 4
#define NONHOLONOMIC 5
#define DYNAMICS 3
#define USE_DYNAMMICS_OBJ

// Flags to control various features of the program
//#define EXPERIMENT
#define REDUCE_RADIUS // Determines whether the radius should be reduced as the tree grows - This misses solutions and saves very little time. Observe the 1D double integrator. -- to be used in conjunction with USE_RANGE
//#define SHOW_COLLISION_CHECKS // Determines whether to show the collision checks
//#define SHOW_COLLISIONS // Determines whether to show the collisions
#define USE_OBSTACLES 6 // Determines whether to create obstacles
#define SHOW_TREE true // Determines whether to render the tree -- set to true or false
#define USE_HEURISTICS // Determines whether heuristics should be used in evaluating sample points
#define NODE_SIZE 0.1 // Determines the base size of noedes use for displaying things like the path, milestones, etc.
//#define ALLOW_ORPHANS // Determines whether orphans are permitted
#define SHOW_PATHS // Determines whether the paths should be displayed
#define SHOW_ROBOT // Determines whether the robot should be displayed
//#define VISUALIZE_LOG // Determines whether paths from the log should be played back
#define MAKE_STILLS -1 // Determines whether stills should be made -- set to -1 to disable
#define STILL_RATE 25 // Determines how frequently stills should be made
#define HUE_SHIFT
//#define GRAPH_PATH
//#define PLOT_PATH // Displays just a single spline
#ifdef REDUCE_RADIUS
#define K_D_TREE_BACKWARD
#define K_D_TREE_FORWARD
#endif

#if MAKE_STILLS > -1
#undef SHOW_PATHS
#undef SHOW_ROBOT
#endif

#endif // __CONFIGURATION_H__