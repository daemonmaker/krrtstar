#include <Eigen/Dense>

int obstacle_group;

void makeWall(float x, float y, float z, int length, double height, bool vertical, bool reverse) {
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
	//double width = (vertical ? length : 0.5);
	//double depth = (vertical ? 0.5 : length);
	//double x_pos = (reverse ? y - width/2 : y + width/2);
	//double y_pos = z + height/2.0;
	//double z_pos = (reverse ? x - depth/2 : x + depth/2);
	//double xw = width;
	//double yw = height;
	//double zw = depth;

	double depth = height;
	height = (vertical ? length : 0.25);
	double width = (vertical ? 0.25 : length);
	double x_pos = (reverse ? x - width/2.0 : x + width/2.0);
	double y_pos = (reverse ? y - height/2.0 : y + height/2.0);
	double z_pos = z; // + height/2.0;
	double xw = width;
	double yw = height;
	double zw = depth;
#endif

	CAL_CreateBox(obstacle_group, xw, yw, zw, x_pos, y_pos, z_pos);
}

class World {
protected:
	int border_group, obstacle_group;
	BOUNDS x_bounds;
	Eigen::Matrix<double,X_DIM,1> x0, x1;

public:
	World()
	{
		this->x0[0] = 0;
		this->x0[1] = 0;
		this->x1[0] = 100;
		this->x1[1] = 100;

		this->x_bounds.resize(X_DIM);
		for (BOUNDS::iterator p = this->x_bounds.begin(); p != this->x_bounds.end(); ++p) {
			p->first = 0.0;
			p->second = 0.0;
		}

		this->x_bounds[0] = std::make_pair(0, 100);
		this->x_bounds[1] = std::make_pair(0, 100);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*
	void setBounds(const BOUNDS* bounds) {
		for (int i = 0; i < bounds->size(); ++i) {
			this->x_bounds[i].first = bounds[i].first;
			this->x_bounds[i].second = bounds[i].second;
		}
	}
	*/

	void setBound(int idx, const BOUND& bound) {
		this->x_bounds[idx] = bound;
	}

	const BOUNDS getBounds() {
		return this->x_bounds;
	}

	const Eigen::Matrix<double,X_DIM,1> getStartState() {
		return x0;
	}

	const Eigen::Matrix<double,X_DIM,1> getFinalState() {
		return x1;
	}

	virtual void buildEnvironment() {
		CAL_CreateGroup(&(this->obstacle_group), 0, true, "Obstacle");

#if (USE_OBSTACLES > 0)
		CAL_SetGroupVisibility(obstacle_group, 0, true, true);
#else
		CAL_SetGroupVisibility(obstacle_group, 0, false, true);
#endif

#if defined(MAKE_STILLS) && (DYNAMICS != QUADROTOR)
		CAL_SetGroupColor(obstacle_group, 0.1, 0.1, 0.1, 1);
#else
		CAL_SetGroupColor(obstacle_group, 0.1, 0.1, 0.1, 0.1);
#endif

		this->_buildEnvironment();
	}

	virtual void positionCamera() {
		double eye_x = 0.0, eye_y = 0.0, eye_z = 0.0;
		double camera_x = 0.0, camera_y = 0.0, camera_z = 0.0;
		double up_x = 1.0, up_y = 0.0, up_z = 0.0;

		camera_x = eye_x = this->x_bounds[0].second/2.0;
		camera_y = eye_y = this->x_bounds[1].second/2.0;
		eye_z = 150;

		up_x = 0.0;
		up_y = 1.0;
		up_z = 0.0;

		CAL_SetViewParams(0, eye_x, eye_y, eye_z, camera_x, camera_y, camera_z, up_x, up_y, up_z);
	}

	virtual int checkCollisions(int robot_group, int * collisions) {
		return CAL_CheckGroupCollision(robot_group, this->obstacle_group, false, collisions);
	}

	template<typename vec>
	bool validateState(const vec& v) {
		return checkBounds(v, this->x_bounds);
	}

protected:
	void makeWall(float x, float y, float z, int length, double height, bool vertical, bool reverse) {
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
		/*
		double width = (vertical ? length : 0.5);
		double depth = (vertical ? 0.5 : length);
		double x_pos = (reverse ? y - width/2 : y + width/2);
		double y_pos = z + height/2.0;
		double z_pos = (reverse ? x - depth/2 : x + depth/2);
		double xw = width;
		double yw = height;
		double zw = depth;
		*/
		double depth = height;
		height = (vertical ? length : 0.25);
		double width = (vertical ? 0.25 : length);
		double x_pos = (reverse ? x - width/2.0 : x + width/2.0);
		double y_pos = (reverse ? y - height/2.0 : y + height/2.0);
		double z_pos = z; // + height/2.0;
		double xw = width;
		double yw = height;
		double zw = depth;
#endif

		CAL_CreateBox(this->obstacle_group, xw, yw, zw, x_pos, y_pos, z_pos);
	}

	virtual void _buildEnvironment() {}
};

class EmptyWorld
	: public World
{
public:
	EmptyWorld(const BOUNDS& x_bounds)
		: World()
	{}

protected:
};

class TwoPathMaze
	: public World
{
public:
	TwoPathMaze()
		: World()
	{
		this->x_bounds[0] = std::make_pair(0, 200);

		this->x0[0] = 15;
		this->x0[1] = 50;
		this->x1[0] = 140;
		this->x1[1] = 50;
	}

protected:
	virtual void _buildEnvironment() {
		double offset = 0;

		// Make bounding box
		this->makeWall(-10, -10, offset, 120, 20, true, false);
		this->makeWall(-10, -10, offset, 220, 20, false, false);
		this->makeWall(210, -10, offset, 120, 20, true, false);
		this->makeWall(-10, 110, offset, 220, 20, false, false);

		// Build long center dividing wall	
		this->makeWall(25, 50, offset, 100, 20, false, false);

		// Build top S-curve
		this->makeWall(25, 50, offset, 25, 20, true, false);
		this->makeWall(50, 110, offset, 35, 20, true, true);
		this->makeWall(75, 50, offset, 25, 20, true, false);
		this->makeWall(100, 110, offset, 35, 20, true, true);

		// Build bottom S-curve	
		this->makeWall(50, -10, offset, 35, 20, true, false);
		this->makeWall(75, 50, offset, 25, 20, true, true);
		this->makeWall(100, -10, offset, 35, 20, true, false);

		// Build box around goal on left
		this->makeWall(125, 25, offset, 50, 20, true, false);
		this->makeWall(125, 25, offset, 50, 20, false, false);
		this->makeWall(125, 75, offset, 50, 20, false, false);
		this->makeWall(210, 50, offset, 60, 20, false, true);
	}
};

class SymmetricRaceTrackMaze
	: public World
{
public:
	SymmetricRaceTrackMaze()
		: World()
	{
		this->x_bounds[0] = std::make_pair(0, 200);

		this->x0[1] = 44;
		this->x0[0] = 10;
		this->x1[1] = 60;
		this->x1[0] = 10;
	}

protected:
	virtual void _buildEnvironment() {
		double offset = 0;
		makeWall(-10, -10, offset, 120, 20, true, false);
		makeWall(-10, -10, offset, 220, 20, false, false);
		makeWall(210, -10, offset, 120, 20, true, false);
		makeWall(-10, 110, offset, 220, 20, false, false);

		CAL_CreateBox(this->obstacle_group, 50, 50, 20, 50, 50, 10);

		int temp;
		CAL_CreateGroup(&temp, 0, false, "Faux hole");
		CAL_SetGroupColor(temp, 1, 1, 1);
		CAL_SetGroupVisibility(temp, 0, true, true);
		CAL_CreateBox(temp, 49.25, 49.25, 22, 50, 50, 10);

		makeWall(-10, 50, offset, 135, 20, false, false);
		makeWall(210, 50, offset, 60, 20, false, true);

		makeWall(125, 25, offset, 50, 20, true, false);
		makeWall(125, 25, offset, 50, 20, false, false);
		makeWall(125, 75, offset, 50, 20, false, false);

		makeWall(100, -10, offset, 35, 20, true, false);
		makeWall(100, 110, offset, 35, 20, true, true);
	}
};

class SimpleRaceTrack
	: public World
{
public:
	SimpleRaceTrack()
		: World()
	{
		this->x_bounds[0] = std::make_pair(0, 200);
		this->x_bounds[1] = std::make_pair(0, 100);

		this->x0[0] = 10;
		this->x0[1] = 44;
		this->x1[0] = 10;
		this->x1[1] = 56;
	}

protected:
	virtual void _buildEnvironment() {
		int obstacle_height = 5;
		double offset = 0;
		makeWall(-10, -10, offset, 120, obstacle_height, true, false);
		makeWall(-10, -10, offset, 220, obstacle_height, false, false);
		makeWall(210, -10, offset, 120, obstacle_height, true, false);
		makeWall(-10, 110, offset, 220, obstacle_height, false, false);

		CAL_CreateBox(this->obstacle_group, 50, 50, obstacle_height, 50, 50, 2.5);
		CAL_CreateBox(this->obstacle_group, 50, 50, obstacle_height, 150, 50, 2.5);
		CAL_CreateBox(this->obstacle_group, 50, 10, obstacle_height, 100, 70, 2.5);

		makeWall(100, -10, offset, 50, obstacle_height, true, false);
		makeWall(-10, 50, offset, 40, obstacle_height, false, false);
		//makeWall(10, 10, offset, 80, obstacle_height, true, false);
		//makeWall(10, 10, offset, 30, obstacle_height, true, true);
	}
};

class HardSMaze
	: public World
{
public:
	HardSMaze()
		: World()
	{}

protected:
	virtual void _buildEnvironment() {
		makeWall(-20, -20, 0, 140, 20, true, false);
		makeWall(-20, -20, 0, 140, 20, false, false);
		makeWall(120, -20, 0, 140, 20, true, false);
		makeWall(-20, 120, 0, 140, 20, false, false);

		makeWall(25, -20, 0, 90, 20, true, false);
		makeWall(75, 120, 0, 90, 20, true, true);
	}
};


class EasySMaze
	: public World
{
public:
	EasySMaze()
		: World()
	{}

protected:
	virtual void _buildEnvironment() {
		makeWall(-20, -20, 0, 140, 20, true, false);
		makeWall(-20, -20, 0, 140, 20, false, false);
		makeWall(120, -20, 0, 140, 20, true, false);
		makeWall(-20, 120, 0, 140, 20, false, false);

		makeWall(25, -20, 0, 70, 20, true, false);
		makeWall(75, 120, 0, 70, 20, true, true);
	}
};


class FourRooms
	: public World
{
public:
	FourRooms()
		: World()
	{
		this->x0[0] = 0;
		this->x0[1] = 0;
		this->x1[0] = 0;
		this->x1[1] = 100;
	}

protected:
	virtual void _buildEnvironment() {
		makeWall(-20, -20, 0, 140, 20, true, false);
		makeWall(-20, -20, 0, 140, 20, false, false);
		makeWall(120, -20, 0, 140, 20, true, false);
		makeWall(-20, 120, 0, 140, 20, false, false);

		makeWall(50, -20, 0, 50, 20, true, false);
		makeWall(-20, 50, 0, 50, 20, false, false);
		makeWall(120, 50, 0, 50, 20, false, true);
		makeWall(50, 120, 0, 50, 20, true, true);
	}
};


class Cylinders
	: public World
{
public:
	Cylinders()
		: World()
	{}

protected:
	virtual void _buildEnvironment() {
		int cylinder_id = 0;
		CAL_CreateCylinder(this->obstacle_group, 5, 10, 25, 25, 5, &cylinder_id);
		CAL_SetObjectOrientation(cylinder_id, 1.57, 0, 0);

		CAL_CreateCylinder(this->obstacle_group, 5, 10, 50, 25, 5, &cylinder_id);
		CAL_SetObjectOrientation(cylinder_id, 1.57, 0, 0);

		CAL_CreateCylinder(this->obstacle_group, 5, 10, 75, 25, 5, &cylinder_id);
		CAL_SetObjectOrientation(cylinder_id, 1.57, 0, 0);

		CAL_CreateCylinder(this->obstacle_group, 5, 10, 25, 50, 5, &cylinder_id);
		CAL_SetObjectOrientation(cylinder_id, 1.57, 0, 0);

		CAL_CreateCylinder(this->obstacle_group, 5, 10, 25, 75, 5, &cylinder_id);
		CAL_SetObjectOrientation(cylinder_id, 1.57, 0, 0);

		CAL_CreateCylinder(this->obstacle_group, 5, 10, 75, 75, 5, &cylinder_id);
		CAL_SetObjectOrientation(cylinder_id, 1.57, 0, 0);
	}
};


class TwoWalls
	: public World
{
public:
	BOUNDS x_bounds_window_1;
	BOUNDS x_bounds_window_2;

	TwoWalls()
		: World()
	{
		this->x_bounds[0] = std::make_pair(-4, 4);
		this->x_bounds[1] = std::make_pair(-4, 4);
		this->x_bounds[2] = std::make_pair(0, 5);

		this->x0[0] = 2.5;
		this->x0[1] = -1;
		this->x0[2] = 4;
		this->x1[0] = -2.5;
		this->x1[1] = 1;
		this->x1[2] = 1;
	}

	virtual void positionCamera() {
	double eye_x = 0.0, eye_y = 0.0, eye_z = 0.0;
	double camera_x = 0.0, camera_y = 0.0, camera_z = 0.0;
	double up_x = 1.0, up_y = 0.0, up_z = 0.0;

		up_x = 0.0;
		up_z = 1.0;


//* isometric
	eye_x = -x_bounds[0].second;
	eye_y = 2*x_bounds[1].second;
	eye_z = 2*x_bounds[2].second;
	camera_x = x_bounds[0].second/2.0;
	camera_y = x_bounds[1].first/2.0;


//* side
//	eye_y = 3*x_bounds[1].second;
//	eye_z = x_bounds[2].second/2.0;
//	camera_z = x_bounds[2].second/2.0;


//* above 
//		eye_z = 3*x_bounds[2].second;
//		up_y = 1;
//		up_x = 0;

		CAL_SetViewParams(0, eye_x, eye_y, eye_z, camera_x, camera_y, camera_z, up_x, up_y, up_z);
	}

protected:
	virtual void _buildEnvironment() {
		CAL_CreateGroup(&border_group, 0, false, "Borders");
		CAL_SetGroupColor(border_group, 0.25, 0.25, 0.25);

		int nl = 1;
		int np[1] = {2};
		float left_x = this->x_bounds[0].first;
		float right_x = this->x_bounds[0].second;
		float back_y = this->x_bounds[1].first;
		float front_y = this->x_bounds[1].second;
		float bottom_z = this->x_bounds[2].first;
		float top_z = this->x_bounds[2].second;
		float p[6];

#define MAKE_LINE(x1, y1, z1, x2, y2, z2) \
	p[0] = x1; \
	p[1] = y1; \
	p[2] = z1; \
	p[3] = x2; \
	p[4] = y2; \
	p[5] = z2; \
	CAL_CreatePolyline(this->border_group, 1, np, p);

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
		Eigen::Matrix<double,6,1> bottom_segment, top_segment, left_segment, right_segment, window;
		bottom_segment = Eigen::Matrix<double,6,1>::Zero();
		top_segment= Eigen::Matrix<double,6,1>::Zero();
		left_segment = Eigen::Matrix<double,6,1>::Zero();
		right_segment = Eigen::Matrix<double,6,1>::Zero();
		window = Eigen::Matrix<double,6,1>::Zero();

		// Get environment stats
		double total_height = this->x_bounds[0].second - this->x_bounds[0].first;
		double total_depth = this->x_bounds[1].second - this->x_bounds[1].first;
		double window_height = 1.0, window_width = 2.0;

#define BUILD_SEGMENTS() \
	bottom_segment[3] = 0.1; \
	bottom_segment[4] = total_depth; \
	bottom_segment[5] = window[2] - window[5]/2.0 - this->x_bounds[2].first; \
	bottom_segment[0] = window[0]; \
	bottom_segment[2] = bottom_segment[5]/2.0; \
	\
	top_segment[3] = bottom_segment[3]; \
	top_segment[4] = total_depth; \
	top_segment[5] = this->x_bounds[2].second - (window[2] + window[5]/2.0); \
	top_segment[0] = window[0]; \
	top_segment[2] = top_segment[5]/2.0 + window[2] + window[5]/2.0; \
	\
	right_segment[3] = bottom_segment[3]; \
	right_segment[4] = abs(this->x_bounds[1].second - (window[1] + window[4]/2.0)); \
	right_segment[5] = window[5]; \
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
	CAL_CreateBox(this->obstacle_group, bottom_segment[3], bottom_segment[4], bottom_segment[5], bottom_segment[0], bottom_segment[1], bottom_segment[2]); \
	CAL_CreateBox(this->obstacle_group, top_segment[3], top_segment[4], top_segment[5], top_segment[0], top_segment[1], top_segment[2]); \
	CAL_CreateBox(this->obstacle_group, left_segment[3], left_segment[4], left_segment[5], left_segment[0], left_segment[1], left_segment[2]); \
	CAL_CreateBox(this->obstacle_group, right_segment[3], right_segment[4], right_segment[5], right_segment[0], right_segment[1], right_segment[2]);

#define CALCULATE_WINDOW_AREA(window, bounds) \
	bounds.resize(X_DIM); \
	bounds[0] = std::make_pair(window[0] - window[3]/2.0, window[0] + window[3]/2.0); \
	bounds[1] = std::make_pair(window[1] - window[4]/2.0, window[1] + window[4]/2.0); \
	bounds[2] = std::make_pair(window[2] - window[5]/2.0, window[2] + window[5]/2.0); \
	for (int CALCULATE_WINDOW_AREA_i = 3; CALCULATE_WINDOW_AREA_i < X_DIM; CALCULATE_WINDOW_AREA_i++) { \
	  bounds[CALCULATE_WINDOW_AREA_i] = x_bounds[CALCULATE_WINDOW_AREA_i]; \
	} //
	//CAL_CreateBox(obstacle_group, window[3], window[4], window[5], window[0], window[1], window[2]);

		// Position window
		window[0] = 1.0; // x pos
		window[1] = 1.0; // y pos
		window[2] = 1.0; // z pos
		window[3] = 1.0; // width - this is really controlling the width of the sample area
		window[4] = 1.0; // depth
		window[5] = 0.5; // height

		CALCULATE_WINDOW_AREA(window, this->x_bounds_window_1);

		BUILD_SEGMENTS();

		// Position window
		window[0] = -1.0; // x pos
		window[1] = -1.0; // y pos
		window[2] = 4.0; // z pos

		CALCULATE_WINDOW_AREA(window, this->x_bounds_window_2);

		BUILD_SEGMENTS();
	}
};