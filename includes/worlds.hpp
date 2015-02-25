#include "state.hpp"
#include "robots.hpp"

#include <iostream>
#include <Eigen/Dense>

#ifndef __WORLDS_HPP__
#define __WORLDS_HPP__

/*
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
*/
class World {
protected:
	int base_group, border_group, obstacle_group;
	BOUNDS x_bounds;
	Eigen::Matrix<double,X_DIM,1> x0, x1;
	double distance_threshold;

public:
	World(int base_group)
		: base_group(base_group), distance_threshold(0.0)
	{
		CAL_CreateGroup(&(this->obstacle_group), base_group, true, "Obstacle");

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

	void setDistanceThreshold(double distance_threshold) {
		this->distance_threshold = distance_threshold;
	}

	double getDistanceThreshold() const {
		return this->distance_threshold;
	}

	void setBound(int idx, const BOUND& bound) {
		this->x_bounds[idx] = bound;
	}

	const BOUNDS getBounds() {
		return this->x_bounds;
	}

	const Eigen::Matrix<double,X_DIM,1> getStartState() const {
		return x0;
	}

	const Eigen::Matrix<double,X_DIM,1> getFinalState() const {
		return x1;
	}

	const void show_obstacles() const {
		CAL_SetGroupVisibility(this->obstacle_group, 0, true, true);
	}

	const void hide_obstacles() const {
		CAL_SetGroupVisibility(this->obstacle_group, 0, false, true);
	}

	const void set_obstacle_color(float r, float g, float b, float a=1) {
		CAL_SetGroupColor(obstacle_group, r, g, b, a);
	}

	virtual void buildEnvironment() {
		//CAL_CreateGroup(&(this->obstacle_group), this->base_group, true, "Obstacle");

#if (USE_OBSTACLES > 0)
		CAL_SetGroupVisibility(this->obstacle_group, 0, true, true);
#else
		CAL_SetGroupVisibility(this->obstacle_group, 0, false, true);
#endif

#if defined(MAKE_STILLS) && (DYNAMICS != QUADROTOR)
		CAL_SetGroupColor(this->obstacle_group, 0.1, 0.1, 0.1, 1);
#else
		CAL_SetGroupColor(this->obstacle_group, 0.1, 0.1, 0.1, 0.1);
#endif

		this->_buildEnvironment();
	}

	virtual void positionCamera(const Eigen::Matrix<double,3,3>& R = Eigen::Matrix<double,3,3>::Identity(), const Eigen::Matrix<double,3,3>& S = Eigen::Matrix<double,3,3>::Identity()) {
		Eigen::Matrix<double,3,1> eye = Eigen::Matrix<double,3,1>::Zero();
		Eigen::Matrix<double,3,1> camera = Eigen::Matrix<double,3,1>::Zero();
		Eigen::Matrix<double,3,1> up = Eigen::Matrix<double,3,1>::Zero();

		camera[0] = eye[0] = (this->x_bounds[0].second - this->x_bounds[0].first)/2.0;
		camera[1] = eye[1] = (this->x_bounds[1].second - this->x_bounds[1].first)/2.0;
		eye[2] = 150;

		up[1] = 1;
		
		CAL_SetViewParams(0, eye[0], eye[1], eye[2], camera[0], camera[1], camera[2], up[0], up[1], up[2]);

		//CAL_ShowView(1);

		camera = S*R*camera;
		eye = S*R*eye;
		up = S*R*up;
		
		CAL_SetViewParams(1, eye[0], eye[1], eye[2], camera[0], camera[1], camera[2], up[0], up[1], up[2]);
	}

	virtual void randPosition(state& s) {
		this->_randPosition(s, this->x_bounds);
	}

	virtual void test() {
	}

/*
	virtual int checkCollisions(int robot_group, int * collisions) {
		return CAL_CheckGroupCollision(robot_group, this->obstacle_group, false, collisions);
	}
*/
	inline const virtual bool checkDistance(Robot * robot, bool visualize = false) const {
		if (robot->computeStdDev(this->obstacle_group, this->distance_threshold, visualize) < this->distance_threshold)
			return true;
		return false;
	}

	virtual int checkCollisions(Robot * robot, int * collisions, bool visualize = false) {
		int result = robot->checkCollision(this->obstacle_group, collisions, visualize);
		if (result != CAL_SUCCESS) {
			std::cout << "CAL_CheckGroupCollision failed (" << result << ")." << std::endl;
			_getchar();
			exit(1);
		}
		return result;
	}

	template<typename vec>
	bool validateState(const vec& v) {
		return checkBounds(v, this->x_bounds);
	}

	void showCollisionCheck(const state& s, bool collision, int collision_hit_group, int collision_free_group) {
		double x_pos = s[0];
		double y_pos = s[1];
#if POSITION_DIM == 3
		double z_pos = s[2];
#else
		double z_pos = 0;
#endif
		if (collision) {
			CAL_CreateSphere(collision_hit_group, 3*NODE_SIZE, x_pos, y_pos, z_pos);
		} else {
			CAL_CreateSphere(collision_free_group, 3*NODE_SIZE, x_pos, y_pos, z_pos);
		}
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

	void inline _randPosition(state& s, const BOUNDS& b) {
		int i = 0;
		s[i] = rand_value(b[i].first, b[i].second);
		++i;
		s[i] = rand_value(b[i].first, b[i].second);
#if POSITION_DIM == 3
		++i;
		s[i] = rand_value(b[i].first, b[i].second);
#endif
	}
};

class EmptyWorld
	: public World
{
public:
	EmptyWorld(int base_group)
		: World(base_group)
	{}

protected:
};

class TwoPathMaze
	: public World
{
public:
	TwoPathMaze(int base_group)
		: World(base_group)
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
	SymmetricRaceTrackMaze(int base_group)
		: World(base_group)
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
		CAL_CreateGroup(&temp, this->base_group, false, "Faux hole");
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
	SimpleRaceTrack(int base_group)
		: World(base_group)
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
	HardSMaze(int base_group)
		: World(base_group)
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
	EasySMaze(int base_group)
		: World(base_group)
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
	FourRooms(int base_group)
		: World(base_group)
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
	Cylinders(int base_group)
		: World(base_group)
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

namespace worlds {
class LudersBoxes
	: public World
{
private:
	float scaler;

public:
	LudersBoxes(int base_group)
		: World(base_group), scaler(1.5)
	{
		this->x0[0] = 0;
		this->x0[1] = -35*(this->scaler);
		this->x1[0] = 0;
		this->x1[1] = 37.5*(this->scaler);

		this->x_bounds.resize(X_DIM);
		for (BOUNDS::iterator p = this->x_bounds.begin(); p != this->x_bounds.end(); ++p) {
			p->first = 0.0;
			p->second = 0.0;
		}

		this->x_bounds[0] = std::make_pair(-50*(this->scaler), 50*(this->scaler));
		this->x_bounds[1] = std::make_pair(-50*(this->scaler), 50*(this->scaler));
	}

protected:
	virtual void _buildEnvironment() {
		int box_id = 0;

		// Make outer walls
		CAL_CreateBox(this->obstacle_group, 100*(this->scaler), 5*(this->scaler), 10*(this->scaler), 0*(this->scaler), -52.5*(this->scaler), 0*(this->scaler)); // Bottom
		CAL_CreateBox(this->obstacle_group, 5*(this->scaler), 100*(this->scaler), 10*(this->scaler), -52.5*(this->scaler), 0*(this->scaler), 0*(this->scaler)); // Left
		CAL_CreateBox(this->obstacle_group, 5*(this->scaler), 100*(this->scaler), 10*(this->scaler), 52.5*(this->scaler), 0*(this->scaler), 0*(this->scaler)); // Right
		CAL_CreateBox(this->obstacle_group, 100*(this->scaler), 5*(this->scaler), 10*(this->scaler), 0*(this->scaler), 52.5*(this->scaler), 0*(this->scaler)); // Top

		// Make small boxes
		CAL_CreateBox(this->obstacle_group, 10*(this->scaler), 10*(this->scaler), 10*(this->scaler), -10*(this->scaler), -15*(this->scaler), 0*(this->scaler)); // Left
		CAL_CreateBox(this->obstacle_group, 10*(this->scaler), 10*(this->scaler), 10*(this->scaler), 10*(this->scaler), -15*(this->scaler), 0*(this->scaler)); // Right

		// Make large boxes
		CAL_CreateBox(this->obstacle_group, 12.5*(this->scaler), 20*(this->scaler), 10*(this->scaler), -15*(this->scaler), 15*(this->scaler), 0*(this->scaler)); // Left
		CAL_CreateBox(this->obstacle_group, 12.5*(this->scaler), 20*(this->scaler), 10*(this->scaler), 15*(this->scaler), 15*(this->scaler), 0*(this->scaler)); // Right
	}

	virtual void positionCamera() {
		CAL_SetViewParams(0, 0, 0, 101*(this->scaler), 0, 0, 0, 0, 1, 0);
	}
};

class VanDenBergPassages
	: public World
{
private:
	float scaler;

public:
	VanDenBergPassages(int base_group)
		: World(base_group), scaler(2)
	{
		this->x0[0] = 7.5*(this->scaler);
		this->x0[1] = 7.5*(this->scaler);
		this->x1[0] = 75*(this->scaler);
		this->x1[1] = 75*(this->scaler);

		this->x_bounds.resize(X_DIM);
		for (BOUNDS::iterator p = this->x_bounds.begin(); p != this->x_bounds.end(); ++p) {
			p->first = 0.0;
			p->second = 0.0;
		}

		this->x_bounds[0] = std::make_pair(0*(this->scaler), 100*(this->scaler));
		this->x_bounds[1] = std::make_pair(0*(this->scaler), 100*(this->scaler));
	}

protected:
	virtual void _buildEnvironment() {
		int box_id = 0;

		// Make outer walls
		CAL_CreateBox(this->obstacle_group, 100*(this->scaler), 5*(this->scaler), 10*(this->scaler), 50*(this->scaler), -2.5*(this->scaler), 0*(this->scaler)); // Bottom
		CAL_CreateBox(this->obstacle_group, 5*(this->scaler), 100*(this->scaler), 10*(this->scaler), -2.5*(this->scaler), 50*(this->scaler), 0*(this->scaler)); // Left
		CAL_CreateBox(this->obstacle_group, 5*(this->scaler), 100*(this->scaler), 10*(this->scaler), 102.5*(this->scaler), 50*(this->scaler), 0*(this->scaler)); // Right
		CAL_CreateBox(this->obstacle_group, 100*(this->scaler), 5*(this->scaler), 10*(this->scaler), 50*(this->scaler), 102.5*(this->scaler), 0*(this->scaler)); // Top

		// Make boxes
		CAL_CreateBox(this->obstacle_group, 2*(this->scaler), 30*(this->scaler), 10*(this->scaler), 1*(this->scaler), 27*(this->scaler), 0*(this->scaler)); // Left
		CAL_CreateBox(this->obstacle_group, 30*(this->scaler), 2*(this->scaler), 10*(this->scaler), 27*(this->scaler), 1*(this->scaler), 0*(this->scaler)); // Right

		// Make large boxes
		CAL_CreateBox(this->obstacle_group, 30*(this->scaler), 30*(this->scaler), 10*(this->scaler), 27.5*(this->scaler), 27.5*(this->scaler), 0*(this->scaler)); // Left
		//CAL_CreateBox(this->obstacle_group, 12.5*(this->scaler), 20*(this->scaler), 10*(this->scaler), 15*(this->scaler), 15*(this->scaler), 0*(this->scaler)); // Right
	}

	virtual void positionCamera() {
		CAL_SetViewParams(0, 50*(this->scaler), 50*(this->scaler), 101*(this->scaler), 50*(this->scaler), 50*(this->scaler), 0, 0, 1, 0);
	}
};
}

class TwoWalls
	: public World
{
public:
	BOUNDS x_bounds_window_1;
	BOUNDS x_bounds_window_2;

	TwoWalls(int base_group)
		: World(base_group)
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
		Eigen::Matrix<double,3,1> eye = Eigen::Matrix<double,3,1>::Zero();
		Eigen::Matrix<double,3,1> camera = Eigen::Matrix<double,3,1>::Zero();
		Eigen::Matrix<double,3,1> up = Eigen::Matrix<double,3,1>::Zero();

		Eigen::Matrix<double,3,1> bounds = Eigen::Matrix<double,3,1>::Zero();
		bounds[0] = x_bounds[0].second;
		bounds[1] = x_bounds[1].second;
		bounds[2] = x_bounds[2].second;

		up[2] = 1;

		//* isometric
		eye[0] = -x_bounds[0].second;
		eye[1] = 2*x_bounds[1].second;
		eye[2] = 2*x_bounds[2].second;
		camera[0] = x_bounds[1].first/2.0;
		camera[1] = x_bounds[1].first/2.0;

		//* side
		//eye[1] = 3*x_bounds[1].second;
		//eye[2] = x_bounds[2].second/2.0;
		//camera[2] = x_bounds[2].second/2.0;

		//* above
		//eye[2] = 3*x_bounds[2].second;
		//up[1] = 1;

		CAL_SetViewParams(0, eye[0], eye[1], eye[2], camera[0], camera[1], camera[2], up[0], up[1], up[2]);
	}

	virtual void randPosition(state& s) {
		double area = rand_value(0, 1);

		BOUNDS *bounds = &(this->x_bounds);
		if (area < 0.1) {
			bounds = &(this->x_bounds_window_1);
		}

		else if (area < 0.2) {
			bounds = &(this->x_bounds_window_2);
		}

		this->_randPosition(s, *bounds);
	}
protected:
	virtual void _buildEnvironment() {
		CAL_CreateGroup(&border_group, this->base_group, false, "Borders");
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

#endif // __WORLDS_HPP__