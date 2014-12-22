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
public:
	World(const BOUNDS& x_bounds)
		: x_bounds(x_bounds)
	{
		//this->x_bounds = BOUNDS();
		this->x_bounds.resize(X_DIM);
	}

	// TODO Remove this! KDtree needs the bounds
	const BOUNDS& getBounds() { return this->x_bounds; }

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

	virtual void setCamera() {
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
	BOUNDS x_bounds;

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

private:
	int obstacle_group;
};

class EmptyWorld
	: public World
{
public:
	EmptyWorld(const BOUNDS& x_bounds)
		: World(x_bounds)
	{
		//this->x_bounds[0] = std::make_pair(0, 100);
		//this->x_bounds[1] = std::make_pair(-10, 10);
	}

protected:
};

class TwoPathMaze
	: public World
{
public:
	TwoPathMaze(const BOUNDS& x_bounds)
		: World(x_bounds)
	{
		//this->x_bounds[0] = std::make_pair(0, 200);
		//this->x_bounds[1] = std::make_pair(0, 100);
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