#include <cmath>

#ifndef __SIMULATION_HPP__
#define __SIMULATION_HPP__

template<class T>
void rand_gaussian_vector(T& v) {
	size_t v_size = v.size();
	double u1, u2, r, t;
	for (int idx = 0; idx < v_size; idx += 2) {
		u1 = rand_value(0, 1);
		u2 = rand_value(0, 1);
		r = sqrt(-2*log(u1));
		t = 2*M_PI*u2;
		v[idx] = r*cos(t);
		v[idx+1] = r*sin(t);
	}
	if (v_size % 2 != 0) {
		u1 = rand_value(0, 1);
		u2 = rand_value(0, 1);
		r = sqrt(-2*log(u1));
		t = 2*M_PI*u2;
		v[v_size - 1] = r*cos(t);
	}
}

/*
class Simulator {
private:
	const dynamics_t &dynamics;
	state deviation;
	state actual;
	motion_noise_covariance_t L_of_M;
	motion_noise_t motion_noise;
	observation_noise_covariance_t L_of_N;
	observation_noise_t observation_noise;
	bool noise_free;
	bool visualize_actual;
	Robot * robot;

public:
	Simulator(const dynamics_t &dynamics, state &x0, bool noise_free = false, bool visualize_actual = VISUALIZE_SIMULATION, Robot * robot = NULL)
		: dynamics(dynamics), deviation(x0), noise_free(noise_free), visualize_actual(visualize_actual), robot(robot)
	{
		if (!(this->noise_free)) {
			Eigen::LLT<natural_dynamics_t> LLT_of_M(dynamics.MotionNoiseCovariance);
			if (LLT_of_M.info() == Eigen::Success) {
				this->L_of_M = LLT_of_M.matrixL();
			} else {
				std::cout << "Motion Noise Covariance is not positive semi-definite. Cannot perform a Cholesky decompotions, try an Eigen decomposition instead." << std::endl;
				_getchar();
				exit(-1);
			}

			Eigen::LLT<observation_noise_covariance_t> LLT_of_N(dynamics.ObservationNoiseCovariance);
			if (LLT_of_N.info() == Eigen::Success) {
				this->L_of_N = LLT_of_N.matrixL();
			} else {
				std::cout << "Observation Noise Covariance is not positive semi-definite. Cannot perform a Cholesky decompotions, try an Eigen decomposition instead." << std::endl;
				_getchar();
				exit(-1);
			}
		}
	}

	void step(const state &state_belief, const control &u) {
		//std::cout << "Step? ";
		//_getchar();

		// Update deviation
		this->deviation = ((dynamics.A)*deviation - (dynamics.B)*(dynamics.L)*u)*deltaT;

		// Ad noise
		if (!(this->noise_free)) {
			rand_gaussian_vector(this->motion_noise);
			this->motion_noise = this->L_of_M*this->motion_noise;
			this->deviation += motion_noise*sqrt(deltaT);
		}
	}

	void observe(state * s) {
		// Calculate the observation
		(*s) = (dynamics.C)*(this->deviation);

		// Add noise
		if (!(this->noise_free)) {
			rand_gaussian_vector(this->observation_noise);
			this->observation_noise = this->L_of_N*this->observation_noise;
			(*s) += observation_noise;
		}
	}

	bool collisionFree(const state &state_nominal) {
		this->actual = this->deviation + state_nominal;
		return collision_free(this->actual, false, false);
	}

	void visualizeActual(const state &state_nominal) {
		this->actual = this->deviation + state_nominal;

		std::cout << "Actual: " << this->actual << std::endl;

		double x_pos = this->actual[0];
		double y_pos = this->actual[1];
#if POSITION_DIM == 3
		double z_pos = this->actual[2];
#else
		double z_pos = 0;
#endif

		vis::markActual(x_pos, y_pos, z_pos);

		if (this->robot != NULL) {
			robot->position(this->actual, true);
		}
	}
};

void visualizeBelief(const state & state_nominal, const state &state_belief) {
	state belief = state_nominal + state_belief;

	std::cout << "Belief: " << state_belief << std::endl;

	double x_pos = belief[0];
	double y_pos = belief[1];
#if POSITION_DIM == 3
	double z_pos = belief[2];
#else
	double z_pos = 0;
#endif

	vis::markBelief(x_pos, y_pos, z_pos);
}
*/
bool simulate(const bool stop_on_collision, const dynamics_t &dynamics, tree_t &tree, nominal_trajectory_t &traj, collision_steps_t &collision_steps, bool visualize_simulation = VISUALIZE_SIMULATION, Robot * robot = NULL) {
	double sqrt_deltaT = sqrt(deltaT);

	double x_pos = 0, y_pos = 0, z_pos = 0;
	state actual = state::Zero();
	state belief = state::Zero();
	double_state_t y = double_state_t::Zero();
	double_noise_t q = double_noise_t::Zero();

	/*
	state state_belief = state::Zero();
	state observation = state::Zero();
	control u = control::Zero();
	Simulator sim(dynamics, state_belief, NOISE_FREE, visualize_simulation, robot);\
	*/
	bool free_path = true;
	for (int current_step = 0; current_step < traj.path.size() - 1; ++current_step) {
		/*
		// Apply LQR control
		u = -(dynamics.L)*(state_belief);
		sim.step(state_belief, u);


		// Observe state
		sim.observe(&observation);

		// Estimate state -- Apply Kalman filter
		state_belief = state_belief + (((dynamics.A) - (dynamics.B)*(dynamics.L))*state_belief + (dynamics.K)*(observation - (dynamics.C)*state_belief))*deltaT;

		// Visualize
		if (visualize_simulation) {
			visualizeBelief(path[current_step].second, state_belief);
			sim.visualizeActual(path[current_step].second);
		}

		// Check for collision
		if (!sim.collisionFree(path[current_step].second)) {
			free_path = false;
			break;
		}
		*/
		y += (dynamics.F)*y*deltaT;
		if (!NOISE_FREE) {
			rand_gaussian_vector(q);
			y += (dynamics.G)*q*sqrt_deltaT;
		}

		actual = (dynamics.X)*y + traj.path[current_step].second;
		if (!collision_free(actual, false, false)) {
			free_path = false;
			++collision_steps[current_step];
			if (stop_on_collision) {
				break;
			}
		} else {
			++collision_steps[current_step];
		}

		if (visualize_simulation) {
			//std::cout << "Actual: " << actual << std::endl;

			x_pos = actual[0];
			y_pos = actual[1];
#if POSITION_DIM == 3
			z_pos = actual[2];
#endif

			vis::markActual(x_pos, y_pos, z_pos);

			belief = (dynamics.Xhat)*y + traj.path[current_step].second;

			//std::cout << "Belief: " << belief << std::endl;

			x_pos = belief[0];
			y_pos = belief[1];
			std::cout << "pos: (" << x_pos << "," << y_pos << ")" << std::endl;
#if POSITION_DIM == 3
			z_pos = belief[2];
#endif

			vis::markBelief(x_pos, y_pos, z_pos);

			if (robot != NULL) {
				robot->position(actual, false);
			}

			Sleep(deltaT*50);
		}
	}

	return free_path;
}

#endif // __SIMULATION_HPP__