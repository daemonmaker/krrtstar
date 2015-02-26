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

public:
	Simulator(const dynamics_t &dynamics, state &x0, bool noise_free = false, bool visualize_actual = VISUALIZE_SIMULATION)
		: dynamics(dynamics), deviation(x0), noise_free(noise_free), visualize_actual(visualize_actual)
	{
		if (!(this->noise_free)) {
			Eigen::LLT<natural_dynamics_t> LLT_of_M(*(dynamics.MotionNoiseCovariance));
			if (LLT_of_M.info() == Eigen::Success) {
				this->L_of_M = LLT_of_M.matrixL();
			} else {
				std::cout << "Motion Noise Covariance is not positive semi-definite. Cannot perform a Cholesky decompotions, try an Eigen decomposition instead." << std::endl;
				_getchar();
				exit(-1);
			}

			Eigen::LLT<observation_noise_covariance_t> LLT_of_N(*(dynamics.ObservationNoiseCovariance));
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
		this->deviation = ((*(dynamics.A))*deviation - (*(dynamics.B))*(*(dynamics.L))*u)*deltaT;

		// Ad noise
		if (!(this->noise_free)) {
			rand_gaussian_vector(this->motion_noise);
			this->motion_noise = this->L_of_M*this->motion_noise;
			this->deviation += motion_noise*sqrt(deltaT);
		}
	}

	void observe(state * s) {
		// Calculate the observation
		(*s) = (*(dynamics.C))*(this->deviation);

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

bool simulate(const dynamics_t &dynamics, tree_t &tree, bool visualize_simulation = VISUALIZE_SIMULATION) {
	// Simulate the trajectory
	state_time_list_t path;
	control_time_list_t controls;
	createNominalTrajectory(tree, &path, &controls);

	state observation = state::Zero();

	double x_pos = 0, y_pos = 0, z_pos = 0;
	state state_belief = state::Zero();
	state state_update = state::Zero();
	control u = control::Zero();
	Simulator sim(dynamics, state_belief, NOISE_FREE, visualize_simulation);
	bool free_path = true;
	for (int current_step = 0; current_step < path.size() - 1; ++current_step) {
		// Apply LQR control
		u = -(*(dynamics.L))*(state_belief);
		sim.step(state_belief, u);

		// Observe state
		sim.observe(&observation);

		// Estimate state -- Apply Kalman filter
		state_belief = state_belief + (((*(dynamics.A)) - (*(dynamics.B))*(*(dynamics.L)))*state_belief + (*(dynamics.K))*(observation - (*(dynamics.C))*state_belief))*deltaT;

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

	}

	return free_path;
}

#endif // __SIMULATION_HPP__