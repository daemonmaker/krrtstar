import numpy as np

import matplotlib.pyplot as plt

import keras
import keras.backend as KK
from keras.models import load_model, Model
from keras.layers import Input, Lambda
from keras.layers.merge import Concatenate
from keras.losses import mean_squared_error as mse

from dynamics import Dynamics
from dynamics.pendulum import *


class NoNeighborException(Exception):
    pass


class DistanceMeasure(object):
    def __init__(self):
        print("Init DistanceMeasure")
        super(DistanceMeasure, self).__init__()


    def measure(self, x0, x1):
        raise NotImplementedError


class EuclideanDistance(DistanceMeasure):
    def __init__(self):
        super(EuclideanDistance, self).__init__()


    def measure(self, x0, x1):
        #return np.sum(np.square(x0 - x1))
        #print("Squared diff shape:", np.square(x0 - x1).shape)
        #print("Squared diff:", np.square(x0 - x1))
        return np.sum(np.square(x0 - x1), axis=1)


class ForwardDynamicsAndRewardDNN(Dynamics, DistanceMeasure):
    def __init__(
            self, x_dim, u_dim, r_dim, model_path=None, model=None
    ):
        print("Init ForwardDynamicsAndRewardDNN")
        super(ForwardDynamicsAndRewardDNN, self).__init__(
            x_dim, u_dim, r_dim
        )

        if model_path is not None:
            self.mdl = load_model(
                model_path,
                custom_objects={'atan2_loss': atan2_loss, 'cos': KK.cos}
            )

        if model is not None:
            self.mdl = model

        x0 = KK.placeholder(shape=(None, self.x_dim), name='x0')
        u = KK.placeholder(shape=(None, self.u_dim), name='u')
        x1, cost = self.mdl([x0, u])
        samp_symb = KK.placeholder(
            shape=(1, self.x_dim),
            name='samp_syb'
        )
        loss = KK.expand_dims(mse(samp_symb, x1), axis=1)
        u_grads = KK.gradients([loss], [u])

        self.meas_fn = KK.function(
            [x0, u, samp_symb],
            [x1, loss] + u_grads
        )

        self.zero_control = None


    def measure(self, x0, x1, u=None):
        if self.zero_control is None:
            self.zero_control = np.zeros((x0.shape[0], self.u_dim))

        if u is None:
            u = self.zero_control

        if x1.shape[0] == 2:
            import ipdb; ipdb.set_trace()
        x1_actual, loss, u_grads = self.meas_fn([x0, u, x1])

        return x1_actual, loss, u_grads


class Tree(object):
    NO_IDX = -1
    GOAL_IDX = 0
    START_IDX = 1

    def __init__(
            self, N, x0, xd, goal_generator_fn, goal_check_fn, distance_measure=None, dyns=None
    ):
        self.N = N

        self.next_node_idx = 0 # Index to the next new node

        self._x_dim = x0.shape[1]

        self._states = np.zeros((N+2, self._x_dim)) # Array of states
        self._controls = np.zeros((N+2, 1)) # Array of states
        self._losses = np.zeros((N+2, 1)) # Array of states

        # Array relating children and  parent nodes. Array position is
        # child node idx and value at that position is parent node idx
        # (instates matrix)
        self._parents = (self.NO_IDX**np.ones((N+2, 1))).astype(int)

        self._children = {} # Linked-list relating node indices

        self._costs_from_start = np.zeros((N, 1)) # Cost to reach
        # indexed node from the start node

        self.add_node(xd, self.NO_IDX, 0, 0)
        self.add_node(x0, self.NO_IDX, 0, 0)

        self.goal_generator_fn = goal_generator_fn
        self.goal_check_fn = goal_check_fn

        if distance_measure is None:
            self.dist = EuclideanDistance()
        else:
            self.dist = distance_measure

        self.dyns = dyns


    def done(self):
        return self.next_node_idx == self.N+2


    def get_goal(self):
        #return np.expand_dims(self._states[self.GOAL_IDX], axis=0)
        return self.goal_generator_fn()


    def check_goal(self, x):
        return self.goal_check_fn(x)


    def samples(self):
        return self.next_node_idx-2


    def add_node(self, x, parent_idx, control=None, loss=None):
        if parent_idx == 0:
            import ipdb; ipdb.set_trace()
        idx = self.next_node_idx
        print("x: ", x, "\tparent (", parent_idx , "): ", self._states[parent_idx], "\tcontrol: ", control, "\tloss: ", loss)
        self._states[idx, :] = x
        self._parents[idx] = parent_idx
        if control is not None:
            self._controls[idx] = control
        if loss is not None:
            self._losses[idx] = loss

        # Start children list
        test = self._children.get(idx)
        assert(test is None)
        self._children[idx] = []

        # Add to parents child list
        if parent_idx != self.NO_IDX:
            test = self._children.get(parent_idx)
            assert(test is not None)
            test.append(idx)

        # TODO add code to maintain self.costs_from_start

        self.next_node_idx += 1

        return idx


    def get_children_idxs(self, parent_idx):
        return self._children[parent_idx]


    def find_nearest(self, x):
        states = self._states[self.START_IDX:self.next_node_idx]
        dists = self.dist.measure(x, states)
        nearest_idx = np.argmin(dists)
        if isinstance(nearest_idx, list):
            nearest_idx = nearest_idx[0]
        nearest_dist = dists[nearest_idx]
        nearest_idx += 1 # Correct for goal idx
        return (nearest_idx, nearest_dist)


    def find_controls(self, x):
        states = self._states[self.START_IDX:self.next_node_idx]
        u_grads_set = np.zeros((1000, states.shape[0], 1))
        u = np.zeros((states.shape[0], self.dyns.u_dim))
        prev_u = np.zeros((states.shape[0], self.dyns.u_dim))
        prev_u_delta = np.zeros((states.shape[0], self.dyns.u_dim))

        for idx in np.arange(1000):
            x1, loss, u_grads = self.dist.measure(states, x, u)
            u_grads_set[idx] = u_grads
            prev_u = u
            u -= u_grads
            if np.all(np.abs(u_grads) < 0.000001):
                break

        return x1, u, loss


    def visualize(self):
        for idx in np.arange(self.N+2):
            parent_idx = self._parents[idx][0]
            if parent_idx != self.NO_IDX:
                x0 = self._states[parent_idx]
                x1 = self._states[idx]
                plt.plot([x0[0], x1[0]], [x0[1], x1[1]], 'k')
        #import ipdb; ipdb.set_trace()
        plt.scatter(self._states[2:, 0], self._states[2:,1])
        plt.scatter(self._states[:2, 0], self._states[:2,1], color='r')
        plt.show()


class UniformSampler(object):
    def __init__(self, bounds):
        assert(bounds, np.matrixlib.defmatrix.matrix)
        self.bounds = bounds

        self.diffs = bounds[1] - bounds[0]
        assert(np.sum(self.diffs <= 0) == 0) # Ensure all bounds
        # encapsulate a positive area.

        self.means = bounds[0] + self.diffs/2.


    def sample(self):
        x_rand = np.asmatrix(np.random.rand(self.bounds.shape[0])-0.5)
        return np.multiply(
            x_rand,
            self.diffs
        ) + self.means


class DummyCollisionChecker(object):
    def __init__(self, obstacles):
        pass


    def check(self, x):
        return False


def rrt(N, tree, samp, cc, r = 1):
    continuing = True
    while continuing:
        # Sample
        checking_goal = np.random.rand() > 0.9
        if checking_goal:
            xi = tree.get_goal()
            print("shape: ", xi.shape)
        else:
            in_collision = True
            while in_collision:
                xi = samp.sample()

                # Collision check sample
                in_collision = cc.check(xi)

        print("Sampling... {}/{}\t{}\r".format(tree.samples(), N, xi),)

        # Find nearest vertex
        nearest_idx, nearest_dist = tree.find_nearest(xi)

        # TODO for now assume straight line connection and no obstacles
        # Connect
        #tree.connect(nearest_idx, 
        # Collision check trajectory

        #print("xi: ",xi,"nearest_dist: ",nearest_dist)
        if nearest_dist < r:
            tree.add_node(xi, nearest_idx)
            if checking_goal:
                break

        if tree.samples() >= N or tree.done():
            continuing = False
        #if tree.samples() >= N:
        #    print("tree.samples(): ",tree.samples())
        #    continuing = False


def drrt(N, tree, dyns, samp, cc, u_bounds = np.array([-1, 1])):
    continuing = True
    while continuing:
        # Sample
        checking_goal = np.random.rand() > 0.95
        if checking_goal:
            print("Checking goal...")
            xi = tree.get_goal()
        else:
            in_collision = True
            while in_collision:
                xi = samp.sample()

                # Collision check sample
                in_collision = cc.check(xi)

        rand_idx = np.random.randint(1, tree.next_node_idx)
        x_rand = tree._states[rand_idx]
        x_rand = tree._states[tree.next_node_idx-1]
        x_rand = np.expand_dims(x_rand, axis=0)
        u_rand = 4.*np.asmatrix(np.random.rand(1,1))-2.
        u_rand = np.asmatrix(-1.)
        res = dyns.mdl.predict([x_rand, u_rand])
        xi = res[0]
        print("xi: ", xi, "\tu_rand: ", u_rand)

        print("Sampling... {}/{}\t{}\r".format(tree.samples(), N, xi),)

        # Find nearest vertex
        #nearest_idx, nearest_dist = tree.find_nearest(xi)
        x1es, controls, losses = tree.find_controls(xi)

        # TODO for now assume straight line connection and no obstacles
        # Connect
        #tree.connect(nearest_idx, 
        # Collision check trajectory

        #print("xi: ",xi,"nearest_dist: ",nearest_dist)
        bottom_bounded = controls >= u_bounds[0]
        top_bounded = controls <= u_bounds[1]
        bounded = bottom_bounded & top_bounded

        if np.any(bounded):
            #parent_idxs = [i+1 for i, x in enumerate(bounded) if x]
            parent_idxs = [i+1 for i in range(bounded.shape[0])]
            bounded_controls = controls[bounded]
            bounded_losses = losses[bounded]

            nearest_idx = np.argmin(bounded_losses)

            x1 = np.expand_dims(x1es[nearest_idx], axis=0)
            control = bounded_controls[nearest_idx]
            loss = bounded_losses[nearest_idx]

            parent_idx = parent_idxs[nearest_idx]

            # Determine idxs of states with bounded controls, i.e. reachable states
            # Determine idx of state with lowest cost?
            tree.add_node(x1, parent_idx, control, loss)
            if checking_goal and tree.check_goal(x1):
                break

        if tree.samples() >= N or tree.done():
            print("Done constructing tree.")
            continuing = False
        #if tree.samples() >= N:
        #    print("tree.samples(): ",tree.samples())
        #    continuing = False


#def krrtstar(N, x0, xd, samp):
#    for idx in np.arange(N):
#        xi = samp.sample()

if __name__ == '__main__':
    keras.activations.cos = KK.cos

    N = 10

    x_dim = 2

    x0 = np.matrix([np.pi, 0], dtype='float32')
    xd = np.matrix([[0, 0]], dtype='float32')

    bounds = np.matrix([[-4*np.pi, -10], [4*np.pi, 10]])
    sampler = UniformSampler(bounds)

    cc = DummyCollisionChecker(None)

    dyns = None
    dyns = Pendulum(
        x_dim=2, u_dim=1, r_dim=1
    )
    #dyns.compare_to_gym()
    dyns = ForwardDynamicsAndRewardDNN(
        x_dim=2, u_dim=1, r_dim=1,
        #model_path = 'models/pendulum_trajectories_1000_500_f_r.h5'
        model = dyns.mdl
    )

    temp = np.zeros((1, 2))
    for idx in range(-10, 11):
        temp[0, 0] = idx*np.pi
        print("Checking (", idx, ")", temp, " ", pendulum_goal_checker(temp))



    tree = Tree(
        N, x0, xd,
        pendulum_goal_generator, pendulum_goal_checker,
        dyns, dyns
    )
    tree.visualize()

    #rrt(N, tree, sampler, cc)
    drrt(N, tree, dyns, sampler, cc, u_bounds = np.array([-2, 2]))

    tree.visualize()

    import ipdb; ipdb.set_trace()
