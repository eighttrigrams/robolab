from immutable.immutable import update, assoc
from runner.render.draw import *
from runner.utils import findByIdentifier
import pybullet
import tensorflow as tf
import numpy as np
import math

c = 0.30
force = 1075
lateral_friction = 5.0

begin = 25

nr_steps_per_game = 8
nr_games_per_update = 12


n_inputs = 4
n_hidden = 4 #
n_outputs = 4 #


PC = pybullet.POSITION_CONTROL
BOX_ID = 'box_id'
CURRENT_STEP_INDEX = 'current_step_index'
CURRENT_GAME_INDEX = 'current_game_index'
ALL_GRADIENTS = 'all_gradients'
ALL_REWARDS = 'all_rewards'
GAME_REWARDS = 'game_rewards'
GAME_GRADIENTS = 'game_gradients'
CURRENT_MOVEMENT_PATTERN = 'current_movement_pattern'
CURRENT_ITERATION_INDEX = 'current_iteration_index'


def make_ops(actions, logits):
    learning_rate = 0.01
    cross_entropy = tf.nn.sigmoid_cross_entropy_with_logits(labels=[actions], logits=logits)
    optimizer = tf.train.AdamOptimizer(learning_rate)
    grads_and_vars = optimizer.compute_gradients(cross_entropy)
    gradients = [grad for grad, variable in grads_and_vars]
    gradient_placeholders = []
    grads_and_vars_feed = []
    for grad, variable in grads_and_vars:
        gradient_placeholder = tf.placeholder(tf.float32, shape=grad.get_shape())
        gradient_placeholders.append(gradient_placeholder)
        grads_and_vars_feed.append((gradient_placeholder, variable))
    training_op = optimizer.apply_gradients(grads_and_vars_feed)
    return gradient_placeholders, grads_and_vars, gradients, training_op


def make_net():
    initializer = tf.contrib.layers.variance_scaling_initializer()

    X = tf.placeholder(tf.float32, shape=[None, n_inputs])
    hidden = tf.layers.dense(X, n_hidden, activation=tf.nn.elu, kernel_initializer=initializer)
    hidden2 = tf.layers.dense(hidden, n_hidden, activation=tf.nn.elu, kernel_initializer=initializer)
    logits = tf.layers.dense(hidden2, n_outputs, kernel_initializer=initializer)
    outputs = tf.nn.sigmoid(logits)
    return X, outputs, logits


def start_session():
    sess = tf.Session()
    sess.run(tf.global_variables_initializer())
    saver = tf.train.Saver()
    saver.restore(sess, "./runner/scenes/4_1.ckpt")
    return sess, saver


def init_physics(scene):
    box_id = (findByIdentifier('ground')(scene))['id']
    pybullet.changeDynamics(box_id, -1, lateralFriction=lateral_friction)

    box_id = (findByIdentifier('hunchback')(scene))['id']
    pybullet.changeDynamics(box_id, -1, lateralFriction=2.0)
    pybullet.changeDynamics(box_id, 0, lateralFriction=lateral_friction)
    pybullet.changeDynamics(box_id, 1, lateralFriction=lateral_friction)
    pybullet.changeDynamics(box_id, 2, lateralFriction=lateral_friction)
    pybullet.changeDynamics(box_id, 3, lateralFriction=lateral_friction)
    pybullet.changeDynamics(box_id, 4, lateralFriction=0.1)
    pybullet.changeDynamics(box_id, 5, lateralFriction=0.1)
    return box_id

def init_scene(scene):
    box_id = init_physics(scene)

    X, outputs, logits = make_net()

    rand = tf.random_uniform([4], minval=0., maxval=1., dtype=tf.float32, seed=None, name=None)
    rand2 = tf.random_uniform([4], minval=0., maxval=1., dtype=tf.float32, seed=None, name=None)
    actions = [
        (tf.cond(rand[0] < 0.25, lambda: rand2[0], lambda: outputs[0][0])),
        (tf.cond(rand[1] < 0.25, lambda: rand2[1], lambda: outputs[0][1])),
        (tf.cond(rand[2] < 0.25, lambda: rand2[2], lambda: outputs[0][2])),
        (tf.cond(rand[3] < 0.25, lambda: rand2[3], lambda: outputs[0][3]))]

    gradient_placeholders, \
    grads_and_vars, \
    gradients, \
    training_op = make_ops(actions, logits)

    sess, saver = start_session()

    return (ImmutableRecord({
        'sess': sess,
        'gradient_placeholders': gradient_placeholders,
        'grads_and_vars': grads_and_vars,
        'gradients': gradients,
        'X': X,
        'training_op': training_op,
        'actions': actions,
        'saver': saver
    }), ImmutableRecord({
        GAME_REWARDS: [],
        GAME_GRADIENTS: [],
        ALL_REWARDS: [],
        ALL_GRADIENTS: [],
        BOX_ID: box_id,
        CURRENT_STEP_INDEX: 0,
        CURRENT_GAME_INDEX: 0,
        CURRENT_MOVEMENT_PATTERN: [0, 0, 0, 0],
        CURRENT_ITERATION_INDEX: 0
    }))


def discount_rewards(rewards, discount_rate): # from the handson ml book
    discounted_rewards = np.empty(len(rewards))
    cumulative_rewards = 0
    for step in reversed(range(len(rewards))):
        cumulative_rewards = rewards[step] + cumulative_rewards * discount_rate
        discounted_rewards[step] = cumulative_rewards
    return discounted_rewards


def discount_and_normalize_rewards(ALL_REWARD, discount_rate): # from the handson ml book
    all_discounted_rewards = [discount_rewards(rewards, discount_rate) for rewards in ALL_REWARD]
    flat_rewards = np.concatenate(all_discounted_rewards)
    reward_mean = flat_rewards.mean()
    reward_std = flat_rewards.std()
    return [(discounted_rewards - reward_mean)/reward_std for discounted_rewards in all_discounted_rewards]


def move_shoulder(position, amount, force, control):
    pos = position
    if ((position + amount > 0) and (position + amount < 1.57)):
        pos = position + amount
        control(pos, force = force)
    return pos


def move_arm(position, amount, force, control):
    pos = position
    if ((position + amount > -0.875) and (position + amount < 1.57)):
        pos = position + amount
        control(pos, force = force)
    return pos


def movements_ongoing(boxId):
    threshold = 0.1
    return pybullet.getJointState(boxId, 0)[1] > threshold \
           or pybullet.getJointState(boxId, 1)[1] > threshold \
           or pybullet.getJointState(boxId, 2)[1] > threshold \
           or pybullet.getJointState(boxId, 3)[1] > threshold


def do_move(current_movement_pattern, boxId):

    control_ = partial(pybullet.setJointMotorControl2, boxId)

    for i, movement_type in enumerate(current_movement_pattern):

        pos = pybullet.getJointState(boxId, i)[0]
        control = partial(control_, i, PC)

        d = c
        if abs(pos - current_movement_pattern[i]) > 0.3:
            d = 0.3
        elif abs(pos - current_movement_pattern[i]) <= 0.3 and  abs(pos - current_movement_pattern[i]) > 0.05:
            d = abs(pos - current_movement_pattern[i]) - 0.04
        else:
            d = c / 50
            if (round(pos, 2) == round(current_movement_pattern[i], 2)):
                d = c / 100
            if (round(pos, 3) == round(current_movement_pattern[i], 3)): continue

        if i == 0 or i == 2:
            if pos < current_movement_pattern[i]: move_shoulder(pos, d, force, control)
            else: move_shoulder(pos, -d, force, control)
        if i == 1 or i == 3:
            if pos < current_movement_pattern[i]: move_arm(pos, d, force, control)
            else: move_arm(pos, -d, force, control)


def get_observation(getJointState):
    obs = [getJointState(0)[0] / 1.57,                    # left shoulder
           (getJointState(1)[0]+0.875) / 2.24,            # left arm
           getJointState(2)[0] / 1.57,                    # right shoulder
           (getJointState(3)[0]+0.875) / 2.24]            # right arm
    return obs


def new_iteration(sess_info, ALL_GRADIENTS, ALL_REWARD, current_iteration_index):
    print("NEXT ITERATION", current_iteration_index)

    ALL_REWARD = discount_and_normalize_rewards(ALL_REWARD, 0.95)

    feed_dict = {}
    for var_index, grad_placeholder in enumerate(sess_info.gradient_placeholders):
        mean_gradients = np.mean(
            [reward * ALL_GRADIENTS[game_index][step][var_index]
             for game_index, rewards in enumerate(ALL_REWARD)
             for step, reward in enumerate(rewards)], axis=0
        )
        feed_dict[grad_placeholder] = mean_gradients
    sess_info.sess.run(sess_info.training_op, feed_dict=feed_dict)
    sess_info.saver.save(sess_info.sess, "./runner/scenes/4_1.ckpt")


def get_reward(boxId):
    y = pybullet.getBasePositionAndOrientation(boxId)[0][1]
    reward = -y
    return reward


def give_rewards(state, reward):
    if reward != -111: old_reward = reward
    else: old_reward = 0
    current = get_reward(state.box_id)

    reward = current - old_reward
    reward = round(reward, 2)

    reward_plus_bonus = round(reward - (pybullet.getBasePositionAndOrientation(state.box_id)[0][1] *
                                        (state.current_step_index+1/nr_steps_per_game)), 2)
    print(reward_plus_bonus)

    state = update(state, GAME_REWARDS, conc(reward_plus_bonus))
    return reward, state


def choose_an_action(state, sess_info):
    # choose an action, based on the obsevation
    obs = get_observation(partial(pybullet.getJointState, state.box_id))
    gradient_vs, current_movement_pattern = sess_info.sess.run(
        [sess_info.gradients, sess_info.actions], feed_dict={sess_info.X: np.array(obs).reshape(1, n_inputs)})

    current_movement_pattern = [
        (current_movement_pattern[0]) * 1.57,
        (current_movement_pattern[1]) * 2.24 - 0.875,
        (current_movement_pattern[2]) * 1.57,
        (current_movement_pattern[3]) * 2.24 - 0.875,
        ]
    #print(current_movement_pattern)

    state = assoc(state, CURRENT_MOVEMENT_PATTERN, current_movement_pattern)
    return gradient_vs, state


def reset_robot(state):
    # set hunchback back to 0, 0
    pybullet.resetJointState(state.box_id, 0, 1.57)
    pybullet.resetJointState(state.box_id, 1, 1.57)
    pybullet.resetJointState(state.box_id, 2, 1.57)
    pybullet.resetJointState(state.box_id, 3, 1.57)
    pybullet.resetBasePositionAndOrientation(
        state.box_id,
        [0.,0.,2.],
        pybullet.getQuaternionFromEuler((0., 0., math.radians(180))))


movements_stopped_for_steps = 0
gradient_vs = -1
reward = -111

conc = lambda y: lambda x: x + [y]
inc = lambda x: x + 1


def apply_changes(scene, i, sess_info, state):

    if i < begin: return state
    global gradient_vs
    global reward

    do_move(state.current_movement_pattern, state.box_id)

    global movements_stopped_for_steps
    if not movements_ongoing(state.box_id):
        movements_stopped_for_steps = movements_stopped_for_steps + 1

        ################### NEXT STEP
        if (movements_stopped_for_steps > 25):
            movements_stopped_for_steps = 0

            if (gradient_vs != -1):
                reward, state = give_rewards(state, reward)
                state = update(state, GAME_GRADIENTS, conc(gradient_vs))

            gradient_vs, state = choose_an_action(state, sess_info)
            state = update(state, CURRENT_STEP_INDEX, inc)

            ################## NEXT GAME
            if (state.current_step_index > nr_steps_per_game):
                state = assoc(state, CURRENT_STEP_INDEX, 0)
                state = update(state, CURRENT_GAME_INDEX, inc)
                print("GAME", state.current_game_index)
                gradient_vs = -1
                reward = -111
                state = update(state, ALL_REWARDS, conc(state.game_rewards))
                state = update(state, ALL_GRADIENTS, conc(state.game_gradients))

                reset_robot(state)

                state = assoc(state, CURRENT_MOVEMENT_PATTERN, [1.57, 1.57, 1.57, 1.57])
                state = assoc(state, GAME_GRADIENTS, [])
                state = assoc(state, GAME_REWARDS, [])

                ################ NEXT ITERATION
                if state.current_game_index == nr_games_per_update:
                    state = update(state, CURRENT_ITERATION_INDEX, inc)
                    state = assoc(state, CURRENT_GAME_INDEX, 0)

                    new_iteration(sess_info, state.all_gradients, state.all_rewards, state.current_iteration_index)

                    state = assoc(state, ALL_GRADIENTS, [])
                    state = assoc(state, ALL_REWARDS, [])

    return state




