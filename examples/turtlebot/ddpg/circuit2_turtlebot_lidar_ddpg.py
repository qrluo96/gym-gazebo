#!/usr/bin/env python

'''
Based on:
https://github.com/vmayoral/basic_reinforcement_learning
https://gist.github.com/wingedsheep/4199594b02138dd427c22a540d6d6b8d
'''
import gym
from gym import wrappers
import gym_gazebo
import time
from distutils.dir_util import copy_tree
import os
import json
import liveplot
from ddpg import *

def detect_monitor_files(training_dir):
    return [os.path.join(training_dir, f) for f in os.listdir(training_dir) if f.startswith('openaigym')]

def clear_monitor_files(training_dir):
    files = detect_monitor_files(training_dir)
    if len(files) == 0:
        return
    for file in files:
        print(file)
        os.unlink(file)

EPISODES = 100000
TEST = 10
def main():
    #REMEMBER!: turtlebot_nn_setup.bash must be executed.
    env = gym.make('GazeboCircuit2TurtlebotLidarNn-v0')
    agent = DDPG(env)
    outdir = '/tmp/gazebo_gym_experiments/'
    path = '/tmp/turtle_c2_dqn_ep'
    plotter = liveplot.LivePlot(outdir)

    continue_execution = False
    #fill this if continue_execution=True
    resume_epoch = '200' # change to epoch to continue from
    resume_path = path + resume_epoch
    monitor_path = resume_path

    steps = 100000

    env._max_episode_steps = steps
    # env = gym.wrappers.Monitor(env, outdir,force=not continue_execution, resume=continue_execution)

    for episode in xrange(EPISODES):
        observation = env.reset()
        #print "episode:",episode
        # Train
        for step in xrange(env.spec.timestep_limit):
            action = agent.noise_action(observation)
            next_state,reward,done,_ = env.step(action)
            agent.perceive(observation,action,reward,next_state,done)
            state = next_state
            if done:
                break
        # Testing:
        if episode % 100 == 0 and episode > 100:
            total_reward = 0
            for i in xrange(TEST):
                state = env.reset()
                for j in xrange(env.spec.timestep_limit):
                    #env.render()
                    action = agent.action(state) # direct action for test
                    state,reward,done,_ = env.step(action)
                    total_reward += reward
                    if done:
                        break
            ave_reward = total_reward/TEST
            print ('episode: ',episode,'Evaluation Average Reward:',ave_reward)

    env.monitor.close()



if __name__ == '__main__':
    main()