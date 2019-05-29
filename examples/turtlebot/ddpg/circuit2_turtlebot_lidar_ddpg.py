#!/usr/bin/env python

import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
from distutils.dir_util import copy_tree
import os
import json
import liveplot
from ddpg import *

TEST = 10
def main():
    #REMEMBER!: turtlebot_nn_setup.bash must be executed.
    env = gym.make('GazeboCircuit2TurtlebotLidarNn-v0')
    
    outdir = '/tmp/gazebo_gym_experiments/'
    # plotter = liveplot.LivePlot(outdir)

    continue_execution = False
    #fill this if continue_execution=True

    epochs = 1000
    steps = 1000

    env._max_episode_steps = steps
    env = gym.wrappers.Monitor(env, outdir,force=not continue_execution, resume=continue_execution)
    
    agent = DDPG(env)

    last100Scores = [0] * 100
    last100ScoresIndex = 0
    last100Filled = False
    stepCounter = 0
    highest_reward = 0

    start_time = time.time()

    print("Start iteration")

    #start iterating from 'current epoch'.
    for epoch in xrange(epochs):
        observation = env.reset()
        cumulated_reward = 0
        done = False
        episode_step = 0

        # run until env returns done
        while not done:
            # env.render()
            # print(observation)
            action = agent.noise_action(observation)

            newObservation, reward, done, _ = env.step(action)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            agent.perceive(observation, action, reward, newObservation, done)

            observation = newObservation
            
            if done:
                last100Scores[last100ScoresIndex] = episode_step
                last100ScoresIndex += 1
                if last100ScoresIndex >= 100:
                    last100Filled = True
                    last100ScoresIndex = 0
                if not last100Filled:
                    print ("EP " + str(epoch) + " - " + format(episode_step + 1) + "/" + str(steps))
                else :
                    m, s = divmod(int(time.time() - start_time), 60)
                    h, m = divmod(m, 60)
                    print ("EP " + str(epoch) + " - " + format(episode_step + 1) + "/" + str(steps) + " Episode steps - last100 Steps : " + str((sum(last100Scores) / len(last100Scores))) + " - Cumulated R: " + str(cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s))

                    if (epoch)%100==0:
                        env._flush(force=True)

            stepCounter += 1
            episode_step += 1

        # if epoch % 100 == 0:
        #     plotter.plot(env)

    env.close()

if __name__ == '__main__':
    main()