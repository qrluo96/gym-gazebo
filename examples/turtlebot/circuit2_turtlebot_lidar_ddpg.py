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

EPOCHS = 1000
TEST = 10
def main():
    #REMEMBER!: turtlebot_nn_setup.bash must be executed.
    env = gym.make('GazeboCircuit2TurtlebotLidarNn-v0')
    agent = DDPG(env)
    
    outdir = '/tmp/gazebo_gym_experiments/'
    plotter = liveplot.LivePlot(outdir)

    continue_execution = False
    #fill this if continue_execution=True

    steps = 1000

    env._max_episode_steps = steps
    env = gym.wrappers.Monitor(env, outdir,force=not continue_execution, resume=continue_execution)

    last100Scores = [0] * 100
    last100ScoresIndex = 0
    last100Filled = False
    stepCounter = 0
    highest_reward = 0

    start_time = time.time()

    print("Start iteration")
    
    #start iterating from 'current epoch'.
    for epoch in xrange(EPOCHS):
        observation = env.reset()
        cumulated_reward = 0
        done = False
        episode_step = 0

        # run until env returns done
        while not done:
            # env.render()
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
                        env._flush()

            stepCounter += 1
            episode_step += 1

        if epoch % 100 == 0:
            plotter.plot(env)

        # # Testing:
        # if epoch % 100 == 0 and epoch > 100:
        #     total_reward = 0
        #     for i in xrange(TEST):
        #         observation = env.reset()
        #         for j in xrange(env.spec.timestep_limit):
        #             #env.render()
        #             action = agent.action(observation) # direct action for test
        #             observation,reward,done,_ = env.step(action)
        #             total_reward += reward
        #             if done:
        #                 break
        #     ave_reward = total_reward/TEST
        #     print ('epoch: ',epoch,'Evaluation Average Reward:',ave_reward)
        #     plotter.plot(env)

    # env.monitor.close()
    env.close()

if __name__ == '__main__':
    main()