import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
from collections import defaultdict

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.qtable = defaultdict(int)
        self.previous_state_action = None
        self.previous_reward = None
        self.success_trials = 0
        self.trials = 0
        self.total_reward = 0

    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required
        self.qtable[self.previous_state_action] = self.previous_reward
        self.previous_state_action = None
        self.previous_reward = None
        self.trials += 1

    def get_success_rate_info(self):
        return "{}/{} = {}%".format(self.success_trials, self.trials, round(float(self.success_trials)/self.trials, 3) * 100)

    def get_average_reward(self):
        return round(float(self.total_reward)/self.trials, 3)

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)

        # TODO: Update state
        ideal_next = self.next_waypoint
        self.state = (inputs['light'], ideal_next)

        # TODO: Select action according to your policy
        qvalues = defaultdict(int)
        for a in (None, 'forward', 'left', 'right'):
            qkey = (self.state, a)
            qvalues[qkey] = self.qtable[qkey]

        epsilon = 1./self.trials # decay epsilon
        if random.random() < epsilon:
            # exploration
            action = random.choice((None, 'forward', 'left', 'right'))
        else:
            # exploitation
            max_q_state_action = [key for max_q in [max(qvalues.values())] for key,q in qvalues.iteritems() if q == max_q]
            # when there are more than one best actions given by policy, randomly choose one
            action = random.choice(max_q_state_action)[1]

        # Execute action and get reward
        reward = self.env.act(self, action)
        self.total_reward += reward

        # TODO: Learn policy based on state, action, reward
        state_action = (self.state, action)
        gamma = 0.9
        alpha = 0.1
        if self.previous_state_action is not None and self.previous_reward is not None:
            q_old = self.qtable[self.previous_state_action]
            self.qtable[self.previous_state_action] = q_old + alpha * (self.previous_reward + gamma * max(qvalues.values()) - q_old)

        self.previous_state_action = state_action
        self.previous_reward = reward

        print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=False)  # specify agent to track
    # NOTE: You can set enforce_deadline=False while debugging to allow longer trials

    # Now simulate it
    sim = Simulator(e, update_delay=0.0005, display=False)  # create simulator (uses pygame when display=True, if available)
    # NOTE: To speed up simulation, reduce update_delay and/or set display=False

    sim.run(n_trials=100)  # run for a specified number of trials
    # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line

    print "LearningAgent overall success rate: {}\n\taverage reward per trial: {}".format(a.get_success_rate_info(), a.get_average_reward())

if __name__ == '__main__':
    run()
