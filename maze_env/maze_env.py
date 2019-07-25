import numpy as np
import time

UNIT = 160
BLOCK_SIZE = 120
MAZE_H = 4
MAZE_W = 4

class Maze:
  def __init__(self):
    self.action_space = ['u', 'd', 'l', 'r']
    self.n_actions = len(self.action_space)
    self.n_states = 2
    self.agent_coord = np.array([0, 0])
    self.hell1_coord = np.array([2, 1])
    # self.hell2_coord = np.array([1, 2])
    self.goal_coord = np.array([2, 2])

    self.viewer = None

  def reset(self):
    self.agent_coord = np.array([0, 0]) 
    self.num_step = 0

    return self.agent_coord

  def step(self, action):
    reward = 0
    done = False
    s = self.agent_coord
    ns = s
    if action == 0: # up
      if s[1] > 0:
        ns = s - np.array([0, 1])
    elif action == 1: # down
      if s[1] < MAZE_H - 1:
        ns = s + np.array([0, 1])
    elif action == 2: # right 
      if s[0] > 0:
        ns = s - np.array([1, 0])
    elif action == 3:
      if s[0] < MAZE_W - 1:
        ns = s + np.array([1, 0])

    self.agent_coord = ns

    if (self.agent_coord == self.goal_coord).all():
      reward = 1
      done = True
    elif (self.agent_coord == self.hell1_coord).all(): # or (self.agent_coord == self.hell2_coord).all():
      reward = -1
      done = True
    elif self.num_step > 200:
      reward = 0
      done = True

    self.num_step += 1

    return self.agent_coord, reward, done, None

  def render(self):
    screen_width = MAZE_W * UNIT
    screen_height = MAZE_H * UNIT
    
    offset = np.array([UNIT/2, UNIT/2])

    if self.viewer is None:
      import rendering
      self.viewer = rendering.Viewer(screen_width, screen_height)

      for c in range(0, MAZE_W * UNIT, UNIT):
        x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
        self.viewer.add_geom( rendering.Line((x0, y0), (x1, y1)) )
      for r in range(0, MAZE_H * UNIT, UNIT):
        x0, y0, x1, y1, = 0, r, MAZE_W * UNIT, r
        self.viewer.add_geom( rendering.Line((x0, y0), (x1, y1)) )

      hell1_center = offset + self.hell1_coord * UNIT
      hell1 = rendering.FilledPolygon([(-BLOCK_SIZE/2, -BLOCK_SIZE/2), (-BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, -BLOCK_SIZE/2)])
      hell1trans = rendering.Transform()
      hell1.add_attr( hell1trans )
      hell1trans.set_translation( hell1_center[0], MAZE_H * UNIT - hell1_center[1] )
      self.viewer.add_geom( hell1 )

      # hell2_center = offset + self.hell2_coord * UNIT
      # hell2 = rendering.FilledPolygon([(-BLOCK_SIZE/2, -BLOCK_SIZE/2), (-BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, -BLOCK_SIZE/2)])
      # hell2trans = rendering.Transform()
      # hell2.add_attr( hell2trans )
      # hell2trans.set_translation( hell2_center[0], MAZE_H * UNIT - hell2_center[1] )
      # self.viewer.add_geom( hell2 )
      
      goal_center = offset + self.goal_coord * UNIT
      goal = rendering.FilledPolygon([(-BLOCK_SIZE/2, -BLOCK_SIZE/2), (-BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, -BLOCK_SIZE/2)])
      goal.set_color(1., 1., 0.)
      goaltrans = rendering.Transform()
      goal.add_attr( goaltrans )
      goaltrans.set_translation( goal_center[0], MAZE_H * UNIT - goal_center[1] )
      self.viewer.add_geom( goal )

      agent = rendering.FilledPolygon([(-BLOCK_SIZE/2, -BLOCK_SIZE/2), (-BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, BLOCK_SIZE/2), (BLOCK_SIZE/2, -BLOCK_SIZE/2)])
      agent.set_color(1., 0., 0.)
      self.agenttrans = rendering.Transform()
      agent.add_attr( self.agenttrans )
      self.viewer.add_geom( agent )

    agent_center = offset + self.agent_coord * UNIT
    self.agenttrans.set_translation( agent_center[0], MAZE_H * UNIT - agent_center[1] )
      
    return self.viewer.render(return_rgb_array = False)

if __name__ == '__main__':
  env = Maze()
  
  for t in range(10):
    s = env.reset()
    done = False
    while not done:
      print(s)
      env.render()
      a = 1
      s, r, done = env.step(a)
