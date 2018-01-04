import numpy as np
import time
import sys
import tkinter as tk

UNIT = 100
SIZE = UNIT / 2 * .8
MAZE_H = 5
MAZE_W = 5


class Maze(tk.Tk, object):
    def __init__(self):
        super(Maze, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('maze')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        self._build_maze()
        
    def hell_coords(self):
        hellcoords=[]
        for i in range(0,3):
            hellcoords.append((1,i))
        for i in range(1,4):
            hellcoords.append((i,4))
        for i in range(1,4):
            hellcoords.append((3,i))
        return hellcoords
   
        
    def _build_maze(self):
        self.canvas = tk.Canvas(self, bg='white',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        # create grids
        for c in range(0, MAZE_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)
        for r in range(0, MAZE_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, MAZE_H * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)

        # create origin
        origin = np.array([UNIT / 2, UNIT / 2])

        # hell
        self.hells_list = []
        for c in self.hell_coords():
            hell_center = origin + np.array(c) * UNIT
            self.hell = self.canvas.create_rectangle(
                hell_center[0] - SIZE, hell_center[1] - SIZE,
                hell_center[0] + SIZE, hell_center[1] + SIZE,
                fill='black')
            self.hells_list.append(self.canvas.coords(self.hell))

        # create oval
        oval_center = origin + UNIT * 4
        self.oval = self.canvas.create_oval(
            oval_center[0] - SIZE, oval_center[1] - SIZE,
            oval_center[0] + SIZE, oval_center[1] + SIZE,
            fill='yellow')

        # create red rect
        self.rect = self.canvas.create_rectangle(
            origin[0] - SIZE, origin[1] - SIZE,
            origin[0] + SIZE, origin[1] + SIZE,
            fill='red')

        # pack all
        self.canvas.pack()

    def reset(self):
        self.update()
        #time.sleep(0.005)
        self.canvas.delete(self.rect)
        origin = np.array([UNIT / 2, UNIT / 2])
        self.rect = self.canvas.create_rectangle(
            origin[0] - SIZE, origin[1] - SIZE,
            origin[0] + SIZE, origin[1] + SIZE,
            fill='red')
        # return observation
        return self.canvas.coords(self.rect)

    def step(self, action):
        s = self.canvas.coords(self.rect)
        base_action = np.array([0, 0])
        if action == 0:   # up
            if s[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1:   # down
            if s[1] < (MAZE_H - 1) * UNIT:
                base_action[1] += UNIT
        elif action == 2:   # right
            if s[0] < (MAZE_W - 1) * UNIT:
                base_action[0] += UNIT
        elif action == 3:   # left
            if s[0] > UNIT:
                base_action[0] -= UNIT

        self.canvas.move(self.rect, base_action[0], base_action[1])  # move agent

        s_ = self.canvas.coords(self.rect)  # next state

        # reward function
        if s_ == self.canvas.coords(self.oval):
            reward = 1
            done = True
        elif s_ in self.hells_list:
            reward = -1
            done = True
        else:
            reward = -0.05
            done = False

        return s_, reward, done

    def render(self):
        #time.sleep(0.03)
        self.update()
        
def update():
    for t in range(10):
        s = env.reset()
        while True:
            env.render()
            a = 1
            s, r, done = env.step(a)
            if done:
                break
            
if __name__ == '__main__':
    env = Maze()
    env.after(100, update)
    env.mainloop()