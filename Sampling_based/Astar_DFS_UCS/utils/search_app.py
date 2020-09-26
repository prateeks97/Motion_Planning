# search_app.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Authors: Pei Xu (peix@g.clemson.edu) and Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import time, random, math
import tkinter as tk
import tkinter.messagebox 


class App(tk.Frame):
    random.seed('cs4420')
    # possible actions moving from one cell to a neighbor cell
    ACTIONS = (
        (-1,  0), # go up
        ( 0, -1), # go left
        ( 1,  0), # go down
        ( 0,  1) # go right
    )
  
    def __init__(self, search_fn_map, master=None):
        super().__init__(master)
        self.search_fn_map = search_fn_map

        self.master.title("Search Algorithms -- CPSC 4420/6420 Clemson University")
        
        self.canvas = tk.Canvas(self.master, bg="white")
        self.bt_new = tk.Button(self.master, text="Random", command=self.new_game)
        self.bt_default = tk.Button(self.master, text="Default", command=self.default_game)
        self.alg_var = tk.StringVar(self.master)
        self.alg_var.set(next(iter(self.search_fn_map.keys())))
        self.listbox_alg = tk.OptionMenu(self.master, self.alg_var, *self.search_fn_map.keys())
        self.bt_search = tk.Button(self.master, text="Search", command=self.search)

        self.master.geometry("800x600")
        self.master.resizable(False, False)
        self.canvas.grid(row=0, columnspan=4,
            sticky=tk.W+tk.E+tk.N+tk.S, padx=10, pady=28
        )
        self.bt_new.grid(row=1, column=0,
            sticky=tk.W, padx=10, pady=(0, 10)
        )
        self.bt_default.grid(row=1, column=1,
            sticky=tk.W, padx=5, pady=(0, 10)
        )
        self.listbox_alg.grid(row=1, column=2,
            sticky=tk.E, padx=10, pady=(0, 10)
        )
        self.bt_search.grid(row=1, column=3,
            sticky=tk.E, padx=10, pady=(0, 10)
        )
        self.master.columnconfigure(0, weight=0)
        self.master.columnconfigure(1, weight=1)
        self.master.columnconfigure(2, weight=1)
        self.master.columnconfigure(3, weight=0)
        self.master.rowconfigure(0, weight=1)
        self.master.rowconfigure(1, weight=0)

        self.game_width, self.game_height = None, None
        self.game_grid, self.start, self.goal = None, None, None
        self.costFn = lambda x: 1
        '''
           To penalize positions on the north side of the map:
           self.costFn = lambda pos: .5 ** pos[0] 
        '''    
        self.canvas.bind("<Configure>", self.default_game)

        
    def default_game(self, event=None):
        self.clear_canvas()

        self.game_width = 30
        self.game_height = 20
        self.start = (15, 5)
        self.goal = (5, 25)
        self.obstacles = []
        for i in range(6, 16):
            self.obstacles.append((i, 21))
        for i in range(6, 20):
            self.obstacles.append((8, i))
        for i in range(12, 21):
            self.obstacles.append((12, i))
        self.obstacles = tuple(self.obstacles)
        self.draw_grid()
        self.draw_start(self.start)
        self.draw_goal(self.goal)
        for coord in self.obstacles:
            self.draw_obstacle(coord)

    def new_game(self, event=None):
        self.clear_canvas()

        self.game_width = 30
        self.game_height = 20
        self.n_obstacles = 80

        self.draw_grid()

        game_grid = [
            [0 for _ in range(self.game_width)] for __ in range(self.game_height)
        ]
        n_obstacles = 0
        self.obstacles = []
        while n_obstacles < self.n_obstacles:
            coord = (random.randint(0, self.game_height-1), random.randint(0, self.game_width-1))
            if game_grid[coord[0]][coord[1]] != 1:
                game_grid[coord[0]][coord[1]] = 1
                n_obstacles += 1
                self.draw_obstacle(coord)
                self.obstacles.append((coord[0], coord[1]))
        self.obstacles = tuple(self.obstacles)

        while True:
            self.start = (random.randint(0, self.game_height-1), random.randint(0, self.game_width-1))
            if game_grid[self.start[0]][self.start[1]] == 0:
                break
        while True:
            self.goal = (random.randint(0, self.game_height-1), random.randint(0, self.game_width-1))
            if game_grid[self.goal[0]][self.goal[1]] == 0 and self.goal != self.start:
                break
                
        self.draw_start(self.start)
        self.draw_goal(self.goal)

        
    def search(self):
        self.bt_search.config(state=tk.DISABLED)
        self.bt_new.config(state=tk.DISABLED)
        self.bt_default.config(state=tk.DISABLED)
        self.listbox_alg.config(state=tk.DISABLED)
        logger = Logger(self, None, None)

        v = self.alg_var.get()
        print("Alg:", v, "Search")
        fn = self.search_fn_map[v]
        p, closed = fn((self.game_height, self.game_width), self.start, self.goal, self.obstacles, self.costFn, logger)

        if p is None or len(p) == 0:
            tk.messagebox.showinfo("", "Failed to find any solution path.")
        else:
            self.draw_path(p)

        print("Closed Set:")
        for e in closed:
            print(e)

        self.bt_search.config(state=tk.NORMAL)
        self.bt_new.config(state=tk.NORMAL)
        self.bt_default.config(state=tk.NORMAL)
        self.listbox_alg.config(state=tk.NORMAL)
    
    def draw_grid(self):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        self.clear_canvas()
        self.canvas.delete("grid_line")

        c_interval = w / self.game_width
        r_interval = h / self.game_height

        for c in range(self.game_width):
            self.canvas.create_line([(c_interval*c, 0), (c_interval*c, r_interval*h)], tag="grid_line")
        for r in range(self.game_height):
            self.canvas.create_line([(0, r_interval*r), (c_interval*w, r_interval*r)], tag="grid_line")

    def draw_start(self, r, c=None):
        if c is None: r, c = r
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        c_interval = w / self.game_width
        r_interval = h / self.game_height
        self.canvas.create_rectangle(
            c_interval*c, r_interval*r,
            c_interval*(c+1), r_interval*(r+1),
            fill="green", tag="start"
        )
        self.canvas.create_text(
            c_interval*(c+0.5), r_interval*(r+0.5),
            text="S", fill="white",
            font=(None, int(math.ceil(min(c_interval, r_interval)*0.5))),
            tag="start"
        )

    def draw_goal(self, r, c=None):
        if c is None: r, c = r
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        c_interval = w / self.game_width
        r_interval = h / self.game_height
        self.canvas.create_rectangle(
            c_interval*c, r_interval*r,
            c_interval*(c+1), r_interval*(r+1),
            fill="blue", tag="goal"
        )
        self.canvas.create_text(
            c_interval*(c+0.5), r_interval*(r+0.5),
            text="G", fill="white",
            font=(None, int(math.ceil(min(c_interval, r_interval)*0.5))),
            tag="goal"
        )

    def draw_obstacle(self, r, c=None):
        if c is None: r, c = r
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        c_interval = w / self.game_width
        r_interval = h / self.game_height
        self.canvas.create_rectangle(
            c_interval*c, r_interval*r,
            c_interval*(c+1), r_interval*(r+1),
            fill="gray", tag="obstacle"
        )

    def draw_closed_set(self, r, c=None):
        if c is None: r, c = r
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        c_interval = w / self.game_width
        r_interval = h / self.game_height
        rad = 0.3*min(c_interval, r_interval)
        self.canvas.create_oval(
            c_interval*(c+0.5)-rad, r_interval*(r+0.5)-rad,
            c_interval*(c+0.5)+rad, r_interval*(r+0.5)+rad,
            fill="red", tag="closed_set"
        )

    def draw_open_set(self, r, c=None):
        if c is None: r, c = r
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        c_interval = w / self.game_width
        r_interval = h / self.game_height
        rad = 0.3*min(c_interval, r_interval)
        self.canvas.create_oval(
            c_interval*(c+0.5)-rad, r_interval*(r+0.5)-rad,
            c_interval*(c+0.5)+rad, r_interval*(r+0.5)+rad,
            tag="open_set"
        )
    
    def clear_canvas(self):
        self.canvas.delete("open_set")
        self.canvas.delete("closed_set")
        self.canvas.delete("start")
        self.canvas.delete("goal")
        self.canvas.delete("obstacle")
        self.canvas.delete("path")
    
    def draw_path(self, movement):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        c_interval = w / self.game_width
        r_interval = h / self.game_height
        r, c = self.start
        for a in movement:
            if a in self.ACTIONS:
                r_, c_ = r+a[0], c+a[1]
                self.canvas.create_line(
                    c_interval*(c+0.5), r_interval*(r+0.5),
                    c_interval*(c_+0.5), r_interval*(r_+0.5),
                    arrow=tk.LAST, dash=(5, 1, 2, 1), tag="path"
                )
                r, c = r_, c_
            else:
                tk.messagebox.showinfo("", "Invalid action `{}` was found.".format(a))
                break
    

class Logger(object):

    def __init__(self, app, closed_set, open_set):
        self.app = app
        self.closed_set = closed_set
        self.open_set = open_set
    
    def flush(self):
        self.app.clear_canvas()
        for r in range(self.app.game_height):
            for c in range(self.app.game_width):
                if (r, c) == self.app.start:
                    self.app.draw_start(r, c)
                if (r, c) == self.app.goal:
                    self.app.draw_goal(r, c)
                if (r, c) in self.app.obstacles:
                    self.app.draw_obstacle(r, c)
                if (r, c) in self.closed_set:
                    self.app.draw_closed_set(r, c)
                if (r, c) in self.open_set:
                    self.app.draw_open_set(r, c)
        self.app.update()
        # time.sleep(0.1)


class AbstractContainer(object):
    """ A wrapper to the raw list object which
    adds a hook to call the visualization logger
    when adding or removing elements from it
    """
    def __init__(self):
        self._container = []
        self.logger = None

    def add(self, item):
        if item not in self._container:
            self._container.append(item)
            if self.logger is not None:
                self.logger.flush()
        else:
            self._container.append(item)

    def has(self, item):
        return self._container.__contains__

    def remove(self, item):
        if item in self._container:
            self._container.remove(item)
            if self.logger is not None:
                self.logger.flush()
    
    def clear(self):
        self._container.clear()
    
    def __contains__(self, item):
        return self._container.__contains__(item)

    def __len__(self):
        return self._container.__len__()
    
    def __iter__(self):
        return self._container.__iter__()
    
    def pop(self, last=True):
        if last:
            e = self._container.pop()
        else:
            e = self._container.pop(0)
        if self.logger is not None:
            self.logger.flush()
        return e


class OrderedSet(AbstractContainer):
    def pop(self, last=True):
        if len(self._container) > 0:
            return super().pop(last)
        raise IndexError("pop from empty set")

    def add(self, item):
        if item not in self._container:
            return super().add(item)


class Stack(OrderedSet):
    def pop(self):
        if len(self._container) > 0:
            return super(OrderedSet, self).pop()
        raise IndexError("pop from empty stack")
    
    def add(self, item):
        super(OrderedSet, self).add(item)


class Queue(OrderedSet):
    def pop(self):
        if len(self._container) > 0:
            return super(OrderedSet, self).pop(last=False)
        raise IndexError("pop from empty queue")

    def add(self, item):
        super(OrderedSet, self).add(item)


class PriorityQueue(AbstractContainer):
    def __init__(self, order=min, f=lambda v:v):
        if order == min or order == "min":
            self.order = min
        elif order == max or order == "max":
            self.order = max
        else:
            raise KeyError("order must be min or max")
        self.f = f

        self._dict = {}
        self.logger = None

    def get(self, item):
        return self._dict.__getitem__(item)

    def put(self, item, value):
        if item not in self._dict:
            self._dict[item] = value
            if self.logger is not None:
                self.logger.flush()
        else:
            self._dict[item] = value

    def has(self, item):
        return self._dict.__contains__(item)

    def remove(self, item):
        if item in self._dict:
            del self._dict[item]
            # if self.logger is not None:
                # self.logger.flush()

    def pop(self):
        if len(self._dict) > 0:
            tar = self.order(self._dict, key=lambda k: self.f(self._dict.get(k)))
            val = self._dict[tar]
            del self._dict[tar]
            # if self.logger is not None:
                # self.logger.flush()
            return tar, val
        raise IndexError("pop from empty priority queue")

    def __iter__(self):
        return self._dict.__iter__()
    
    def __contains__(self, item):
        return self._dict.__contains__(item)

    def __len__(self):
        return self._dict.__len__()

    def __getitem__(self, key):
        return self._dict.__getitem__(key)
    
    def __setitem__(self, key, value):
        return self._dict.__setitem__(key, value)
    
    def __delitem__(self, key):
        return self._dict.__delitem__(key)