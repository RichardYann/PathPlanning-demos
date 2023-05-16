import math
from os import forkpty
from sys import maxsize

class State(object):
    def __init__(self,x,y):   #_init__不要写错？
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"
        self.h = 0
        self.k = 0

    def cost(self,state):
        if self.state == "#" or state.state == "#":
            return maxsize
        return math.sqrt(math.pow((self.x - state.x),2) + math.pow((self.y - state.y),2))

    def set_state(self,state):
        if state not in ["S","E",".","#","+","*"]:
            return
        self.state = state
    
class Map(object):
    def __init__(self,row,col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i,j))   #调用了State的__init__
            map_list.append(tmp)
        return map_list

    def print_map(self):
        for i in range(self.row):
            tmp = ""
            for j in range(self.col):
                tmp += self.map[i][j].state + " "
            print(tmp)

    def get_neighbors(self,state):
        state_list = []
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row or state.y + j < 0 or state.y +j >= self.col:
                    continue  
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self,point_list):
        for x,y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue
            self.map[x][y].set_state("#")


class Dstar(object):

    def __init__(self,maps):
        self.map = maps
        self.open_list = set()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list,key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self,state,h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k,h_new)
        elif state.t == "close":
            state.k = min(state.k,h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self,state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self,x):   #这个其实是把指向障碍物的结点h值变为无穷大的过程
        if x.t == "close":
            self.insert(x,x.parent.h + x.cost(x.parent))  #这里的insert不只修改了h值 同时还把该结点重新加入open_list,
                                                            # 这样找openlist中最小k结点就为该节点

    def run(self,start,end):
        self.open_list.add(end)
        while True:
            self.process_state()
            if start.t == "close":
                break
        start.set_state("S")
        s =start
        while s != end:
            s = s.parent
            s.set_state("+")
        s.set_state("E")
        print("未发生变化的搜索路径：")
        self.map.print_map()
        self.map.set_obstacle([(3,3)])   #函数的参数是一个列表
        tmp = start
        while tmp != end:
            tmp.set_state("*")
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("E")
        print("障碍物发生变化时的搜索路径：")
        self.map.print_map()

    def modify(self,state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

    def process_state(self):
        x = self.min_state()
        if x is None:
            return -1
        k_old = self.get_kmin()
        self.remove(x)
        print("**************x state",x.x,x.y,k_old,x.h)
        if k_old == x.h:
            for y in self.map.get_neighbors(x):
                # if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \   #这里犯了一个巨大的错误 and y != end前面有一个括号
                # or y.parent != x and x.h + x.cost(y) < y.h and y != end:        #说明这个条件是必须要满足的！！！和前面并列 这里单独排除end点好像因为end点tag也是new
                if (y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y)) and y != end:
                    y.parent = x
                    self.insert(y,x.h + x.cost(y))
            print('k_old == x.h',x.h,x.k)
        elif k_old < x.h:
            for y in self.map.get_neighbors(x):
                if  y.h <= k_old and y.h + x.cost(y) < x.h:
                    x.parent = y
                    x.h = y.h + x.cost(y)           # h = k 时要用insert加入openlist 而 h > k 时不用，只要修改代价和父指针
                elif y.h > x.k and y.h + x.cost(y) < x.h and y.parent != x and y.t == "close":
                    x.parent = y 
                    x.h = y.h + x.cost(y)
            print("k_old < x.h",k_old,x.h) 

        return self.get_kmin()

if __name__ == "__main__":
    m = Map(6, 7)
    m.set_obstacle([(1, 1), (1,2), (2, 1), (2, 2), (3, 1),(3,2),(4,3)])
    start = m.map[5][1]
    end = m.map[0][6]
    dstar = Dstar(m)
    dstar.run(start,end) 

                      

                     

