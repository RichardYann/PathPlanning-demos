#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2018/12/13 0013 22:30
# @Author  : 心一
# @Site    :
# @File    : D_star.py
# @Software: PyCharm
 
import math
from sys import maxsize # 导入最大数，2^63-1
 
 
class State(object):
 
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"
        self.h = 0
        self.k = 0  # k即为f
 
    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize  # 存在障碍物时，距离无穷大
        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))
 
    def set_state(self, state):
        if state not in ["S", ".", "#", "E", "*","+"]:
            return
        self.state = state
 
 
class Map(object):
    '''
    创建地图
    '''
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()
 
    def init_map(self):
        # 初始化map
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list
 
    def print_map(self):
        for i in range(self.row):
            tmp = ""
            for j in range(self.col):
                tmp += self.map[i][j].state + " "
            print(tmp)
 
    def get_neighbers(self, state):
        # 获取8邻域
        state_list = []
        for i in [-1, 0,1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list
 
    def set_obstacle(self, point_list):
        # 设置障碍物的位置
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue
            self.map[x][y].set_state("#")
 
 
class Dstar(object):
 
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()  # 创建空集合
 
    def process_state(self):
        '''
        D*算法的主要过程
        :return:
        '''
        x = self.min_state()    # 获取open list列表中最小k的节点
        if x is None:
            return -1
        # k_old = self.get_kmin() #获取open list列表中最小k节点的k值
        k_old = x.k
        # print('myold',k_old)
        self.remove(x)  # 从openlist中移除
        
        # print('**************x state',x.x,x.y,x.k,x.h)
        # 判断openlist中
        if k_old < x.h:
            for y in self.map.get_neighbers(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):  #y.h <= k_old 表示选择离终点更近领域 因为终点到起点的k值不断增加 所以k值越小离终点越近 如果找不到这样的点可能就需要进一步的扩散辐射
                                                              #但是他这里只要满足条件就修改代价和指针 因此不能保证是最优路径 因为可能有多个领域满足条件 x.h > y.h + x.cost(y)只要领域不是障碍物就可以满足
                    x.parent = y                                #x.h > y.h + x.cost(y)用于排除领域中的障碍物
                    x.h = y.h + x.cost(y)                #修改后 y.h还是小于x.h 但是h.x就能够降下来了 不再是无穷大
                    # print('my k_old < x.h',x.k,x.h)
                    # print('my kmin',self.get_kmin())    #周围8个领域全部都会遍历到 一定是8个点中代价最小的
                elif y.parent != x and x.h > y.h + x.cost(y) and y.h > x.k and y.t == "close" :
                    x.parent = y                                #x.h > y.h + x.cost(y)用于排除领域中的障碍物
                    x.h = y.h + x.cost(y)                    
            print('k_old < x.h',x.k,x.h)
        elif k_old == x.h:
            for y in self.map.get_neighbers(x):
                if (y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y)) and y != end:
                    y.parent = x
                    #if y.x == 17 and  y.y == 1
                    #print('Debug: ',y.x,y.y,y.k,y.h)
                    self.insert(y, x.h + x.cost(y))
            print('k_old == x.h',x.h,x.k)
        # else:
        #     for y in self.map.get_neighbers(x):
        #         if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
        #             y.parent = x
        #             self.insert(y, x.h + x.cost(y))
        #         else:
        #             if y.parent != x and y.h > x.h + x.cost(y):
        #                 self.insert(x, x.h)
        #             else:
        #                 if y.parent != x and x.h > y.h + x.cost(y) \
        #                         and y.t == "close" and y.h > k_old:
        #                     self.insert(y, y.h)
        #     print('k_old > x.h',x.h,x.k)
        return self.get_kmin()
 
    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)  # 获取openlist中k值最小对应的节点
        return min_state
 
    def get_kmin(self):
        # 获取openlist表中k(f)值最小的k
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min
 
    def insert(self, state, h_new):    # ?
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.k, h_new)  #这里h和k是一样的 所以取哪个应该都行
            # state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)
 
    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)
 
    def modify_cost(self, x):
        if x.t == "close":  # 是以一个openlist，通过parent递推整条路径上的cost
            self.insert(x, x.parent.h + x.cost(x.parent))  #
 
    def run(self, start, end):
        self.open_list.add(end)
        while True:
            self.process_state()
            if start.t == "close":
                break
        start.set_state("S")
        s = start
        while s != end:
            s = s.parent
            s.set_state("+")
        s.set_state("E")
        print('障碍物未发生变化时，搜索的路径如下：')
        self.map.print_map()
        tmp = start # 起始点不变
        # self.map.set_obstacle([(9, 3), (9, 4), (9, 5), (9, 6), (9, 7), (9, 8)]) # 障碍物发生变化
        self.map.set_obstacle([(3, 3)]) # 障碍物发生变化
        # self.map.print_map()
        '''
        从起始点开始，往目标点行进，当遇到障碍物时，重新修改代价，再寻找路径
        '''
        while tmp != end:
            tmp.set_state("*")
            # self.map.print_map()
            # print("")
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("E")
        print('障碍物发生变化时，搜索的路径如下(*为更新的路径)：')
        self.map.print_map()
 
    def modify(self, state):
        '''
        当障碍物发生变化时，从目标点往起始点回推，更新由于障碍物发生变化而引起的路径代价的变化
        :param state:
        :return:
        '''
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break
 
 
if __name__ == '__main__':
    # m = Map(20, 20)
    # m.set_obstacle([(4, 3), (4, 4), (4, 5), (4, 6), (5, 3), (6, 3), (7, 3)])
    # start = m.map[1][2]
    # end = m.map[17][11]

    m = Map(6, 7)
    m.set_obstacle([(1, 1), (1,2), (2, 1), (2, 2), (3, 1),(3,2),(4,3)])
    start = m.map[5][1]
    end = m.map[0][6]

    # start = m.map[0][6]
    # end = m.map[5][1]
    dstar = Dstar(m)
    dstar.run(start, end)
    # m.print_map()