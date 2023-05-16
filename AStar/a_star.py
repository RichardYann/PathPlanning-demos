"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]  障碍物的x坐标列表
        oy: y position list of Obstacles [m]  障碍物的y坐标列表
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid        #Node初始化需要的参数为index！！！
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
#当使用print输出对象的时候，若定义了__str__(self)方法打印对象时就会从这个方法中打印出return的z字符串数据。
        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(  
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):  #把calc那些常用函数放在前面更好 符合逻辑 可读性强 有代码提示
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)
        print("goal_node:",goal_node)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node  #字典一对一存储 
                                                                 #字典要用[]来取值

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,  #min（，key）其中key是对前面数据判别最小值需要做的处理 lambda定义以o为参数的函数
                                                                     open_set[
                                                                         o]))

            # c_id = min(
            #     open_set,
            #     key=lambda o: open_set[o].cost)     #这样就是dijkstra算法

            current = open_set[c_id]

            # show graph   
            if show_animation:  # pragma: no cover  注释：不覆盖
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")  #c青绿色 的叉号
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [ exit(0) if event.key == 'escape' else None])  #键鼠相应事件 key_release_event按下键盘
                if len(closed_set.keys()) % 10 == 0:       # .keys以列表返回字典的所有键 不用也可以
                    plt.pause(0.001)  #每处理完10个结点暂停0.001s 不暂停则没有动画效果直接显示结果

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                # goal_node = current  也可以这样代替 但是逻辑性没那么强
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current   #open set和closed set中结点存放的位置一样

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)  #这里的c_id设置了初始的parent指针
                                                                          #这里的cost是累计cost 加上了之前结点的cost
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node         #！！！这里同时更新了代价和父结点指针 是整个结点直接进行了替换 并不是只更新了代价

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]      #一定要定义成列表的形式
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]    #closed set中存放了所有路径回溯的结点
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod #无需声明对象可以直接调用类的函数
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic（启发性/启发函数）
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)    #平方和开根号
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)  #round函数四舍五入

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)  #乘以x_width而不是y_width

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x) #node.x max_x x_width是index ，min_x是实际值
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False              

        # collision check
        if self.obstacle_map[node.x][node.y]:     #这个判断不包含上面四个判断 超出范围很多这个检测不到
            return False

        return True
n.cost
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)
n.cost
        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]         #第一个定义行数，即y，第二个定义列数，即x
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)


    # print(__file__+"start!!!")
    # sx = 80
    # sy = 30
    # gx = 80
    # gy = 60
    # grid_size = 2
    # robot_radius =1

    # ox,oy = [],[]
    # for i in range (0,101):
    #     ox.append(i)
    #     oy.append(0)
    # for i in range (0,101):
    #     ox.append(i)
    #     oy.append(100)
    # for i in range (0,101):
    #     ox.append(0)
    #     oy.append(i)
    # for i in range (0,101):
    #     ox.append(100)
    #     oy.append(i)
    # for i in range (20,101):
    #     ox.append(i)
    #     oy.append(50)
    # for i in range (50,81):
    #     ox.append(60)
    #     oy.append(i)


    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")  #k为黑色
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")  #x,y轴刻度的分辨率一样

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        # plt.pause(0.001)  #这里的暂停效果不好，直接显示结果
        plt.show()  #显示所有打开的图形


if __name__ == '__main__':
    main()
