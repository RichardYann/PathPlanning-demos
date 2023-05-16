"""

Hybrid A* path planning

author: Zheng Zh (@Zhengzh)

"""

import heapq
import math
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

sys.path.append(os.path.dirname(os.path.abspath(__file__))  
                + "/../ReedsSheppPath")   #添加路径  os.path.abspath(__file__)当前py文件的绝对路径
try:
    from dynamic_programming_heuristic import calc_distance_heuristic
    import reeds_shepp_path_planning as rs
    from car import move, check_car_collision, MAX_STEER, WB, plot_car
except Exception:
    raise

XY_GRID_RESOLUTION = 2.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]  yaw偏航 以15度为单位旋转 
MOTION_RESOLUTION = 0.1  # [m] path interpolate resolution
N_STEER = 20  # number of steer command
VR = 1.0  # robot radius

SB_COST = 100.0  # switch back penalty cost
BACK_COST = 5.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle change penalty cost
H_COST = 5.0  # Heuristic cost

show_animation = True


class Node:

    def __init__(self, x_ind, y_ind, yaw_ind, direction,
                 x_list, y_list, yaw_list, directions,
                 steer=0.0, parent_index=None, cost=None):
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind  #yaw感觉是朝向 steer感觉是方向变化（偏航）
        self.direction = direction
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.directions = directions
        self.steer = steer
        self.parent_index = parent_index
        self.cost = cost


class Path:

    def __init__(self, x_list, y_list, yaw_list, direction_list, cost):
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.direction_list = direction_list
        self.cost = cost


class Config:

    def __init__(self, ox, oy, xy_resolution, yaw_resolution):
        min_x_m = min(ox)
        min_y_m = min(oy)
        max_x_m = max(ox)
        max_y_m = max(oy)

        ox.append(min_x_m)     #所以下标取-1就是取ox最大值，-2就是最小值（非index 实际值）
        oy.append(min_y_m)
        ox.append(max_x_m)
        oy.append(max_y_m)

        self.min_x = round(min_x_m / xy_resolution)  #index
        self.min_y = round(min_y_m / xy_resolution)
        self.max_x = round(max_x_m / xy_resolution)
        self.max_y = round(max_y_m / xy_resolution)

        self.x_w = round(self.max_x - self.min_x)  #index
        self.y_w = round(self.max_y - self.min_y)

        self.min_yaw = round(- math.pi / yaw_resolution) - 1   #yaw到底是什么  是index  ？？？
        self.max_yaw = round(math.pi / yaw_resolution)         #index!
        self.yaw_w = round(self.max_yaw - self.min_yaw)


def calc_motion_inputs():
    for steer in np.concatenate((np.linspace(-MAX_STEER, MAX_STEER,   #拼接作用 传入的参数必须是一个多个数组的元组或者列表 另外需要指定拼接的方向，默认是 axis = 0
                                             N_STEER), [0.0])):       #通过定义均匀间隔创建数值序列。其实，需要指定间隔起始点、终止端，以及指定分隔值总数（包括起始点和终止点）；最终函数返回间隔类均匀分布的数值序列。
        for d in [1, -1]:                                             #MAX_STEER = 0.6  # [rad] maximum steering angle  N_STEER=20 将左右最大转向范围均分成20份  d应该指的是方向
            yield [steer, d]
                                                                      #如果要返回的数据是通过for等循环生成的迭代器类型数据（如列表、元组），return只能在循环外部一次性地返回，yeild则可以在循环内部逐个元素返回。

def get_neighbors(current, config, ox, oy, kd_tree):               #kd——tree？？？
    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config, ox, oy, kd_tree)
        if node and verify_index(node, config):
            yield node


def calc_next_node(current, steer, direction, config, ox, oy, kd_tree):    #steer应该是方向转变
    x, y, yaw = current.x_list[-1], current.y_list[-1], current.yaw_list[-1]  #-1 列表的最后一个元素 路径中最后一个点的信息 下面x，y list会不断append

    arc_l = XY_GRID_RESOLUTION * 1.5   #步长？ 3
    x_list, y_list, yaw_list = [], [], []
    for _ in np.arange(0, arc_l, MOTION_RESOLUTION):
        x, y, yaw = move(x, y, yaw, MOTION_RESOLUTION * direction, steer)  #yaw为偏航角度 distance = MOTION_RESOLUTION * direction  MOTION_RESOLUTION = 0.1  # [m] path interpolate resolution路径插值分辨率
        x_list.append(x)                                                   #direction为1或-1表示方向
        y_list.append(y)                    #这里在进行结点间的路径插值 难道也就是steering actions？ yes 为什么满足运动学约束？
        yaw_list.append(yaw)

    if not check_car_collision(x_list, y_list, yaw_list, ox, oy, kd_tree):
        return None

    d = direction == 1  #先进行判断 d为bool
    x_ind = round(x / XY_GRID_RESOLUTION)
    y_ind = round(y / XY_GRID_RESOLUTION)
    yaw_ind = round(yaw / YAW_GRID_RESOLUTION)

    added_cost = 0.0

    # SB_COST = 100.0  # switch back penalty cost
    # BACK_COST = 5.0  # backward penalty cost
    # STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
    # STEER_COST = 1.0  # steer angle change penalty cost
    # H_COST = 5.0  # Heuristic cost

    if d != current.direction:
        added_cost += SB_COST

    # steer penalty
    added_cost += STEER_COST * abs(steer)  #鼓励走直线

    # steer change penalty
    added_cost += STEER_CHANGE_COST * abs(current.steer - steer) #steer为方向  #鼓励不转向

    cost = current.cost + added_cost + arc_l  #arc_l是什么 是结点扩张的步长   每次设置扩张结点时cost初始化就会有不考虑障碍物的非完整性cost（好像没加上back cost） 后面再加上A*的运动学损失

    node = Node(x_ind, y_ind, yaw_ind, d, x_list,
                y_list, yaw_list, [d],
                parent_index=calc_index(current, config),
                cost=cost, steer=steer)

    return node


def is_same_grid(n1, n2):
    if n1.x_index == n2.x_index \
            and n1.y_index == n2.y_index \
            and n1.yaw_index == n2.yaw_index:
        return True
    return False


def analytic_expansion(current, goal, ox, oy, kd_tree):  #使用RS曲线进行解析扩张 加速搜索 路径与目标终点重合
    start_x = current.x_list[-1]
    start_y = current.y_list[-1]
    start_yaw = current.yaw_list[-1]  #-1还是不理解

    goal_x = goal.x_list[-1]
    goal_y = goal.y_list[-1]
    goal_yaw = goal.yaw_list[-1]

    max_curvature = math.tan(MAX_STEER) / WB   #最大弯曲度 MAX_STEER = 0.6  # [rad] maximum steering angle， WB = 3.  # rear to front wheel 
    paths = rs.calc_paths(start_x, start_y, start_yaw,
                          goal_x, goal_y, goal_yaw,
                          max_curvature, step_size=MOTION_RESOLUTION)  #计算reeds sheep曲线

    if not paths:
        return None

    best_path, best = None, None

    for path in paths:
        if check_car_collision(path.x, path.y, path.yaw, ox, oy, kd_tree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path   #只返回rs中最佳路径 损失最小

    return best_path


def update_node_with_analytic_expansion(current, goal,
                                        c, ox, oy, kd_tree):
    path = analytic_expansion(current, goal, ox, oy, kd_tree)

    if path:
        if show_animation:
            plt.plot(path.x, path.y)
        f_x = path.x[1:]
        f_y = path.y[1:]
        f_yaw = path.yaw[1:]

        f_cost = current.cost + calc_rs_path_cost(path) ###
        f_parent_index = calc_index(current, c)

        fd = []
        for d in path.directions[1:]:
            fd.append(d >= 0)

        f_steer = 0.0   ###
        f_path = Node(current.x_index, current.y_index, current.yaw_index,
                      current.direction, f_x, f_y, f_yaw, fd,
                      cost=f_cost, parent_index=f_parent_index, steer=f_steer)
        return True, f_path    #返回的是包含路径列表的结点

    return False, None
    # SB_COST = 100.0  # switch back penalty cost
    # BACK_COST = 5.0  # backward penalty cost
    # STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
    # STEER_COST = 1.0  # steer angle change penalty cost
    # H_COST = 5.0  # Heuristic cost

def calc_rs_path_cost(reed_shepp_path):  #“考虑运动约束不考虑障碍物”的启发信息就是大名鼎鼎的Reeds-Shepp曲线的长度！！！第一个启发
    #使用第一种（“考虑约束不考虑障碍物”）是为了防止机器人从错误的方向到达目标，作者说实际试验中使用这种启发比不使用的效率提高了一个数量级，看来是必不可少了。使用第二种（“考虑障碍物不考虑约束”）是为了防止在断头路或者U型障碍物里浪费时间。
    # Reeds-Shepp曲线和A星得到的最优解这两个启发信息既然不依赖障碍物，那就可以提前把所有离散节点的启发信息一次性都算好存起来，然后在搜索过程中直接查询，这种用空间换时间的策略也是一种解决办法。启发信息必须满足一些要求，最重要的一个要求是，不能高估，可以低估。
    # 从两个低估里取最大值当然还是低估，所以仍然是合理的。
    # 这么定义的启发信息跟混合A星其实没什么关系（不依赖混合A星的状态）。也就是说，你可以用在其它搜索算法上（随便用不要钱），当然你要是找到更牛逼的启发信息也可以换成你自己的。
    # 先介绍它的明星父母：探索树方法和A星算法。探索树方法也是一个比较通用的方法，有好多变种，最有名的就是RRT（快速探索随机树）方法
    # 狭义的混合A星只包含搜索的过程，广义的混合A星把后处理也加上了。
    # 因为每次的扩展都会产生有限的子节点，同理子节点的子节点也是有限个的，所以直接对子节点应用A星搜索的代价计算方法和选择方法就可以了。所以混合A星使用栅格真正的目的是限制节点对空间的覆盖。（太妙了！！！）（所以他虽然使用了栅格地图，仍然是一种探索树扩展）
    # 　在百度的无人驾驶项目中的混合A*代码中，直接将扩展的距离设置成了栅格的对角线，如下。因为对角线是一个栅格能容纳的最大距离（仅对直线来说），这样便可以保证子节点与父节点不会占据一个栅格。（很妙 但感觉还有其他办法）
    cost = 0.0
    for length in reed_shepp_path.lengths:  #为什么有lengths
        if length >= 0:  # forward
            cost += length
        else:  # back
            cost += abs(length) * BACK_COST

    # switch back penalty
    for i in range(len(reed_shepp_path.lengths) - 1):
        # switch back
        if reed_shepp_path.lengths[i] * reed_shepp_path.lengths[i + 1] < 0.0:  #惩罚换向
            cost += SB_COST

    # steer penalty
    for course_type in reed_shepp_path.ctypes:    #reed shepp曲线的转向必然是最大转向（最小半径转向）  而进行结点扩张时分了很多转向角（前向20 后向20）
        if course_type != "S":  # curve
            cost += STEER_COST * abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    n_ctypes = len(reed_shepp_path.ctypes)
    u_list = [0.0] * n_ctypes
    for i in range(n_ctypes):
        if reed_shepp_path.ctypes[i] == "R":  #方向 运动方式
            u_list[i] = - MAX_STEER
        elif reed_shepp_path.ctypes[i] == "L":
            u_list[i] = MAX_STEER

    for i in range(len(reed_shepp_path.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(u_list[i + 1] - u_list[i])   #abs(u_list[i + 1] - u_list[i]只能为0 或 2 MAX_STEER

    return cost


def hybrid_a_star_planning(start, goal, ox, oy, xy_resolution, yaw_resolution):
    """
    start: start node
    goal: goal node
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xy_resolution: grid resolution [m]
    yaw_resolution: yaw angle resolution [rad]
    """

    start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    tox, toy = ox[:], oy[:]

    obstacle_kd_tree = cKDTree(np.vstack((tox, toy)).T)   #vertical stack

    config = Config(tox, toy, xy_resolution, yaw_resolution)

    start_node = Node(round(start[0] / xy_resolution),
                      round(start[1] / xy_resolution),
                      round(start[2] / yaw_resolution), True,
                      [start[0]], [start[1]], [start[2]], [True], cost=0)   #列表的初始值就是结点位姿信息 可以用-1取到
    goal_node = Node(round(goal[0] / xy_resolution),
                     round(goal[1] / xy_resolution),
                     round(goal[2] / yaw_resolution), True,
                     [goal[0]], [goal[1]], [goal[2]], [True])

    openList, closedList = {}, {}

    h_dp = calc_distance_heuristic(           #close list
        goal_node.x_list[-1], goal_node.y_list[-1],
        ox, oy, xy_resolution, VR)

    pq = []
    openList[calc_index(start_node, config)] = start_node
    heapq.heappush(pq, (calc_cost(start_node, h_dp, config),  #heap堆 优先队列让你能够以任意顺序添加对象，并随时（可能是在两次添加对象之间）找出（并删除）最小的元素。
                        calc_index(start_node, config)))
    final_path = None

    while True:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)   #删除并返回最小值，因为堆的特征是heap[0]永远是最小的元素，所以一般都是删除第一个元素。
        if c_id in openList:
            current = openList.pop(c_id)
            closedList[c_id] = current
        else:
            continue

        if show_animation:  # pragma: no cover
            plt.plot(current.x_list[-1], current.y_list[-1], "xc") #c青色   结点扩张的步长为3 所以每隔3距离一个点
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if len(closedList.keys()) % 1 == 0:
                plt.pause(0.1)

        is_updated, final_path = update_node_with_analytic_expansion(
            current, goal_node, config, ox, oy, obstacle_kd_tree)

        if is_updated:
            print("path found")
            break

        for neighbor in get_neighbors(current, config, ox, oy,
                                      obstacle_kd_tree):
            neighbor_index = calc_index(neighbor, config)
            if neighbor_index in closedList:
                continue
            if neighbor not in openList \
                    or openList[neighbor_index].cost > neighbor.cost:
                heapq.heappush(
                    pq, (calc_cost(neighbor, h_dp, config),   
                         neighbor_index))
                openList[neighbor_index] = neighbor

    path = get_final_path(closedList, final_path)
    return path


def calc_cost(n, h_dp, c):  #n neighbor c config  h_dp = calc_distance_heuristic 返回距离启发的closed_set
    ind = (n.y_index - c.min_y) * c.x_w + (n.x_index - c.min_x)
    if ind not in h_dp:
        return n.cost + 999999999  # collision cost
    return n.cost + H_COST * h_dp[ind].cost    # n.cost包含了初始化结点时设置的不考虑障碍物损失 加上运动学损失


def get_final_path(closed, goal_node):
    reversed_x, reversed_y, reversed_yaw = \
        list(reversed(goal_node.x_list)), list(reversed(goal_node.y_list)), \
        list(reversed(goal_node.yaw_list))
    direction = list(reversed(goal_node.directions))
    nid = goal_node.parent_index
    final_cost = goal_node.cost

    while nid:
        n = closed[nid]
        reversed_x.extend(list(reversed(n.x_list)))
        reversed_y.extend(list(reversed(n.y_list)))
        reversed_yaw.extend(list(reversed(n.yaw_list)))
        direction.extend(list(reversed(n.directions)))

        nid = n.parent_index

    reversed_x = list(reversed(reversed_x))
    reversed_y = list(reversed(reversed_y))
    reversed_yaw = list(reversed(reversed_yaw))
    direction = list(reversed(direction))

    # adjust first direction
    direction[0] = direction[1]

    path = Path(reversed_x, reversed_y, reversed_yaw, direction, final_cost)

    return path  #这里的路径是起点到终点的顺序


def verify_index(node, c):
    x_ind, y_ind = node.x_index, node.y_index
    if c.min_x <= x_ind <= c.max_x and c.min_y <= y_ind <= c.max_y:
        return True

    return False


def calc_index(node, c):
    ind = (node.yaw_index - c.min_yaw) * c.x_w * c.y_w + \
          (node.y_index - c.min_y) * c.x_w + (node.x_index - c.min_x)

    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind


def main():
    print("Start Hybrid A* planning")

    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    # Set Initial parameters
    start = [10.0, 10.0, np.deg2rad(90.0)]
    goal = [50.0, 50.0, np.deg2rad(-90.0)]

    print("start : ", start)
    print("goal : ", goal)

    if show_animation:
        plt.plot(ox, oy, ".k")
        rs.plot_arrow(start[0], start[1], start[2], fc='g')  #rs reeds sheep
        rs.plot_arrow(goal[0], goal[1], goal[2])

        plt.grid(True)
        plt.axis("equal")

    path = hybrid_a_star_planning(
        start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    x = path.x_list
    y = path.y_list
    yaw = path.yaw_list

    if show_animation:
        for i_x, i_y, i_yaw in zip(x, y, yaw):
            plt.cla()
            plt.plot(ox, oy, ".k")
            plt.plot(x, y, "-r", label="Hybrid A* path")
            plt.grid(True)
            plt.axis("equal")
            plot_car(i_x, i_y, i_yaw)
            # plt.pause(0.0001)
            plt.pause(0.0000001)

    print(__file__ + " done!!")


if __name__ == '__main__':
    main()
