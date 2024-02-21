import matplotlib.pyplot as plt
import random
import math
import copy
import rospy
import time
import numpy as np
from numpy import genfromtxt
from numpy import array
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

show_animation = True

class RRT():
    """
    Class for RRT Planning
    """
    
    def __init__(self, start, goal, obstacles,
                 randArea, expandDis=1.0, goalSampleRate=5, maxIter = 500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacles:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacles = obstacles

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacles):
                continue

            self.nodeList.append(newNode)
            print("nNodelist:", len(self.nodeList))

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacles:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-10, 10, -10, 10])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacles):

        for (ox, oy, size) in obstacles:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
	
	f = open("obstacles.txt", "r")
	obstacles = []
	for i in range(8):	
		x 	= int(f.readline())
		y 	= int(f.readline())
		cap = int(f.readline())
		obstacles.append((int(x), int(y), int(cap)))
	f.close()
    # Set Initial parameters
	rrt = RRT(start=[0, 0], goal = [3, 4], randArea = [-15, 15], obstacles = obstacles)
	path = rrt.Planning(animation = show_animation)

	# Draw final path
	if show_animation:
		rrt.DrawGraph()
		plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
		plt.grid(True)
		plt.show()
		
if __name__ == '__main__':
    
    rospy.init_node('turtlebot3')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
     
    f = open("obstacles.txt", "r")
    obstacles = []
    for i in range(8):    
        x  = int(f.readline())
        y  = int(f.readline())
        cap = int(f.readline())
        obstacles.append((int(x), int(y), int(cap)))
              
    f.close()
    # Set Initial parameters
    obstacles = array(obstacles)
    rrt = RRT(start=[0, 0], goal = [3, 4], randArea = [-15, 15], obstacles = obstacles)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()

    rate = rospy.Rate(1)
    while True:
        twist.linear.x = 0.7
        twist.angular.z = 0.3
        pub.publish(twist)
        rate.sleep()
    
    
