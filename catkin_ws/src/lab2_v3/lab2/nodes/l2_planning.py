#!/usr/bin/env python3
#Standard Libraries
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import disk
from scipy.linalg import block_diag
from scipy.spatial import KDTree
import matplotlib.pyplot as plt



def load_map(filename):
    im = mpimg.imread("../maps/" + filename)
    if len(im.shape) > 2:
        im = im[:,:,0]
    im_np = np.array(im)  #Whitespace is true, black is false
    #im_np = np.logical_not(im_np)    
    return im_np


def load_map_yaml(filename):
    with open("../maps/" + filename, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict

#Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost):
        self.point = point # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost # The cost to come to this node
        self.children_ids = [] # The children node ids of this node
        return

#Path Planner 
class PathPlanner:
    #A path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_point, stopping_dist):
        #Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        #Get the metric bounds of the map
        self.bounds = np.zeros([2,2]) #m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[1] * self.map_settings_dict["resolution"]
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[0] * self.map_settings_dict["resolution"]

        #Robot information
        self.robot_radius = 0.22 #m
        self.vel_max = 0.5 #m/s (Feel free to change!)
        self.rot_vel_max = 0.2 #rad/s (Feel free to change!)

        #Goal Parameters
        self.goal_point = goal_point #m
        self.stopping_dist = stopping_dist #m

        #Trajectory Simulation Parameters
        self.timestep = 1.0 #s
        self.num_substeps = 10

        #Planning storage
        self.nodes = [Node(np.zeros((3,1)), -1, 0)]

        #RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] **2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1/2) ** (1/2) * (self.lebesgue_free / self.zeta_d) ** (1/2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5
        
        #Pygame window for visualization
        self.window = pygame_utils.PygameWindow(
            "Path Planner", (1000, 1000), self.occupancy_map.shape, self.map_settings_dict, self.goal_point, self.stopping_dist, map_filename)
        return

    #Functions required for RRT
    def sample_map_space(self):
        #Return an [x,y] coordinate to drive the robot towards
        x = int(np.random.uniform(self.bounds[0, 0], self.bounds[0, 1]))
        y = int(np.random.uniform(self.bounds[1, 0], self.bounds[1, 1]))
        
        return x, y
        # lavalle sampling
        # x_prime = int(np.random.normal(x, 0.1))
        # y_prime = int(np.random.normal(y, 0.1))

        # pt1 = self.point_to_cell(np.array([[x], [y]]))
        # pt2 = self.point_to_cell(np.array([[x_prime], [y_prime]]))

        # if self.occupancy_map[pt1[0, 0], pt1[0, 1]] and not self.occupancy_map[pt2[0, 0], pt2[0, 1]]:
        #     return x, y
        # elif not self.occupancy_map[pt1[0, 0], pt1[0, 1]] and self.occupancy_map[pt2[0, 0], pt2[0, 1]]:
        #     return x_prime, y_prime
        # else:
        #     return False
    
    def check_if_duplicate(self, point):
        #Check if point is a duplicate of an already existing node
        
        for node in self.nodes:
            if (node.point[0:2] == point).all():
                return True
        return False
    
    def closest_node(self, point):
        #Returns the index of the closest node
        
        # points = np.array([node.point[0:2].flatten()  for node in self.nodes])
        # kd_tree = KDTree(points)
        # _, index = kd_tree.query(point)
        min_dist = np.inf
        index = -1
        for i, node in enumerate(self.nodes):
            dist = np.linalg.norm(point - node.point[0:2])
            if dist < min_dist:
                min_dist = dist
                index = i
        return index
    
    def simulate_trajectory(self, node_i, point_s):
        #Simulates the non-holonomic motion of the robot.
        #This function drives the robot from node_i towards point_s. This function does has many solutions!
        #node_i is a 3 by 1 vector [x;y;theta] this can be used to construct the SE(2) matrix T_{OI} in course notation
        #point_s is the sampled point vector [x; y]
        
        vel, rot_vel = self.robot_controller(node_i, point_s)

        robot_traj = self.trajectory_rollout(vel, rot_vel, node_i, point_s)
        return robot_traj
    
    def robot_controller(self, node_i, point_s):
        #This controller determines the velocities that will nominally move the robot from node i to node s
        #Max velocities should be enforced
        
        vel, rot_vel = 0, 0
        k1 = 0.005
        k2 = 0.07
        # k1 = 0.5
        # k2 = 0.5
        vel = np.linalg.norm(k1 * (point_s - node_i[0:2])).item()
        rot_vel = (k2 * (np.arctan2(point_s[1] - node_i[1], point_s[0] - node_i[0]) - node_i[2])).item()
        if vel > self.vel_max:
            vel = self.vel_max
        elif vel < -self.vel_max:
            vel = -self.vel_max
        if rot_vel > self.rot_vel_max:
            rot_vel = self.rot_vel_max
        elif rot_vel < -self.rot_vel_max:
            rot_vel = -self.rot_vel_max
        # print(f"vel: {vel}, rot_vel: {rot_vel}")
        return vel, rot_vel
    
    def trajectory_rollout(self, vel, rot_vel, start_point, end_point):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions
        
        traj = np.zeros((3, self.num_substeps))
        traj[:,0] = start_point.flatten()
        for i in range(1, self.num_substeps):
            A = np.array([[np.cos(traj[2, i-1]), 0], 
                        [np.sin(traj[2, i-1]), 0], 
                        [0, 1]])
            q_dot = A @ np.array([vel, rot_vel])
            traj[:,i] = traj[:,i-1] + self.timestep * q_dot
            if np.linalg.norm(traj[0:2] - end_point) < self.stopping_dist:
                break

        return traj
    
    def point_to_cell(self, point):
        #Convert a series of [x,y] points in the map to the indices for the corresponding cell in the occupancy map
        #point is a 2 by N matrix of points of interest

        [x_0,y_0,theta] = self.map_settings_dict["origin"]

        #rotate frame
        num_points = point.shape[1]
        resolution = self.map_settings_dict["resolution"]
        x_coords = np.array(point[0,:] - x_0*np.ones((1,num_points)))/resolution
        # y_coords = np.array(y_0*np.ones((1,num_points)) - point[1,:])/resolution + self.map_shape[1]
        y_coords = np.array(point[1,:] - y_0*np.ones((1,num_points)))/resolution
        x_coords = np.floor(x_coords).astype(int)
        y_coords = np.floor(y_coords).astype(int)

        return np.column_stack((x_coords.T, y_coords.T))

    # def points_to_robot_circle(self, points):
    #     #Convert a series of [x,y] points to robot map footprints for collision detection
    #     #Hint: The disk function is included to help you with this function
    #     # print(points)
    #     x = []
    #     y = []
    #     robo_points = self.point_to_cell(points)
    #     for robo_point in robo_points:
    #         rr, cc = disk(robo_point, self.robot_radius/self.map_settings_dict["resolution"])
    #         x.extend(rr)
    #         y.extend(cc)

    #     return np.column_stack((x, y))
    
    def points_to_robot_circle(self, points):
        # Convert a series of [x,y] points to robot map footprints for collision detection
        # Hint: The disk function is included to help you with this function
        # print("POINTS TO ROBOT CIRCLE: get the pixel locations of the robot path")

        # get the converted points, it's around these points we'll place the circles
        converted_points = self.point_to_cell(points)

        # extract from self the relevant parameters
        res = self.map_settings_dict["resolution"]
        scaled_rad = self.robot_radius / res

        # for each of the converted points, find the circle around them
        rows, cols = [], []
        for i in range(converted_points.shape[1]):
            centre = converted_points[0][i], converted_points[1][i]
            rr, cc = disk(centre, scaled_rad)
            rows.append(rr)
            cols.append(cc)

        # this now holds all potentially occupied points
        # ** there could be points outside of the 1600x1600 area!
        included_points = np.vstack((np.concatenate(rows), np.concatenate(cols))).T

        # plot_points_felicia(included_points, self.occupancy_map)

        return included_points
    

    #RRT* specific functions
    def ball_radius(self):
        #Close neighbor distance
        card_V = len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V ) ** (1.0/2.0), self.epsilon)
    
    def connect_node_to_point(self, node_i, point_f):
        #Given two nodes find the non-holonomic path that connects them
        #Settings
        #node is a 3 by 1 node
        #point is a 2 by 1 point
        print("TO DO: Implement a way to connect two already existing nodes (for rewiring).")
        x = np.linspace(node_i[0], point_f[0], self.num_substeps)
        y = np.linspace(node_i[1], point_f[1], self.num_substeps)
        theta = np.zeros(self.num_substeps)
        theta[0] = node_i[2]
        for i in range(1, self.num_substeps):
            theta[i] = np.arctan2(y[i] - y[i-1], x[i] - x[i-1])

        return np.vstack((x, y, theta))
    
    def cost_to_come(self, trajectory_o):
        #The cost to get to a node from lavalle 
        print("TO DO: Implement a cost to come metric")
        return 0
    
    def update_children(self, node_id):
        #Given a node_id with a changed cost, update all connected nodes with the new cost
        print("TO DO: Update the costs of connected nodes after rewiring.")
        
        return
    
    def collision_check(self, robot_occupancy):
        # print(self.occupancy_map.shape)
        for robot_point in robot_occupancy:
            x, y = robot_point
            
            if x < 0 or x >= self.map_shape[1] or y < 0 or y >= self.map_shape[0]:
                return True
            
            if not self.occupancy_map[y, x]:
                return True
        
        return False

    #Planner Functions
    def rrt_planning(self):
        #This function performs RRT on the given map and robot
        #You do not need to demonstrate this function to the TAs, but it is left in for you to check your work
        # plt.figure(figsize=(10, 10))

        # Display the map as a background
        plt.imshow(self.occupancy_map, cmap='gray_r', 
                extent=[self.bounds[0, 0], self.bounds[0, 1], self.bounds[1, 0], self.bounds[1, 1]],
                origin='lower')

        plt.plot(self.nodes[0].point[0], self.nodes[0].point[1], 'rx', markersize=10)  # Start (red x)
        plt.plot(self.goal_point[0], self.goal_point[1], 'gx', markersize=10)  # Goal (green x)
        # print(self.map_shape)
        for i in range(10000): #Most likely need more iterations than this to complete the map!
            #Sample map space
            point = self.sample_map_space()
            
            # print(point)
            if not self.check_if_duplicate(point):
                #Get the closest point
                closest_node_id = self.closest_node(point)
                # print(closest_node_id)
                
                #Simulate driving the robot towards the closest point
                trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)
                
                plt.scatter(trajectory_o[0, -1], trajectory_o[1, -1], c='g', s=5)
                #Check for collisions
                robot_occupancy = self.points_to_robot_circle(trajectory_o)
                
                # if not self.collision_check(robot_occupancy):
                if True:
                    # print('enter')
                    # new_node = Node(trajectory_o[:, -1].reshape(3,1), closest_node_id, 0)
                    new_node = Node(trajectory_o[:, -1].reshape(3,1), closest_node_id, 0)
                    self.nodes.append(new_node)               
                
                    # **Plot the new node and edge**
                    plt.plot([self.nodes[closest_node_id].point[0], new_node.point[0]], 
                        [self.nodes[closest_node_id].point[1], new_node.point[1]], 'b-', alpha=0.5, linewidth=0.5)  # Edge (blue line)
                    # print(new_node.point[0], new_node.point[1])
                    plt.scatter(new_node.point[0], new_node.point[1], c='r', s=5)  # Node (red dot)
                
                    # Update plot dynamically
                    if i % 100 == 0:  # Update every 100 iterations for performance
                        # plt.draw()
                        plt.pause(0.01)

                    #Check if goal has been reached
                    if np.linalg.norm(new_node.point[0:2] - self.goal_point) <= self.stopping_dist:
                        print("Goal reached!")
                        break
            

        return self.nodes
    
    def rrt_star_planning(self):
        #This function performs RRT* for the given map and robot        
        for i in range(1): #Most likely need more iterations than this to complete the map!
            #Sample
            point = self.sample_map_space()

            #Closest Node
            closest_node_id = self.closest_node(point)

            #Simulate trajectory
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            #Check for Collision
            print("TO DO: Check for collision.")

            #Last node rewire
            print("TO DO: Last node rewiring")

            #Close node rewire
            print("TO DO: Near point rewiring")

            #Check for early end
            print("TO DO: Check for early end")
        return self.nodes
    
    def recover_path(self, node_id = -1):
        path = [self.nodes[node_id].point]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            path.append(self.nodes[current_node_id].point)
            current_node_id = self.nodes[current_node_id].parent_id
        path.reverse()
        return path

def main():
    #Set map information
    # map_filename = "willowgarageworld_05res.png"
    # map_setings_filename = "willowgarageworld_05res.yaml"

    map_filename = "myhal.png"
    map_setings_filename = "myhal.yaml"
   
    #robot information
    # goal_point = np.array([[42], [-44]]) # m goal for willowgarage
    goal_point = np.array([[7], [0]]) # m goal point for myhal

    stopping_dist = 0.5 #m

    #RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist)
    # nodes = path_planner.rrt_star_planning()
    nodes = path_planner.rrt_planning()
    node_path_metric = np.hstack(path_planner.recover_path())

    #Leftover test functions
    np.save("shortest_path.npy", node_path_metric)


if __name__ == '__main__':
    main()
