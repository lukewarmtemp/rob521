#!/usr/bin/env python3

######################################################
# Standard Libraries
######################################################

import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import disk
from scipy.linalg import block_diag

import matplotlib.pyplot as plt
import math
from PIL import Image

# np.random.seed(19)
np.random.seed(18)
# np.random.seed(42)

markers = 1
line = 0.5

######################################################
# Visualisers
######################################################

def plot_trajectory_with_orientation(trajectory, point_s):
    """
    Plots the trajectory with (x, y) points and small lines indicating the orientation (theta).
    
    :param trajectory: A 3xN numpy array where each column is [x, y, theta] for each timestep.
    """

    print(point_s)
    # Loop through each timestep in the trajectory
    for i in range(trajectory.shape[1]):
        x, y, theta = trajectory[:, i]
        
        # Plot the (x, y) as a dot
        plt.plot(x, y, 'bo')  # 'bo' means blue color, circle marker
        
        # Calculate the end point of the orientation line
        line_length = 2  # Length of the line to represent orientation
        dx = line_length * np.cos(theta)
        dy = line_length * np.sin(theta)
        
        # Plot the orientation line
        plt.plot([x, x + dx], [y, y + dy], 'r-', lw=1)

    plt.scatter(point_s[0], point_s[1], c='r', marker='x') 

    # Set plot labels and display
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Trajectory with Orientation')
    plt.grid(True)
    plt.axis("equal")
    plt.show()

def plot_points_felicia(array, mapp):

    if np.any(array < 0) or np.any(array >= 1600):
        print("Array contains values outside the valid range.")
    else:
        print("All values are within the valid range.")
    
    print(array.shape)

    # display purposes
    plt.imshow(mapp)
    plt.scatter(array[0], array[1], c='r', marker='x')  
    plt.show()

def display_the_bounds(array, min_x, max_x, min_y, max_y):

    plt.imshow(array, cmap='gray', origin='upper')  # Show array as an image
    plt.colorbar(label="Values")

    # Plot horizontal boundary lines
    plt.axhline(y=min_y - 0.5, color='r', linestyle='--', label="Top Bound")
    plt.axhline(y=max_y + 0.5, color='b', linestyle='--', label="Bottom Bound")

    # Plot vertical boundary lines
    plt.axvline(x=min_x - 0.5, color='g', linestyle='--', label="Left Bound")
    plt.axvline(x=max_x + 0.5, color='y', linestyle='--', label="Right Bound")

    # Labels and display settings
    plt.legend()
    plt.xlabel("Y Axis (Columns)")
    plt.ylabel("X Axis (Rows)")
    plt.title("Array with Zero Bounds")
    plt.show()

def convert_png_to_pgm(input_path, output_path):
    # Open the PNG image and convert to grayscale (if not already)
    img = Image.open(input_path).convert("L")
    image_resized = img.resize((200, 200))
    image_resized.save("resized_image.png")

    # Convert to NumPy array
    img_array = np.array(image_resized)

    print(img_array.shape)

    # Threshold the image: set pixels to either 0 or 255
    img_array = np.where(img_array > 128, 255, 0).astype(np.uint8)

    # Save as PGM (Portable GrayMap)
    Image.fromarray(img_array).save(output_path, format="PPM")

def showing_nodes_and_background(array, nodes, closest_node, point, goal):
    plt.imshow(array, cmap='gray', origin='upper')  # Display background

    print("num nodes:", len(nodes))
    for node in nodes:
        x, y = node.point[:2]  # Extract x, y from the node's point attribute
        plt.plot(x, y, 'bo', markersize=markers)  # Plot each node as a red dot

        # Check if the node has a traj attribute and plot its points
        if hasattr(node, 'traj'):  # Ensure traj exists
            traj = node.traj  # Assuming traj is a 3xN array
            # Extract x, y from traj (ignoring theta for plotting)
            traj_x, traj_y = traj[0, :], traj[1, :]
            plt.plot(traj_x, traj_y, 'b-', linewidth=line)  # Connect trajectory points with a red line

        # Check if the node has an area attribute and fill it
        if hasattr(node, 'area'):  # Ensure area exists
            area = node.area  # Assuming area is a 2xN array of [x, y] points
            area_x, area_y = area[0, :], area[1, :]
            plt.plot(area_x, area_y, 'r.', alpha=0.01)

    # Highlight the closest node in green
    closest_x, closest_y = closest_node.point[:2]
    plt.plot(closest_x, closest_y, 'go', markersize=markers, label="Closest Node")  # Green dot

    # Highlight the given point in blue
    point_x, point_y = point
    plt.plot(point_x, point_y, 'mo', markersize=markers, label="Target Point")  # Blue dot

    point_x, point_y = goal.flatten()
    plt.plot(point_x, point_y, 'ko', markersize=markers, label="Goal Point")  # Black dot

    # Add labels for better visualization
    plt.xticks([])
    plt.yticks([])
    # plt.legend(loc="best", fontsize=3)  # Add a legend to differentiate the points
    plt.show()

def showing_nodes_and_background_star(array, nodes, closest_node, point, goal):
    plt.imshow(array, cmap='gray', origin='upper')  # Display background

    print("num nodes:", len(nodes))
    for node in nodes:
        x, y = node.point[:2].flatten()  # Extract x, y from the node's point attribute
        parent_node = nodes[node.parent_id]
        parx, pary = parent_node.point[:2].flatten()
        
        # Plot the node as a blue dot
        plt.plot(x, y, 'bo', markersize=markers)

        # print(node.point[:2].shape, parent_node.point[:2].shape, node.parent_id, x, y, parx, pary)

        # Draw a line connecting the node to its parent
        if node.parent_id != -1:
            plt.plot([parx, x], [pary, y], 'b-', linewidth=line)  # Blue line with a thin width

        # # If the node has an 'area' attribute, plot its area
        # if hasattr(node, 'area'):  # Ensure area exists
        #     area = node.area  # Assuming area is a 2xN array of [x, y] points
        #     area_x, area_y = area[0, :], area[1, :]
        #     plt.plot(area_x, area_y, 'r.', alpha=0.01)

    # Highlight the closest node in green
    closest_x, closest_y = closest_node.point[:2]
    plt.plot(closest_x, closest_y, 'go', markersize=markers, label="Closest Node")  # Green dot

    # Highlight the given point in magenta
    point_x, point_y = point
    plt.plot(point_x, point_y, 'mo', markersize=markers, label="Target Point")  # Magenta dot

    # Highlight the goal in black
    goal_x, goal_y = goal.flatten()
    plt.plot(goal_x, goal_y, 'ko', markersize=markers, label="Goal Point")  # Black dot

    # Add labels for better visualization
    plt.xticks([])
    plt.yticks([])
    # plt.legend(loc="best", fontsize=8)  # Add a legend to differentiate the points
    plt.show()


def color_squares_on_grid(processed_set, color_value=0):

    # Create a copy of the array to avoid modifying the original
    array = np.ones((200, 200))*255
    modified_array = np.copy(array)
    
    for x, y in processed_set:
        # Check if the coordinates are within the bounds of the array
        if 0 <= x < modified_array.shape[1] and 0 <= y < modified_array.shape[0]:
            modified_array[y, x] = color_value  # Set the value to color the square
    
    plt.imshow(modified_array, cmap='gray', origin='upper')
    plt.show()

    return modified_array

def plot_path_with_background(background_image, path):
    # Load the background image
    if isinstance(background_image, str):
        bg_img = mpimg.imread(background_image)
    else:
        bg_img = background_image

    # Extract x, y points from the path
    x_points = path[0, :]
    y_points = path[1, :]

    # Create the plot
    plt.figure()
    plt.imshow(bg_img, cmap='gray', origin='upper')  # Display the background image
    plt.plot(x_points, y_points, 'b-o', markersize=markers, linewidth=line)  # Plot the path points
    
    # Add labels and title
    plt.xticks([])
    plt.yticks([])
    plt.show()


######################################################
# Map Loading Functions
######################################################

def load_map(filename):
    import os
    print(os.getcwd())
    print(filename)
    im = mpimg.imread("../maps/" + filename)
    if len(im.shape) > 2:
        im = im[:,:,0]
    im_np = np.array(im) 
    return im_np

def load_map_yaml(filename):
    with open("../maps/" + filename, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict


######################################################
# Class Functions
######################################################

class Node:
    def __init__(self, point, parent_id, cost):
        self.point = point                  # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id          # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost                    # The cost to come to this node
        self.children_ids = []          # The children node ids of this node
        self.traj = np.zeros((3, 1))    # Trajectory to get there from parent
        self.area = np.zeros((2, 1))    # Holds colours of its surrounding neighbours
        return




class PathPlanner:
    
    # a path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_pix, first_node, stopping_dist):

        # Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape
        self.map_settings_dict = load_map_yaml(map_setings_filename)
        # Get the metric bounds of the map
        self.bounds = np.zeros([2,2]) #m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[1] * self.map_settings_dict["resolution"]
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[0] * self.map_settings_dict["resolution"]

        # Robot information
        self.robot_radius = 0.22    #m
        self.vel_max = 0.5          #m/s (Feel free to change!)
        self.rot_vel_max = 0.2      #rad/s (Feel free to change!)
        self.pix_vel = 3    
        self.pix_rot_vel = 0.1
        self.scaled_rad = self.robot_radius / self.map_settings_dict["resolution"]

        # Goal Parameters
        self.goal_pix = goal_pix
        self.stopping_dist = stopping_dist  #m

        # Trajectory Simulation Parameters
        self.timestep = 1.0                 #s
        self.num_substeps = 10

        # Planning storage
        self.nodes = [first_node]
        self.processed = set()

        # RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] **2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1/2) ** (1/2) * (self.lebesgue_free / self.zeta_d) ** (1/2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5
        
        # Bound the image depending on the image and choose ratios
        if "willowgarageworld" in map_filename:
            zero_indices = np.argwhere(self.occupancy_map == 0)
            self.min_y, self.min_x = zero_indices.min(axis=0)  # first 0
            self.max_y, self.max_x = zero_indices.max(axis=0)  # last 0
            self.ratios = [0.1, 0.15, 0.7] # (open, goal, gauss, bridge)
            self.variance = 15
            self.node_making = [4, 8]
            self.rad_scale = 2
        elif "myhal" in map_filename:
            self.min_y, self.min_x = 0, 0
            self.max_y, self.max_x = self.map_shape[0], self.map_shape[1]
            self.ratios = [0.3, 0.5, 1] # (open, goal, gauss, bridge)
            self.variance = 20
            self.node_making = [2, 3]
            self.rad_scale = 1
        else:
            self.min_y, self.min_x = 0, 0
            self.max_y, self.max_x = self.map_shape[0], self.map_shape[1]
            self.ratios = [0.3, 0.5, 1] # (open, goal, gauss, bridge)
            self.variance = 20
            self.node_making = [4, 8]
            self.rad_scale = 2

        # Pygame window for visualization
        # self.window = pygame_utils.PygameWindow(
        #     "Path Planner", (500, 500), self.occupancy_map.shape, self.map_settings_dict, self.goal_point, self.stopping_dist)
        
        return

    # converts (2, N) points from real world to matrix integer form
    def point_to_cell(self, points):
        # extract from the yaml map the origin and resolution
        [x_0, y_0, _] = self.map_settings_dict["origin"]
        res = self.map_settings_dict["resolution"]
        y_size = self.map_shape[1]
        # for each point in the array, get rid of the offset and change scaling
        converted_points = np.zeros(points.shape)
        converted_points[0] = ((points[0] - x_0) / res).astype(int)
        converted_points[1] = ((y_0 - points[1]) / res + y_size).astype(int)
        # return the array version of the points
        return converted_points

    # Converts matrix integer form (2, N) points back to real-world coordinates
    def cell_to_point(self, cells):
        # extract the origin and resolution from the map settings
        [x_0, y_0, _] = self.map_settings_dict["origin"]
        res = self.map_settings_dict["resolution"]
        y_size = self.map_shape[1]
        # For each cell in the array, reverse the scaling and offset transformations
        converted_points = np.zeros(cells.shape)
        converted_points[0] = cells[0] * res + x_0
        converted_points[1] = (y_size - cells[1]) * res + y_0
        # Return the array version of the points
        return converted_points

    # finds the surrounding robot areas for (2, N) points (perform map or pre-mapped)
    def points_to_robot_circle(self, points, scaled_rad):
        # get the converted points, it's around these points we'll place the circles (or use given ones)
        # converted_points = self.point_to_cell(points)
        converted_points = points.reshape((2, 1))
        # for each of the converted points, find the circle around them
        rows, cols = [], []
        for i in range(converted_points.shape[1]):
            centre = converted_points[0][i], converted_points[1][i]
            rr, cc = disk(centre, scaled_rad)
            rows.append(rr)
            cols.append(cc)
        # this now holds all potentially occupied points
        # ** there could be points outside of the 1600x1600 area!
        included_points = np.vstack((np.concatenate(rows), np.concatenate(cols)))
        return included_points

    # first performs all turning steps, then all walking steps, returns floats, unicycle model
    def trajectory_rollout(self, vel, rot_vel, start_point, num_steps = None, timestep = None, stopping_dist = None):
        # create a blank array for the trajectory start and load in the first node
        if num_steps == None:
            num_steps = self.num_substeps
        if timestep == None:
            timestep = self.timestep
        if stopping_dist == None:
            stopping_dist = self.stopping_dist
        traj = np.zeros((3, num_steps))
        traj[:,0] = start_point.flatten()
        for i in range(1, num_steps):
            A = np.array([[np.cos(traj[2, i-1]), 0], 
                        [np.sin(traj[2, i-1]), 0], 
                        [0, 1]])
            q_dot = A @ np.array([vel, rot_vel])
            traj[:,i] = traj[:,i-1] + timestep * q_dot
            if traj[2,i] > np.pi:
                traj[2,i] -= 2 * np.pi
            if traj[2,i] < -np.pi:
                traj[2,i] += 2 * np.pi
        return traj[:, 1:]
    
    # def trajectory_rollout(self, vel, rot_vel, start_point, end_point, num_steps = None, timestep = None, stopping_dist = None):
    #     trajectory = np.zeros((3, num_steps+1))
    #     cur_pose = start_point
    #     trajectory[:,0] = cur_pose
    #     del_time = timestep / num_steps
    #     for steps in range(num_steps):
    #         x_dot = vel * np.cos(trajectory[2,steps])
    #         y_dot = vel * np.sin(trajectory[2,steps])
    #         theta_dot = rot_vel
    #         trajectory[:,steps+1] = trajectory[:,steps] + np.array([x_dot, y_dot, theta_dot]) * del_time
    #     return trajectory[:,2:]

    # using a beginning (x, y, theta) and ending (x, y), determine walking steps required
    def robot_controller(self, node_i, point_s):
        # extract all the numbers out of the class inputs points and nodes 
        x_i, y_i, theta_i = node_i.point.flatten()
        x_s, y_s = point_s
        res = self.map_settings_dict["resolution"]
        vel_max, rot_vel_max = self.pix_vel, self.pix_rot_vel
        # find the required dist to travel and the angle that we would need to turn
        lin_req = np.sqrt((x_s - x_i)**2 + (y_s - y_i)**2)
        rot_req = np.arctan2(y_s-y_i, x_s-x_i) - theta_i
        # rotation could be negative, flip velocity if needed
        if rot_req < 0: rot_vel_max *= -1
        # using the ratios, calculate how we would need to prioritise moving in a balanced way
        time_lin = round(lin_req/vel_max)
        time_rot = round(np.abs(rot_req/rot_vel_max))
        return vel_max, rot_vel_max, time_lin, time_rot

    # simulates moving from node i to node s using a holonomic model
    def simulate_trajectory(self, node_i, point_s):
        # get the velocities
        vel_max, rot_vel_max, _, _ = self.robot_controller(node_i, point_s)
        # simulate trying to get to that point
        robot_traj = self.trajectory_rollout(vel_max, rot_vel_max, node_i.point)
        return robot_traj

    # samples a point from the world (slightly guided)
    def sample_map_space(self):
        # set the bounds of the x and y (can be adjusted)
        min_y, min_x, max_y, max_x = self.min_y, self.min_x, self.max_y, self.max_x
        p1, p2, p3 = self.ratios
        variance = self.variance

        def sample_on_map(min_y, min_x, max_y, max_x):
            sampled_point = np.zeros((2, 1))
            sampled_point[0, 0] = np.random.randint(min_x, max_x)
            sampled_point[1, 0] = np.random.randint(min_y, max_y)
            return sampled_point.astype(int)

        # sampling idea: 50% guassian non uniform, 30% from the open, 10% straight up the goal
        prob = np.random.random()
        # open sampling
        if prob < p1:
            while True:
                p = sample_on_map(min_y, min_x, max_y, max_x)
                if self.check_if_duplicate(p)==False and self.occupancy_map[p[1, 0], p[0, 0]] != 0: return p.astype(int)
        # goal point
        elif prob < p2: 
            return self.goal_pix.astype(int)
        # gaussian distribution-based sampling
        elif prob < p3: 
            while True:
                # sample a random point on the map
                p = sample_on_map(min_y, min_x, max_y, max_x)
                # gaussian-distributed point around the first point
                q = np.zeros((2, 1))
                q[0, 0] = np.random.normal(p[0, 0], variance)
                q[1, 0] = np.random.normal(p[1, 0], variance)
                q = q.astype(int)
                # for a second point in bounds
                if (min_x <= q[0, 0] < max_x and min_y <= q[1, 0] < max_y):
                    # get intensities on the image
                    p_int = self.occupancy_map[p[1, 0], p[0, 0]]
                    q_int = self.occupancy_map[q[1, 0], q[0, 0]]
                    # if only one of the points has non-zero intensity, return that one
                    if (p_int == 0 and q_int != 0 and self.check_if_duplicate(q)==False): return q 
                    if (p_int != 0 and q_int == 0 and self.check_if_duplicate(p)==False): return p
        # bridge distribution-based sampling
        else: 
            while True:
                # sample a random point on the map
                p = sample_on_map(min_y, min_x, max_y, max_x)
                # gaussian-distributed point around the first point
                q = np.zeros((2, 1))
                q[0, 0] = np.random.normal(p[0, 0], variance)
                q[1, 0] = np.random.normal(p[1, 0], variance)
                q = q.astype(int)
                # for a second point in bounds
                if (min_x <= q[0, 0] < max_x and min_y <= q[1, 0] < max_y):
                    # get intensities on the image
                    p_int = self.occupancy_map[p[1, 0], p[0, 0]]
                    q_int = self.occupancy_map[q[1, 0], q[0, 0]]
                    # if both points are in obs and their centre is free
                    if (p_int == 0 and q_int == 0): 
                        mid = (p + q) // 2
                        mid_int = self.occupancy_map[mid[1, 0], mid[0, 0]]
                        if (mid_int != 0 and self.check_if_duplicate(mid)==False): return mid

    # checks if this new point already exists as a node we've done before
    def check_if_duplicate(self, point):
        # compare each point to determine if there are duplicates
        new = tuple(point.astype(int).flatten())
        if new in self.processed: return True
        return False

    # sifting through the list, finds and returns index of closest node
    def closest_node(self, point):
        # O(n) runtime, go through the list and return the closest
        x0, y0 = point
        chosen_index, min_dist = 0, np.inf
        for i, node in enumerate(reversed(self.nodes)):
            x, y, theta = node.point
            dist = np.sqrt((x-x0)**2 + (y-y0)**2)
            if dist < min_dist:
                # since we are iterating in reverse, adjust the index
                chosen_index = len(self.nodes) - 1 - i
                min_dist = dist
        return chosen_index
    
    # from retracing parent nodes, finds the path
    def recover_path(self, node_id = -1):
        path = [self.nodes[node_id].point]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            add_node = self.nodes[current_node_id].point.reshape((3,))
            path.append(add_node)
            current_node_id = self.nodes[current_node_id].parent_id
        return np.array(path[::-1]).T

    ########################################################

    # for each point along a trajectory, we check the viability, if they all pass, then we're good!
    def collision_check(self, x, y, theta = None, input_map = np.array([]), scaled_rad = None):
        # if this point on the path goes off the page, we're done
        if input_map.all() == None or input_map.size == 0:
            input_map = self.occupancy_map
            map_shape = self.map_shape
        else:
            map_shape = input_map.shape

        if scaled_rad == None:
            scaled_rad = self.scaled_rad
        
        if not (0 <= x <= map_shape[1]-1 and 0 <= y <= map_shape[0]-1): 
            return True
        # if any of the surrounding points are off the edge, we're also done
        included_points = self.points_to_robot_circle(points=np.array([x, y]), scaled_rad=scaled_rad)
        out_of_range_x = np.any((included_points[0, :] < 0) | (included_points[0, :] >= map_shape[1]-1))
        out_of_range_y = np.any((included_points[1, :] < 0) | (included_points[1, :] >= map_shape[0]-1))
        out_of_range = out_of_range_x or out_of_range_y
        if out_of_range: 
            return True
        # if any of the surrounding points are in an obstacle, we're also done
        for mini_index in range(included_points.shape[1]):
            test_x, test_y = included_points[:, mini_index].flatten()
            if input_map[test_y, test_x] == 100: 
                print(test_x, test_y)
                return True
        # only if we make it here is the point safe to add as a next 
        return False

    # take one point, create a node for it and add all its attributes
    def adding_a_node(self, trajectory_o, prev_index, index, closest_node, closest_node_id, node_id):
        # extract the info to add
        new_pt = trajectory_o[:, index]
        new_node_included_area = self.points_to_robot_circle(np.array([new_pt[0], new_pt[1]]), self.scaled_rad*self.rad_scale)
        # create the new node and add its attributes
        new_node = Node(new_pt, closest_node_id, 0)
        new_node.traj = trajectory_o[:, prev_index:index]
        new_node.area = new_node_included_area
        new_node.cost = closest_node.cost + self.partial_cost_to_come(closest_node, new_pt[:2])
        print("adding new node! ID:", node_id, "prev_ID:", closest_node_id, "cost:", new_node.cost, new_pt.flatten())
        # to its predeccesor, add this child for path retracing
        closest_node.children_ids.append(node_id)
        # add new node to the list of nodes accounted for
        self.nodes.append(new_node)
        # handle adding the area for duplicates
        self.processed.add(tuple(new_pt[:2].astype(int)))
        for i in range(new_node_included_area.shape[1]):
            x, y = new_node_included_area[:, i]
            self.processed.add((int(x), int(y)))
            
        # if len(self.nodes) % 500 ==0:
        # showing_nodes_and_background(self.occupancy_map, self.nodes, closest_node, new_pt[:2], self.goal_pix[:2])
        return new_node

    # RRT - with some better sampling and collison checking using forward moving
    def rrt_planning(self):

        # gives the next avaiable id in the nodes list
        avail_id, iterations = 1, 0
        res = self.map_settings_dict["resolution"]
        start_here, how_often = self.node_making

        # showing_nodes_and_background(self.occupancy_map, self.nodes, Node(np.zeros((2,1)), 1, 1), np.zeros((2,1)), self.goal_pix[:2])

        # run the loop until convergence
        while True:
            iterations +=1 
            print("iter #", iterations)

            # sample map space
            point = self.sample_map_space()
            x_point, y_point = point.flatten()
            # get the closest point
            closest_node_id = self.closest_node(point)
            closest_node = self.nodes[closest_node_id]
            # simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(closest_node, point.flatten())

            # print("probelm node! ID:", avail_id, "prev_ID:", closest_node_id, self.nodes[closest_node_id].point.flatten(), point.flatten())

            # for each point in the trajectory we check for collisions
            can_add_point = True

            for index in range(trajectory_o.shape[1]):
                # extract along the trajectory
                x, y, theta = trajectory_o[:, index].flatten()
                did_collide = self.collision_check(int(x), int(y), theta)
                # if the point is safe we continue checking to the end, if at any point it breaks, we end
                if did_collide: can_add_point = False; break
                
            # no collisions detected means we can add the final node of the trajectory
            if can_add_point == True:
                
                prev_index, prev_id, last_node = 0, closest_node_id, closest_node
                for index in range(start_here, trajectory_o.shape[1], how_often):

                    new_node = self.adding_a_node(trajectory_o, prev_index, index, last_node, prev_id, avail_id) 
                    # increase the id for the next node
                    prev_index, prev_id, last_node = index, avail_id, new_node
                    avail_id += 1

                # check if this new node could be the goal!
                if index != trajectory_o.shape[1]:
                    new_node = self.adding_a_node(trajectory_o, prev_index, -1, last_node, prev_id, avail_id)
                    avail_id += 1
                
                goal_reached = np.linalg.norm(new_node.point[:2] - self.goal_pix[:2].flatten()) <= self.stopping_dist/res
                if goal_reached: print("goal reached!"); break

                # if iterations % 500 == 0:
                # if len(self.nodes) % 200 ==0:
                #     showing_nodes_and_background(self.occupancy_map, self.nodes, closest_node, point, self.goal_pix[:2])

        showing_nodes_and_background_star(self.occupancy_map, self.nodes, closest_node, point, self.goal_pix[:2])
        return self.nodes

    ########################################################

    # assuming it can be holonomic, it's very similar to the simulation
    def connect_node_to_point(self, node_i, point_f):
        return self.simulate_trajectory(node_i, point_f)
    
    # at each node the cost to come is the number of timesteps taken on the trajectory to here
    def partial_cost_to_come(self, node_i, point_f):
        vel_max, rot_vel_max, time_lin, time_rot = self.robot_controller(node_i, point_f)
        return time_lin
    
    # defines a ball in which nodes need to be reconsidered for rewiring and re-eval
    def ball_radius(self):
        card_V = 10 #len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V ) ** (1.0/2.0), self.epsilon)
    
    # recursively update all children nodes costs to be consistent and correct
    def update_children(self, node_id):
        # extract this updated cost for this node and its children
        updated_cost = self.nodes[node_id].cost
        children_node_ids = self.nodes[node_id].children_ids
        # check if we've reached the base case
        if len(children_node_ids) == 0: return
        # for each of its children, the new cost is the updated cost + it's own cost_to_come
        for child_node_id in children_node_ids:
            child_node = self.nodes[child_node_id]
            child_node.cost = updated_cost + self.partial_cost_to_come(self.nodes[node_id], child_node.point[:2])
            self.update_children(child_node_id)
        return

    # returns ids of nodes within the ball radius for the node of interest
    def find_nodes_inside_ball(self, node_point):
        # get all the points in the ball around the centre point
        radius = 500 #int(self.ball_radius() / self.map_settings_dict["resolution"])
        inside_points = self.points_to_robot_circle(node_point, radius)
        # put all the points in a set so its easy to search through
        points_set = set(zip(inside_points[0].astype(int), inside_points[1].astype(int)))
        # now look through all current nodes and check if they're in the set
        inside_node_ids = []
        # print("looking through # nodes", len(self.nodes))
        for other_id in range(len(self.nodes)):
            other_point = self.nodes[other_id].point[:2]
            other_tuple = tuple(other_point.flatten().astype(int))
            if other_tuple in points_set: 
                inside_node_ids.append(other_id)
        return inside_node_ids

    # runs rewiring for one node and reports other nodes that may need rewiring
    # find nodes of which this new node can be the parent and they become its children
    def rewiring_a_node(self, node_id):
        # get the ball around the node and extract ids of potential nodes
        # print("entering rewiring")
        node = self.nodes[node_id]
        new_parent_cost = node.cost
        inside_node_ids = self.find_nodes_inside_ball(node.point[:2])
        # if there are no points, no children to consider
        if len(inside_node_ids) == 0: return []
        # for each potential node, test to see if it could benefit
        tampered_node_ids = []

        # print("rewiring", node_id, inside_node_ids)
        for potential_child_id in inside_node_ids:
            # try connecting and find cost to come
            potential_child_node = self.nodes[potential_child_id]
            potential_child_point = potential_child_node.point[:2]
            partial_cost = self.partial_cost_to_come(node, potential_child_point.flatten())

            # print("potential", potential_child_id, new_parent_cost, partial_cost, potential_child_node.cost)
            # if this new cost to come is an improvment 
            if new_parent_cost+partial_cost < potential_child_node.cost:
                # now we need to check no collisions
                traj = self.connect_node_to_point(node, potential_child_point.flatten())
                # for each point in the trajectory we check for collisions
                can_add_point = True
                if traj.shape[1] == 0: can_add_point = False
                if can_add_point == True:
                    for index in range(traj.shape[1]):
                        # extract along the trajectory
                        x, y, theta = traj[:, index].flatten()
                        did_collide = self.collision_check(int(x), int(y), theta)
                        # if the point is safe we continue checking to the end, if at any point it breaks, we end
                        if did_collide: can_add_point = False; break
                if can_add_point == True:
                    # print("rewire!")
                    # input()
                    # update the child's costs and trajectory and its children
                    potential_child_node.cost = new_parent_cost+partial_cost
                    potential_child_node.traj = self.connect_node_to_point(node, potential_child_point.flatten())
                    self.update_children(potential_child_id)
                    # remove the previous parent thinking this is their child
                    og_parent_id = potential_child_node.parent_id
                    og_parent_node = self.nodes[og_parent_id]
                    og_parent_node.children_ids = [x for x in og_parent_node.children_ids if x != potential_child_id]
                    # add the new parent
                    potential_child_node.parent_id = node_id
                    # remember this node was rewired... need to check around this parent...abs
                    tampered_node_ids.append(og_parent_id)

                    # showing_nodes_and_background(self.occupancy_map, self.nodes, closest_node, new_pt[:2], self.goal_pix[:2])
        # input()
        return tampered_node_ids

    # take one point, create a node for it and add all its attributes
    def adding_a_lctc_node(self, trajectory_o, index, node_id):
        # extract the info to add
        new_pt = trajectory_o[:, index]
        # find it's closest
        closest_node_id = self.lowest_ctc_node(new_pt[:2])
        closest_node = self.nodes[closest_node_id]
        mini_traj = self.simulate_trajectory(closest_node, new_pt[:2].flatten())

        new_node_included_area = self.points_to_robot_circle(np.array([new_pt[0], new_pt[1]]), self.scaled_rad*self.rad_scale)
        # create the new node and add its attributes
        new_node = Node(new_pt, closest_node_id, 0)
        new_node.traj = mini_traj
        new_node.area = new_node_included_area
        new_node.cost = closest_node.cost + self.partial_cost_to_come(closest_node, new_pt[:2])
        # print("adding new node! ID:", node_id, "prev_ID:", closest_node_id, "cost:", new_node.cost, new_pt.flatten())
        # to its predeccesor, add this child for path retracing
        closest_node.children_ids.append(node_id)
        # add new node to the list of nodes accounted for
        self.nodes.append(new_node)
        # handle adding the area for duplicates
        self.processed.add(tuple(new_pt[:2].astype(int)))
        for i in range(new_node_included_area.shape[1]):
            x, y = new_node_included_area[:, i]
            self.processed.add((int(x), int(y)))
            
        # if len(self.nodes) % 200 == 0:
        #     showing_nodes_and_background_star(self.occupancy_map, self.nodes, closest_node, new_pt[:2], self.goal_pix[:2])

        return new_node

    # runs rewiring for one node and reports other nodes that may need rewiring
    def lowest_ctc_node(self, point):
        # get the ball around the node and extract ids of potential nodes
        inside_node_ids = self.find_nodes_inside_ball(point)
        # if there are no points, just default for RRT
        if len(inside_node_ids) == 0: 
            # print("triggered")
            return self.closest_node(point)
        # print("inside the ball!", inside_node_ids)
        # for each potential node, test to see if it could be a match
        lowest_ctc, best_rewire_node_id = np.inf, -2
        for potential_parent_id in inside_node_ids:
            # try connecting and find cost to come
            potential_parent_node = self.nodes[potential_parent_id]
            cost = potential_parent_node.cost + self.partial_cost_to_come(potential_parent_node, point.flatten())

            # print("parent", potential_parent_id, potential_parent_node.cost, cost)
            # store the potential lowest cost to come
            if cost < lowest_ctc:
                # now we need to check no collisions
                traj = self.connect_node_to_point(potential_parent_node, point.flatten())
                # for each point in the trajectory we check for collisions
                can_add_point = True
                if traj.shape[1] == 0: can_add_point = False
                if can_add_point == True:
                    for index in range(traj.shape[1]):
                        # extract along the trajectory
                        x, y, theta = traj[:, index].flatten()
                        did_collide = self.collision_check(int(x), int(y), theta)
                        # if the point is safe we continue checking to the end, if at any point it breaks, we end
                        if did_collide: can_add_point = False; break
                if can_add_point == True:
                    lowest_ctc, best_rewire_node_id = cost, potential_parent_id
        # after trying them all, maybe rewire nodes, fix cost and parent child connections
        if best_rewire_node_id == -2: 
            # print("WE HAVE A HUGE ERROR")
            # input()
            return self.closest_node(point)

        # print(best_rewire_node_id)
        return best_rewire_node_id

    def rewiring_consistency(self, node_id):
        # start a queue to check consistency
        rewiring_queue = [node_id]
        while len(rewiring_queue) != 0:
            # offload a node to check
            curr_id = rewiring_queue[0]
            rewiring_queue = rewiring_queue[1:]
            # rewire a node and extract neighbours that may need fixing
            tampered_with = self.rewiring_a_node(curr_id)
            # add them to the queue
            rewiring_queue += tampered_with
        return

    def rrt_star_planning(self):
      
        # gives the next avaiable id in the nodes list
        avail_id, iterations = 1, 0
        res = self.map_settings_dict["resolution"]
        start_here, how_often = self.node_making

        goal_id = None  # Initialize goal ID
        min_iterations = 100 
        # showing_nodes_and_background(self.occupancy_map, self.nodes, Node(np.zeros((2,1)), 1, 1), np.zeros((2,1)), self.goal_pix[:2])

        # run the loop until convergence
        while True:
            iterations +=1 
            print("iter #", iterations)

            # sample map space
            point = self.sample_map_space()
            x_point, y_point = point.flatten()
            # get the LOWEST COST TO COME point
            closest_node_id = self.closest_node(point)
            closest_node = self.nodes[closest_node_id]
            # simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(closest_node, point.flatten())

            # print("probelm node! ID:", avail_id, "prev_ID:", closest_node_id, self.nodes[closest_node_id].point.flatten(), point.flatten())

            # for each point in the trajectory we check for collisions
            can_add_point = True

            if trajectory_o.shape[1] == 0: continue

            for index in range(trajectory_o.shape[1]):
                # extract along the trajectory
                x, y, theta = trajectory_o[:, index].flatten()
                did_collide = self.collision_check(int(x), int(y), theta)
                # if the point is safe we continue checking to the end, if at any point it breaks, we end
                if did_collide: can_add_point = False; break
                
            # no collisions detected means we can add the final node of the trajectory
            if can_add_point == True:

                prev_index, prev_id, last_node = 0, closest_node_id, closest_node
                for index in range(start_here, trajectory_o.shape[1], how_often):

                    new_node = self.adding_a_lctc_node(trajectory_o, index, avail_id) 
                    self.rewiring_consistency(avail_id)
                    # increase the id for the next node
                    prev_index, prev_id, last_node = index, avail_id, new_node
                    avail_id += 1

                # check if this new node could be the goal!
                if index != trajectory_o.shape[1]:
                    new_node = self.adding_a_lctc_node(trajectory_o, -1, avail_id)
                    self.rewiring_consistency(avail_id)
                    avail_id += 1
                
                goal_reached = np.linalg.norm(new_node.point[:2] - self.goal_pix[:2].flatten()) <= self.stopping_dist/res
                # if goal_reached: print("goal reached!"); break

                if goal_reached:
                    goal_id = len(self.nodes)-1
                    print("Goal Reached! **************************")

            if goal_id is not None and iterations >= min_iterations:
            # if goal_id is not None:
                print("Enough Iterations!")
                break

            # if iterations % 100:
            #     break
                # if iterations % 500 == 0:
                # if len(self.nodes) % 200 ==0:
            # if iterations % 101:
            #     showing_nodes_and_background(self.occupancy_map, self.nodes, closest_node, point, self.goal_pix[:2])

        showing_nodes_and_background_star(self.occupancy_map, self.nodes, closest_node, point, self.goal_pix[:2])
        return self.nodes, goal_id

    

######################################################
# Main Function
######################################################

def main():

    # # map information (testing only)
    # map_filename = "simplest_map.png"
    # map_setings_filename = "simplest_map.yaml"
    # # robot information
    # goal_pix = np.array([[190], [190]])
    # first_node = Node(np.array([[10],[10],[0]]), -1, 0)
    # stopping_dist = 0.5 #m
    # rrt_path = "/Users/felicialiu/Desktop/DEV/ROB521_MobileRobotics/LAB2/lab2/maps/simplest_map_coords.npy"
    # rrt_star_path = "/Users/felicialiu/Desktop/DEV/ROB521_MobileRobotics/LAB2/lab2/maps/simplest_map_rrtstar_coords.npy"

    # # map information (random seed 18)
    # map_filename = "simple_map.png"
    # map_setings_filename = "simple_map.yaml"
    # # robot information
    # goal_pix = np.array([[1550], [1550]])
    # first_node = Node(np.array([[50],[50],[0]]), -1, 0)
    # stopping_dist = 0.5 #m
    # rrt_path = "./catkin_ws/src/lab2_v3/lab2/maps/simple_map_coords.npy"
    # rrt_star_path = "./catkin_ws/src/lab2_v3/lab2/maps/maps/simple_map_rrtstar_coords.npy"

    # # map information (random seed 42)
    # map_filename = "willowgarageworld_05res.png"
    # map_setings_filename = "willowgarageworld_05res.yaml"
    # # robot information
    # goal_pix = np.array([[1250], [1500]])
    # first_node = Node(np.array([[420],[615],[0]]), -1, 0)
    # stopping_dist = 0.5 #m
    # rrt_path = "/Users/felicialiu/Desktop/DEV/ROB521_MobileRobotics/LAB2/lab2/maps/willowgarageworld_05res_coords.npy"
    # rrt_star_path = "/Users/felicialiu/Desktop/DEV/ROB521_MobileRobotics/LAB2/lab2/maps/willowgarageworld_05res_rrtstar_coords.npy"

    # # map information (random seed 19)
    map_filename = "myhal.png"
    map_setings_filename = "myhal.yaml"
    # robot information
    goal_pix = np.array([[153], [6]])
    first_node = Node(np.array([[6],[43],[0]]), -1, 0)
    stopping_dist = 0.2 #m
    rrt_path = "../maps/myhal_coords.npy"
    rrt_star_path = "../maps/myhal_rrtstar_coords.npy"

    ########################################################

    # RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_pix, first_node, stopping_dist)

    ########################################################

    nodes = path_planner.rrt_planning()
    node_path_metric = path_planner.recover_path()
    plot_path_with_background(path_planner.occupancy_map, node_path_metric)
    np.save(rrt_path, node_path_metric)

    ########################################################

    nodes, goal_id = path_planner.rrt_star_planning()
    node_path_metric = path_planner.recover_path(goal_id)
    plot_path_with_background(path_planner.occupancy_map, node_path_metric)
    np.save(rrt_star_path, node_path_metric)

    ########################################################

    # path_name = rrt_path
    # cells = np.load(path_name)
    # points = path_planner.cell_to_point(cells)
    # np.save(path_name.replace(".npy", "_coords.npy"), points)

    # path_name = rrt_star_path
    # cells = np.load(path_name)
    # points = path_planner.cell_to_point(cells)
    # np.save(path_name.replace(".npy", "_coords.npy"), points)

    ########################################################

    plot_path_with_background(path_planner.occupancy_map, path_planner.point_to_cell(np.load(rrt_path)))
    plot_path_with_background(path_planner.occupancy_map, path_planner.point_to_cell(np.load(rrt_star_path)))



######################################################
# Main Execution
######################################################

if __name__ == '__main__':
    print("*********************\n\n")
    main()
    print("\ndone\n*********************")


######################################################
# Clean Up
######################################################
