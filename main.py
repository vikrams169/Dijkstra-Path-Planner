'''

Link to the GitHub Repository:
https://github.com/vikrams169/Dijkstra-Path-Planner

'''

# Importing the required libraries
import os
import shutil
from heapq import *
import numpy as np
import pygame
import cv2
import imageio
import time

# Defining colour values across the RGB Scale
WHITE = (255,255,255)
BLACK = (0,0,0)
GRAY = (128,128,128)
GREEN = (0,255,0)
RED = (255,0,0)
BLUE = (0,0,255)
ORANGE = (255,164.5,0)

# Size of the grid and dimensions of the grid block
GRID_SIZE = (1200,500)
WIDTH_X = 1
WIDTH_Y = 1

# Names of the directory where the animation frames and the video generated would be saved to
FRAME_DIRECTORY = "animation_frames"
VIDEO_NAME = "sample_video.mp4"

# Node Info Explanation
'''
for each cell in the grid (indexed by node_states[row_index][column_index]), there are three dimensions
Index 0: Node state i.e. -1 for not explored yet, 0 for open, 1 for closed
Index 1: Parent node row index
Index 2: Parent node column index
Index 3: Distance from the source node
'''
node_info = -1*np.ones((GRID_SIZE[0],GRID_SIZE[1],4)) # Information about node distance, parents, and status
min_heap = [] # Min-heap to store nodes initialized as a list
heapify(min_heap) # Making the min-heap list into a heap using the heapq library
start_pos = None # Start position of the planner
goal_pos = None # Goal position of the planner
solved = False # Boolean to check whether the path has been fully solved or not
iteration = 0 # Number of times Dijkstra has been called so far
frame_number = 0 # Current number of frames that have been saved

# Playing the generated video
def play_video(video_file):
    vid = cv2.VideoCapture(video_file)
    while vid.isOpened():
        frame_present, frame = vid.read()
        if frame_present:
            cv2.imshow('Animation Video',frame)
            if cv2.waitKey(25) & 0xFF == ord('q'): 
                break
        else:
            break
    vid.release()
    cv2.destroyAllWindows()

# Function to make a video from a set of frames in a directory
def make_video(frames_directory,video_name):
    num_frames = len(os.listdir(frames_directory))
    with imageio.get_writer(video_name,mode='I',fps=50) as writer:
        for i in range(num_frames):
            image = imageio.imread(frames_directory+"/frame"+str(i)+".jpg")
            writer.append_data(image)

# Saving a frame to a directory
def save_frame(viz_window):
    global frame_number
    pygame.display.update()
    pygame.image.save(viz_window,FRAME_DIRECTORY+"/frame"+str(frame_number)+".jpg")
    frame_number += 1

# Return a boolean value of whether a grid cell lies within an obstacle (or in its clearance)
def in_obstacle(viz_window,loc):
    if np.array_equal(viz_window.get_at(loc)[:3],GRAY) or np.array_equal(viz_window.get_at(loc)[:3],BLACK):
        return True
    else:
        return False

# Returning whether a current grid cell is the goal or not
def reached_goal(loc):
    if np.array_equal(loc,goal_pos):
        return True
    else:
        return False
    
# Backtracking to find the path between the start and goal locations
def compute_final_path(viz_window):
    path_nodes = [goal_pos]
    while not np.array_equal(path_nodes[-1],start_pos):
        parent_loc = path_nodes[-1]
        path_nodes.append((int(node_info[parent_loc[0],parent_loc[1],1]),int(node_info[parent_loc[0],parent_loc[1],2])))
    for loc in path_nodes:
        pygame.draw.rect(viz_window,GREEN,(loc[0],loc[1],3*WIDTH_X,3*WIDTH_Y))
    for i in range(50):
        save_frame(viz_window)

# Adding a node to the min-heap
def add_node_to_heap(viz_window,parent_loc,new_loc,op):
    global min_heap
    if new_loc[0] < 0 or new_loc[0] >= GRID_SIZE[0] or new_loc[1] < 0 or new_loc[1] >= GRID_SIZE[1] or in_obstacle(viz_window,(new_loc[0],new_loc[1])) or node_info[new_loc[0],new_loc[1],0] == 1:
        return
    dist = None
    if op == 0:
        dist = node_info[parent_loc[0],parent_loc[1],3] + 1
    else:
        dist = node_info[parent_loc[0],parent_loc[1],3] + 1.4
    heappush(min_heap,(dist,new_loc,parent_loc))

# Adding all the neighbors of the current node to the min-heap
def add_neighbors(viz_window,loc):
    add_node_to_heap(viz_window,loc,(loc[0]-1,loc[1]),0)
    add_node_to_heap(viz_window,loc,(loc[0],loc[1]-1),0)
    add_node_to_heap(viz_window,loc,(loc[0]+1,loc[1]),0)
    add_node_to_heap(viz_window,loc,(loc[0],loc[1]+1),0)
    add_node_to_heap(viz_window,loc,(loc[0]-1,loc[1]-1),1)
    add_node_to_heap(viz_window,loc,(loc[0]+1,loc[1]-1),1)
    add_node_to_heap(viz_window,loc,(loc[0]-1,loc[1]+1),1)
    add_node_to_heap(viz_window,loc,(loc[0]+1,loc[1]+1),1)

# Processing the current node that was returned by popping the min-heap
def process_node(viz_window,node):
    global node_info, solved, iteration
    dist, loc, parent_loc = node
    if node_info[loc[0],loc[1],0] == 1:
        return
    node_info[loc[0],loc[1],0] = 1
    node_info[loc[0],loc[1],1] = parent_loc[0]
    node_info[loc[0],loc[1],2] = parent_loc[1]
    node_info[loc[0],loc[1],3] = dist
    pygame.draw.rect(viz_window,BLUE,(loc[0],loc[1],WIDTH_X,WIDTH_Y))
    if reached_goal(loc):
        solved = True
        compute_final_path(viz_window)
    add_neighbors(viz_window,loc)
    iteration += 1
    if iteration%300 == 0:
        save_frame(viz_window)

# Wrapper function for the Dijkstra Algorithm
def dijkstra(viz_window):
    global min_heap,solved
    heappush(min_heap,(0,start_pos,start_pos))
    while not solved:
        node = heappop(min_heap)
        process_node(viz_window,node)

# Initializing start and goal positions using user input
def initialize_start_and_goal_pos(viz_window):
    global start_pos, goal_pos
    print("Please enter the start and goal positions (in the coordinate system with the origin at the bottom left corner) starting from index 0")
    print("Make sure to to add locations within the 1200mm*500mm grid map avoiding obstacles accounting for a 5mm clearance")
    while(start_pos is None or goal_pos is None):
        start_x = int(input("Enter the X coordinate of the starting position: "))
        start_y = GRID_SIZE[1] - int(input("Enter the Y coordinate of the starting position: "))
        goal_x = int(input("Enter the X coordinate of the goal position: "))
        goal_y = GRID_SIZE[1] - int(input("Enter the Y coordinate of the goal position: "))
        if start_x < 0 or start_x >= GRID_SIZE[0] or start_y < 0 or start_y >= GRID_SIZE[1] or in_obstacle(viz_window,(start_x,start_y)):
            print("Try again with a valid set of values")
            continue
        if goal_x < 0 or goal_x >= GRID_SIZE[0] or goal_y < 0 or goal_y >= GRID_SIZE[1] or in_obstacle(viz_window,(goal_x,goal_y)):
            print("Try again with a valid set of values")
            continue
        start_pos = (start_x,start_y)
        goal_pos = (goal_x,goal_y)

# Initializing the map with obstacles in PyGame
def initialize_map(viz_window):
    wall1_bloated = [(0,0),(5,0),(5,500),(0,500)]
    wall2_bloated = [(0,0),(1200,0),(1200,5),(0,5)]
    wall3_bloated = [(1195,0),(1200,0),(1200,500),(1195,500)]
    wall4_bloated = [(0,495),(1200,495),(1200,500),(0,500)]
    obstacle1 = [(100,0),(175,0),(175,400),(100,400)]
    obstacle1_bloated = [(95,0),(180,0),(180,405),(95,405)]
    obstacle2 = [(275,100),(350,100),(350,500),(275,500)]
    obstacle2_bloated = [(270,95),(355,95),(355,500),(270,500)]
    obstacle3 = [(650,100),(int(650+75*(3**0.5)),175),(int(650+75*(3**0.5)),325),(650,400),(int(650-75*(3**0.5)),325),(int(650-75*(3**0.5)),175)]
    obstacle3_bloated = [(650,95),(int(650+75*(3**0.5)+2.5*(3**0.5)),int(175-2.5*(3**0.5))),(int(650+75*(3**0.5)+2.5*(3**0.5)),int(325+2.5*(3**0.5))),(650,405),(int(650-75*(3**0.5)-2.5*(3**0.5)),int(325+2.5*(3**0.5))),(int(650-75*(3**0.5)-2.5*(3**0.5)),int(175-2.5*(3**0.5)))]
    obstacle4 = [(900,50),(1100,50),(1100,450),(900,450),(900,375),(1020,375),(1020,125),(900,125)]
    obstacle4_bloated = [(895,45),(1105,45),(1105,455),(895,455),(895,370),(1015,370),(1015,130),(895,130)]
    obstacles = [obstacle1,obstacle2,obstacle3,obstacle4]
    obstacles_bloated = [wall1_bloated,wall2_bloated,wall3_bloated,wall4_bloated,obstacle1_bloated,obstacle2_bloated,obstacle3_bloated,obstacle4_bloated]
    for obstacle in obstacles_bloated:
        pygame.draw.polygon(viz_window,GRAY,obstacle)
    for obstacle in obstacles:
        pygame.draw.polygon(viz_window,BLACK,obstacle)
    save_frame(viz_window)

def main():

    global node_states, start_pos, goal_pos

    # Creating a directory to save the animation frames
    os.mkdir(FRAME_DIRECTORY)

    # Initializing the grid world as a pygame display window,
    pygame.display.set_caption('Dijkstra Path Finding Algorithm Visualization')
    viz_window = pygame.display.set_mode(GRID_SIZE,flags=pygame.HIDDEN)
    viz_window.fill(WHITE)
    save_frame(viz_window)

    # Initializing the map with obstacles
    initialize_map(viz_window)
    save_frame(viz_window)
    # Initializing the start and goal positions of the path from user input
    initialize_start_and_goal_pos(viz_window)

    print("\nStarting the Pathfinding Process! This may take upto a minute.\n")

    # Running the Dijkstra Algorithm
    start_time = time.time()
    dijkstra(viz_window)
    end_time = time.time()
    print("\nSuccess! Found the Optimal Path\n")
    print("\nTime taken for the pathfinding process using Dijkstra's Algorithm: ",end_time-start_time," seconds\n")

    #Making the video from the animation frames
    print("\nNow Generating the video for animation\n")
    make_video(FRAME_DIRECTORY,VIDEO_NAME)
    print("\nSaved the video as sample_video.mp4 in the same directory as this code file!\n")

    # Removing all the video frames
    shutil.rmtree(FRAME_DIRECTORY)

    # Playing the video
    print("\nPlaying the video animation of the path computation\n")
    play_video(VIDEO_NAME)

if __name__ == "__main__":
    main()