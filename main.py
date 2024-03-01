from heapq import *
import numpy as np
import pygame

# Defining colour values across the RGB Scale
WHITE = (255,255,255)
BLACK = (0,0,0)
GRAY = (128,128,128)
GREEN = (0,255,0)
RED = (255,0,0)
BLUE = (0,0,255)
ORANGE = (255,164.5,0)

GRID_SIZE = (1200,500)
WIDTH_X = 1
WIDTH_Y = 1

FRAME_DIRECTORY = "animation_frames"

# Node Info Explanation
'''
for each cell in the grid (indexed by node_states[row_index][column_index]), there are three dimensions
Index 0: Node state i.e. -1 for not explored yet, 0 for open, 1 for closed
Index 1: Parent node row index
Index 2: Parent node column index
Index 3: Distance from the source node
'''
node_info = -1*np.ones((GRID_SIZE[0],GRID_SIZE[1],4))
min_heap = []
heapify(min_heap)
start_pos = None
goal_pos = None
solved = False
frame_number = 0

def save_frame(viz_window):
    global frame_number
    pygame.display.update()
    pygame.image.save(viz_window,FRAME_DIRECTORY+"/frame"+str(frame_number)+".jpg")
    frame_number += 1

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

def in_obstacle(viz_window,loc):
    if np.array_equal(viz_window.get_at(loc)[:3],GRAY) or np.array_equal(viz_window.get_at(loc)[:3],BLACK):
        return True
    else:
        return False

def reached_goal(loc):
    if np.array_equal(loc,goal_pos):
        return True
    else:
        return False

def add_node(viz_window,parent_node,new_loc):
    global node_info
    pass

def add_neighbors(viz_window_node):
    global node_info

def process_node(viz_window,node):
    global node_info
    dist, loc, parent_loc = node
    if node_info[loc[0],loc[1],0] == 1:
        return
    node_info[loc[0],loc[1],0] = 1
    node_info[loc[0],loc[1],1] = parent_loc[0]
    node_info[loc[0],loc[1],2] = parent_loc[1]
    node_info[loc[0],loc[1],3] = dist
    add_neighbors(viz_window,node)

def dijkstra(viz_window):
    global min_heap,solved
    heappush(min_heap,(0,start_pos,start_pos))
    while not solved:
        node = heappop(min_heap)
        process_node(viz_window,node)

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
    print(start_pos)
    print(goal_pos)

def main():

    global node_states, start_pos, goal_pos

    # Initializing the grid world as a pygame display window,
    pygame.display.set_caption('Dijkstra Path Finding Algorithm Visualization')
    viz_window = pygame.display.set_mode(GRID_SIZE)
    viz_window.fill(WHITE)
    save_frame(viz_window)
    initialize_map(viz_window)
    #pygame.pixelarray(viz_window)[10:20,10:20] = GREEN
    save_frame(viz_window)
    #print(in_obstacle(viz_window,(100,100)))
    initialize_start_and_goal_pos(viz_window)

if __name__ == "__main__":
    main()