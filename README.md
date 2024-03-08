# Dijkstra-Path-Planner

## ENPM661: Project 2 Submission

### Vikram Setty (119696897)

An implementation of Dijkstra's Algorithm to plan optimal paths between a known starting and goal position in an obstacle-ridden grid world

## Overview
This project uses Dijkstra's Algorithm to plan paths between a user-input start and goal position in a 1200*500 obstacle-ridden gird world.

On giving start and goal location coordinates, the path planner computes the shortest path using Dijkstra's Algorithm while avoiding obstacles (with a clearance of 5 mm).

On finding the final path, the planner makes a video with intermediate frames and displays it as a pop-up animation.

This project majorly uses PyGame for generating the visualiziation and OpenCV and ImageIO for displaying the animation.

## Dependencies
The dependencies for this Python 3project include the following:
<ul>
<li> NumPy
<li> PyGame
<li> OpenCV
<li> ImageIO (amd imageio-ffmpeg)
</ul>
They can be installed using the following commands.

```sh
    pip3 install numpy
    pip3 install pygame
    pip3 install opencv-python
    pip3 install imageio
    pip3 install imageio-ffmpeg
```

## Running the Code
To run the code, execute the following command
```sh
    python3 dijkstra_Vikram_Setty.py
```
