# Dijkstra-Path-Planner

## ENPM661: Project 2 Submission

### Vikram Setty (119696897)

An implementation of Dijkstra's Algorithm to plan optimal paths between a known starting and goal position in an obstacle-ridden 1200*500 grid world.

The link to the GitHub Repository can be found [here](https://github.com/vikrams169/Dijkstra-Path-Planner).


## Overview
On giving start and goal location coordinates, the path planner computes the shortest path using Dijkstra's Algorithm while avoiding obstacles (with a clearance of 5 mm).

On finding the final path, the planner makes a video with intermediate frames and displays it as a pop-up animation.

This project majorly uses PyGame for generating the visualiziation and OpenCV and ImageIO for displaying the animation.

## Dependencies
The dependencies for this Python 3 project include the following:
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
On doing so, the terminal should prompt for the coordinate positions of start and goal locations which the user has to enter. Note a couple points:
<ul>
<li> Enter integer values
<li> Use the coordinate system considering the bottom-left of the window/map as the origin
<li> If any of the coordinate locations you have enetered is not valid i.e. out of map bounds, or within an obstacle/its clearance, you will be prompted to enter all the coordinate locations again till they are valid. Note that even the walls/boundaries of the grid world have a clearance of 5 grid cells.
</ul>

A sample set of start and goal positions to enter (that goes from one corner of the grid to the other) include the one below. This particular case can execute within 10-50 seconds depending upon system specifications.
<ul>
<li> Start Position: (6,494)
<li> Goal Position: (1194,6)
</ul>

After the program accepts your start and goal locations, it will start computing the path. It will keep on adding intermediate frames to a newly created `animation_frames` directory. Ater computing the final path, it will generate and display a video `sample_video.mp4`from the saved frames and delete all the individual frames themselvers along with the `animation_frames` directory.
