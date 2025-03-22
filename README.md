# CAF_STAR_STAR
Implementation CAF-RRT* algorithm for a mobile Robot

The following project is a application to simulate the use of the novel algorithm CAF-RRT* presented in the following  paper:
https://ieeexplore.ieee.org/document/9969595

## AUTHORS of implementation
jcrespo 121028099
saigopal 119484128
ugaurav 120386640

## DEPENDENCIES and PACKAGES
- python 3.11.7
(pip installer used)
- numpy 1.26.3
- opencv-python-headless 4.9.0.80
- tqdm 4.66.2
- matplotlib 3.6.0

## LIBRARIES
 - numpy
 - time
 - numpy
 - cv2
 - math
 - tqdm
 - matplotlib.pyplot

## INSTRUCTIONS

### The following code will generate a simulation of the CAF-RRT* applied to solve and maze problem

- Install previously the linux(Ubuntu) the ffmepg package running on the terminal the command:
	sudo apt install ffmpeg

- Install all necessary dependencies and libraries with pip for example. Recommended to run in your own python environment.

- Using linux terminal locate the folder CAF_RRT_STAR/sim_maze with the python file 'running.py'

- In this file modify values of INIT_STATE, GOAL_STATE and OPTION as specified in the commentaries. for OPTION = 3 use file variable to specify the name of the image to use as maze.

- Run the command 'python3 running.py'

- While program is running information of the node count will display

- The program informs success or not. If success, it will display optimization and animation creation processes. At the end, a video file called 'simulation.mp4' which animates the node exploration,rewiring and initial,optimal,smooth path determined.

## Simulation Examples

![Screenshot 2024-05-06 180511](https://github.com/user-attachments/assets/54fdd7f8-824d-4ba2-b716-202be2e5e84f)

![Screenshot 2024-05-06 181911](https://github.com/user-attachments/assets/01a49b98-f924-4cbc-8f32-73e01ca63fef)


