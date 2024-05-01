from caf_rrt_star import plan_caf_rrt_star
from solution import generate_path
from simulation import create_simulation
INITIAL= (500,1650,0)
GOAL = (5750,500,0)
WIDTH = 6000
HEIGHT = 2000
BORDER = 10
result = plan_caf_rrt_star(INITIAL, GOAL, (BORDER, WIDTH, HEIGHT))
solution_path = generate_path((BORDER, WIDTH, HEIGHT), **result)
create_simulation(INITIAL, GOAL, result, solution_path, (BORDER, WIDTH, HEIGHT))
