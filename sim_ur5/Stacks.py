from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward
import time


def No_colide(cube_positions, x, y, index):
    for cube_index in range(len(cube_positions)):
        if (index != cube_index):
            a = 0 <= (cube_positions[cube_index][0] - x) < 0.04
            b = 0 <= (x - cube_positions[cube_index][0]) < 0.04
            c = 0 <= (y - cube_positions[index][1]) < 0.04
            d = 0 <= (cube_positions[index][1] - y) < 0.04
            if (a or b) and (c or d):
                print(f"*************Cube {index + 1} and Cube {cube_index + 1} are too close")
                print(f"*************Cube[{index + 1}]  x, y = {x,y} and cube_positions[{cube_index + 1}] = {cube_positions[cube_index][0], cube_positions[cube_index][1]}")
                return False
            #print(f"Cube {index + 1} and Cube {cube_index + 1} are not close")
            #print(f"Cube[{index + 1}]  x, y = {x,y} and cube_positions[{cube_index + 1}] = {cube_positions[cube_index][0], cube_positions[cube_index][1]}")
    return True

def stack_cubes(cube_positions, target_location):
    # Create the simulation environment and the executor
    env = SimEnv()
    executor = MotionExecutor(env)
    # Add blocks to the world by enabling the randomize argument and setting the block position in the reset function of the SimEnv class 
    env.reset(randomize=False, block_positions=cube_positions)

    #Check if target position is reachable
    workspace_limits_x = [-0.9, -0.54]
    workspace_limits_y = [-1.0, -0.55]
    if not (workspace_limits_x[0] <= target_location[0] <= workspace_limits_x[1]) or not (workspace_limits_y[0] <= target_location[1] <= workspace_limits_y[1]):
        print("Target location is not reachable")
        return
    for cube_index in range(len(cube_positions)):
        if not (workspace_limits_x[0] <= cube_positions[cube_index][0] <= workspace_limits_x[1]) or not (workspace_limits_y[0] <= cube_positions[cube_index][1] <= workspace_limits_y[1]):
            print(f"Cube {cube_index + 1} is not reachable ,{cube_positions[cube_index]}")
            return 

    #Take only cubes that needed to be stacked - relocated 
    cube_positions = [pos for pos in cube_positions if not (pos[0] == target_location[0] and pos[1] == target_location[1])]

    # Sort cubes by Z value (from highest to lowest) - in case some cubes stack on top of each other and needed to relocate
    sorted_cubes_with_index = sorted(enumerate(cube_positions), key=lambda item: item[1][2], reverse=True)
    
    # Extract sorted indices
    sorted_indices = [index for index, _ in sorted_cubes_with_index]
    
    #Check how many cubes need to be moved
    num_cubes_to_move = len(sorted_indices)
    if num_cubes_to_move == 0:
        print("No cubes to move")
        return
    
    # moveJ is utilized when the robot's joints are clear to you but use carefully because there is no planning here
    move_to = [1.305356658502026, -0.7908733209856437, 1.4010098471710881, 4.102251451313659, -1.5707962412281837, -0.26543967541515895]
    executor.moveJ("ur5e_2", move_to)

    #Check if target location is clear, if not need to move cubes.
    target_x_range = [target_location[0] - 0.02, target_location[0] + 0.02] 
    target_y_range = [target_location[1] - 0.02, target_location[1] + 0.02] 
    index_array = []
    for cube_index in range(len(cube_positions)):
        if (target_x_range[0] <=  cube_positions[cube_index][0] <= target_x_range[1]) or (target_y_range[0] <=  cube_positions[cube_index][1] <= target_y_range[1]):
            index_array.append(cube_index)
 
    if len(index_array) > 0:
        print(f"Target location is not clear, cubes need to be moved {index_array}")
        # Sort indices based on the Z value of corresponding cube positions (highest to lowest)
        sorted_cubes_with_index_move = sorted(index_array, key=lambda idx: cube_positions[idx][2], reverse=True)

        # Extract only the sorted indices
        sorted_indices_move = [index for index in sorted_cubes_with_index_move]

        for cnt, index in enumerate(sorted_indices_move):
            z = cube_positions[index][2]
            x = cube_positions[index][0]
            y = cube_positions[index][1]
            i = 0.1 * (1 + cnt)
            while True:
                x_current = x
                y_current = y
                No_collision = No_colide(cube_positions, x, y, index)
                if (target_x_range[0] <=  cube_positions[index][0] <= target_x_range[1]) or not No_collision:
                    if x + i < workspace_limits_x[1]:
                        x += i
                    else:
                        x -= i
                    No_collision = No_colide(cube_positions, x, y, index)

                if (target_y_range[0] <=  cube_positions[index][1] <= target_y_range[1]) or not No_collision:
                    if y + i < workspace_limits_x[1]:
                        y += i
                    else:
                        y -= i
                    No_collision = No_colide(cube_positions, x, y, index)
                i = i/2

                if y_current == y and x_current == x and No_collision:
                    break
            print(f"moving cube number {index + 1} to {x}, {y}")       
            executor.pick_up("ur5e_2", cube_positions[index][0], cube_positions[index][1], z + 0.12)
            executor.plan_and_move_to_xyz_facing_down("ur5e_2", [x,y,0.15])
            executor.put_down("ur5e_2", x, y, 0.2)
            cube_positions[index] = [x, y, 0.03]
    
        # Sort cubes by Z value (from highest to lowest) - in case some cubes stack on top of each other and needed to relocate
        sorted_cubes_with_index = sorted(enumerate(cube_positions), key=lambda item: item[1][2], reverse=True)   
        # Extract sorted indices
        sorted_indices = [index for index, _ in sorted_cubes_with_index]




    for i, cube_index in enumerate(sorted_indices):
        # Compute stacking position (z increases with each cube for safety)
        current_target = [target_location[0], target_location[1], target_location[2] + 0.12 + 0.02 * (i + 4 - num_cubes_to_move)]
        
        # Pick up the cube
        x, y, z = cube_positions[cube_index]
        print(f"Pick up cube at {x}, {y}, {z}")
        executor.pick_up("ur5e_2", x, y, z + 0.12)

        # Move to the target location
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", current_target)
        x, y, z = target_location[0], target_location[1], target_location[2] + 0.17 + 0.03 * (i + 4 - num_cubes_to_move)
        executor.put_down("ur5e_2", x, y, z)
        if i < len(sorted_indices) - 1:
            next_cube_index = sorted_indices[i + 1]    # Get the next sorted cube index
            print(f"Moving to next cube at index {next_cube_index}")
            x, y, z = cube_positions[next_cube_index]  # Access cube_positions using the sorted index
            executor.plan_and_move_to_xyz_facing_down("ur5e_2", [x, y, 0.15])
    
    final_pos_robot = [target_location[0] - 0.1 , target_location[1] - 0.1, 0.3]  
    executor.plan_and_move_to_xyz_facing_down("ur5e_2", final_pos_robot)
    executor.wait(5)

    # Wait for stability
    start_time = time.time()
    while time.time() - start_time < 5:
        pass  
        


        
