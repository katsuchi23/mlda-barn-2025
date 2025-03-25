import time
import argparse
import subprocess
import os
from os.path import join
import sys
import numpy as np
import rospy
import rospkg
from gazebo_simulation import GazeboSimulation
from jackal_helper.msg import TrainingData

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

def terminate_processes(processes):
    """Properly terminate all processes"""
    print("[CLEANUP] Terminating all processes...")
    for process in processes:
        if process:
            try:
                process.terminate()
                time.sleep(0.5)
                if process.poll() is None:
                    process.kill()
            except Exception as e:
                print(f"[ERROR] Failed to terminate process: {e}")
    print("[CLEANUP] All processes terminated.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=259)
    parser.add_argument('--gui', action="store_true")
    parser.add_argument('--out', type=str, default="out.txt")
    parser.add_argument('--kul_weight', type=float, default=0.7, help='Weight for KUL algorithm (0.0-1.0)')
    parser.add_argument('--rl_weight', type=float, default=0.3, help='Weight for RL algorithm (0.0-1.0)')
    args = parser.parse_args()
    
    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################
    
    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
    # os.environ["DISPLAY"] = "-"
    # os.environ["DISPLAY"] = ":0"

    if args.world_idx < 300:  # static environment from 0-299
        world_name = "BARN/world_%d.world" %(args.world_idx)
        INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 10]  # relative to the initial position
    elif args.world_idx < 360:  # Dynamic environment from 300-359
        world_name = "DynaBARN/world_%d.world" %(args.world_idx - 300)
        INIT_POSITION = [0, 0, 3.14]  # in world frame
        GOAL_POSITION = [-8, 0]  # relative to the initial position
    else:
        raise ValueError("World index %d does not exist" %args.world_idx)
    
    # Store all processes to ensure proper cleanup
    processes = []
    
    try:
        print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world_name))   
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('jackal_helper')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")
        
        launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
        world_name = join(base_path, "worlds", world_name)
        
        gazebo_process = subprocess.Popen([
            'roslaunch',
            launch_file,
            'world_name:=' + world_name,
            'gui:=true'
        ])
        processes.append(gazebo_process)
        
        rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
        rospy.set_param('/use_sim_time', True)

        # Initialize the node
        pub = rospy.Publisher('/training_episode_result', TrainingData, queue_size=10)
        msg = TrainingData()
        
        # GazeboSimulation provides useful interface to communicate with gazebo  
        gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
        
        init_coor = (INIT_POSITION[0], INIT_POSITION[1])
        goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
        
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = True
        
        # check whether the robot is reset, the collision is False
        while compute_distance(init_coor, curr_coor) > 0.1 or collided:
            gazebo_sim.reset() # Reset to the initial position
            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = gazebo_sim.get_hard_collision()
            time.sleep(1)

        ##########################################################################################
        ## 1. Launch your navigation stack
        ## (Customize this block to add your own navigation stack)
        ##########################################################################################
        
        launch_file = join(base_path, '..', 'jackal_helper/launch/move_base_rl_kul.launch')
        nav_stack_process = subprocess.Popen([
            'roslaunch',
            launch_file,
            'kul_weight:=' + str(args.kul_weight),
            'rl_weight:=' + str(args.rl_weight)
        ])
        processes.append(nav_stack_process)
        
        # Make sure your navigation stack recives the correct goal position defined in GOAL_POSITION
        import actionlib
        from geometry_msgs.msg import Quaternion
        from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
        nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = 'odom'
        mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
        mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
        mb_goal.target_pose.pose.position.z = 0
        mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        nav_as.wait_for_server()
        print("goal: ", mb_goal)
        nav_as.send_goal(mb_goal)


        ##########################################################################################
        ## 2. Start navigation
        ##########################################################################################
        
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        
        # check whether the robot started to move
        while compute_distance(init_coor, curr_coor) < 0.1:
            curr_time = rospy.get_time()
            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            time.sleep(0.01)
        
        # start navigation, check position, time and collision
        start_time = curr_time
        start_time_cpu = time.time()
        collided = False
        step_count = 0
        collision_count = 0
        
        print("[INFO] Starting navigation from", init_coor, "to", goal_coor)
        
        # loop inside this
        while compute_distance(goal_coor, curr_coor) > 1 and curr_time - start_time < 100:
            step_count += 1
            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = gazebo_sim.get_hard_collision()
            
            # Only log every 100 steps or on collision
            if step_count % 100 == 0:
                dist_to_goal = compute_distance(goal_coor, curr_coor)
                elapsed = curr_time - start_time
                print(f"[STEP {step_count}] Position: ({curr_coor[0]:.2f}, {curr_coor[1]:.2f}) | Distance to goal: {dist_to_goal:.2f} | Time: {elapsed:.2f}s")
            
            if collided:
                collision_count += 1
                print(f"[COLLISION {collision_count}] Detected at position ({curr_coor[0]:.2f}, {curr_coor[1]:.2f})")
                msg.status = "collided"
                pub.publish(msg)

                reset_count = 0
                while compute_distance(init_coor, curr_coor) > 0.1 or collided:
                    reset_count += 1
                    if reset_count % 5 == 0:  # Only log every 5 reset attempts
                        print(f"[RESET {reset_count}] Position: ({curr_coor[0]:.2f}, {curr_coor[1]:.2f}) | Collision status: {collided}")
                    
                    gazebo_sim.reset() # Reset to the initial position
                    pos = gazebo_sim.get_model_state().pose.position
                    curr_coor = (pos.x, pos.y)
                    collided = gazebo_sim.get_hard_collision()

                print(f"[INFO] Reset complete after {reset_count} attempts. Continuing navigation.")
                msg.status = "continue"
                pub.publish(msg)
            
            curr_time = rospy.get_time()

        # Cancel any pending goals to avoid callbacks after completion
        nav_as.cancel_all_goals()
        
        ##########################################################################################
        ## 3. Report metrics and generate log (similar to run_rviz_kul.py)
        ##########################################################################################
        
        print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
        success = compute_distance(goal_coor, curr_coor) <= 1
        if collided:
            status = "collided"
        elif curr_time - start_time >= 100:
            status = "timeout"
        else:
            status = "succeeded"
            success = True
        print("Navigation %s with time %.4f (s)" %(status, curr_time - start_time))
        
        if args.world_idx >= 300:  # DynaBARN environment which does not have a planned path
            path_length = GOAL_POSITION[0] - INIT_POSITION[0]
        else:
            path_file_name = join(base_path, "worlds/BARN/path_files", "path_%d.npy" %args.world_idx)
            path_array = np.load(path_file_name)
            path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
            path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
            path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
            path_length = 0
            for p1, p2 in zip(path_array[:-1], path_array[1:]):
                path_length += compute_distance(p1, p2)
        
        # Navigation metric: 1_success *  optimal_time / clip(actual_time, 2 * optimal_time, 8 * optimal_time)
        optimal_time = path_length / 2
        actual_time = curr_time - start_time
        nav_metric = int(success) * optimal_time / np.clip(actual_time, 2 * optimal_time, 8 * optimal_time)
        
        print("\n" + "="*80)
        print("NAVIGATION RESULTS")
        print("="*80)
        print(f"Final position: ({curr_coor[0]:.2f}, {curr_coor[1]:.2f})")
        print(f"Distance to goal: {compute_distance(goal_coor, curr_coor):.2f}")
        print(f"Path length: {path_length:.2f}")
        print(f"Optimal time: {optimal_time:.2f}")
        print(f"Actual time: {actual_time:.2f}")
        print(f"Total collisions: {collision_count}")
        print(f"Navigation metric: {nav_metric:.4f}")
        print("="*80 + "\n")
        
        # Write results to file
        with open(args.out, "a") as f:
            f.write("%d %d %d %d %.4f %.4f\n" %(args.world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, nav_metric))
        
        msg.status = status
        msg.score = nav_metric * 100  # Scale the navigation metric to a 0-100 scale
        pub.publish(msg)
        
        # Give ROS time to process final messages
        time.sleep(1)
        
    except Exception as e:
        print(f"[ERROR] An exception occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        terminate_processes(processes)
        gazebo_process.terminate()
        gazebo_process.wait()
        nav_stack_process.terminate()
        nav_stack_process.wait()
        sys.exit(200) if success else sys.exit(400)
        try:
            rospy.signal_shutdown("Navigation completed")
        except:
            pass
        
        print("[EXIT] Script completed.")
