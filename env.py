import piarm
import time
import numpy as np
import cv2
import random

class MyArm2D:

    def __init__(self, move_robot = False):

        self.move_robot = move_robot

        if self.move_robot:
            self.robot = piarm.PiArm()
            self.open_connection()
            self.DEFAULT = [500, 500, 500, 500, 500, 500]
        
        self.num_members = 3
        self.adjustable_joints = [3,4,5]
        
        self.initial_height = 73 # height in mm of motor 5's axle
        self.lengths = {
            "h_0":   73,
            "a":     97.5,
            "b":     96,
            "c":     160
        }

        self.base_width = 110
        self.base_height = 45

        # All the angles are with respect to the vertical 
        self.max_angles = [90 for _ in range(self.num_members)]
        self.min_angles = [-90 for _ in range(self.num_members)]
        self.min_angles[0] = 0 # To prevent it from hitting the base of the arm
        self.angles = 90*np.ones(self.num_members) # angles of motor 3, 4 and 5 ranging between 
                                  # min_angle and max_angle

        self.member_thickness = 30
        
        self.img_width = 1000
        self.x_offset = int(self.img_width/2)
        self.y_offset = self.lengths["h_0"]
        self.img_height = int(sum(list(self.lengths.values())) + self.y_offset + 20)

        self.img = np.zeros((self.img_height, self.img_width, 3))

        self.timestep = 0
        self.max_timestep = 200

        # This is to check that all the joints (except for the last one) is above
        # the ground
        self.min_joint_heights = [20, 20, 10]

        self.goal_coords = [None, None]
        self.update_goal_coords()

        self.joint_positions = [[0,0] for i in range(self.num_members + 1)]
        self.update_positions()

        self.distance2goal = None
        self.update_distance_2_goal()

    def __del__(self):
        print("Closing connection...")
        if self.move_robot:
            self.close_connection()

    def open_connection(self):
        if self.robot.alive:
            raise Exception("Robot is already switched on")
        self.robot.connect("COM3")
        if self.robot.alive:
            print("Success connecting to robot")
            return True
        else:
            print("Failed to connect to robot")
            return False

    def move_to_default_pos(self):
        if self.robot.alive:
            for ID in range(1, 7):
                self.robot.servoWrite(ID, int(self.DEFAULT[ID - 1]), 500)
            return True
        else:
            return False

    def move_to_pos(self):

        # First, convert the angles in degrees between -90º and +90º
        # to angles between 125 and 875

        # 90 -> 500
        # 0 -> 125

        angles_deg = self.angles - 90
        angles_deg[2] -= angles_deg[1]
        angles_deg[1] -= angles_deg[0]
        
        angles_piarm = [int(500 + (375/90)*angle_deg) for angle_deg in angles_deg]

        angles_piarm[0] = 1000 - angles_piarm[0]
        angles_piarm[1] = 1000 - angles_piarm[1]

        print("Angles in degrees: ", angles_deg)
        print("Moving arms with angles: ", angles_piarm)

        if self.robot.alive:	
            for ID in range(3, 6):
                self.robot.servoWrite(8 - ID, int(angles_piarm[ID - 3]), 500)
            time.sleep(1)
            return True
        else:
            return False

    def close_connection(self):
        if not self.robot.alive:
            raise Exception("Robot is already switched off")
        self.robot.disconnect()
        if not self.robot.alive:
            print("Success disconnecting from robot")
            return True
        else:
            print("Failed to disconnect from robot")
            return False

    def update_goal_coords(self):

        max_length = sum(list(self.lengths.values())[1:])

        r = random.uniform(0.8*max_length,max_length)
        theta = random.uniform(-np.pi/4, np.pi/2) 

        x = r * np.sin(theta)
        y = r * np.cos(theta)

        self.goal_coords = [int(x), int(y)]

    def update_distance_2_goal(self):
        gripper_pos = self.joint_positions[-1]
        
        self.distance2goal = np.sqrt(sum([(gripper_pos[i] - self.goal_coords[i])**2 for i in range(2)]))


    def update_positions(self):
        """
            Positions are with respect to the origin (0,0), right underneath
            motor 5. It is positive if it is away from the origin.
        """
        
        self.joint_positions[0] = [0, self.lengths["h_0"]]
        self.joint_positions[1] = [
            self.joint_positions[0][0] + self.lengths["a"] * np.sin(np.deg2rad(self.angles[0])),
            self.joint_positions[0][1] + self.lengths["a"] * np.cos(np.deg2rad(self.angles[0]))
        ]
        self.joint_positions[2] = [
            self.joint_positions[1][0] + self.lengths["b"] * np.sin(np.deg2rad(self.angles[1])),
            self.joint_positions[1][1] + self.lengths["b"] * np.cos(np.deg2rad(self.angles[1]))
        ]
        self.joint_positions[3] = [
            self.joint_positions[2][0] + self.lengths["c"] * np.sin(np.deg2rad(self.angles[2])),
            self.joint_positions[2][1] + self.lengths["c"] * np.cos(np.deg2rad(self.angles[2]))
        ]

        # Convert to integers
        self.joint_positions = [[int(x[0]),int(x[1])] for x in self.joint_positions]

    def move_arm(self, actions):
        """
            The inputs are the new set of angles [theta0, theta1, theta2]
        """
        for i, action in enumerate(actions): 
            self.angles[i:] += action
        
        
        for member_index in range(1,self.num_members):
            self.max_angles[member_index] = self.angles[member_index - 1] + 90
            self.min_angles[member_index] = self.angles[member_index - 1] - 90

        self.update_positions()

        self.update_distance_2_goal()

    def render(self):

        self.img = np.zeros((self.img_height, self.img_width, 3))

        # Render the floor
        self.img = cv2.rectangle(self.img, (0,0), (self.img_width, self.y_offset), (0,255,0), -1)

        # Render the base of the arm
        self.img = cv2.rectangle(self.img,
                                 (int(self.x_offset - self.base_width/2), self.y_offset),
                                 (int(self.x_offset - self.base_width/2 + self.base_width), self.y_offset + self.base_height),
                                 (0, 165, 255),
                                 -1)

        goal_x, goal_y = self.goal_coords
        
        self.img = cv2.circle(self.img, (goal_x + self.x_offset, goal_y + self.y_offset), int(self.member_thickness/2), (128, 0, 128), 5)
        
        for member_id in range(self.num_members):
            first_joint = self.joint_positions[member_id].copy()
            second_joint = self.joint_positions[member_id + 1].copy()

            first_joint[0] += self.x_offset
            first_joint[1] += self.y_offset

            second_joint[0] += self.x_offset
            second_joint[1] += self.y_offset

            self.img = cv2.line(self.img, tuple(first_joint), tuple(second_joint), (255,0,0), self.member_thickness)
            self.img = cv2.circle(self.img, tuple(first_joint), int(self.member_thickness/2), (255,255,0), -1)

        # Flip image upside down
        self.img = cv2.flip(self.img, 0)

        self.img = cv2.putText(self.img,
                               "Distance: " + str(round(self.distance2goal,2)),
                               (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX,
                               1,
                               (255,255,255),
                               2)

        cv2.imshow("Arm", self.img)
        cv2.moveWindow("Arm",20,50)

    def reset(self):

        self.angles = 90*np.ones(self.num_members)

        self.update_positions()

        self.img = np.zeros((self.img_height, self.img_width, 3))

        self.timestep = 0

        self.update_goal_coords()

        self.render()

        if self.move_robot:
            self.move_to_default_pos()

    def check_arm_angles(self):
        for member_index in range(self.num_members):
            if self.angles[member_index] < self.min_angles[member_index]:
                return False
            if self.angles[member_index] > self.max_angles[member_index]:
                return False
        return True

    def check_arm_positions(self):
        for joint_index in range(1,len(self.joint_positions)):
            member_pos = self.joint_positions[joint_index][1]
            min_height = self.min_joint_heights[joint_index-1]
            
            if member_pos < min_height:
                return False
        return True

    def get_reward(self, forbidden_action):

        if forbidden_action:
            reward_scaling_factor = 2
        else:
            reward_scaling_factor = 1

        return - self.distance2goal * reward_scaling_factor 
    
    def step(self, actions):

        self.move_arm(actions)

        forbidden_action = False

        okay_angles = self.check_arm_angles()

        okay_positions = self.check_arm_positions()

        if not okay_angles:
            print("An angle threshold was exceeded")
            self.move_arm(-actions)
            forbidden_action = True

        if not okay_positions:
            print("A position threshold was exqqceeded")
            self.move_arm(-actions)
            forbidden_action = True

        self.render()

        if self.move_robot:
            self.move_to_pos()

        r = self.get_reward(forbidden_action)
        
        self.timestep += 1

        is_done = self.timestep >= self.max_timestep

        return self.angles, r, is_done
            