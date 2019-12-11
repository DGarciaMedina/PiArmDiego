import piarm
import time
import numpy as np
import cv2

class MyArm2D:

    def __init__(self):
        
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
        self.x_offset = self.img_width/2
        self.y_offset = self.lengths["h_0"]
        self.img_height = int(sum(list(self.lengths.values())) + self.y_offset + 20)

        self.img = np.zeros((self.img_height, self.img_width, 3))

        self.timestep = 0
        self.max_timestep = 200

        # This is to check that all the joints (except for the last one) is above
        # the ground
        self.min_joint_heights = [10, 10, 0]

        self.joint_positions = [[0,0] for i in range(self.num_members + 1)]
        self.update_positions()

    def update_positions(self):
        """
            Positions are with respect to the origin (0,0), right underneath
            motor 5. It is positive if it is away from the origin.
        """
        
        self.joint_positions[0] = [self.x_offset, self.y_offset + self.lengths["h_0"]]
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

        self.timestep += 1

        if self.timestep >= self.max_timestep:
            self.reset()

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
        
        for member_id in range(self.num_members):
            first_joint = tuple(self.joint_positions[member_id])
            second_joint = tuple(self.joint_positions[member_id + 1])

            self.img = cv2.line(self.img, first_joint, second_joint, (255,0,0), self.member_thickness)
            self.img = cv2.circle(self.img, first_joint, int(self.member_thickness/2), (255,255,0), -1)

        # Flip image upside down
        self.img = cv2.flip(self.img, 0)

        cv2.imshow("Arm", self.img)
        cv2.moveWindow("Arm",20,50)

    def reset(self):

        self.angles = 90*np.ones(self.num_members)

        self.update_positions()

        self.img = np.zeros((self.img_height, self.img_width, 3))

        self.timestep = 0

        self.render()

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
            
            if member_pos < min_height + self.y_offset:
                return False
        return True

    def step(self, actions):

        self.move_arm(actions)

        okay_angles = self.check_arm_angles()

        okay_positions = self.check_arm_positions()

        if not okay_angles:
            print("An angle threshold was exceeded")
            self.move_arm(-actions)

        if not okay_positions:
            print("A position threshold was exceeded")
            self.move_arm(-actions)

        self.timestep += 1

        if self.timestep > self.max_timestep:
            self.reset()