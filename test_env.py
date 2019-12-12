from env import MyArm2D
import cv2
import numpy as np

arm = MyArm2D(move_robot=True)
arm.reset()

is_done = False

while 1:
    
    k = cv2.waitKey(40)
    # UP
    if k == 82:
        s, r, is_done = arm.step(np.array([5, 0, 0]))
        print(r)
    # DOWN
    elif k == 84:
        s, r, is_done = arm.step(np.array([-5, 0, 0]))
        print(r)
    # RIGHT
    elif k == 83:
        s, r, is_done = arm.step(np.array([0, 5, 0]))
        print(r)
    # LEFT
    elif k == 81:
        s, r, is_done = arm.step(np.array([0, -5, 0]))
        print(r)
    # a/A
    elif k == ord("a") or k == ord("A"):
        s, r, is_done = arm.step(np.array([0, 0, 5]))
        print(r)
    # s/S
    elif k == ord("s") or k == ord("S"):
        s, r, is_done = arm.step(np.array([0, 0, -5]))
        print(r)
    # q/Q (exit)
    elif k == ord("q") or k == ord("Q"):
        break

    if is_done:
        arm.reset()

cv2.destroyAllWindows()
