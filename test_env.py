from env import MyArm2D
import cv2
import numpy as np

arm = MyArm2D("COM4", move_robot=False)
arm.reset()

is_done = False

while 1:
    
    k = cv2.waitKey(40)
    if k == ord("a"):
        s, r, is_done = arm.step(np.array([5, 0, 0]))
        print(s, r)
    elif k == ord("b"):
        s, r, is_done = arm.step(np.array([-5, 0, 0]))
        print(s, r)
    elif k == ord("c"):
        s, r, is_done = arm.step(np.array([0, 5, 0]))
        print(s, r)
    elif k == ord("d"):
        s, r, is_done = arm.step(np.array([0, -5, 0]))
        print(s, r)
    elif k == ord("e"):
        s, r, is_done = arm.step(np.array([0, 0, 5]))
        print(s, r)
    elif k == ord("f"):
        s, r, is_done = arm.step(np.array([0, 0, -5]))
        print(s, r)
    # q/Q (exit)
    elif k == ord("q") or k == ord("Q"):
        break

    if is_done:
        arm.reset()
        is_done = False

cv2.destroyAllWindows()
