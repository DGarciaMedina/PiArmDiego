from env import MyArm2D
import cv2
import numpy as np

arm = MyArm2D()

while 1:
    arm.render()

    k = cv2.waitKey(40)
    # UP
    if k == 82:
        arm.step(np.array([5, 0, 0]))
    # DOWN
    elif k == 84:
        arm.step(np.array([-5, 0, 0]))
    # RIGHT
    elif k == 83:
        arm.step(np.array([0, 5, 0]))
    # LEFT
    elif k == 81:
        arm.step(np.array([0, -5, 0]))
    # a/A
    elif k == ord("a") or k == ord("A"):
        arm.step(np.array([0, 0, 5]))
    # s/S
    elif k == ord("s") or k == ord("S"):
        arm.step(np.array([0, 0, -5]))
    # q/Q (exit)
    elif k == ord("q") or k == ord("Q"):
        break

cv2.destroyAllWindows()
