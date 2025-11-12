import cv2
import numpy as np
import time

def orthogonal_2d_vector(vector):
    """Output: orthogonal unit vector of input vector in 2D"""
    ortho_vector = np.array([vector[1], -vector[0]])
    ortho_vector = ortho_vector / np.linalg.norm(ortho_vector)

    return ortho_vector
    
def circle_center(p0, v0, p1):
    """Output: virtual center of the circle go through p0 and p1.
    And use v1 as tangent. """
    
    ortho_v0 = orthogonal_2d_vector(v0)

    mid_p0_p1 = (p0 + p1) * 0.5

    v01 = p1 - p0
    ortho_v01 = orthogonal_2d_vector(v01)

    x0 = p0[0]
    y0 = p0[1]
    xm = mid_p0_p1[0]
    ym = mid_p0_p1[1]

    vx = ortho_v0[0]
    vy = ortho_v0[1]

    k = 2*xm**2 + 2*ym**2 - 2*x0*xm - 2*y0*ym
    k = k - (2*x0*xm - 2*x0**2 + 2*y0*ym - 2*y0**2)
    k = k/ (2 * (vx*xm - vx*x0 + vy*ym - vy*y0))

    cc = p0 + k * ortho_v0

    return cc

p0 = np.array([100, 30])

v0 = np.array([-0.2, 1])

p1 = np.array([280, 180])


image = np.zeros((480, 640, 3), np.uint8)
image2 = cv2.circle(image, p0, radius=10, color=(0, 0, 255), thickness=-1)
image2 = cv2.circle(image, p1, radius=10, color=(150, 50, 155), thickness=-1)

t = time.time()

cc = np.rint(circle_center(p0, v0, p1)).astype(int)

rr = np.rint(np.linalg.norm(p0 - cc)).astype(int)

exptime = time.time() - t
print("exptime:", exptime)
image3 = cv2.circle(image, (cc[0], cc[1]), radius=rr, color=(0, 255, 255), thickness=1)


cv2.imshow("Red Point on Black Image", image3)


cv2.waitKey(0)
cv2.destroyAllWindows()
