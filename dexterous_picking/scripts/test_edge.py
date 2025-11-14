import numpy as np
import matplotlib.pyplot as plt
import cv2

def closest_edge_point(pt, edges):
    closest_distance = float('inf')
    closest_point = None
    for edge in edges:
        p1, p2 = edge
        dist = np.linalg.norm(np.cross(p2 - p1, p1 - pt)) / np.linalg.norm(p2 - p1)
        if dist < closest_distance:
            closest_distance = dist
            closest_edge = edge
            closest_point = p1 + ((np.dot(pt - p1, p2 - p1)) / np.dot(p2 - p1, p2 - p1)) * (p2 - p1)
    return closest_edge, closest_point

# Load the image
image = cv2.imread('your_image_path_here.jpg') # Replace with your image path
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Replace these example edge coordinates with your table edge coordinates
edges = [
    (np.array([10, 10]), np.array([10, 200])),
    (np.array([10, 200]), np.array([300, 200])),
    (np.array([300, 200]), np.array([300, 10])),
    (np.array([300, 10]), np.array([10, 10]))
]

# Example given point on the plate
given_point = np.array([150, 100])

edge, perpendicular_point = closest_edge_point(given_point, edges)

# Plotting
fig, ax = plt.subplots()
ax.imshow(image)

# Plot the given point
ax.plot(given_point[0], given_point[1], 'bo')

# Plot the closest edge
ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'g-')

# Plot the perpendicular point on the edge
ax.plot(perpendicular_point[0], perpendicular_point[1], 'ro')

plt.show()
