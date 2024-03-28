import numpy as np
import matplotlib.pyplot as plt

def regular_pentagon_points():
    # Vertices of the regular pentagon
    vertices = [[-1, 0]]  # Vertex at (-1, 0)

    # Generate other vertices in a counter-clockwise manner
    for i in range(1, 5):
        angle = (i * 2 * np.pi / 5) + np.pi  # Shifted by 180 degrees to start from (-1, 0)
        x = np.cos(angle)
        y = np.sin(angle)
        vertices.append([x, y])

    # Number of equidistant points on each side (excluding the vertices)
    num_points = 50
    
    # Initialize list to store coordinates of equidistant points
    equidistant_points = []
    
    # Compute equidistant points on each side of the pentagon
    for i in range(5):
        start_point = vertices[i]
        end_point = vertices[(i + 1) % 5]
        
        for j in range(num_points):
            t = (j + 1) / (num_points + 1)  # Parameter for interpolation
            x = (1 - t) * start_point[0] + t * end_point[0]
            y = (1 - t) * start_point[1] + t * end_point[1]
            equidistant_points.append([x, y])
        
        # Add start vertex to the end of the list
        equidistant_points.append(start_point)
    
    return equidistant_points

# Generate equidistant points
points = regular_pentagon_points()

# Plotting the regular pentagon and equidistant points
plt.figure(figsize=(8, 8))
vertices = np.array(regular_pentagon_points()[:5])  # Extracting vertices for plotting
plt.plot(vertices[:, 0], vertices[:, 1], 'b-')  # Plotting sides of the pentagon
plt.plot(vertices[:, 0], vertices[:, 1], 'bo')  # Plotting vertices of the pentagon
plt.plot([point[0] for point in points], [point[1] for point in points], 'ro')  # Plotting equidistant points
plt.title('Regular Pentagon with 50 Equidistant Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.axis('equal')
plt.show()

