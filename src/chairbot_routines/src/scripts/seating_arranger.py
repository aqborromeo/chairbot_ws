import subprocess
import rospy
import math
import json

def get_object_detection(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def convert_bbox_to_vertices(yolo_bbox, res=0.05):
    # Extract the bounding box coordinates
    x_min, y_min, x_max, y_max = yolo_bbox
    x_min, y_min, x_max, y_max = x_min * res, y_min * res, x_max * res, y_max * res
    
    
    # Create the vertices in clockwise order
    vertices = [
        (x_min, y_min),  # Top-left
        (x_max, y_min),  # Top-right
        (x_max, y_max),  # Bottom-right
        (x_min, y_max)   # Bottom-left
    ]
    
    return vertices

def obb_to_polygon(obb):
    # Extract the center, angle, and dimensions from the OBB
    center = obb['center']
    angle = obb['angle']
    width = obb['dimensions']['width']
    height = obb['dimensions']['height']
    
    # half dimensions
    half_width = width / 2
    half_height = height / 2
    
    # corners of the OBB
    corners = [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height)
    ]
    
    # Rotate and translate the corners
    polygon = []
    for corner in corners:
        # Rotate the corner
        x_rotated = (corner[0] * math.cos(math.radians(angle))) - (corner[1] * math.sin(math.radians(angle)))
        y_rotated = (corner[0] * math.sin(math.radians(angle))) + (corner[1] * math.cos(math.radians(angle)))
        
        # Translate to the center
        x_translated = x_rotated + center['x']
        y_translated = y_rotated + center['y']
        
        polygon.append((x_translated, y_translated))
    
    return polygon


def calculate_markers(polygon_coords, w, spacing=1.5, perpendicular_distance_factor=0.5):
    markers = []

    # Iterate through each edge of the polygon
    num_vertices = len(polygon_coords)
    for i in range(num_vertices):
        # Get the current vertex (p1) and the next vertex (p2)
        p1 = polygon_coords[i]
        p2 = polygon_coords[(i + 1) % num_vertices]

        # Calculate direction vector
        direction = (p2[0] - p1[0], p2[1] - p1[1])
        
        # Calculate the length of the direction vector
        length = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
        
        # Normalize the direction vector
        if length == 0:
            continue  # Skip if the edge length is zero (coincident points)
        
        unit_direction = (direction[0] / length, direction[1] / length)
        
        # Calculate perpendicular vector (90 degrees rotation) pointing outward
        perp_vector = (unit_direction[1], -unit_direction[0])

        # Calculate the angle of the perpendicular vector in radians
        angle = math.atan2(perp_vector[1], perp_vector[0])
        
        # Calculate marker placement distance from edge
        marker_distance = w / 2 + perpendicular_distance_factor * w  # Total distance from edge to marker
        
        # Calculate number of markers based on spacing
        num_markers = int(length // (spacing * w))  # Number of markers
        
        for j in range(1, num_markers):  # Start from 1 to num_markers inclusive
            # Calculate marker position
            marker_position = (
                p1[0] + (j * (direction[0] / num_markers)) + (marker_distance * perp_vector[0]),
                p1[1] + (j * (direction[1] / num_markers)) + (marker_distance * perp_vector[1]),
                angle
            )
            markers.append(marker_position)
    return markers

def main():
    rospy.init_node('seating_arranger', anonymous=True)
    
    robot_base_width = 0.7

    # Uncomment for bounding box format
    # bounding_boxes = get_object_detection('src/chairbot_routines/src/data/bbox_detections.json')['bboxes']

    # Uncomment for OBB format
    bounding_boxes = get_object_detection('src/chairbot_routines/src/data/obb_detections.json')['obb']
    
    seating_positions = []
    if bounding_boxes:
        for i, bbox in enumerate(bounding_boxes):
            # Uncomment for bounding box format
            # polygon = convert_bbox_to_vertices(bbox)

            # Uncomment for OBB format
            polygon = obb_to_polygon(bbox)
            seating_positions.extend(calculate_markers(polygon, w=robot_base_width, spacing=1.5, perpendicular_distance_factor=0.25))

    
    with open('src/chairbot_routines/src/data/waypoints.json', 'w') as json_file:
        json.dump(seating_positions, json_file, indent=4)

if __name__ == '__main__':
    main()