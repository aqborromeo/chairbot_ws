import subprocess
import rospy
import math

def run_object_detection():
    # Run the object detection script
    # process = subprocess.Popen(['rosrun', 'your_package_name', 'object_detection_script.py'], 
    #                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # stdout, stderr = process.communicate()
    
    # if process.returncode != 0:
    #     rospy.logerr("Object detection script failed: %s", stderr)
    #     return None
    
    # bounding_boxes = parse_bounding_boxes(stdout)
    bounding_boxes = {'bboxes': [[166.5989990234375, 162.5854949951172, 198.4309844970703, 299.68951416015625]], 'bboxes_labels': ['window'], 'polygons': [], 'polygons_labels': []}
    return bounding_boxes

def convert_bbox_to_vertices(yolo_bbox):
    # Extract the bounding box coordinates
    x_min, y_min, x_max, y_max = yolo_bbox
    
    # Create the vertices in clockwise order
    vertices = [
        (x_min, y_min),  # Top-left
        (x_max, y_min),  # Top-right
        (x_max, y_max),  # Bottom-right
        (x_min, y_max)   # Bottom-left
    ]
    
    return vertices

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
        perp_vector = (-unit_direction[1], unit_direction[0])

        # Calculate the angle of the perpendicular vector in radians
        angle = math.atan2(perp_vector[1], perp_vector[0])
        
        # Calculate marker placement distance from edge
        marker_distance = w / 2 + perpendicular_distance_factor * w  # Total distance from edge to marker
        
        # Calculate number of markers based on spacing
        num_markers = int(length // spacing)  # Number of markers
        
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
    
    robot_base_width = 2.00
    bounding_boxes = run_object_detection()
    
    seating_positions = []
    if bounding_boxes and bounding_boxes['bboxes']:
        for i, bbox in enumerate(bounding_boxes['bboxes']):
            polygon = convert_bbox_to_vertices(bbox)
            seating_positions.append(calculate_markers(polygon, w=robot_base_width))

if __name__ == '__main__':
    main()