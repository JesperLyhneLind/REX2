import cv2
import particle
import camera
import numpy as np
import time
from time import sleep
from timeit import default_timer as timer
import sys
import drive_functionality
import numpy.random as rand
import time
import math


# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True # Whether or not we are running on the Arlo robot
def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot
if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("robot.py")
try:
    import robot
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False
# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)
# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [6, 7]
landmarks = {
    7: (0.0, 0.0),  # Coordinates for landmark 1
    6: (0.0, 300.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks
landmarksSeen = []
def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)
    return (255.0*r, 255.0*g, 255.0*b)
def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""
    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250
    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]
    world[:] = CWHITE # Clear background to white
    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())
    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)
    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)
    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
                                 ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)
def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*rand.ranf() - 100.0, 600.0*rand.ranf() - 250.0, np.mod(2.0*np.pi*rand.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)
    return particles
# Main program #
try:
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)
        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)
    # Initialize particles
    num_particles = 1000
    particles = initialize_particles(num_particles)
    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec
    # Initialize the robot (XXX: You do this)
    if onRobot:
        otto = robot.Robot()
    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)
    # Draw map
    draw_world(est_pose, particles, world)
    print("Opening and initializing camera")
    if camera.isRunningOnArlo():
        cam = camera.Camera(0, 'arlo', useCaptureThread = True)
    else:
        cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)
    while True:
        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
        # Use motor controls to update particles
        # XXX: Make the robot drive
        #FLYT PARTIKLER
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
                [p.move_particle(0, 10, 0) for p in particles]
                sleep(0.18)
            elif action == ord('x'): # Backwards
                velocity -= 4.0
                [p.move_particle(0, -10, 0) for p in particles]
                sleep(0.18)
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left      
                sleep(0.18)
            elif action == ord('d'): # Right
                angular_velocity -= 0.2
                [p.move_particle(5, 0, 0.45) for p in particles]   
                sleep(0.18)


        particle.add_uncertainty(particles, 8, 0.025) #noise sigmas are centimeter and radians
        # Fetch next frame
        
        colour = cam.get_next_frame()
        # Detect objects
        d_objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        def distance_observation_model(d_M, d_i, sigma_d):
            # Calculate the Gaussian PDF
            pdf_value = (1 / np.sqrt(2 * np.pi * sigma_d**2)) * math.exp(-(d_M - d_i)**2 / (2 * sigma_d**2))
            return pdf_value
        
        def angle_observation_model(phi_M, phi_i, sigma_theta):
            # Calculate the Gaussian PDF
            pdf_value = (1 / np.sqrt(2 * np.pi * sigma_theta**2)) * math.exp(-(phi_M - phi_i)**2 / (2 * sigma_theta**2))
            return pdf_value
        if not isinstance(d_objectIDs, type(None)):
            # List detected objects
            objectIDs = np.unique(d_objectIDs)
            print("Object ID = ", objectIDs, ", Distance = ", dists, ", angle = ", angles)
            
            #     # XXX: Do something for each detected object - remember, the same ID may appear several times.
            #     # Use the camera function to get the measured distance
            #     objectType, distance, angle, colourProb = cam.get_object(colour)
                            
            #objectType, distance, angle, colourProb = cam.get_object(colour)
            for par in particles:
                par.setWeight(1.0)
                for i in range(len(objectIDs)):
                    if objectIDs[i] in landmarkIDs:
                        particle_distance = np.sqrt(((landmarks[objectIDs[i]])[0] - par.getX())**2 + ((landmarks[objectIDs[i]])[1] - par.getY())**2)
                        sigma_d = 8 # try value 20cm
                        p_d = distance_observation_model(dists[i], particle_distance, sigma_d)
                        #angle
                        sigma_theta = 0.025# try value 0.3 radians
                        uvec_robot = [((landmarks[objectIDs[i]])[0] - par.getX()) / particle_distance, 
                                    ((landmarks[objectIDs[i]])[1] - par.getY()) / particle_distance]
                        uvec_orientation = [np.cos(par.getTheta()), np.sin(par.getTheta())]
                        
                        #fortegn skal kun byttes rundt, hvis på webcam?
                        uvec_orientation_ortho = [-np.sin(par.getTheta()), np.cos(par.getTheta())]
                        
                        phi_i = np.sign(np.dot(uvec_robot, uvec_orientation_ortho))*np.arccos(np.dot(uvec_robot,uvec_orientation)) 
                        
                        p_phi = angle_observation_model(angles[i], phi_i, sigma_theta)
                        p_x = p_d * p_phi
                        #update weights
                        par.setWeight(par.getWeight() * p_x)
            # Normalize particle weights
            total_weight = sum([p.getWeight() for p in particles])
            normalized_weights = []     
            for par in particles:
                par.setWeight(par.getWeight() / total_weight)
                normalized_weights.append(par.getWeight())
            # Resampling
            r_particles = rand.choice(a=particles, replace=True, p=normalized_weights, size=len(particles))
            #particles = [copy.deepcopy(p) for p in r_particles]
            particles = [particle.Particle(p.getX(), p.getY(), p.getTheta(), p.getWeight()) for p in r_particles]
            # Draw detected objects
            cam.draw_aruco_objects(colour)
            #landmarksSeen.append(landmarks_in_map) # Has the robot already seen one box 
            print(np.std(normalized_weights))
            if np.std(normalized_weights) < 0.006:
                break
            
            landmarks_in_map = list(filter(lambda x: x in landmarkIDs, objectIDs))
            for i in landmarks_in_map:
                if not landmarksSeen.__contains__(i):
                    landmarksSeen.append(i) # Has the robot already seen one box
            
            print("landmarks_in_map", landmarks_in_map)
            print("landmarksSeen", landmarksSeen)
            if len(landmarks_in_map) == 1 and len(landmarksSeen) < 2: 
                drive_functionality.turn(drive_functionality.Direction.Right, 30)
                [p.move_particle(0, 0, math.radians(30)) for p in particles]  
            
            
        
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)
    
            # Show frame
            cv2.imshow(WIN_RF1, colour)
            # Show world
            cv2.imshow(WIN_World, world)
        
        
    
finally: 
    # Make sure to clean up even if an exception occurred
    # Close all windows
    cv2.destroyAllWindows()
    # Clean-up capture thread
    cam.terminateCaptureThread()
print("done")
print("est_pose:", est_pose.getX(), est_pose.getY())