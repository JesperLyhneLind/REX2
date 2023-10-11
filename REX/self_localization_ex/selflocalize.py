import cv2
import particle
import camera
import numpy as np
import time
from time import sleep
from timeit import default_timer as timer
import sys
import numpy.random as rand

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
    sys.path.append("../REX/robot.py")


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
landmarkIDs = [1, 2]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (300.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks





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
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
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
    otto = robot.Robot()

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    draw_world(est_pose, particles, world)

    print("Opening and initializing camera")
    camera = camera.Camera()
    if camera.isRunningOnArlo():
        cam = camera.Camera(0, 'arlo', useCaptureThread = True)
    else:
        cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)

    # SIR-algorithm from q1.py
    def norm(x, my, sig):
        return (1/(np.sqrt(2*np.pi))*sig) * (np.e**((-1/2)*((x-my)**2/sig**2)))

    def p(x):
        return 0.3 * norm(x, 2.0, 1) + 0.4 * norm(x, 5.0, 2) + 0.3 * norm(x, 9.0, 1) 

    def q(x):
        if 0 <= x < 16:
            return 1/15
        else:
            return 0

    data = rand.uniform(low=0.0, high=15.0, size=20) 
    data1 = rand.uniform(low=0.0, high=15.0, size=100) 
    data2 = rand.uniform(low=0.0, high=15.0, size=1000) 


    def SIR(data, p, q):
        w = []
        w_norm = []
        for elem in data:
            w.append(p(elem)/q(elem))

        for elem in w:
            w_norm.append(elem/(sum(w)))

        return rand.choice(a=data, replace=True, p=w_norm, size=len(data))

    while True:

        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
                print(otto.go_diff(velocity, velocity, 1, 1))
                sleep(0.18)
            elif action == ord('x'): # Backwards
                velocity -= 4.0
                print(otto.go_diff(velocity, velocity, 0, 0))
                sleep(0.18)
            elif action == ord('s'): # Stop
                velocity = 0.0
                print(otto.stop())
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
                print(otto.go_diff(angular_velocity, angular_velocity, 1, 0))
                sleep(0.18)
            elif action == ord('d'): # Right
                angular_velocity -= 0.2
                print(otto.go_diff(angular_velocity, angular_velocity, 0, 1))
                sleep(0.18)
        
        # Use motor controls to update particles
        # XXX: Make the robot drive
        # XXX: You do this
        # LOOK ABOVE.

        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        
        def distance_observation_model(d_M, x_i, y_i, d_i, sigma_d):
            # Calculate the Gaussian PDF
            pdf_value = (1 / np.sqrt(2 * np.pi * sigma_d**2)) * np.exp(-(d_M - d_i)**2 / (2 * sigma_d**2))
            return pdf_value

        if not isinstance(objectIDs, type(None)):
            # List detected objects
            for i in range(len(objectIDs)):
                print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
                # XXX: Do something for each detected object - remember, the same ID may appear several times.

                # Use the camera function to get the measured distance
                measured_distance = cam.get_object(objectIDs)[1]

                # Find the corresponding true distance to the landmark
                true_distance = np.sqrt((landmarks[objectID][0] - est_pose.getX())**2 + (landmarks[objectID][1] - est_pose.getY())**2)

                # Use the distance observation model to update particle weights
                for p in particles:
                    particle_distance = np.sqrt((landmarks[objectID][0] - p.getX())**2 + (landmarks[objectID][1] - p.getY())**2)
                    observation_model = distance_observation_model(measured_distance, p.getX(), p.getY(), particle_distance, add_uncertainty(particles, sigma, sigma_d))
                    p.setWeight(p.getWeight() * observation_model)

                #objectIDs = list(set(objectIDs)) # removes duplicates of IDs from list.
                #dists = list(set(dists)) # removes duplicates of distances.
                #angles = list(set(angles)) # removes duplicates of angles.

            # Normalize particle weights
            total_weight = sum([p.getWeight() for p in particles])
            for p in particles:
                p.setWeight(p.getWeight() / total_weight)

            # Compute particle weights
            # XXX: You do this



            # Bruger SIR fra q1
            # Spørgsmål til Kim: Skal q have det samme interval som det vi lavede i q1.py
            SIR(particles, q1.p, q1.q)


            # Resampling
            # XXX: You do this
            particles = SIR([p.getWeight() for p in particles], p, q)
            particles = SIR_particles(particles, lambda x: x.getWeight(), lambda x: 1.0)  # Assuming a uniform proposal distribution

            # Draw detected objects
            cam.draw_aruco_objects(colour)
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

