import cv2
import numpy as np
import particle
import robot
import math
from time import sleep
from enum import Enum
import camera
import sys
import numpy.random as rand
import time
import math
import robot


#cam = camera.Camera(0, 'arlo', useCaptureThread = True)
otto = robot.Robot()
class Direction(Enum):
    Left = 1
    Right = 2

sys.path.append("robot.py")
otto = robot.Robot()
landmarksSeen = []



# Landmarks.
landmarkIDs = [1, 2, 3, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    4: (400.0, 300.0)  # Coordinates for landmark 4
}
landmarks_inOrder = [1,2,3,4,1]
landmarks_index = 0

def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*rand.ranf() - 100.0, 600.0*rand.ranf() - 250.0, np.mod(2.0*np.pi*rand.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)
    landmarksSeen = []
    return particles


# Checks if there's any object in the path of the robot.  
def check():
    #while(True):
    Front_sensor = otto.read_front_ping_sensor()
    Right_sensor = otto.read_right_ping_sensor()
    Left_sensor = otto.read_left_ping_sensor()

        # if Left_sensor < 300 or Right_sensor < 300 or Front_sensor < 400:
        #     print("Left: " + str(Left_sensor))
        #     print("Front: " + str(Front_sensor))
        #     print("Right: " + str(Right_sensor))
    return Left_sensor, Right_sensor, Front_sensor
        
# Turns the robot angle degrees.
def turn(dir: Direction, angle: int):
    if dir == Direction.Left:
        print(otto.go_diff(40, 40, 0, 1))
        sleep(angle/65) 
        print(otto.stop())
        sleep(0.18)
    else:
        print(otto.go_diff(40, 40, 1, 0))
        sleep(angle/65)
        print(otto.stop())
        sleep(0.18)

# Drives a certain amount of meters until obstacle is detected in front of it.
def iDrive(meters):
    print(otto.go_diff(70, 71, 1, 1))
    print("driving meters ", meters)
    start = time.perf_counter()
    while True:
        Left_sensor, Right_sensor, Front_sensor = check()
        if Front_sensor < 400 or Left_sensor < 400 or Right_sensor < 400:
            print(otto.stop())
            print("Left: " + str(Left_sensor))
            print("Front: " + str(Front_sensor))
            print("Right: " + str(Right_sensor))
            print("oh no, obstacle detected!! :( miv)")
            return 1 # Otto stopped because of obstacle
        if (time.perf_counter() - start > (2.6*meters)):
            print(otto.stop())
            sleep(0.18)
            print("oh yes I drived meters ", meters)
            return 0 # Otto drived intended distance
   



# Drives the robot and checks which direction to go for avoiding an object.
def drive(): 
    #otto.go_diff(70, 71, 1, 1)
    Left_sensor, Right_sensor, Front_sensor = check()

    if Left_sensor >= Right_sensor:
        print("left")
        turn(Direction.Left, 45)
        iDrive(1)
        turn(Direction.Right, 90)
        iDrive(1)
        turn(Direction.Left, 45)
        iDrive(1)
        

    elif Right_sensor > Left_sensor:
        print("Right")
        turn(Direction.Right, 45)
        iDrive(1)
        turn(Direction.Left, 90)
        iDrive(1)
        turn(Direction.Right, 45)
        iDrive(1)
    else:
        pass

def distance_observation_model(d_M, d_i, sigma_d):
    # Calculate the Gaussian PDF
    pdf_value = (1 / np.sqrt(2 * np.pi * sigma_d**2)) * math.exp(-(d_M - d_i)**2 / (2 * sigma_d**2))
    return pdf_value

def angle_observation_model(phi_M, phi_i, sigma_theta):
    # Calculate the Gaussian PDF
    pdf_value = (1 / np.sqrt(2 * np.pi * sigma_theta**2)) * math.exp(-(phi_M - phi_i)**2 / (2 * sigma_theta**2))
    return pdf_value

def self_localize(landmarks, landmarkIDs):
    particles = initialize_particles(1000)
    while True:# and time_running < 15: #stop loop after 15 seconds
        particle.add_uncertainty(particles, 15, 0.25) #noise sigmas are centimeter and radians
        # Fetch next frame
        
        colour = camera.Camera(0, 'arlo', useCaptureThread = True).get_next_frame()
        # Detect objects
        d_objectIDs, dists, angles = camera.Camera(0, 'arlo', useCaptureThread = True).detect_aruco_objects(colour)

        if not isinstance(d_objectIDs, type(None)):
            print("estimating pose")
            # List detected objects
            objectIDs, indices = np.unique(d_objectIDs, return_index=True)
            unique_dists = [dists[i] for i in indices]
            unique_angles = [angles[i] for i in indices]  

            print("Object ID = ", objectIDs, ", Distance = ", unique_dists, ", angle = ", unique_angles)
                                       
            #objectType, distance, angle, colourProb = cam.get_object(colour)
            for par in particles:
                par.setWeight(1.0)
                for i in range(len(objectIDs)):
                    if objectIDs[i] in landmarkIDs:
                        particle_distance = np.sqrt(((landmarks[objectIDs[i]])[0] - par.getX())**2 + ((landmarks[objectIDs[i]])[1] - par.getY())**2)
                        #sigma_d = 5 # try value 20cm
                        sigma_d = 15 # try value 20cm
                        p_d = distance_observation_model(dists[i], particle_distance, sigma_d)                       
                        #angle
                        #sigma_theta = 0.03# try value 0.3 radians
                        sigma_theta = 0.25# try value 0.3 radians
                        uvec_robot = [((landmarks[objectIDs[i]])[0] - par.getX()) / particle_distance, 
                                    ((landmarks[objectIDs[i]])[1] - par.getY()) / particle_distance]
                        uvec_orientation = [np.cos(par.getTheta()), np.sin(par.getTheta())]
                        
                        #fortegn skal kun byttes rundt, hvis på webcam?
                        uvec_orientation_ortho = [-np.sin(par.getTheta()), np.cos(par.getTheta())]
                        
                        phi_i = np.sign(np.dot(uvec_robot, uvec_orientation_ortho))*np.arccos(np.dot(uvec_robot,uvec_orientation)) 
                        
                        p_phi = angle_observation_model(angles[i], phi_i, sigma_theta)
                        #print("p_phi:", p_phi)
                        if p_phi == 0.0:
                            print("p_phi = 0")
                            exit
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
            camera.Camera(0, 'arlo', useCaptureThread = True).draw_aruco_objects(colour)
            #landmarksSeen.append(landmarks_in_map) # Has the robot already seen one box 
            
            landmarks_in_map = list(filter(lambda x: x in landmarkIDs, objectIDs))
            for i in landmarks_in_map:
                if not landmarksSeen.__contains__(i):
                    landmarksSeen.append(i) # Has the robot already seen one box
            
            print("landmarks_in_map", landmarks_in_map)
            print("landmarksSeen", landmarksSeen)
            if len(landmarks_in_map) == 1 and len(landmarksSeen) < 2: 
                print("have only seen one known landmark")
                turn(Direction.Right, 30)
                sleep(1)
                [p.move_particle(0, 0, -math.radians(30)) for p in particles]  
            elif len(landmarksSeen) >= 2: 
                print("saw at least two known landmarks UwU")
                if np.std(normalized_weights) < 0.0015:
                    print("done")
                    break
                print("My std is too high:", np.std(normalized_weights))
            else: #he only sees boxes that are not in dictionary
                print("no boxes seen")
                turn(Direction.Right, 30)
                sleep(1)
                [p.move_particle(0, 0, -math.radians(30)) for p in particles] 
            
        else:
            # No observation - reset weights to uniform distribution
            print("no landmarks seen") 
            turn(Direction.Right, 30)
            sleep(1)
            [p.move_particle(0, 0, -math.radians(30)) for p in particles]  
            for p in particles:
                p.setWeight(1.0/len(particles))
    est_pose = particle.estimate_pose(particles) 
    print("est_pose:", est_pose.getX(), est_pose.getY())
    return est_pose
        
# Funtion for finding the orientation from the robot towards its next goal in degrees.
def orientation(id_index):
    # The estimate of the robots current pose
    robot_pose = self_localize(landmarks, landmarkIDs)

    # Calculate the vector, that the robot should drive to in order to visit the goal.
    print("goal: ", (landmarks[id_index]))
    vec_posX = (landmarks[id_index])[0] - robot_pose.getX() # x-coordinate
    vec_posY = (landmarks[id_index])[1] - robot_pose.getY() # y-coordinate

    # Calculate the new theta.
    vec_theta = math.degrees(math.atan2(vec_posY, vec_posX) - robot_pose.getTheta())
 
    # Returns the orientation-vector.
    return vec_theta, vec_posX, vec_posY # degrees instead of radians.

# Avoids an object and drives the robot 0.3m if there's nothing detected in front of it.
def avoid():
    Left_sensor, Right_sensor, Front_sensor = check()

    if Left_sensor >= Right_sensor:
        print("Turning left - avoid, 45")
        turn(Direction.Left, 45)
    else:
        print("Turning right - avoid, 45")
        turn(Direction.Right, 45)

    iDrive(0.5)
        
# Turns the robot and drives towards the goal while avoiding objects.
def driveAlongVec(vecX, vecY, theta, goalID):
    distance = math.sqrt(vecX**2 + vecY**2) # pythagorean theorem.
    print("driving " + str(distance-40) + " cm to goal along " + str(vecX) + " " + str(vecY))
    # Let the robot face the goal.
    if np.sign(theta) == 1:
        turn(Direction.Right, theta) # right.
        print("turning right with degrees ", theta)
    elif np.sign(theta) == -1:
        turn(Direction.Left, abs(theta)) # left.
        print("turning right with degrees ", abs(theta))
    # If yes go until sensor sensors
        # if canYouSeeTarget(goalID, cam):
        #     print("I can see the target!!")
        #     res = drive_functionality.iDrive(40) # Drives until reaching landmark.
        #     return 2
    # If not do as before:
    # Drives the robot towards the goal, while there's longer than 0,4m to the goal.
    if iDrive((distance-50)/100) == 1:
        print("avoiding")
        avoid()
        return 0 # Ends with avoid
    else:
        print("I drived distance:)")
        return 1 # Intended drived distance.

# Turns the robot towards goal.
while landmarks_index < 5:
    #Check orientation
    vec_t, vec_x, vec_y = orientation(landmarks_inOrder[landmarks_index], camera.Camera(0, 'arlo', useCaptureThread = True))
    # If it has seen the goal and drives blindly or the distance has been reached successfully
    if driveAlongVec(vec_x, vec_y, vec_t, landmarks_inOrder[landmarks_index], camera.Camera(0, 'arlo', useCaptureThread = True)) == 1 : 
        print("\n NOW INCREMENTED!!!!! \n")
        landmarks_index += 1