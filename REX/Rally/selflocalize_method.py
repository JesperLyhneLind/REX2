import cv2
import particle
import camera
import numpy as np
from time import sleep
from timeit import default_timer as timer
import sys
import numpy.random as rand
import time
import math
import drive_functionality
import robot
from enum import Enum

class Direction(Enum):
    Left = 1
    Right = 2

sys.path.append("robot.py")
cam = camera.Camera(0, 'arlo', useCaptureThread = True)
otto = robot.Robot()
landmarksSeen = []

def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*rand.ranf() - 100.0, 600.0*rand.ranf() - 250.0, np.mod(2.0*np.pi*rand.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles


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
        particle.add_uncertainty(particles, 8, 0.25) #noise sigmas are centimeter and radians
        # Fetch next frame
        
        colour = cam.get_next_frame()
        # Detect objects
        d_objectIDs, dists, angles = cam.detect_aruco_objects(colour)

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
                        sigma_d = 8 # try value 20cm
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
            cam.draw_aruco_objects(colour)
            #landmarksSeen.append(landmarks_in_map) # Has the robot already seen one box 
            
            landmarks_in_map = list(filter(lambda x: x in landmarkIDs, objectIDs))
            for i in landmarks_in_map:
                if not landmarksSeen.__contains__(i):
                    landmarksSeen.append(i) # Has the robot already seen one box
            
            print("landmarks_in_map", landmarks_in_map)
            print("landmarksSeen", landmarksSeen)
            if len(landmarks_in_map) == 1 and len(landmarksSeen) < 2: 
                print("have only seen one known landmark")
                drive_functionality.turn(drive_functionality.Direction.Right, 30)
                sleep(1)
                [p.move_particle(0, 0, math.radians(30)) for p in particles]  
            elif len(landmarksSeen) >= 2: 
                print("saw at least two known landmarks")
                if np.std(normalized_weights) < 0.0008:
                    print("done")
                    break
                print("std too high:", np.std(normalized_weights))
            else: #he only sees boxes that are not in dictionary
                print("no boxes seen")
                drive_functionality.turn(drive_functionality.Direction.Right, 30)
                sleep(1)
                [p.move_particle(0, 0, math.radians(30)) for p in particles] 
            
        else:
            # No observation - reset weights to uniform distribution
            print("no landmarks seen")
            drive_functionality.turn(drive_functionality.Direction.Right, 30)
            sleep(1)
            [p.move_particle(0, 0, -math.radians(30)) for p in particles]  
            for p in particles:
                p.setWeight(1.0/len(particles))
    est_pose = particle.estimate_pose(particles) 
    print("est_pose:", est_pose.getX(), est_pose.getY())
    return est_pose
