import cv2
import particle
import camera
import numpy as np
import time
from time import sleep
from timeit import default_timer as timer
import sys
import numpy.random as rand
import time
import math
import robot

sys.path.append("robot.py")
cam = camera.Camera(0, 'arlo', useCaptureThread = True)
otto = robot.Robot()

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

def self_localize(landmarks, landmarkIDs, num_particles):
    time1 = time.time()
    while True:# and time_running < 15: #stop loop after 15 seconds
        particle.add_uncertainty(particles, 2, 0.025) #noise sigmas are centimeter and radians
        # Fetch next frame
        
        colour = cam.get_next_frame()
        # Detect objects
        d_objectIDs, dists, angles = cam.detect_aruco_objects(colour)

        if not isinstance(d_objectIDs, type(None)):
            # List detected objects
            objectIDs = np.unique(d_objectIDs)
            for par in particles:
                par.setWeight(1.0)
                for i in range(len(objectIDs)):
                    if objectIDs[i] in landmarkIDs:
                        particle_distance = np.sqrt(((landmarks[objectIDs[i]])[0] - par.getX())**2 + ((landmarks[objectIDs[i]])[1] - par.getY())**2)
                        sigma_d = 2
                        p_d = distance_observation_model(dists[i], particle_distance, sigma_d)
                        sigma_theta = 0.025
                        
                        uvec_robot = [((landmarks[objectIDs[i]])[0] - par.getX()) / particle_distance, 
                                    ((landmarks[objectIDs[i]])[1] - par.getY()) / particle_distance]
                        uvec_orientation = [np.cos(par.getTheta()), np.sin(par.getTheta())]

                        uvec_orientation_ortho = [-np.sin(par.getTheta()), np.cos(par.getTheta())]
                        
                        phi_i = np.sign(np.dot(uvec_robot, uvec_orientation_ortho))*np.arccos(np.dot(uvec_robot,uvec_orientation)) 
                        
                        p_phi = angle_observation_model(angles[i], phi_i, sigma_theta)
                        
                        p_x = p_d * p_phi

                        par.setWeight(par.getWeight() * p_x)

            # Normalize particle weights
            total_weight = sum([p.getWeight() for p in particles])
            normalized_weights = []     
            for par in particles:
                par.setWeight(par.getWeight() / total_weight)
                normalized_weights.append(par.getWeight())
            
            # Resampling
            r_particles = rand.choice(a=particles, replace=True, p=normalized_weights, size=len(particles))
            particles = [particle.Particle(p.getX(), p.getY(), p.getTheta(), p.getWeight()) for p in r_particles]

            if np.std(normalized_weights) < 0.00115:
                return particles
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)
    
        #time_running = time1-time.time()
