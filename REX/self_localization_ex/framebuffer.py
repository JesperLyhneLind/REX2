# import copy
import threading


class FrameBuffer(object):
    """This class represents a framebuffer with a front and back buffer storing frames. 
    Access to the class is thread safe (controlled via an internal lock)."""
    
    def __init__(self):
        self.frameBuffer = [None, None]  # Initialize the framebuffer with None references
        self.currentBufferIndex = 0 
        self.lock = threading.Lock()
        
    def get_frame(self):
        """Return latest frame from the framebuffer"""
        
        # Obtain lock and release it when done
        with self.lock:
            if self.frameBuffer[self.currentBufferIndex] is not None:
                # return copy.deepcopy(self.frameBuffer[self.currentBufferIndex])
                return self.frameBuffer[self.currentBufferIndex]
            else:
                return None
                
    def new_frame(self, frame):
        """Add a new frame to the frame buffer"""
        # self.frameBuffer[int(not self.currentBufferIndex)] = copy.deepcopy(frame)
        self.frameBuffer[int(not self.currentBufferIndex)] = frame
        
        # Obtain lock and release it when done
        with self.lock:
            # Switch buffer index
            self.currentBufferIndex = int(not self.currentBufferIndex)
