import pyjevois
if pyjevois.pro: import libjevoispro as jevois
else: import libjevois as jevois
import cv2
import numpy as np
from collections import deque
#import imutils

#yellowLower=(25,86,50)
#yellowHigher=(64,255,255)

#yellowLower=(0,195,0) #0 100 80 
#yellowHigher=(100,255,255) #100 255 255

yellowLower=(15,50,50) 
yellowHigher=(64,255,255) #100 255 255

buffer_size=0
pts = deque(maxlen=buffer_size)

class Tracking:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("timer", 100, jevois.LOG_INFO)

        # Create an ArUco marker detector:
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

    # ###################################################################################################
    ## Process function with GUI output (JeVois-Pro mode):
    def processGUI(self, inframe, helper):
        # Start a new display frame, gets its size and also whether mouse/keyboard are idle:
        idle, winw, winh = helper.startFrame()

        # Draw full-resolution color input frame from camera. It will be automatically centered and scaled to fill the
        # display without stretching it. The position and size are returned, but often it is not needed as JeVois
        # drawing functions will also automatically scale and center. So, when drawing overlays, just use image
        # coordinates and JeVois will convert them to display coordinates automatically:
        #x, y, iw, ih = helper.drawInputFrame("c", inframe, False, False)
        
        # Get the next camera image for processing (may block until it is captured), as greyscale:
        inimg = inframe.getCvRGBA()
        h = int(inimg.shape[0]/2)
        w = int(inimg.shape[1]/2)
        dim = (w,h)
        
        inimg = cv2.resize(inimg,dim,interpolation=cv2.INTER_AREA)
        
        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()
        
        blurred = cv2.GaussianBlur(inimg,(11,11),0)
        hsv = cv2.cvtColor(blurred,cv2.COLOR_RGB2HSV)
        
        #apply mask
        mask = cv2.inRange(hsv,yellowLower,yellowHigher)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
        cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        center = None
		# only proceed if at least one contour was found
        if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			# only proceed if the radius meets a minimum size
            if radius > 1 :
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
                cv2.circle(inimg, (int(x), int(y)), int(radius),
                    (255, 255, 0,255), 2)
                cv2.circle(inimg, center, 5, (255, 0, 0,255), -1)
		# update the points queue
        pts.appendleft(center)
		
		# loop over the set of tracked points
        for i in range(1, len(pts)):
			# if either of the tracked points are None, ignore
		    # them
            if pts[i - 1] is None or pts[i] is None:
                continue
            # otherwise, compute the thickness of the line and
		    # draw the connecting lines
            thickness = int(np.sqrt(buffer_size / float(i + 1)) * 2.5)
            cv2.line(inimg, pts[i - 1], pts[i], (255,0, 0, 255), thickness)
        
        # Draw the edges as an overlay on top of the full-resolution camera input frame. It will automatically be
        # re-scaled and centered to match the last-drawn full-resolution frame:
        # Flags here are: rgb = True, noalias = False, isoverlay=True

        helper.drawImage("edge",inimg, True, False, True)
        

        # Write frames/s info from our timer:
        fps = self.timer.stop()
        helper.iinfo(inframe, fps, winw, winh);

        # End of frame:
        helper.endFrame()
