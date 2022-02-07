import pyjevois
if pyjevois.pro: import libjevoispro as jevois
else: import libjevois as jevois
import cv2
import numpy as np
from collections import deque
import imutils

#yellowLower=(29,86,6)
#yellowHigher=(64,255,255)

yellowLower=(0,100,80) #0 86 20
yellowHigher=(100,255,255) #100 255 255

pts = deque(maxlen=0)

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
    ## Process function with USB output (Legacy mode):
    def process(self, inframe, outframe):
        # Get the next camera image for processing (may block until it is captured) and here convert it to OpenCV BGR by
        # default. If you need a grayscale image instead, just use getCvGRAYp() instead of getCvBGRp(). Also supported
        # are getCvRGBp() and getCvRGBAp():
        inimg = inframe.getCvBGRp()
        
        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()

        blurred = cv2.GaussianBkur(inframe,(11,11),0)
        hsv= cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        #apply mask
        mask = cv2.inRange(hsv,yellowLower,yellowUpper)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        outframe.send(mask)

    # ###################################################################################################
    ## Process function with GUI output (JeVois-Pro mode):
    def processGUI(self, inframe, helper):
        # Start a new display frame, gets its size and also whether mouse/keyboard are idle:
        idle, winw, winh = helper.startFrame()

        # Draw full-resolution color input frame from camera. It will be automatically centered and scaled to fill the
        # display without stretching it. The position and size are returned, but often it is not needed as JeVois
        # drawing functions will also automatically scale and center. So, when drawing overlays, just use image
        # coordinates and JeVois will convert them to display coordinates automatically:
        x, y, iw, ih = helper.drawInputFrame("c", inframe, False, False)
        
        # Get the next camera image for processing (may block until it is captured), as greyscale:
        inimg = inframe.getCvBGRp()

        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()

        

        # Edges is a greyscale image. To display it as an overlay, we convert it to RGBA, with zero alpha (transparent)
        # in background and full alpha on edges. We just duplicate our edge map 4 times for A, B, G, R:
        blurred = cv2.GaussianBlur(inimg,(11,11),0)
        hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        #apply mask
        mask = cv2.inRange(hsv,yellowLower,yellowHigher)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
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
            if radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
                cv2.circle(inimg, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(inimg, center, 5, (0, 0, 255), -1)
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
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(inimg, pts[i - 1], pts[i], (0, 0, 255), thickness)
        
        # Draw the edges as an overlay on top of the full-resolution camera input frame. It will automatically be
        # re-scaled and centered to match the last-drawn full-resolution frame:
        # Flags here are: rgb = True, noalias = False, isoverlay=True
        output= cv2.cvtColor(inimg,cv2.COLOR_BGR2RGB)
        test1=cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
        test=np.concatenate((mask,test1),axis=1)
        helper.drawImage("ballTracking",test, True, False, True)
        

        # Write frames/s info from our timer:
        fps = self.timer.stop()
        helper.iinfo(inframe, fps, winw, winh);

        # End of frame:
        helper.endFrame()
        
    # ###################################################################################################
    ## Process function with no USB output (Headless mode):
    def processNoUSB(self, inframe):
        # Get the next camera image at the processing resolution (may block until it is captured) and here convert it to
        # OpenCV GRAY by default. Also supported are getCvRGBp(), getCvBGRp(), and getCvRGBAp():
        inimg = inframe.getCvGRAYp()

        # Detect ArUco markers:
        corners, ids, rej = cv2.aruco.detectMarkers(inimg, self.dict, parameters = self.params)

        # Nothing to display in headless mode. Instead, just send some data to serial port:
        if len(corners) > 0:
            for (marker, id) in zip(corners, ids):
                jevois.sendSerial("Detected ArUco ID={}".format(id))

        



