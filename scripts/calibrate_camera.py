#!/usr/bin/env python

import cv2
import sys
import numpy as np

def main(img_path):

    try:

        # Try to open image as BGR (default behaviour)
        img = cv2.imread(img_path)
        
        # Check if image was opened
        
        if img == None:
            raise
    except:
        print('Incorrect image path!')  
        sys.exit() # Show the image
        
    # cv2.imshow('Calibration Image',img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    
    # Convert to HSV
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
    # Get image size
    h,w,d = imgHSV.shape
    print 'Image Dimensions: (' + str(h) + ',' + str(w) + ')' 

    # Image center
    center = (int(h/2),int(w/2))

    # Threshold only the color we want
    # Green bowl
    # MIN = np.array([25,60,10])
    # MAX = np.array([85,255,255])

    Blue block
    MIN = np.array([90,30,10])
    MAX = np.array([140,160,160])

    imgThresh = cv2.inRange(imgHSV, MIN, MAX)
    # output = cv2.bitwise_and(img, img, mask = imgThresh)
        
    cv2.imshow('Threshold Image',imgThresh)
    cv2.waitKey()
    cv2.destroyAllWindows()
     
    # Find centroid with zeroth order moment
    m = cv2.moments(imgThresh)
    centroid = (int(m['m10']/m['m00']), int(m['m01']/m['m00'])) 
    print 'Object centroid at: ' + str(centroid)

    # Show image with centroid overlay
    # img
    # cv2.circle(img,centroid,5,(120,255,255),-1)  
    # cv2.imshow('Threshold Image w/ Centroid',img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

    # Print the x,y delta distances
    delta = tuple(np.subtract(centroid,center))
    print 'Object centroid delta from center: ' + str(delta)

    # Getting Orientation
    imgBlur = cv2.medianBlur(img,5)
    thresh = cv2.adaptiveThreshold(imgBlur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    cv2.imshow('Adaptive Threshold',thresh)
    cv2.waitKey()
    cv2.destroyAllWindows()

    # contours,hierarchy = cv2.findContours(imgThresh, 1, 2)
    # cnt = contours[0]
    # M = cv2.moments(cnt)

    # #<--- Harris Corner Detector --->
    # # Create grayscale image used for Harris detection
    # biImage = cv2.bitwise_and(imgThresh, imgThresh, mask = imgThresh)
    # # grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # # Compute Harris Corner Scores
    # biImage = np.float32(biImage)
    # dst = cv2.cornerHarris(biImage,14,15,0.04)
    # # print 'Harris Corner Dimensions:' + str(dst.shape)

    # #Find Max 4 corners
    # numPts = 10
    # blockCorners = np.zeros((numPts, 3))
    # minScore = np.amin(dst)
    # for i in range(numPts):
    # # Assumes dst is an 2D array
    #     maxHarris = np.unravel_index(dst.argmax(), dst.shape)
    #     # maxHarris = np.argmax(dst) + 1
    #     # maxHarris = np.array([maxHarris/w-1,maxHarris-(maxHarris/w)*w-1])
    #     blockCorners[i,:2] = maxHarris[0:2]
    #     blockCorners[i,2] = dst[maxHarris[0], maxHarris[1]]
    #     dst[maxHarris[0],maxHarris[1]] = minScore
    #     img
    #     cv2.circle(img,maxHarris,5,(120,255,255),-1)
    # print 'Block Corners : ' + str(blockCorners)
    # import pdb; pdb.set_trace()

    # cv2.imshow('Threshold Image Corners',img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()

    # print 'block corners' + str(blockCorners)

    # Find COM and Orientation
    # blockCOM = np.mean(blockCorners, axis=0)
    # print 'Cube centroid at: ' + str(blockCOM)

    # leftNodes = [blockCorners[:,0]<blockCOM[0]]                 #NODES TO THE LIEFT OF COM_x 
    # orientation = leftNodes[0,:] - leftNodes[1,:]
    # orientation = orientation / np.linalg.norm(orientation)  #Unit Vector
    # orientation = np.arctan2(orientation[1],orientation[0])
    # print 'Cube Orientation at: ' + str(orientation)
    # <----------------------------------------------------->




if __name__ == '__main__':

    if len(sys.argv) == 2:
        main(sys.argv[1])
    else:
        print('Please specify path to calibration image')
        sys.exit()
