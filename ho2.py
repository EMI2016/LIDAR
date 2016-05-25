import serial
import numpy as np
import cv2
import glob
import pickle
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

def valores(angle):
    
    if (angle<45 and angle>=0) or (angle>135 and angle<=180) :
        x2=-320
        y2=x2*np.sin(angle)
    elif angle==45:
        x2=320
        y2=480
    elif (angle>45 and angle<90) or (angle>90 and angle<135):
        y2=480
        x2=y2*np.cos(angle)
    elif angle==135:
        x2=-320 
        y2=480
    print 'coor',x2,y2 
    dist=((x2)**2+(y2)**2)**0.5
    return dist


def linea_l(x1,y1,x2,y2,disp,a1,b1):
       
    dis1=((x1-x2)**2+(y1-y2)**2)**0.5
    cv2.line(img,(x1,y1),(x2,y2),(75,50,100),5)
    
    if dis1 > disp:
        disp=dis1
        a=((x1+x2)/2)
        b=((y1+y2)/2)
	       
        cv2.circle(img,(a,b),5,(45,0,255),1) 
        
        if (disp<537*480.0/3000.0):
            
            print 'Distancia',disp
            print ("No es vaca")

        if (disp>=537*480.0/3000.0) and (disp<=543*480.0/3000.0):
            
            if a>a1 and b>b1:
                a1=a
                b1=b  
            print 'Punto medio',a1,b1
            anglew= np.arctan(y2-y1/x2-x1)	
            print 'Angulo',anglew
            print 'Distancia',disp
            print ("Solo hay una vaca")
                             
        if (disp>543*480.0/3000.0):   
            
            print 'Distancia',disp
            print ("Hay mas de una vaca")     
            xa=x1+543*480/300*np.cos(np.arctan(y2-y1/x2-x1))
            yb=y1+543*480/300*np.sin(np.arctan(y2-y1/x2-x1))
            a1=(xa+x1)/2
            b1=(yb+y1)/2
            print 'Punto medio',a1,b1                  
        
    return [disp,a1,b1]

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

file = open('B.npz', 'rb')
mtx, dist, rvecs, tvecs =pickle.load(file)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
            

uart_port = '/dev/ttyACM0'
uart_speed = 19200
cap = cv2.VideoCapture(1)

if __name__ == '__main__':
    
    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)
    laser.laser_on()
    disp=0 
    a1=0
    b1=0
           
    while(True):
        
        #Capture fram-by-frame
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image=np.zeros((480,640))
        angles, distances, timestamp=laser.get_scan()
        angles=np.array(angles)
        distances=np.array(distances)

        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

        if ret == True:
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)
        # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            img = draw(img,corners,imgpts)
	
        for i in range(len(distances)):
             if (angles[i]>-29.4 and angles[i]<29.4):
                
                x=distances[i]*np.sin(angles[i]*np.pi/180.0)*480.0/3000.0
                y=distances[i]*np.cos(angles[i]*np.pi/180.0)*480.0/3000.0

                if(x>-319 and x<319 and y>0 and y<479):

                    image[479-y,319+x]=255
            
        k = cv2.waitKey(1)
        
        img=np.uint8(image)
       
        mask = np.zeros(img.shape, dtype=np.uint8)
        img1 = np.uint8(img)
        size = np.size(img1)
        skel = np.zeros(img1.shape,np.uint8)
        
        element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))

        done=False

        #skeletonization        

        while(not done):

            eroded = cv2.erode(img1,element)
            temp = cv2.dilate(eroded,element)
            temp = cv2.subtract(img1,temp)
            skel = cv2.bitwise_or(skel,temp)
            img1 = eroded.copy()

            zeros = size - cv2.countNonZero(img1)

            if zeros==size:

                done = True

        lines = cv2.HoughLinesP(skel,9,np.pi/180,2, None,35*480/3000,100*480/3000)
        
        try:
           
            for x1,y1,x2,y2 in lines[0]:

                r=np.sqrt((x1-x2)**2+(y1-y2)**2)
    
                if (r>20*480/3000):
                                       
                    if abs(x1-x2)>30*480/3000 and abs(y1-y2)>30*480/3000:         

                        [disp,a1,b1]=linea_l(x1,y1,x2,y2,disp,a1,b1) 
                                     
          
        except Exception as e:
            print(str(e))
        
        cv2.imshow("skel",skel)
        cv2.imshow("img",img)
        
        if k==27:
            break
    laser.laser_off()


