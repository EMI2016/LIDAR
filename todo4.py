import serial
import numpy as np
import cv2
import glob
import pickle
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

signal1_cascade = cv2.CascadeClassifier('signal1_cascade_v3.xml')
signal2_cascade = cv2.CascadeClassifier('signal2_cascade.xml')
signal3_cascade = cv2.CascadeClassifier('signal3_cascade2.xml')
cap = cv2.VideoCapture(0)
bandera=0
q=0
w=0
l=0
m=0
pend=0
uart_port = '/dev/ttyACM0'
uart_speed = 19200

def valores(angle):
    
    if (angle==45):
        x2=x/2
        y2=y
    if (angle==135):
        x2=-x/2
        y2=y
    if (angle==0):
        x2=x/2
        y2=0
    if (angle==90):
        x2=0
        y2=y
    if (angle==180):
        x2=-x/2
        y2=0
    print 'coor',x2,y2 
    dist=((x2)**2+(y2)**2)**0.5
    return dist

def escalar (x1,y1,x2,y2,img,c,d):
    #t 'k',c
    a=((x1+x2)/2)
    b=((y1+y2)/2)
    #dista=(((a-500)**2+(b-1000)**2)**0.5)*3
   # d1=dista-0.12*1000.0/3.0
    #a1=a*d1/dista
    #b1=b*d1/dista
    #print a1
    #print b1
     
    if a<500:
        es=(a-500)
    else: 
        es=(500-a)
    er=(1000-b)
    c.append(es)
    d.append(er)
    #print 'dist',dista
    cv2.line(img,(a,b),(500,1000),(75,50,100),8)
    cv2.line(img,(x1,y1),(x2,y2),(50,70,50),5)
    

def esquelet (image):

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

    return [skel,img]

def analisis_patron(bandera):

    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    signal1=signal1_cascade.detectMultiScale(gray,1.3,5)
    signal2=signal2_cascade.detectMultiScale(gray,1.3,5)
    signal3=signal3_cascade.detectMultiScale(gray,1.3,5)
    for (x,y,w,h) in signal1:
      
        if bandera==1:
            print "Detecto patron 1"
            bandera=0

    for (x,y,w,h) in signal2:
      
        if bandera==1:
            print "Detecto patron 2"
            bandera=0

    for (x,y,w,h) in signal3:
       
        if bandera==1:
            print "Detecto patron 3"
            bandera=0

    if (len(signal1) == 0)&(len(signal2) == 0)&(len(signal3) == 0)& (bandera ==0):
        print "No detecto patron"
        bandera =1

def analisis_vaca(q,w,l,m):

    ret, frame = cap.read()
    image=np.zeros((1000,1000))
    angles, distances, timestamp=laser.get_scan()
    angles=np.array(angles)
    distances=np.array(distances)

    for i in range(len(distances)):

        if (angles[i]>-90 and angles[i]<90):
            x=distances[i]*np.sin(angles[i]*np.pi/180.0)*1000.0/3000.0
            y=distances[i]*np.cos(angles[i]*np.pi/180.0)*1000.0/3000.0

            if(x>-499 and x<499 and y>0 and y<999):

                image[999-y,499+x]=255
            
    k = cv2.waitKey(1)
        
    [skel,img] = esquelet (image)

    #lines = cv2.HoughLinesP(skel,9,np.pi/180,2, None,35*1000/3000,100*1000/3000)
    #lines = cv2.HoughLinesP(skel,9,np.pi/180,2, None,300*1000/3000,700*1000/3000)
    #lines = cv2.HoughLinesP(skel,9,np.pi/180,2, None,50*1000/3000,700*1000/3000)
    lines = cv2.HoughLinesP(skel,2,np.pi/180,50, None,55,70)

    try:
#        i=0    
        for x1,y1,x2,y2 in lines[0]:
         
               
            r=np.sqrt((x1-x2)**2+(y1-y2)**2)
    
            if (r>400*1000/3000 and r<=550*1000/3000):
            #a=o
            #if (r>55*3 and r<=70*3):    
             #   a=a+1
                #print 'h', x1
                escalar (x1,y1,x2,y2,img,c,d)
                #i=i+1
     #           print 'e',x2
                #if (c[i+1]>(c[i]+56.0*1.0/3.0)) or (c[i+1]<(c[i]-56.0*1.0/3.0)) or (d[i+1]>(d[i]+26.0*1.0/3.0)) or (d[i+1]<(d[i]-26.0*1.0/3.0)):
                    #i=i+1
                    #print 'i'
                    #l=format((c[i+1]*3/1000.0),'.2f')
                    #m=format((d[i+1]*3/1000.0),'.2f')
                #if a>1:
                    
            #if (r>600*1000/3000): 
             #   f=2
              #  print 'dos',f                         
                   
        q=format((c[0]*3/1000.0),'.2f')  
        w=format((d[0]*3/1000.0),'.2f') 
        #q=q-0.25*np.cos(p)
        #w=w-0.25*np.sin(p)
        #l=format((c[15]/1000.0),'.2f')
        #m=format((d[15]/1000.0),'.2f')
    except Exception as e:
        z=1

    cv2.imshow("img",img)
    
    return [q,w,k]             
            



if __name__ == '__main__':
    
    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)
    laser.laser_on()
    c=[]
    d=[]
    g=0       
    while(1):
        
  
        
        #analisis_patron(bandera)
        
        [a,b,k]=analisis_vaca(q,w,l,m)
        #print 'coor',[(c[g],d[g])]      
        print 'Vaca1', [a,b]   
        #print 'Vaca2', [e,f] 
        #g=g+1            
        if k==27:
            break
        
    laser.laser_off()


