# Read serial data and track moving targets
#
import tkinter as tk
from os import times_result
from tkinter import simpledialog
import serial
import cv2
import pickle
import binascii
from PIL import Image, ImageTk
import numpy as np
import math
import time
from datetime import datetime
import struct
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg



real_targe_area=25 # Projected area of the test object (cm²)

# Create serial port object
ser = serial.Serial('COM6', 460800, timeout=1)  # Modify COM port number and baud rate as required

# Create video capture object
#Camera supports 1080×720, 800×600, 640×480 at 30Hz and 60Hz
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640) #Set camera frame width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480) #Set camera frame height
cap.set(cv2.CAP_PROP_FPS,30)


# Create tracker
tracker = cv2.TrackerCSRT_create() #CSRT tracker
#tracker = cv2.TrackerKCF_create() #CSRT tracker
#tracker = cv2.legacy.TrackerKCF_create()  #KCF tracker
#tracker = cv2.legacy.TrackerMOSSE_create()
#tracker = cv2.TrackerDaSiamRPN() #DaSiamRPN tracker

# Initialize background subtractor
#bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=True)
bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=15, varThreshold=30, detectShadows=True)



Starttime=time.time()

#Global variables
DJPWM_FW=1500
DJPWM_FY=1500


#Data storage arrays
G_capMB_X = []
G_capMB_Y = []
G_capMB_Z = []
G_capMB_time = []
GTab_mb_imgx= []
GTab_mb_imgy= []
GTab_MBFW= []
GTab_MBFY= []
GTab_djjiao_FW= []
GTab_djjiao_FY= []
GTab_dpwm_fw= []
GTab_dpwm_fy= []
GTab_distance_CSB=[]
GTab_distance_JG=[]
GTab_Gpix_targe_area=[]
GTab_pix_to_dis=[]


G_CapState=0
G_DJTraceFlag=0
G_CaptaceModeFlag=1

Gpix_targe_area=1
MBdistance=0

G_man_auto_st =1  #initial manual mode
G_mbtracesureFlag = 0

index=0

capStateFlag=0
CNT=0
manm=0

Gpix_targe_area=1

# Create main window
root = tk.Tk()
root.title("Video Processing")
root.geometry("800x800+600+10")


window1=tk.Tk()
window1.title("MCU Status Monitor")
window1.minsize(400, 200)
window1.geometry("410x200+180+10")

window3 = tk.Tk()
window3.title("Trajectory Plot")
window3.geometry("410x410+180+220")

# Create a Matplotlib figure
fig = Figure(figsize=(6, 4), dpi=100)
ax = fig.add_subplot(111, projection='3d')

# Create a Tkinter canvas
canvas = FigureCanvasTkAgg(fig, master=window3)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

# Create a separate 3D figure and coordinate axes
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')



# get camera frame rate

fps = cap.get(cv2.CAP_PROP_FPS)
print(f"Camera FPS: {fps} FPS")
frame_width=cap.get(cv2.CAP_PROP_FRAME_WIDTH)
print(f"Frame width: {frame_width} pixels")
frame_hight=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Frame height: {frame_hight}  pixels")

dpix_to_Angle=100.0/(math.sqrt(frame_width**2+frame_hight**2))
print(f"1 pixel corresponds to: {dpix_to_Angle}  degrees ")



def coordinatechange(imgx,imgy): #Convert camera coordinates to Earth reference frame
    # Calculate coordinates relative to the center of the field of view
    dx=imgx-frame_width*0.5
    dy=imgy-frame_hight*0.5
    k=100.0/math.sqrt(frame_width**2+frame_hight**2)
    jiao_dz=dx*k*3.1415926/180.0 #Convert to angle in radians corresponding to pixels
    jiao_dy=-dy*k*3.1415926/180.0 #Convert to angle in radians corresponding to pixels
    #Convert to angle in detection coordinate system
    tcq_dq=math.sqrt(jiao_dz**2+jiao_dy**2)
    tcq_gama=math.atan2(jiao_dz,jiao_dy)

    tcq_x=math.cos(tcq_dq)
    tcq_y=math.sin(tcq_dq)*math.cos(tcq_gama)
    tcq_z=math.sin(tcq_dq)*math.sin(tcq_gama)
    # Transform to ground frame
    djjiao_FW = (DJPWM_FW - 1500) * (270.0 / 2000.0)
    djjiao_FY = 90.0 + (DJPWM_FY - 1500) * (270.0 / 2000.0)
    sheta=-djjiao_FY/180.0*3.1415926
    fai=-djjiao_FW/180.0*3.1415926
    mb1_array=np.array([[tcq_x],[tcq_y],[tcq_z]])
    L_sheta=np.array([[math.cos(sheta),math.sin(sheta),0],[-math.sin(sheta),math.cos(sheta),0],[0,0,1]])
    L_fai=np.array([[math.cos(fai),0,-math.sin(fai)],[0,1,0],[math.sin(fai),0,math.cos(fai)]])
    A1=L_sheta @ mb1_array
    mb0_arry=L_fai @ A1

    x0=mb0_arry[0]
    y0=mb0_arry[1]
    z0=mb0_arry[2]
    DDMB_FY=math.asin(y0/math.sqrt(x0**2+y0**2+z0**2))*180.0/3.1415926
    DDMB_FW=math.atan2(-z0,x0)*180.0/3.1415926

    #print("MB_FY:",DDMB_FY)
    #print("MB_FW:",DDMB_FW)

    return  DDMB_FW,DDMB_FY



def djtracemb(dpwm_fw,dpwm_fy):
    global  DJPWM_FY
    global  DJPWM_FW

    ser.write(struct.pack('B', 0xEC))
    ser.write(struct.pack('B', 0x91))
    ser.write(struct.pack('B', 0x55))
    ser.write(struct.pack('H',(DJPWM_FW+dpwm_fw)))
    ser.write(struct.pack('H',(DJPWM_FY+dpwm_fy)))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('B', 0x0D))
    ser.write(struct.pack('B', 0x0A))


def updatambpos(mb_imgx,mb_imgy): #Update target position information
    global G_capMB_X, G_capMB_Y, G_capMB_Z
    global frame_width,frame_hight
    global dpix_to_Angle

    MBFW,MBFY=coordinatechange(mb_imgx,mb_imgy) # Convert to geodetic azimuth

    djjiao_FW = (DJPWM_FW - 1500) * (270.0 / 2000.0)
    djjiao_FY = 90.0 + (DJPWM_FY - 1500) * (270.0 / 2000.0)

    d_jiaoFW=MBFW-djjiao_FW  # Angle between target and FOV center in geodetic coordinates
    d_jiaoFY=MBFY-djjiao_FY

    k = 0.6  # Servo control coefficient
    dpwm_fw=int(d_jiaoFW*(2000.0/270.0)*k) #Note the sign convention : relationship between motor system and vision coordinate system
    dpwm_fy=int(d_jiaoFY*(2000.0/270.0)*k)

    global G_DJTraceFlag
    if G_DJTraceFlag==1:
       djtracemb(dpwm_fw, dpwm_fy)

    global MBdistance,Starttime,Gpix_targe_area

    L0=real_targe_area**0.5
    Lpix1=Gpix_targe_area**0.5
    mbangle=Lpix1*dpix_to_Angle/180.0*3.1415926
    #("mbangle",mbangle)
    pix_to_dis=L0/mbangle

    print("mbangle", mbangle)
    print("Lpix1",pix_to_dis)


    global G_CaptaceModeFlag

    if (G_CaptaceModeFlag==1)|(G_CaptaceModeFlag==2):
        MBdistance0=pix_to_dis
    else:
        MBdistance0=250

    tcmb_y=MBdistance0*math.sin(MBFY/180.0*3.1415926)
    tcmb_x = MBdistance0 * math.cos(MBFY / 180.0 * 3.1415926)*math.cos(MBFW/180.0*3.1415926)
    tcmb_z = -MBdistance0 * math.cos(MBFY / 180.0 * 3.1415926)*math.sin(MBFW/180.0*3.1415926)

    print("MBdistance0", MBdistance0)

    print("tcmb_x",tcmb_x)
    print("tcmb_y", tcmb_y)


    G_capMB_X.append(tcmb_x)
    G_capMB_Y.append(tcmb_y)
    G_capMB_Z.append(tcmb_z)
    plot_trajectory()

    #Store data
    GTab_mb_imgx.append(mb_imgx)
    GTab_mb_imgy.append(mb_imgy)
    GTab_MBFW.append(MBFW)
    GTab_MBFY.append(MBFY)
    GTab_djjiao_FW.append(djjiao_FW)
    GTab_djjiao_FY.append(djjiao_FY)
    GTab_dpwm_fw.append(dpwm_fw)
    GTab_dpwm_fy.append(dpwm_fy)
    dtime=time.time()-Starttime
    G_capMB_time.append(dtime)
    GTab_distance_CSB.append(MBdistance)
    GTab_Gpix_targe_area.append(Gpix_targe_area)
    GTab_pix_to_dis.append(pix_to_dis)
    #GTab_distance_CSB = []
    #GTab_distance_JG = []


def savetofile_now_data():
    #Save data
    k = int(time.time())
    filenamne = "data/save_" + str(k) + ".pkl"
    with open(filenamne, 'wb') as f:
        # save multiple variables using the pickle.dump method
        pickle.dump(GTab_mb_imgx, f)
        pickle.dump(GTab_mb_imgy, f)
        pickle.dump(GTab_MBFW, f)
        pickle.dump(GTab_MBFY, f)
        pickle.dump(GTab_djjiao_FW, f)
        pickle.dump(GTab_djjiao_FY, f)
        pickle.dump(GTab_dpwm_fw, f)
        pickle.dump(GTab_dpwm_fy, f)
        pickle.dump(G_capMB_X, f)
        pickle.dump(G_capMB_Y, f)
        pickle.dump(G_capMB_Z, f)
        pickle.dump(G_capMB_time, f)
        pickle.dump(GTab_distance_CSB, f)
        pickle.dump(GTab_distance_JG, f)
        pickle.dump(GTab_Gpix_targe_area, f)
        pickle.dump(GTab_pix_to_dis,f)





def movemb_trace(frame):
    global capStateFlag,CNT, bbox,Gpix_targe_area
    global G_CaptaceModeFlag,manm
    CNT=CNT+1

    if G_CaptaceModeFlag == 1:   #Only detect moving background (no tracking)
        manm = 0  # Initial tracking box selection: 0: background motion 1: circle detection
    if G_CaptaceModeFlag == 2:   #Detect circles only (no tracking)
        manm = 1
    if G_CaptaceModeFlag == 3:  # Detect and track moving objects
        manm = 0
    if G_CaptaceModeFlag == 4:  # Detect and track circular targets
        manm = 1

    if  capStateFlag==0:
        if manm == 0:
            # Apply background subtractor
            fgmask = bg_subtractor.apply(frame)
            # Use morphological closing/opening to reduce noise
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel)
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
            #Find connected components in the foreground
            contours, _ = cv2.findContours(fgmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Iterate over contours
            MBCapFlag=0

            bboxmax=[]
            S_mbara=0
            for contour in contours:

                # Ignore small contours to reduce noise
                if cv2.contourArea(contour) < 300:
                    continue
                # Ignore overly large contours to reduce noise
                if cv2.contourArea(contour) > 120000:
                    continue

                # Compute bounding box of the contour
                bbox= cv2.boundingRect(contour)
                x = bbox[0]
                y = bbox[1]
                w = bbox[2]
                h = bbox[3]
                if (w/h>4.0)|(h/w>4.0):
                    continue

                if cv2.contourArea(contour)>=S_mbara:
                    S_mbara =cv2.contourArea(contour)
                    bboxmax=bbox
                MBCapFlag = 1
                # Draw bounding box on the original frame
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if MBCapFlag==1:
                bbox= bboxmax #Use the largest one
                Gpix_targe_area=S_mbara
                x = bbox[0]
                y = bbox[1]
                w = bbox[2]
                h = bbox[3]
                sx = x + 0.5 * w
                sy = y + 0.5 * h
                # Draw bounding box on the original frame
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
                if (G_CaptaceModeFlag==1):
                    updatambpos(sx, sy)

            # Display foreground mask and original framewith bounding boxes
            cv2.imshow('Foreground', fgmask)
            cv2.imshow('Frame', frame)

            global  G_mbtracesureFlag,G_man_auto_st
            #print("ddd",G_CaptaceModeFlag,G_man_auto_st,MBCapFlag)
            if G_CaptaceModeFlag==3:
               # print("ddd22", G_CaptaceModeFlag)
                if G_man_auto_st==1:
                    if (G_mbtracesureFlag==1)&(MBCapFlag==1):# Manual selection 
                        capStateFlag = 1
                        G_mbtracesureFlag= 0

                if G_man_auto_st == 0:   #Automatic selection
                    if (CNT>7)&(MBCapFlag==1):
                        x = bbox[0]
                        y = bbox[1]
                        w = bbox[2]
                        h = bbox[3]
                        #frame_width
                        #frame_hight
                        if (x > 10)&(y > 10)&((x + w) < (frame_width - 10))&((y + h) < (frame_hight - 10)):
                            capStateFlag = 1


        if manm == 1:
            frame, bbox, flag = find_largest_circle(frame)
            if flag:
                if G_CaptaceModeFlag==2:
                    x=bbox[0]
                    y=bbox[1]
                    w=bbox[2]
                    h=bbox[3]
                    sx = x + 0.5 * w
                    sy = y + 0.5 * h
                    Gpix_targe_area=3.14159*((w*0.5)**2)
                    updatambpos(sx, sy) #Plot trajectory

                if G_CaptaceModeFlag == 4:
                    if (G_man_auto_st == 1):#Manual tracking mode
                        if (G_mbtracesureFlag == 1): #Confirm target (manual)
                            capStateFlag = 1
                            G_mbtracesureFlag=0
                    if (G_man_auto_st == 0):#Automatic
                        capStateFlag = 1
        #    # Manually select the initial tracking ROI
        #    global bbox
        #    bbox = cv2.selectROI(frame, False)  # 返回 (x_min, y_min, w, h)
        #    capStateFlag = 1
        label.config(text="Detecting target...")
        #print("Run state 0 %d", capStateFlag)

    if capStateFlag == 1:
        #print("Run state 1 %d", capStateFlag)
        print(bbox)
        # Initialize tracker
        tracker.init(frame, bbox)
        # Observe the selected tracking ROI
        rio = frame[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])]
        cv2.imshow('roi', rio)
        capStateFlag=2

    if capStateFlag == 2:

        # Update tracker
        success, bbox = tracker.update(frame)
        # If tracking succeeds, draw the bounding box
        if success:
            x, y, w, h = [int(i) for i in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            sx=x+0.5*w
            sy=y+0.5*h
            updatambpos(sx,sy)
            label.config(text="Tracking target~")
        # Display current frame
        #cv2.imshow()'Tracking', frame
        #rint("Run state2 %d", capStateFlag)

    return  frame

# Circle detection function within the motion module
def find_largest_circle(image):
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #Median filtering
    #gray = cv2.medianBlur(gray, 5)
     #Use Gaussian blur to reduce noise
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    #edges = cv2.Canny(gray, threshold1=30, threshold2=100)

    cv2.imshow('gray', gray)
    # Apply Hough transform to detect circles
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.0, 15,
                               param1=100, param2=50, minRadius=5, maxRadius=100)
    # Find the circle with the largest radius
    largest_circle = None
    mybox=[0,0,0,0]

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            if largest_circle is None or r > largest_circle[2]:
                largest_circle = (x, y, r)
        # If the largest circle is found, draw it
        if largest_circle is not None:
            cv2.circle(image, (largest_circle[0], largest_circle[1]), largest_circle[2], (0, 255, 0), 4)
            cv2.circle(image, (largest_circle[0], largest_circle[1]), 2, (0, 128, 255), 3)
            sx = largest_circle[0]
            sy = largest_circle[1]
            R = largest_circle[2]
            x=int(sx-R)
            y=int(sy-R)
            w=int(2*R)
            h=int(2*R)
            #cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            mybox[0] = x
            mybox[1] = y
            mybox[2] = w
            mybox[3] = h

             # Update label text

            sss=' Center Coordinates X:'+str(sx)+' Center Coordinates Y:'+str(sy)+'R:'+str(R)
            label.config(text=sss)
            return image,mybox,largest_circle
    else:
        label.config(text="Not detected")
        return image,mybox,largest_circle


# Define circular detection function
def detect_largest_circle(image):
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #Median filtering
    gray = cv2.medianBlur(gray, 5)
    # Use Gaussian blur to reduce noise
    gray = cv2.GaussianBlur(gray, (9, 9), 2)
    # Apply Hough transform to detect circles
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 5,
                               param1=100, param2=30, minRadius=2, maxRadius=200)
    # Find the circle with the largest radius
    largest_circle = None
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            if largest_circle is None or r > largest_circle[2]:
                largest_circle = (x, y, r)
        # If the largest circle is found, draw it
        if largest_circle is not None:
            cv2.circle(image, (largest_circle[0], largest_circle[1]), largest_circle[2], (0, 255, 0), 4)
            cv2.circle(image, (largest_circle[0], largest_circle[1]), 2, (0, 128, 255), 3)

            sx=largest_circle[0]
            sy=largest_circle[1]
            R=largest_circle[2]
             # Update label text
            sss=' Center Coordinates X:'+str(sx)+' Center Coordinates Y:'+str(sy)+'R:'+str(R)
            label.config(text=sss)

            updatambpos(sx, sy)

            return image
    else:
        label.config(text="Not detected")
        return image

# Update video frame function
def update_frame():

    ret, frame = cap.read()
    # Select the initial region to track
    #bbox = cv2.selectROI(frame, False)
    # Initialize tracker
    #tracker = cv2.TrackerCSRT_create()
    #tracker.init(frame, bbox)

    if ret:

        #frame = detect_circles(frame)
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        #result_frame = detect_largest_circle(frame)
        result_frame=movemb_trace(frame)
        # Update tracker
        #success, bbox = tracker.update(frame)
        # If tracking succeeds, draw the bounding box
        #if success:

        #    x, y, w, h = [int(i) for i in bbox]
        #    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #cv2.imshow('Camera', result_frame)
        cvimage = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(cvimage)
        imgtk = ImageTk.PhotoImage(image=img)
        panel.imgtk = imgtk
        panel.config(image=imgtk)
        #print("Image read time",time.time())
    root.after(5, update_frame)


def retocap():#Reacquire target
    global G_capMB_X,G_capMB_Y,G_capMB_Z,G_capMB_time,G_CapState
    G_capMB_X.clear()
    G_capMB_Y.clear()
    G_capMB_Z.clear()
    G_capMB_time.clear()
    GTab_mb_imgx.clear()
    GTab_mb_imgy.clear()
    GTab_MBFW.clear()
    GTab_MBFY.clear()
    GTab_djjiao_FW.clear()
    GTab_djjiao_FY.clear()
    GTab_dpwm_fw.clear()
    GTab_dpwm_fy.clear()
    GTab_distance_CSB.clear()
    GTab_distance_JG.clear()
    GTab_Gpix_targe_area.clear()
    GTab_pix_to_dis.clear()



    plot_trajectory()
    G_CapState= 0
    global capStateFlag, CNT
    capStateFlag=0
    CNT=0
    #global tracker
    tracker = None  # Set tracker to None




def djtrace_start():
    global G_DJTraceFlag
    if G_DJTraceFlag==0:
        G_DJTraceFlag = 1
        djtrace_start_button.config(text="Motor Tracking ON ")
        djtrace_start_button.config(bg="green")
    else:
        G_DJTraceFlag = 0
        djtrace_start_button.config(text="Motor Tracking OFF ")
        djtrace_start_button.config(bg="red")



def manna_auto_sel():

    global G_man_auto_st
    #State changed
    if G_man_auto_st==0:
        G_man_auto_st=1
        manna_auto_button.config(text="Manual Target Capture")
        manna_auto_button.config(bg="yellow")
    else:
        G_man_auto_st = 0
        manna_auto_button.config(text="Automatic Target Capture")
        manna_auto_button.config(bg="green")



def targesure():
    global G_mbtracesureFlag
    G_mbtracesureFlag=1

def on_radio_button_clicked():
    # Get the selected radio button value
    selected_value = radio_value_var.get()
    print("Selected radio value:", selected_value)

    global G_CaptaceModeFlag
    if selected_value=="A" :
        G_CaptaceModeFlag=1
    if selected_value=="B" :
        G_CaptaceModeFlag=2
    if selected_value=="C" :
        G_CaptaceModeFlag=3
    if selected_value=="D" :
        G_CaptaceModeFlag=4





# Create video display area
panel = tk.Label(root)
panel.pack()

label = tk.Label(root, text="Hello, Tkinter!")
label.pack()


# Create a StringVar to store radio button value
radio_value_var = tk.StringVar()
#Create radio buttons and bind them to the same StringVar
radio1 = tk.Radiobutton(root, text="Moving Object Detection", variable=radio_value_var, value="A", command=on_radio_button_clicked)
radio1.pack()
radio2 = tk.Radiobutton(root, text="Circular Target Detection", variable=radio_value_var, value="B", command=on_radio_button_clicked)
radio2.pack()

radio3 = tk.Radiobutton(root, text="Moving Object Detection + Tracking", variable=radio_value_var, value="C", command=on_radio_button_clicked)
radio3.pack()

radio4 = tk.Radiobutton(root, text="Circular Target Detection + Tracking", variable=radio_value_var, value="D", command=on_radio_button_clicked)
radio4.pack()
# Set the default selected radio button
radio1.select()

recap_button = tk.Button(root, text="Reacquire Target", command=retocap)
recap_button.pack()  #grid(row=2, column=2)
manna_auto_button = tk.Button(root, text="Manual/Auto Tracking (Manual)", command=manna_auto_sel)
manna_auto_button.pack()  #grid(row=2, column=2)
manna_auto_button.config(bg="yellow")
targesure_button = tk.Button(root, text="Confirm Target", command=targesure)
targesure_button.pack()  #grid(row=2, column=2)


djtrace_start_button = tk.Button(root, text="Motor Tracking OFF", command=djtrace_start)
djtrace_start_button.pack(side=tk.BOTTOM)  #grid(row=2, column=2)
djtrace_start_button.config(bg="red")

savedate_button = tk.Button(root, text="Save Current Trajectory Data", command=savetofile_now_data)
savedate_button.pack(side=tk.RIGHT)  #grid(row=2, column=2)
savedate_button.config(bg="cyan")





# Start updating video frames
update_frame()



#-----------------------Serial data TX/RX processing-----------------------------
#def send_data():

    #data = simpledialog.askstring("Input", "Please enter the data to send:")
    #if data:
     #   ser.write(0xEc)

def move_up():
    global  DJPWM_FY
    global  DJPWM_FW
    number = int(entry.get())
    ser.write(struct.pack('B', 0xEC))
    ser.write(struct.pack('B', 0x91))
    ser.write(struct.pack('B', 0x55))
    ser.write(struct.pack('H',(DJPWM_FW+0)))
    ser.write(struct.pack('H',(DJPWM_FY+number)))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('B', 0x0D))
    ser.write(struct.pack('B', 0x0A))

def move_down():
    global  DJPWM_FY
    global  DJPWM_FW
    number = int(entry.get())
    ser.write(struct.pack('B', 0xEC))
    ser.write(struct.pack('B', 0x91))
    ser.write(struct.pack('B', 0x55))
    ser.write(struct.pack('H',(DJPWM_FW+0)))
    ser.write(struct.pack('H',(DJPWM_FY-number)))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('B', 0x0D))
    ser.write(struct.pack('B', 0x0A))

def move_left():
    global  DJPWM_FY
    global  DJPWM_FW
    number = int(entry.get())
    ser.write(struct.pack('B', 0xEC))
    ser.write(struct.pack('B', 0x91))
    ser.write(struct.pack('B', 0x55))
    ser.write(struct.pack('H',(DJPWM_FW+number)))
    ser.write(struct.pack('H',(DJPWM_FY+0)))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('B', 0x0D))
    ser.write(struct.pack('B', 0x0A))

def move_right():
    global  DJPWM_FY
    global  DJPWM_FW
    number = int(entry.get())
    ser.write(struct.pack('B', 0xEC))
    ser.write(struct.pack('B', 0x91))
    ser.write(struct.pack('B', 0x55))
    ser.write(struct.pack('H',(DJPWM_FW-number)))
    ser.write(struct.pack('H',(DJPWM_FY+0)))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('B', 0x0D))
    ser.write(struct.pack('B', 0x0A))
def move_Reset():
    ser.write(struct.pack('B', 0xEC))
    ser.write(struct.pack('B', 0x91))
    ser.write(struct.pack('B', 0xAA))
    ser.write(struct.pack('H',(1500)))
    ser.write(struct.pack('H',(1500)))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('H',0))
    ser.write(struct.pack('B', 0x0D))
    ser.write(struct.pack('B', 0x0A))

def find_non_zero_min(numbers):
    non_zero_numbers = [num for num in numbers if num != 0]
    if non_zero_numbers:
        return min(non_zero_numbers)
    else:
        return 88  # Or a default value of your choice, e.g., float('inf')

def RCserDataProcd(data):
    MCUTIME=(data[1]<<24)+(data[2]<<16)+(data[3]<<8)+(data[4]<<8)
    global  DJPWM_FW
    global  DJPWM_FY
    DJPWM_FW = (data[5] << 8) + data[6]
    DJPWM_FY = (data[7] << 8) + data[8]
    CSBTime_CH1 = (data[9]<<8) + data[10]
    CSBTime_CH2 = (data[11] << 8) + data[12]
    CSBTime_CH3 = (data[13] << 8) + data[14]
    CSBTime_CH4 = (data[15] << 8) + data[16]
    Now_SampleCH=(data[17] << 8) + data[18]
    Now_SampleCH_State=(data[19] << 8) + data[20]
    #print('time %d',MCUTIME)
    #print('DJPWM_FW %d', DJPWM_FW)
    #print('DJPWM_FY %d', DJPWM_FY)
    #print('CSBTime_CH1 %d', CSBTime_CH1)
    #print('CSBTime_CH2 %d', CSBTime_CH2)
    #print('CSBTime_CH3 %d', CSBTime_CH3)
    #print('CSBTime_CH4 %d', CSBTime_CH4)
    #print('Now_SampleCH %d', Now_SampleCH)
    current_time_ms = int(round(time.time() * 1000))
    #print('PCtime %d',current_time_ms)

    djjiao_FW=  (DJPWM_FW - 1500) * (270.0/2000.0)
    djjiao_FY =90.0+ (DJPWM_FY - 1500) * (270.0 / 2000.0)
    showstr=' Azimuth angle: '+f"{djjiao_FW:.2f}"+' Elevation angle: '+f"{djjiao_FY:.2f}"
    SerShow.config(text=showstr)
    k=1e-6*340.0*0.5
    disCH1 = CSBTime_CH1 * k
    disCH2 = CSBTime_CH2 * k
    disCH3 = CSBTime_CH3 * k
    disCH4 = CSBTime_CH4 * k

    global MBdistance
    L=[disCH1,disCH2,disCH3,disCH4]


    if (disCH1==0)&(disCH2==0)&(disCH3==0)&(disCH4==0):
        A=0
    else:
        A=find_non_zero_min(L)
        if(A>5):
          A=0

    MBdistance = A

    #print("MBdistance",MBdistance)

    #showstr=' cH1:'+f"{disCH1:.2f}"+' cH2:'+f"{disCH2:.2f}"+' cH3:'+f"{disCH3:.2f}"+' cH4:'+f"{disCH4:.2f}"
    showstr='MBdistance:'+f"{MBdistance:.2f}"
    SerShow2.config(text=showstr)
#Read serial data
def read_data():
    if ser.in_waiting > 0:
        #data = ser.readline().decode('utf-8').rstrip()
        #print("Received data:", data)
        data = ser.read(ser.in_waiting)
        if data[0]==0xEA:
            RCserDataProcd(data)
            #hex_data = binascii.hexlify(data).decode('utf-8')
            #print(hex_data)
        #else:
           # label.config(text="Invalid data")




def updata_MCUData():
    read_data()  # Read serial data
    window1.after(5, updata_MCUData)


#window1 = tk.Tk()

#Status panel
#send_button = tk.Button(window1, text="Send", command=send_data)
#send_button.pack()
SerShow= tk.Label(window1, text="Motor Angles")
SerShow.grid(row=0, column=0)
SerShow2= tk.Label(window1, text="Distance Display")
SerShow2.grid(row=1, column=0)
#Create motion control buttons
moveup_button = tk.Button(window1, text="Move Up", command=move_up)
moveup_button.grid(row=2, column=2)

movedown_button = tk.Button(window1, text="Move Down", command=move_down)
movedown_button.grid(row=4, column=2)

moveleft_button = tk.Button(window1, text="Move Left", command=move_left)
moveleft_button.grid(row=3, column=1)

moveright_button = tk.Button(window1, text="Move Right", command=move_right)
moveright_button.grid(row=3, column=3)

moveright_button = tk.Button(window1, text="Reset", command=move_Reset)
moveright_button.grid(row=4, column=0)

# Create Entry widget
entry = tk.Entry(window1,width=30, bg="lightblue", fg="darkblue")
entry.grid(row=3, column=0) # Add the widget to the window
entry.insert(0, "100")

updata_MCUData()






#Window 3: Trajectory Plot
def plot_trajectory():

    #ax.plot(np.array(trajectory)[:, 0], np.array(trajectory)[:, 1], np.array(trajectory)[:, 2])
    #plt.draw()
    #plt.pause(0.001)
   # Clear current figure
    ax.clear()
    if len(G_capMB_X)>0:
        # Draw updated trajectory
        ax.plot(G_capMB_X, G_capMB_Y, G_capMB_Z, 'r-')

        # Update the x axis ranges
        #if len(G_capMB_X)>5:
        #    ax.set_xlim(min(G_capMB_X), max(G_capMB_X))
        #    ax.set_ylim(min(G_capMB_Y), max(G_capMB_Y))
        #    ax.set_zlim(min(G_capMB_Z), max(G_capMB_Z))
    if len(G_capMB_X)>5: #Predict landing point
        # Slice time window
        cu_capMB_X=G_capMB_X[-5:]
        cu_capMB_Y=G_capMB_Y[-5:]
        cu_capMB_Z=G_capMB_Z[-5:]
        cu_capMB_time=G_capMB_time[-5:]
        nowtime=G_capMB_time[-1]
        #Compute coefficients
        coefs_x = np.polyfit(cu_capMB_time, cu_capMB_X, 2)  # 2nd order
        coefs_y = np.polyfit(cu_capMB_time, cu_capMB_Y, 2)  # 2nd order
        coefs_z = np.polyfit(cu_capMB_time, cu_capMB_Z, 2)  # 2nd order
        #Compute predicted values

        result = np.roots(coefs_y)
        YCtime=nowtime+5
        if isinstance(result[0], (int, float)) & isinstance(result[0], (int, float)): #print("Solution exists")
           if result[1]>nowtime:
               YCtime=result[1]
           else:
               if result[0]>nowtime:
                   YCtime = result[0]
        if YCtime<nowtime:
            YCtime=nowtime+5

        #print("YCtime0",YCtime)
        tt = np.linspace(nowtime, YCtime, 20)

        YC_X = np.polyval(coefs_x,tt)
        YC_Y = np.polyval(coefs_y,tt)
        YC_Z = np.polyval(coefs_z, tt)
        ax.plot(YC_X, YC_Y, YC_Z, 'b:')


    # Redraw figure
    canvas.draw()
    #window3.after(20, plot_trajectory)



plot_trajectory()


# Run main loop
root.mainloop()
window3.mainloop()
window1.mainloop()

#Release resources
cap.release()
cv2.destroyAllWindows()
ser.close()


