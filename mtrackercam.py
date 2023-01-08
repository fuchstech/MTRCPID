import cv2
import  numpy as np
from threading import Thread


cap = cv2.VideoCapture(0)


# create a dictionary of all trackers in OpenCV that can be used for tracking
OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.legacy.TrackerCSRT_create,
	"kcf": cv2.legacy.TrackerKCF_create,
	"boosting": cv2.legacy.TrackerBoosting_create,
	"mil": cv2.legacy.TrackerMIL_create,
	"tld": cv2.legacy.TrackerTLD_create,
	"medianflow": cv2.legacy.TrackerMedianFlow_create,
	"mosse": cv2.legacy.TrackerMOSSE_create
}


# Create MultiTracker object
trackers = cv2.legacy.MultiTracker_create()
x,y,w,h = 0,0,0,0
def getV():
    while 1:
        if x+y != 0:
            print(x,y,w,h,"Xcenter=",x+w/2,"Ycenter=",y+h/2)
g = Thread(target=getV)
g.start()
while True:
    frame = cap.read()[1]

    if frame is None:
        break


    (success, boxes) = trackers.update(frame)
    #print(success,boxes)
    # loop over the bounding boxes and draw then on the frame
    if success == False:
        bound_boxes = trackers.getObjects()
        idx = np.where(bound_boxes.sum(axis= 1) != 0)[0]
        bound_boxes = bound_boxes[idx]
        trackers = cv2.legacy.MultiTracker_create()
        for bound_box in bound_boxes:
            trackers.add(tracker,frame,bound_box)

    for i,box in enumerate(boxes):
        (x, y, w, h) = [int(v) for v in box]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame,'Obje',(x+10,y-3),cv2.FONT_HERSHEY_PLAIN,1.5,(255,255,0),2)
    
    cv2.imshow('Frame', frame)
    k = cv2.waitKey(30)

    if k == ord("s"):
        # select the bounding box of the object we want to track (make
        # sure you press ENTER or SPACE after selecting the ROI)
        roi = cv2.selectROI("Frame", frame, fromCenter=False,
                            showCrosshair=True)
        # create a new object tracker for the bounding box and add it
        # to our multi-object tracker
        tracker = OPENCV_OBJECT_TRACKERS['kcf']()
        trackers.add(tracker, frame, roi)


cap.release()
cv2.destroyAllWindows()