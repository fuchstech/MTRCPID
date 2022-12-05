import cv2
dispW= 640
dispH= 480
flip=2
camSet = 0
cam=cv2.VideoCapture(camSet)
dispW= int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
dispH= int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
BW= int(0.15*dispW)
BH= int(0.15*dispH)
posX=10
posY=270
dx=2
dy=2
while True:
    ret, frame= cam.read()
    roi= frame[posY:posY+BH, posX:posX+BW].copy()
    frame= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame= cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    frame= cv2.rectangle(frame,(posX,posY),(posX+BW,posY+BH),(0,0,0),3)
    frame[posY:posY+BH, posX:posX+BW]=roi
    
    posX=posX+dx
    posY=posY+dy
    if(posX+BW>=640 or posX<=0):
        dx=dx*(-1)
    if(posY+BH>=480 or posY<=0):
        dy=dy*(-1)
    
    cv2.imshow('nanoCam', frame)
    if cv2.waitKey(1) == ord('q'):
        break
cam.release()
cv2.destroyAllWindows()