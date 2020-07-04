# In this task Cascade Classifier will be used to detect faces in a photo
import cv2

# The classifier path
faceCascade = cv2.CascadeClassifier("FrontalFace.xml")

# The image path
image = cv2.imread("TestImg.jpg")

# To convert the image into grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Look for faces in the image using the loaded cascade file
faces = faceCascade.detectMultiScale(gray, 1.02, 20)

for (x,y,w,h) in faces:
    # Create rectangular frame around faces in the image
    cv2.rectangle(image,(x,y),(x+w,y+h),(255, 153, 51),2)
# Will show the output in a file called 'output.jpg'	
cv2.imwrite('output.jpg', image)

# Uncomment this line if you want to show the output image immediately 
# without being in a file but if did so you should comment the previous line
# cv2.imshow('output.jpg', image)
# cv2.waitKey()