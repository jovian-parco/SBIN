import tensorflow as tf
import numpy as np
import cv2 as cv

class WasteClassifier:
    def __init__(self):
        self.VGG16 = tf.keras.models.load_model('/home/jovian/SBIN/VGG16.h5')
        self.image = ""
        self.predict = ""
    def capture_img(self):
        cam = cv.VideoCapture(0)
        result, self.image = cam.read()
        if result:
            return self.image
        else:
            print("No image detected. Please! try again")

    def classify(self):
        self.capture_img()
        pic = cv.resize(self.image,(150, 150))/255
        pic = np.expand_dims(pic, axis=0)
        classes = self.VGG16.predict(pic)
        self.predict =np.argmax(classes, axis=1)
        return self.predict, self.image

shit=WasteClassifier()
shit.classify() 
cv.imshow("test",shit.image)
cv.waitKey(0)
cv.destroyAllWindows()
print(shit.predict)