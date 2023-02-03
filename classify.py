import tensorflow as tf
import numpy as np
import cv2 as cv

VGG16 = tf.keras.models.load_model('/home/jovian/SBIN/VGG16.h5')

def use_model(img):

    pic = cv.imread(img)
    pic = cv.resize(pic,(150, 150))/255
    pic = np.expand_dims(pic, axis=0)
    classes = VGG16.predict(pic)
    predict =np.argmax(classes, axis=1)
    return predict
 
print(use_model('/home/jovian/SBIN/test.png'))