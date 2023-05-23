import serial
import time
import cv2
import os
import numpy as np
import sys
from datetime import datetime
from tensorflow.lite.python.interpreter import Interpreter

if __name__ == "__main__":
    min_conf_threshold = float(0.5)
    resW, resH = ('1280'), ('720')
    imW, imH = int(resW), int(resH)
    PATH_TO_CKPT = 'detect.tflite'

    PATH_TO_LABELS = 'waste_label_map.txt'

    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
    interpreter = Interpreter(model_path=PATH_TO_CKPT)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]

    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5

    boxes_idx, classes_idx, scores_idx = 1, 3, 0

    ser = serial.Serial('COM5', 9600)  # /dev/ttyACM0
    ser.flush()

    cap = cv2.VideoCapture(0)


    is_window_open = False


    if not os.path.exists("history"):
        os.makedirs("history")

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "cam_detected":
                ret, frame = cap.read()
                if ret:
                    now = datetime.now()
                    dt_string = now.strftime("%Y-%m-%d_%H-%M-%S")
                    image_file_name = f"{dt_string}.jpg"
                    image_file_path = os.path.join("history", image_file_name)
                    cv2.imwrite("captured_image.jpg", frame) 
                    image = cv2.imread("captured_image.jpg")
                    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    imH, imW, _ = image.shape
                    image_resized = cv2.resize(image_rgb, (width, height))
                    input_data = np.expand_dims(image_resized, axis=0)
                    if floating_model:
                        input_data = (np.float32(input_data) - input_mean) / input_std
                    interpreter.set_tensor(input_details[0]['index'], input_data)
                    interpreter.invoke()

                    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0]  # Bounding box coordinates of detected objects
                    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0]  # Class index of detected objects
                    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0]  # Confidence of detected objects

                    detections = []
                    for i in range(len(scores)):
                        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
                            ymin = int(max(1, (boxes[i][0] * imH)))
                            xmin = int(max(1, (boxes[i][1] * imW)))
                            ymax = int(min(imH, (boxes[i][2] * imH)))
                            xmax = int(min(imW, (boxes[i][3] * imW)))

                            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

                            object_name = labels[int(classes[i])]
                            label = '%s: %d%%' % (object_name, int(scores[i] * 100))
                            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                            label_ymin = max(ymin, labelSize[1] + 10)
                            cv2.rectangle(image, (xmin, label_ymin - labelSize[1] - 10),
                                        (xmin + labelSize[0], label_ymin + baseLine - 10),
                                        (255, 255, 255), cv2.FILLED)
                            cv2.putText(image, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                        (0, 0, 0), 2)

                            detections.append([object_name, scores[i], xmin, ymin, xmax, ymax])


                    cv2.imwrite(image_file_path, image)



