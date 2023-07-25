import serial
import time
import cv2
import os
import numpy as np
from datetime import datetime
from tensorflow.lite.python.interpreter import Interpreter

developer_mode = False

if developer_mode:
    custom_model_path = '/home/sbin/Desktop/Detection_Deploy/detect_custom.tflite'
    custom_label_path = '/home/sbin/Desktop/Detection_Deploy/label_map_custom.txt'
    custom_min_conf_threshold = 0.5
    custom_box_color = (0, 255, 255)  # Yellow

    coco_model_path = '/home/sbin/Desktop/Detection_Deploy/detect_coco.tflite'
    coco_label_path = '/home/sbin/Desktop/Detection_Deploy/label_map_coco.txt'
    coco_min_conf_threshold = 0.5
    coco_box_color = (0, 255, 0)  # Green
    history_dir = "/home/sbin/Desktop/Detection_Deploy/history_developer"
else:
    custom_model_path = '/home/sbin/Desktop/Detection_Deploy/detect_custom.tflite'
    custom_label_path = '/home/sbin/Desktop/Detection_Deploy/label_map_custom.txt'
    custom_min_conf_threshold = 0.5
    custom_box_color = (0, 255, 255)  # Yellow
    history_dir = "/home/sbin/Desktop/Detection_Deploy/history"

if not developer_mode:
    coco_model_path = '/home/sbin/Desktop/Detection_Deploy/detect_coco.tflite'  
    coco_label_path = '/home/sbin/Desktop/Detection_Deploy/label_map_coco.txt' 
    coco_min_conf_threshold = 0.5  
    coco_box_color = (0, 255, 0)  

ser = serial.Serial('/dev/ttyUSB0', 9600) #'/dev/ttyUSB0'
ser.flush()

def initialize_interpreter(model_path):
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter

def load_labels(path):
    with open(path, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
    return labels

def preprocess_image(image, width, height, input_mean, input_std, floating_model):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height))
    input_data = np.expand_dims(image_resized, axis=0)
    if floating_model:
        input_data = (np.float32(input_data) - input_mean) / input_std
    return input_data

def detect_objects(interpreter, image, labels, min_conf_threshold, box_color):
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    boxes_idx, classes_idx, scores_idx = 1, 3, 0

    image_resized = cv2.resize(image, (int(resW), int(resH)))

    input_data = preprocess_image(image_resized, input_details[0]['shape'][2], input_details[0]['shape'][1],
                                  input_mean=127.5, input_std=127.5, floating_model=(input_details[0]['dtype'] == np.float32))
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0]
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0]
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0]

    detections = []
    for i in range(len(scores)):
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
            imH, imW, _ = image.shape
            ymin = int(max(1, (boxes[i][0] * imH)))
            xmin = int(max(1, (boxes[i][1] * imW)))
            ymax = int(min(imH, (boxes[i][2] * imH)))
            xmax = int(min(imW, (boxes[i][3] * imW)))

            detections.append([labels[int(classes[i])], scores[i], xmin, ymin, xmax, ymax])

            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), box_color, 2)
            object_name = labels[int(classes[i])]
            label = '%s: %d%%' % (object_name, int(scores[i] * 100))
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            label_xmin = max(xmin, xmax - labelSize[0] - 10)
            cv2.rectangle(image, (label_xmin - 5, ymin - labelSize[1] - 10),
                          (xmax + 5, ymin), box_color, cv2.FILLED)
            cv2.putText(image, label, (label_xmin, ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 0), 2)

    return detections, image

def capture_image(output_dir):
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H-%M-%S")
    image_file_name = f"{dt_string}.jpg"
    image_file_path = os.path.join(output_dir, image_file_name)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(resW))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(resH))
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(image_file_path, frame)
    #cap.release()
    return image_file_path

def process_detection(detections, ser):
    bio_objects = [
        "paper_carton",
        "paper_crumpled",
        "paper_cup",
        "paper_container",
        "paper_sheet",
        "paper_tissue"
    ]
    non_bio_objects = [
        "pen",
        "plastic_bag",
        "plastic_bottle",
        "plastic_cup",
        "plastic_lid",
        "plastic_straw",
        "plastic_utensils",
        "candy_wrapper"
    ]

    if len(detections) == 1:
        object_name = detections[0][0]
        if object_name == "facemask" or object_name not in bio_objects + non_bio_objects:
            ser.write("reject".encode())
            print("reject")
        elif object_name in bio_objects:
            ser.write("bio".encode())
            print("bio")
        elif object_name in non_bio_objects:
            ser.write("non_bio".encode())
            print("non_bio")
    elif len(detections) > 1 or len(detections) < 1:
        ser.write("reject".encode())
        print("reject")
    else:
        ser.write("no object detected".encode())
        print("no object detected")

def create_history_directory(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

if __name__ == "__main__":
    resW, resH = '1280', '720'
    input_mean = 127.5
    input_std = 127.5

    create_history_directory(history_dir)

    time.sleep(3)
    ser.write("rpi ready".encode())
    print("rpi ready")

    interpreter_custom = initialize_interpreter(custom_model_path)
    labels_custom = load_labels(custom_label_path)
    interpreter_coco = initialize_interpreter(coco_model_path)
    labels_coco = load_labels(coco_label_path)

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            if line == "cam_detected":
                time.sleep(2)
                image_file_path = capture_image(history_dir)
                image = cv2.imread(image_file_path)

                detections_custom, image_custom = detect_objects(
                    interpreter_custom, image, labels_custom, custom_min_conf_threshold, custom_box_color)
                
                if developer_mode:
                    detections_coco, image_coco = detect_objects(
                        interpreter_coco, image, labels_coco, coco_min_conf_threshold, coco_box_color)
                    detections = detections_custom #+ detections_coco
                    image = cv2.addWeighted(image_custom, 1, image_coco, 0.5, 0)
                else:
                    detections = detections_custom

                process_detection(detections, ser)
                cv2.imwrite(image_file_path, image)
