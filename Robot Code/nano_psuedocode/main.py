'''
Vidur Modgil
Nano Code
'''

import tflite_runtime.interpreter as tflite
import cv2
import numpy as np
from PIL import Image

def main():

    interpreter = tflite.Interpreter("./ballTracker.tflite")
    cam = cv2.VideoCapture()
    cam.open(0, cv2.CAP_DSHOW)

    while (ret):
        ret, frame = cam.read()
        input_details = interpreter.get_input_details()
        height = input_details[0]['shape'][1]
        width = input_details[0]['shape'][2]
        img = Image.fromarray(frame).resize((width, height))
        input_data = np.expand_dims(img, 0)
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_details = interpreter.get_output_details()
        output = interpreter.get_tensor(output_details[0]['index'])
        result = np.squeeze(output)
        top_k = results.argsort()[-5:][::-1]
        for i in top_k:
            print(i)

if __name__ == '__main__': 
    main()
