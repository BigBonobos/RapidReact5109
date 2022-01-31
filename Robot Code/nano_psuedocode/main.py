'''
Vidur Modgil
Nano Code
'''

import tflite_runtime.interpreter as tflite
import cv2
import numpy as np
from PIL import Image
from networktables import NetworkTablesInstance
    

def main():

    interpreter = tflite.Interpreter("./ballTracker.tflite")
    resolution = 1280
    cam = cv2.VideoCapture()
    cam.open(0, cv2.CAP_DSHOW)
    ntwrkInst = NetworkTablesInstance.getDefault()
    ballAlign = ntwrkInst.getTable("ballAlignment")
    alliance = ballAlign.getEntry("alliance").getString()
    velocity = ballAlign.getEntry("tVelocity")

    if (alliance != None):
        ret = True
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
            top_k = result.argsort()[-5:][::-1]
            targetBall = None
            currentArea = 0
            for i in top_k:
                if(i[4] == alliance):
                    centralPosX = (i[0] + i[2])/2
                    centralPosY = (i[1] + i[3])/2
                    area = abs((i[2] - i[0]) * (i[3] - i[1]))
                    if (area > currentArea):
                        currentArea = area
                        targetBall = (centralPosX, centralPosY)
                        ballLength = i[2] - i[0]
            xSpeed = 22.86*targetBall[0]/ballLength
            ySpeed = (resolution*22.86)/(ballLength*2*(3**0.5))
            velocity.setDoubleArray([xSpeed, ySpeed])
    else:
        velocity.setDoubleArray([0.0, 0.0])

if __name__ == '__main__': 
    main()
