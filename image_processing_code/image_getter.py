'''
Part of Project for Getting Images to put into Axon
Doing this because I'm bored as fuck
Vidur Modgil
5109 Gladiator Robotics
'''

# Imports
from youtube_dl import YoutubeDL
import cv2
import os


def main() :
    videos = []
    ytdl = YoutubeDL()
    
    for i in range(3):
        videos.append(f"https://www.youtube.com/watch?v=OXkeAkWwex8&list=PLZT9pIgNOV6a_P3_HDTolvzprSdYB_zd3&index={i + 1}")
    ytdl.download(videos)

    for (dirpath, dirnames, filenames) in os.walk(os.getcwd()):

        for file in filenames:

            extension = file.split(".")[1]

            if (extension == "mp4"):

                cam = cv2.VideoCapture(file)
                if not os.path.exists("./image_data"):
                    os.makedirs("./image_data")

                ret = True
                current_frame = 0

                while (ret):
                    ret, frame = cam.read()

                    name = f"./image_data/frame {current_frame}.jpg"
                    print(f"Creating {name}")

                    cv2.imwrite(name, frame)

                    current_frame += 1

                cam.release()
                cv2.destroyAllWindows()





if __name__ == '__main__':
    main()