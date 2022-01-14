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
from superviselyClient import Supervisely


def main():
    api_key = input("Enter your api key for supervisely: ")
    project_id = input("Enter you supervisely project_id: ")
    playlist_url = input("Enter playlist url here: ")
    playlist_url = playlist_url.split("index")[0]
    playlist_url = playlist_url + "index="
    sly = Supervisely(api_key, project_id)
    ytdl = YoutubeDL()

    # Initialization of object
    videos = []
    
    # Gets Video URLS from playlist
    for i in range(3):
        print(playlist_url + str(i + 1))
        videos.append(playlist_url + str(i + 1))

    # Downloads videos from generated list
    ytdl.download(videos)
    sly.create_dataset("FRC Game Piece Sorting", "Machine Vision Labelling for FRC")

    # Gets all files into list
    for (dirpath, dirnames, filenames) in os.walk(os.getcwd()):

        # Iterates through all files in directory
        for file in filenames:

            # Splits file string by . to get file extension
            split_file = file.split(".")
            extension = split_file[len(split_file) - 1]

            # only run code if file is mp3
            if (extension == "mp4" or extension == "webm" or extension == "mkv"):
                
                # Reads video into cv2 Video Object
                cam = cv2.VideoCapture(file)

                # Creates Directory to store images
                if not os.path.exists(f"./image_data {split_file[0]}"):
                    os.makedirs(f"./image_data {split_file[0]}")

                # Initializes variables needed in next part of the code
                ret = True
                current_frame = 0

                # While there are frames, run
                while (ret):
                    try:
                        # reads individual imgae
                        ret, frame = cam.read()

                        # Saves indiv images
                        name = f"./image_data {split_file[0]}/frame {current_frame}.jpg"
                        sly.upload_image(name)
                        print(f"Creating {name}")
                        cv2.imwrite(name, frame)
                        current_frame += 1
                    except:
                        ret = False

                cam.release()
    cv2.destroyAllWindows()





if __name__ == '__main__':
    main()