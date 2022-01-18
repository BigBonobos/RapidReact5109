'''
Part of Project for Getting Images to put into Axon
Vidur Modgil
5109 Gladiator Robotics
'''

# Imports
from youtube_dl import YoutubeDL
import cv2
import os
from superviselyClient import Supervisely
from concurrent.futures import ThreadPoolExecutor

def create_and_upload_frame(current_frame: int, filename: list, cam, sly, subcounter):
    # reads individual image

    if (subcounter == 4):
        ret, frame = cam.read()

        # Saves indiv images
        name = f"./image_data {filename[0]}/frame {current_frame}.jpg"
        print(f"Creating {name}")
        cv2.imwrite(name, frame)
        sly.upload_image(name)
        os.remove(name)
    else:
        ret = True
    
    return ret

def main():
    api_key = input("Enter your api key for supervisely: ")
    project_id = int(input("Enter your supervisely project_id: "))
    playlist_url = input("Enter playlist url here: ")
    try:
        playlist_url = playlist_url.split("index")[0]
    except:
        playlist_url = playlist_url
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
    sly.create_dataset("FRC Game Piece Sorting16", "Machine Vision Labelling for FRC")

    # Gets all files into list
    for (_, _, filenames) in os.walk(os.getcwd()):

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
                subcounter = 0

                # While there are frames, run
                while (ret):
                    try:
                        ret = True
                        with ThreadPoolExecutor() as pool:
                            pool.submit(create_and_upload_frame, current_frame, split_file, cam, sly, subcounter)
                        current_frame += 1
                        if (subcounter == 4):
                            subcounter = 0
                        else:
                            subcounter += 1
                    except Exception as e:
                        print(f"Error: {e}")
                        ret = False

                cam.release()
    cv2.destroyAllWindows()





if __name__ == '__main__':
    main()