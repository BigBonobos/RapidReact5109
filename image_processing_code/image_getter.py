'''
Part of Project for Getting Images to put into Axon
Doing this because I'm bored as fuck
Vidur Modgil
5109 Gladiator Robotics
'''

# Imports
import requests
import youtube_dl


def main() :
    ytdl = youtube_dl.YoutubeDL()
    ytdl.download(["https://www.youtube.com/watch?v=OXkeAkWwex8"])


if __name__ == '__main__':
    main()