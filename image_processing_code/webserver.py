from flask import Flask
from flask import request
from image_getter import upload_video


app = Flask(__name__)

@app.route("/api/get_image")
def get_image():
    if (request.method == "POST"):
        request_data = request.get_json()
        api_key = request_data['api_key']
        project_id = request_data['project_id']
        playlist_url = request_data['playlist_url']
        amt_videos = int(request_data['amt_videos'])
        upload_video(api_key, project_id, plalist_url, amt_videos)
        return "Uploading"


if(__name__ == '__main__'):
    app.run(debug=True, port=3001)