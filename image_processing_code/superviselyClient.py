'''
Vidur Modgil
Gladiator Robotics
Team 5109
Supervisely Client interaction for python
'''

# Preprossescor Directives
import requests
import base64
from json import dumps
import re


def decode_base64(data, altchars=b'+/'):
    """Decode base64, padding being optional.

    :param data: Base64 data as an ASCII byte string
    :returns: The decoded byte string.

    """
    data = re.sub(rb'[^a-zA-Z0-9%s]+' % altchars, b'', data)  # normalize
    missing_padding = len(data) % 4
    if missing_padding:
        data += b'='* (4 - missing_padding)
    return base64.b64decode(data, altchars)

class Supervisely:
    def __init__(self, api_key: str, project_id: int):
        self.headers = {
            "x-api-key": api_key
        }
        self.base_url = "https://app.supervise.ly/public/api/v3/"
        self.project_id = project_id
        self.dataset_id = None  

    def create_dataset(self, datasetName: str, description: str):
        body = {
            "projectId": self.project_id,
            "name": datasetName,
            "description": description
        }
        resp = requests.post(self.base_url + "datasets.add", json=body, headers=self.headers)
        json_final = resp.json()
        print(json_final)
        self.dataset_id = json_final["id"]
    
    def upload_image(self, image_path):
        resp = None
        with open(image_path, "rb") as file:
            image_name = image_path.split("/")
            image_name = image_name[len(image_name) - 1]
            headers = self.headers.copy()
            headers['Content-Type'] = "application/octet-stream"
            resp = requests.post(self.base_url + "images.upload", data=file, headers=headers)
        resp = resp.json()
        print(resp)
        hashFinal = resp['hash']
        data = {
            "datasetId": self.dataset_id,
            "images": [{
                "hash": hashFinal,
                "name": image_name,
            }],
            "generate_unique_names": True
        }
        resp = requests.post(self.base_url + "images.bulk.add", json=data, headers=self.headers)
        print(resp.text)