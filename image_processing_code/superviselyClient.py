'''
Vidur Modgil
Gladiator Robotics
Team 5109
Supervisely Client interaction for python
'''

# Preprossescor Directives
import requests
from json import dumps
import base64


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
            body = file.read()
            headers = self.headers.copy()
            headers['Content-Type'] = "application/octet-stream"
            resp = requests.post(self.base_url + "images.upload", data=body, headers=headers)
        resp = resp.json()
        hashFinal = resp['hash']
        data = {
            "dataset_id": self.dataset_id,
            "images": [{
                "hash": hashFinal
            }],
            generate_unique_names: True
        }
        resp = requests.post(self.base_url + "images.bulk.add", json=body, headers=self.headers)