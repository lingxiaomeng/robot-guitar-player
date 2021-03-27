from simple_http_server import request_map

from yolov3.get_image import ImageUtils
main = ImageUtils()


@request_map("/detect")
def detect():
    point = main.get_image()
    return f"<html><body>{point}<body><html>"
