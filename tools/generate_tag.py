import apriltag
import cv2

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

img = detector.detection_engine.generate_tag(0, 500)

cv2.imwrite(
"/home/adhithyan_rajeevan24/ardupilot_gazebo/models/apriltag/materials/textures/tag.png",
img
)
