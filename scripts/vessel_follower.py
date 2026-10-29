#!/usr/bin/env python3
# Noeud qui détecte le navire cible avec YOLO dans l'image de la caméra avant
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

BOAT_CLASS_ID = 8  # Identifiant de la classe "boat" dans le jeu de données COCO


class VesselFollower(Node):
    def __init__(self):
        super().__init__('vessel_follower')
        self.bridge = CvBridge()  # Convertit les images ROS <-> OpenCV
        self.model = YOLO('yolov8n.pt')  # Téléchargé automatiquement au 1er lancement
        self.image_pub = self.create_publisher(Image, '/vessel_follower/image', 10)
        self.create_subscription(Image, '/wamv/sensors/camera/front/image', self.on_image, 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Détection limitée à la classe "boat"
        results = self.model(frame, classes=[BOAT_CLASS_ID], verbose=False)
        # plot() dessine boîtes + classe + confiance sur une copie de l'image
        annotated = results[0].plot()
        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(VesselFollower())


if __name__ == '__main__':
    main()
