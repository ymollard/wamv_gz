#!/usr/bin/env python3
# Noeud qui détecte le navire cible avec YOLO et oriente le WAM-V vers lui avec un PD
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO

BOAT_CLASS_ID = 8  # Identifiant de la classe "boat" dans le jeu de données COCO


class VesselFollower(Node):
    def __init__(self):
        super().__init__('vessel_follower')
        # Paramètres modifiables en direct avec : ros2 param set /vessel_follower kp 2.0
        self.declare_parameter('kp', 1.0)       # Gain proportionnel
        self.declare_parameter('kd', 0.2)       # Gain dérivé
        self.declare_parameter('forward', 2.0)  # Poussée constante vers l'avant

        self.bridge = CvBridge()  # Convertit les images ROS <-> OpenCV
        self.model = YOLO('yolov8n.pt')  # Téléchargé automatiquement au 1er lancement
        self.previous_error = 0.0
        self.previous_time = None

        self.image_pub = self.create_publisher(Image, '/vessel_follower/image', 10)
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
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

        boxes = results[0].boxes
        if len(boxes) == 0:
            # Cible perdue : on coupe les moteurs
            self.send_thrust(0.0, 0.0)
            self.previous_time = None
            return

        # Boîte la plus sûre, puis son centre horizontal en pixels
        best = boxes.conf.argmax()
        x_center = float(boxes.xywh[best][0])

        # Erreur entre -1 (cible tout à gauche) et +1 (cible tout à droite)
        half_width = msg.width / 2
        error = (x_center - half_width) / half_width

        # Dérivée de l'erreur, calculée avec l'horodatage des images
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        derivative = 0.0
        if self.previous_time is not None and now > self.previous_time:
            derivative = (error - self.previous_error) / (now - self.previous_time)
        self.previous_error = error
        self.previous_time = now

        # Commande PD
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        forward = self.get_parameter('forward').value
        turn = kp * error + kd * derivative

        # Cible à droite (turn > 0) : moteur gauche plus fort pour tourner à droite
        self.send_thrust(forward + turn, forward - turn)
        self.get_logger().info(f'🐧 erreur={error:+.2f} dérivée={derivative:+.2f} rotation={turn:+.2f}')

    def send_thrust(self, left, right):
        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))


def main():
    rclpy.init()
    rclpy.spin(VesselFollower())


if __name__ == '__main__':
    main()
