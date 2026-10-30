#!/usr/bin/env python3
# Noeud qui détecte le navire cible avec YOLO et oriente le WAM-V vers lui avec un PD.
# YOLO est lent : on ne le lance qu'une image sur N, et entre deux on suit
# le navire avec une simple recherche de motif OpenCV (matchTemplate), très rapide.
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO

BOAT_CLASS_ID = 8      # Identifiant de la classe "boat" dans le jeu de données COCO
SEARCH_MARGIN = 50     # Pixels autour de la dernière boîte où chercher le navire
MIN_MATCH_SCORE = 0.5  # En dessous, le suivi est considéré comme perdu


class VesselFollower(Node):
    def __init__(self):
        super().__init__('vessel_follower')
        # Paramètres modifiables en direct avec : ros2 param set /vessel_follower kp 2.0
        self.declare_parameter('kp', 1.0)         # Gain proportionnel
        self.declare_parameter('kd', 0.2)         # Gain dérivé
        self.declare_parameter('forward', 2.0)    # Poussée constante vers l'avant
        self.declare_parameter('yolo_every', 6)   # YOLO une image sur N

        self.bridge = CvBridge()  # Convertit les images ROS <-> OpenCV
        self.model = YOLO('yolov8n.pt')  # Téléchargé automatiquement au 1er lancement
        self.previous_error = 0.0
        self.previous_time = None
        self.frame_count = 0
        self.template = None  # Imagette du navire découpée par YOLO
        self.box = None       # Dernière boîte connue : (x, y, largeur, hauteur)

        self.image_pub = self.create_publisher(Image, '/vessel_follower/image', 10)
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.create_subscription(Image, '/wamv/sensors/camera/front/image', self.on_image, 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_count += 1

        # YOLO si on n'a rien à suivre, ou périodiquement pour recaler le suivi
        yolo_every = self.get_parameter('yolo_every').value
        if self.template is None or self.frame_count % yolo_every == 0:
            label = self.detect_with_yolo(frame)
        else:
            label = self.track_with_template(frame)

        if self.box is not None:
            x, y, w, h = self.box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

        if self.box is None:
            # Cible perdue : on coupe les moteurs
            self.send_thrust(0.0, 0.0)
            self.previous_time = None
            return

        # Erreur entre -1 (cible tout à gauche) et +1 (cible tout à droite)
        x_center = self.box[0] + self.box[2] / 2
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
        self.get_logger().info(f'🐧 [{label}] erreur={error:+.2f} dérivée={derivative:+.2f} rotation={turn:+.2f}')

    def detect_with_yolo(self, frame):
        # Détection limitée à la classe "boat"
        boxes = self.model(frame, classes=[BOAT_CLASS_ID], verbose=False)[0].boxes
        if len(boxes) == 0:
            self.template = None
            self.box = None
            return ''

        # Boîte la plus sûre
        best = boxes.conf.argmax()
        x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[best]]
        self.box = (x1, y1, x2 - x1, y2 - y1)
        # copy() : l'imagette ne doit pas être modifiée quand on dessine sur frame
        self.template = frame[y1:y2, x1:x2].copy()
        return f'YOLO boat {float(boxes.conf[best]):.2f}'

    def track_with_template(self, frame):
        # Zone de recherche : la dernière boîte agrandie de SEARCH_MARGIN, sans sortir de l'image
        x, y, w, h = self.box
        x1 = max(x - SEARCH_MARGIN, 0)
        y1 = max(y - SEARCH_MARGIN, 0)
        x2 = min(x + w + SEARCH_MARGIN, frame.shape[1])
        y2 = min(y + h + SEARCH_MARGIN, frame.shape[0])
        search_area = frame[y1:y2, x1:x2]

        # Score de ressemblance pour chaque position possible (1 = identique)
        scores = cv2.matchTemplate(search_area, self.template, cv2.TM_CCOEFF_NORMED)
        _, best_score, _, best_position = cv2.minMaxLoc(scores)
        if best_score < MIN_MATCH_SCORE:
            # Suivi perdu : YOLO sera relancé à l'image suivante
            self.template = None
            self.box = None
            return ''

        self.box = (x1 + best_position[0], y1 + best_position[1], w, h)
        return f'suivi {best_score:.2f}'

    def send_thrust(self, left, right):
        self.left_pub.publish(Float64(data=left))
        self.right_pub.publish(Float64(data=right))


def main():
    rclpy.init()
    rclpy.spin(VesselFollower())


if __name__ == '__main__':
    main()
