import cv2
from cv2 import aruco
import paho.mqtt.client as mqtt
from concurrent.futures import ThreadPoolExecutor
from include.Constants import IMAGE_SIZE, CAMERA_INDEX
from include.Planner import Paths_planner
from include.MarkersAnalizer import MarkerAnalizer
import include.mqtt_utils as mqtt_utils
from time import time


# Init aruco parametrs
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()

# Init cv2 stream and window parameters
stream = cv2.VideoCapture(CAMERA_INDEX)
stream.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_SIZE)
stream.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_SIZE)
stream.set(cv2.CAP_PROP_FPS, 30)
cv2.namedWindow("Capture", cv2.WINDOW_NORMAL)
cv2.moveWindow("Capture", 50, 50)
RUN_CAPTURE = True

# Init marker analizer
analizer = MarkerAnalizer()

# Init path planner
planner = Paths_planner()
PATH_CREATED = False
actual_path = None
actual_pt = None

# Init mqtt
SPIIRAS_IP = "192.168.0.116"
MOTO_IP = "192.168.43.51"
IGOR_GARAGE_IP = "10.1.30.38"
IVAN_GARAGE_IP = "10.1.30.46"

SERVER_IP = MOTO_IP
PORT = 1883
PLATFORMS_SET = {5,}
FEEDBACK_TOPIC = "esp32/feedback/+"
CONNECTON_TOPIC = "Connected"
MSG_TOPIC = "platforms/"

NUMBER_OF_CLIENTS = 1

MAX_WORKERS = 10
WORKER_TIME = 0.1
MESSAGES_QOS = 2
LAST_SENDING = time()
SINDING_PERIOD = 0.5

client = mqtt.Client("")

ON_FINISH = 0

def resize_image_to_square_size(image):
    w, h, _ = image.shape
    if h != w:
        h_to_cut = (h - w)//2
        sqr_image = image[0:w, h_to_cut:h - h_to_cut]
        return sqr_image
    else:
        return image


client = mqtt.Client("BOSS")
pool = ThreadPoolExecutor(max_workers=MAX_WORKERS)
mqtt_utils.start_connection_client(client, SERVER_IP, PORT, CONNECTON_TOPIC)
client.subscribe(FEEDBACK_TOPIC, qos=MESSAGES_QOS)
client.loop(WORKER_TIME)

while RUN_CAPTURE:
    ret, img = stream.read()
    if ret:
        img = resize_image_to_square_size(img)
        img_to_show = img.copy()

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners_np, ids_np, _ = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        aruco.drawDetectedMarkers(img_to_show, corners_np, ids_np)

        if type(ids_np) != type(None):
            ids = list(map(lambda x: x[0], ids_np))
            corners = list(map(lambda x: x[0], corners_np))
            markers_dict = dict(zip(ids, corners))

            analizer.parse_fields_objects_by_id(markers_dict)

            robots = analizer.get_robots()

            if len(robots):
                if not PATH_CREATED:
                    goals = analizer.get_goals()
                    obstacles = analizer.get_obstacles()
                    planner.set_robots(robots)
                    planner.set_targets(goals)
                    planner.set_obstacles(obstacles)

                    paths, time_el = planner.multiple_paths_planning()

                    if len(paths):
                        mqtt_utils.send_msg(client, 5, msg_topic="on_finish/", qos=2, msg=ON_FINISH)
                        for path_id in paths.keys():
                            PATH_CREATED = True
                            actual_path = planner.path_to_point_list(paths[path_id])
                            actual_path.pop(0)
                            actual_path.pop(0)
                            actual_pt = actual_path.pop(0)

                else:
                    robot = robots[list(robots.keys())[0]]
                    robot_position = robot.get_position()
                    angle = robot.get_angle_to_point(actual_pt)
                    now = time()
                    if now - LAST_SENDING > SINDING_PERIOD:
                        mqtt_utils.send_msg(client, 5, msg_topic=MSG_TOPIC, qos=2, msg=angle)
                        LAST_SENDING = time()
                    on_point = analizer.on_position(robot_position, actual_pt)
                    cv2.circle(img_to_show, (int(actual_pt.x), int(actual_pt.y)), 8, (0, 255, 0), 3)
                    if on_point:
                        try:
                            actual_pt = actual_path.pop(0)
                            mqtt_utils.send_msg(client, 5, msg_topic="on_finish/", qos=2, msg=ON_FINISH)
                        except:
                            ON_FINISH = 1
                            mqtt_utils.send_msg(client, 5, msg_topic="on_finish/", qos=2, msg=ON_FINISH)
                            print("Finish!")
                            RUN_CAPTURE = False

        if not isinstance(None, type(actual_path)):
            for pt in actual_path:
                cv2.circle(img_to_show, (int(pt.x), int(pt.y)), 5, (255, 255, 0), 8)

        cv2.imshow("Capture", img_to_show)

        if cv2.waitKey(10) & 0xFF == 27:
            RUN_CAPTURE = not RUN_CAPTURE
    else:
        print("Can't connect to camera with index << {} >>".format(CAMERA_INDEX))
        RUN_CAPTURE = not RUN_CAPTURE

stream.release()
cv2.destroyAllWindows()