#!/usr/bin/env python

import sys
import cozmo
import datetime
import time
import numpy as np
import re
import asyncio
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
from cozmo.util import degrees, distance_mm, speed_mmps


class ImageClassifier:

    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir + "*.bmp", load_func=self.imread_convert)

        # create one large array of image data
        data = io.concatenate_images(ic)

        # extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return (data, labels)

    def extract_image_features(self, data):
        l = []
        for im in data:
            im_gray = color.rgb2gray(im)

            im_gray = filters.gaussian(im_gray, sigma=0.4)

            f = feature.hog(im_gray, orientations=10, pixels_per_cell=(48, 48), cells_per_block=(4, 4),
                            feature_vector=True, block_norm='L2-Hys')
            l.append(f)

        feature_data = np.array(l)
        return (feature_data)

    def train_classifier(self, train_data, train_labels):
        self.classifer = svm.LinearSVC()
        self.classifer.fit(train_data, train_labels)

    def predict_labels(self, data):
        predicted_labels = self.classifer.predict(data)
        return predicted_labels


def stateMachine(state):
    switcher = {
        "IDLE": Idle,
        "DRONE": drone,
        "ORDER": order,
        "INSPECTION": inspection
    }


def Idle(clf, robot):
    robot.say_text("Show me a Picture!").wait_for_completed()
    time.sleep(2)
    latest_image = robot.world.latest_image
    new_image = latest_image.raw_image
    new_image.save("./temp/temp_.bmp")
    # (test_raw, temp_) = img_clf.load_data_from_folder('./temp/')
    test_raw = []
    test_raw.append(io.imread('./temp/temp_.bmp'))
    test_data = clf.extract_image_features(test_raw)
    label = clf.predict_labels(test_data)
    label = str(label[0]).upper()
    print(label)
    if label == "NONE":
        return "IDLE"
    else:
        robot.say_text(str(label)).wait_for_completed()
    if label == "DRONE":
        return "DRONE"
    elif label == "ORDER":
        return "ORDER"
    elif label == "INSPECTION":
        return "INSPECTION"
    else:
        return "IDLE"

    robot.say_text(str(label)).wait_for_completed()


def drone(robot):
    # Move lift down and tilt the head up
    robot.move_lift(0)
    robot.set_head_angle(degrees(0)).wait_for_completed()

    # look around and try to find a cube
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cube = None

    try:
        cube = robot.world.wait_for_observed_light_cube(timeout=30)
        print("Found cube: %s" % cube)
    except asyncio.TimeoutError:
        print("Didn't find a cube")
    finally:
        # whether we find it or not, we want to stop the behavior
        look_around.stop()

    if cube:
        action = robot.pickup_object(cube, num_retries=3)
        action.wait_for_completed()
        action = robot.drive_straight(distance_mm(150), speed_mmps(50))
        action.wait_for_completed()
        action = robot.place_object_on_ground_here(cube)
        action.wait_for_completed()
        action = robot.drive_wheels(-100, -100, duration=1.5)

    return "IDLE"


def order(robot):
    robot.drive_wheels(105, 241, duration=4.5)
    return "IDLE"


def inspection(robot):
    lift_speed = 0.1
    for _ in range(4):
        action = robot.drive_straight(distance_mm(200), speed_mmps(50))
        #action.wait_for_completed()
        while not action.has_succeeded :
            if robot.lift_height.distance_mm <= cozmo.robot.MIN_LIFT_HEIGHT.distance_mm + 0.5 :
                lift_speed = 0.1
            elif robot.lift_height.distance_mm >= cozmo.robot.MAX_LIFT_HEIGHT.distance_mm - 0.5 :
                lift_speed = -0.1
            else :
                lift_speed = lift_speed
            robot.move_lift(lift_speed)
        action = robot.turn_in_place(degrees(90))
        while not action.has_succeeded :
            if robot.lift_height.distance_mm <= cozmo.robot.MIN_LIFT_HEIGHT.distance_mm + 0.5 :
                lift_speed = 0.1
            elif robot.lift_height.distance_mm >= cozmo.robot.MAX_LIFT_HEIGHT.distance_mm - 0.5 :
                lift_speed = -0.1
            else :
                lift_speed = lift_speed
            robot.move_lift(lift_speed)
    return "IDLE"


def run(sdk_conn):
    img_clf = ImageClassifier()
    (train_raw, train_labels) = img_clf.load_data_from_folder('./photos/')
    # (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    # test_data = img_clf.extract_image_features(test_raw)

    # train model
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n", metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

    robot = sdk_conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.say_text("I'm HERE").wait_for_completed()

    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    state = "IDLE"

    while True:
        if state == "DRONE":
            state = drone(robot)
        elif state == "ORDER":
            state = order(robot)
        elif state == "INSPECTION":
            state = inspection(robot)
        else:
            state = Idle(img_clf, robot)

    # time.sleep(.5)
    # latest_image = robot.world.latest_image
    # new_image = latest_image.raw_image
    # new_image.save("./temp/temp_.bmp")
    # #(test_raw, temp_) = img_clf.load_data_from_folder('./temp/')
    # test_raw = []
    # test_raw.append(io.imread('./temp/temp_.bmp'))
    # test_data = img_clf.extract_image_features(test_raw)
    # label = img_clf.predict_labels(test_data)
    # label = str(label).upper()
    # robot.say_text(str(label)).wait_for_completed()

    # myargs = sys.argv[1:]
    #
    # if len(myargs) <= 1:
    #     sys.exit("Incorrect arguments")
    #
    # num_images_per_type = int(myargs[0])  # number of images to take of each type of object
    #
    # print("Taking ", num_images_per_type, "images each of ", myargs[1:])
    #
    # for type in myargs[1:]:
    #     for i in range(num_images_per_type):
    #         time.sleep(.5)
    #         latest_image = robot.world.latest_image
    #         new_image = latest_image.raw_image
    #
    #         robot.say_text(type + str(i)).wait_for_completed()
    #
    #         timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")
    #
    #         new_image.save("./imgs/" + str(type) + "_" + timestamp + ".bmp")

    time.sleep(4)


if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
