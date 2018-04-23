import sys
import cozmo
import datetime
import time
import numpy as np
import re
import asyncio
from sklearn import svm, metrics
from sklearn.externals import joblib
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
    def trainAndSave() :
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

        joblib.dump(img_clf.classifier, 'classifier.pk1')


    def run(sdk_conn):
        robot = sdk_conn.wait_for_robot()
        robot.camera.image_stream_enabled = True
        robot.camera.color_image_enabled = False
        robot.camera.enable_auto_exposure()
        robot.say_text("I'm HERE").wait_for_completed()
