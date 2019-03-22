#!/usr/bin/env python

import cv2
from cv2 import aruco
import pickle
from Transforms import Transformations


class Detector:
    parameters = aruco.DetectorParameters_create()
    length_of_axis = 0.1
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    def __init__(self, size, calibration, marker_list):
        with open(calibration, 'rb') as pkl:
            ret, mtx, dist = pickle.load(pkl)
            self.cal = (mtx, dist)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.size_of_marker = size  # 0.144  # side lenght of the marker in meter
        self.markers_list = marker_list

    def draw_text(self, image, corner, *args):
        i = 0
        max = sorted([len(str(arg)) for arg in args], reverse=True)[0]
        cv2.rectangle(image,
                      (int(corner[0]), int(corner[1])),
                      (int(corner[0]) + max*6+20, int(corner[1]) + len(args)*12+30),
                      (0, 0, 0), -1)
        for arg in args:
            cv2.putText(image, arg, (int(corner[0])+10, int(corner[1])+i+20),
                        cv2.FONT_HERSHEY_SIMPLEX, .35, (255, 255, 255), 1)
            i += 13

    def find_ar(self, frame):
        tvecs = {}

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        imaxis = cv2.cvtColor(gray.copy(), cv2.COLOR_GRAY2BGR)
        if not corners:
            self.draw_text(imaxis, (5, 5), "No Markers Detected!", "Searching for IDs: 0, 2, 9")
            return imaxis, None

        for corner, id in zip(corners, ids):
            if id[0] in self.markers_list:
                cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=self.criteria)
                (rvec, ((tvecs[id[0]],),), _) = aruco.estimatePoseSingleMarkers(corner,
                                                                                self.size_of_marker,
                                                                                *self.cal)
                imaxis = aruco.drawDetectedMarkers(imaxis, [corner, ], id)
                imaxis = aruco.drawAxis(imaxis, self.cal[0], self.cal[1], rvec, tvecs[id[0]], self.length_of_axis)
                self.draw_text(imaxis, corner[0][0],
                               "ID: {}".format(id[0]),
                               "X: {:5.2}".format(tvecs[id[0]][0]),
                               "Y: {:5.2}".format(tvecs[id[0]][1]),
                               "Z: {:5.2}".format(tvecs[id[0]][2])
                               )

        txt = []
        for mrk in sorted(tvecs.keys()):
            txt += ["ID Found: {}".format(mrk),
                    "    X: {:5.2}".format(tvecs[mrk][0]),
                    "    Y: {:5.2}".format(tvecs[mrk][1]),
                    "    Z: {:5.2}".format(tvecs[mrk][2]),
                    ""]
        if txt:
            self.draw_text(imaxis, (5, 5), *txt)

        return imaxis, tvecs


if __name__ == "__main__":
    detector = Detector(.075,
                        "/home/ardoo/catkin_ws/src/my_pcl_tutorial/src/Calibration_Parameters.pickle",
                        [0, 8, 9])
    tr = Transformations([2500, 1000, 600], [2500, -1000, 600], [2900, 1000, 650])
    cap = cv2.VideoCapture(0)

    while True:
        ret, img = cap.read()
        imaxis, translations = detector.find_ar(img)
        if translations:
            if len(translations) == 3:
                # print(translations)
                print(tr.calculate_transformation(*translations.values()))

        cv2.imshow("KIR", imaxis)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
