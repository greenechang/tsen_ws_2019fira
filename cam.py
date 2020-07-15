#!/usr/bin/env python

import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    cv2.imshow('asdf',frame)
    cv2.waitKey(1)
