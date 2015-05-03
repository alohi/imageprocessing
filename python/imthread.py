
import os
import threading
import cv2


class CapThread(threading.Thread):

    maxRetries = 20

    def __init__(self, thread_id, name, video, thread_lock):
        threading.Thread.__init__(self)
        self.thread_if = thread_id
        self.video = video
        self.thread_lock = thread_lock


    def run(self):
        print 'starting thread'
        window_name = self.name
        cv2.namedWindow(window_name)
        video = cv2.VideoCapture(self.video)

        while True:
            ret, frame = video.read()
            if not ret:
                break
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(4000)
            if key == 27:
                break
            cv2.destroyWindow(window_name)
            print self.name + ' Exiting'
