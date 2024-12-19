# from pathlib import Path
# import argparse
# import cv2
# import matplotlib.cm as cm
# import torch
import numpy as np
# import math
import time
import socket
import threading
# from datetime import datetime, timedelta
# from collections import namedtuple, deque
# from enum import Enum
from scipy.spatial.transform import Rotation as R
import pyigtl

# HOST = '192.168.122.157'
HOST = '192.168.0.136'
# HOST = '127.0.0.1'

# HundredsOfNsToMilliseconds = 1e-4
# MillisecondsToSeconds = 1e-3

# torch.set_grad_enabled(False)

class pnp():
    def __init__(self):

        self.cmd = "get"
        self.key = "get"
        self.calibPos = [0, 0, 0, 0, 1, 0, 0]
        # self.calibPos = [0, 0, 0, -0.707, 0, 0, 0.707]
        self.camPos = [0, 0, 0, 0, 0, 0, 1.0]
        self.corPos = [0, 0, 0, 0, 0, 0, 1.0]
        self.new_cmd = False
        self.calibed = False
        t1 = threading.Thread(target=self.calib_comm)
        t2 = threading.Thread(target=self.keyboard_read)
        t1.start()
        t2.start()

    def comp_obj(self):
        client = pyigtl.OpenIGTLinkClient("192.168.0.136", 18945)
        # rec_rot = [0, -1, 0, 0]
        rec_rot =[1,0,0,0]
        # rec_rot =[0,0,1,0]
        rr = R.from_quat(rec_rot)
        rr = np.float32(rr.as_matrix())
        while True:
            link = client.is_connected()
            if link:
                message = client.wait_for_message("VirtualToRas", timeout=5)
                if message != None:
                    VirtualToRas = message.matrix
                    trans = list(VirtualToRas[:3, 3]/1000)
                    rot_mat = VirtualToRas[:3, :3]
                    rot_mat = np.matmul(rot_mat, rr)
                    rot = R.from_matrix(rot_mat)
                    rot = list(R.as_quat(rot))
                    break
        print(trans, rot)
        return trans, rot
    
    def calib_comm(self):
        self.calib_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.calib_sock.connect((HOST, 25003))
        # self.calib_sock.connect(("127.0.0.1", 25003))
        while True:
            time.sleep(0.5)
            if self.new_cmd:
                self.cmd = self.key
                self.new_cmd = False
            else:
                self.cmd = "get"
            if self.cmd == "q":
                break
                # print("calibed:", self.cmd)
            # time.sleep(0.5) #sleep 0.5 sec
            if self.cmd == "c":
                trans, rot = self.comp_obj()
                # trans[1] += 1.6
                trans[2] = -trans[2]
                rot[0] = -rot[0]
                rot[1] = -rot[1]
                self.cmd = "e" + str(tuple(trans + rot))
                print(self.cmd)
            # print(self.cmd)
            self.calib_sock.sendall(self.cmd.encode("UTF-8")) #Converting string to Byte, and sending it to C#
            receivedPos = self.calib_sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String

    def keyboard_read(self):
        while True:
            self.key = str(input("ctrl:"))
            self.new_cmd = True
            if self.key == "q":
                break
            elif self.key == "c":
                self.err = 10000


if __name__ == "__main__":
    p = pnp()
