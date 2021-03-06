#!/usr/bin/env python

import zmq
import sys
import signal
import cPickle as pickle
from PIL import Image

class netCat:
    def __init__(self, ip_address = '127.0.0.1', port = '6127'):

        self._ip = ip_address
        self._port = port

        self.zmqc = zmq.Context()

class netCatSender(netCat):
    def __init__(self, img_fp, ip_address='127.0.0.1', port='6127'):
        netCat.__init__(self, ip_address, port)

        self._img_file = img_fp
        self.pub = self.zmqc.socket(zmq.PUB)
        print("binding to tcp://%s:%s" % (self._ip, self._port))
        self.pub.bind('tcp://%s:%s' % (self._ip, self._port))

    def send(self):
        img = Image.open(self._img_file)
        img_pickled = pickle.dumps(img)

        print("sending {fp}".format(fp=self._img_file))
        self.pub.send(img_pickled)
        print("done sending {fp}".format(fp=self._img_file))

class netCatReceiver(netCat):
    def __init__(self, ip_address='127.0.0.1', port='6127'):
        netCat.__init__(self, ip_address, port)
        self.shown = False

        self.sub = self.zmqc.socket(zmq.SUB)
        print("connecting to tcp://%s:%s" % (self._ip, self._port))
        try:
            self.sub.setsockopt(zmq.SUBSCRIBE, '')
            self.sub.connect('tcp://%s:%s' % (self._ip, self._port))
            print("connected!")
        except:
            print("failed to connect")

    def recv(self):
        img_pickled = self.sub.recv()
        img = pickle.loads(img_pickled)
        if not self.shown:
            img.show()
            self.shown = True
        print ("cat received")

class netCatLauncher:
    def __init__(self):
        if len(sys.argv) == 1:
            self.sender = None
            self.receiver = netCatReceiver()
        # usage: netCat.py (image_file)
        elif len(sys.argv) == 2:
            self.sender = netCatSender(img_fp=sys.argv[1])
            self.receiver = None
        # usage: netCat.py (ip, port)
        elif len(sys.argv) == 3:
            self.sender = None
            self.receiver = netCatReceiver(\
                ip_address=sys.argv[1],    \
                port=sys.argv[2]           \
            )
        # usage: netCat.py (image_file, ip, port)
        elif len(sys.argv) == 4:
            self.sender = netCatSender(\
                img_fp=sys.argv[1],    \
                ip_address=sys.argv[2],\
                port=sys.argv[3]       \
            )
            self.receiver = None
        else:
            print("receive: ./netCat.py \n transmit: ./netCat.py cat_image")

    def run(self):
        while True:
            try:
                if self.sender is not None:
                    self.sender.send()
                if self.receiver is not None:
                    self.receiver.recv()
            except KeyboardInterrupt:
                nc.sit()
                break
            except:
                if self.sender is not None:
                    print "failed to send"
                if self.receive is not None:
                    print "failed to receive"

    def sit(self):
        print("good kitty!")

if __name__ == "__main__":
    nc = netCatLauncher()
    nc.run()
