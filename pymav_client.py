#!/usr/bin/python

from pymavlink import mavutil

from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

import sys
import queue
import threading
import pmt

from asl_sdr_hackfest.zmq_sub import ZMQ_sub, ZMQ_sub_timeout

class PyMav_client(object):
    def __init__(self, zmq_context = None, remote_port = 6130):
        self.zmq_client = ZMQ_sub(remote_port)
        self.msg_queue = queue.Queue()

        self.is_running = True

        self.mc = mavutil.mavlink_connection("tcp:localhost:{}".format(remote_port), retries=9000)
        print "connected"
        self.mc.setup_logfile("/dev/null")
        self.logger = mavutil.mavlogfile("/dev/stdout", write=True)

        self.listener = threading.Timer(0, self.listen)
        self.listener.start()

    def listen(self):
        while self.is_running:
            try:
                # data = self.mc.recv_msg()
                msg = self.zmq_client.recv()
                # meta = pmt.to_python(pmt.car(msg))
                # self.msg_queue.put_nowait(data)
                # self.logger.write(data)
                # print (meta)
                # print len(data)
                # print len(msg)
            except queue.Full:
                print "Queue is full"
            except ZMQ_sub_timeout:
                print "Receive timed out"
            # except TypeError:
            #     print "Nope"
                # continue
            except:
                raise

if __name__ == "__main__":
    try:
        print ("using port %s" % sys.argv[1])
        pmc = PyMav_client(sys.argv[1]);
    except KeyboardInterrupt:
        pmc.is_running = False
    except:
        print ("using port 5763")
        # pmc = PyMav_client();
