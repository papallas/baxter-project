#!/usr/bin/env python
import socket
import time
from manipulation.srv import *
import rospy

global mainCount
mainCount = 1
global connection
connection = False
global sendData
sendData = False
global request
request = []
global conn

def handle_command(req):
    print "handling command"
    global sendData
    sendData = True
    get_command()
    while sendData == True:
        rospy.sleep(1)
    global request
    print "returned: ",request
    return RequestCommandResponse(request)

def wait_for_connect():
    global mainCount, connection, sendData, request, conn
    if connection == False:
        conn, addr = serversocket.accept()
        print("Got a connection from %s" % str(addr))
        connection = True

def get_command():
    global connection, sendData, conn, request
    if connection == True and sendData == True:
        string = "Get choice\n"
        conn.send(string.encode('ascii'))
        print "Sent to client to start getting customer order"
        conn.close()

        conn, addr = serversocket.accept()

        data = conn.recv(1024)
        print data
        print "\n"
        words = data.split()
        blueNum = int(words[0])
        redNum = int(words[3])
        greenNum = int(words[6])
        request = [blueNum,redNum,greenNum]

        conn.close()
        connection = False
        sendData = False

        wait_for_connect()


if __name__ == '__main__':
    rospy.init_node('app_server', anonymous = True)

    global mainCount, connection, sendData, request, conn

    if mainCount == 1:
        # create a socket object
        serversocket = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # get local machine name
        host = socket.gethostname()

        port = 8000

        # bind to the port
        serversocket.bind(('10.41.79.177', port))

        # queue up to 1 requests
        serversocket.listen(1)
        print "Server now listening"
        mainCount = mainCount + 1

    if connection == False:
        wait_for_connect()


    print "Connection: ",connection,", sendData: ",sendData

    t = rospy.Service('get_command', RequestCommand, handle_command)

    #Keep from exiting until this node is stopped
    rospy.spin()
