#!/usr/bin/env python
import socket
import time
from manipulation.srv import *
import rospy
import netifaces

class Server:
    def __init__(self):
        # Initialise variables that keep track of the initialisation/connections
        self.mainCount = 1
        self.connection = False
        self.sendData = False
        self.request = []
        self.conn = False

        # Set up the server socket initially
        self.server_setup()
        # Wait for the initial client connection
        self.wait_for_connection()

    def server_setup(self):
        # Create an initial socket to a generically available IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8",80))
        hostip = s.getsockname()[0]
        # Get a list of interfaces and IP addresses to retrieve the main
        # IP address that the client needs to enter initially - print to screen
        interfaces = netifaces.interfaces()
        for i in interfaces:
            if i == 'lo':
                continue
            iface = netifaces.ifaddresses(i).get(netifaces.AF_INET)
            if iface != None:
                for j in iface:
                    print j['addr']
                    mainaddr = j['addr']
        s.close()

        # Using a generic port number
        port = 8000
        # Create a socket object and add setup options for repeated connections
        self.serversocket = socket.socket(
                   socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the port
        self.serversocket.bind(('0.0.0.0', port))

        # Listen for up to 1 device to connect at once
        self.serversocket.listen(1)
        print "Server now listening"


    def wait_for_connection(self):
        # If there are no connections to the server, wait for a client
        # to connect to the server
        if self.connection == False:
            self.conn, addr = self.serversocket.accept()
            self.connection = True
            print "connection made with client"

    def get_sweet_command(self):
        if self.connection == True and self.sendData == True:
            # Send a string message to the client asking for a command
            string = "Get choice\n"
            self.conn.send(string.encode('ascii'))
            # Close connection after sending
            self.conn.close()

            # After that accept a new connection which will receive a command
            self.conn, addr = self.serversocket.accept()
            # Receive a string command of a maximum size from the client
            data = self.conn.recv(1024)

            # Split the string command into three respective numbers and create
            # an array with the request data
            words = data.split()
            blueNum = int(words[0])
            redNum = int(words[3])
            greenNum = int(words[6])
            self.request = [blueNum,redNum,greenNum]

            # Close the connection after receiving the command
            self.conn.close()
            # Reset the connection variable and send data variable for reuse
            self.connection = False
            self.sendData = False

            # Wait for the next connection to the server
            self.wait_for_connection()

    def handle_order(self, req):
        print "handle order request from main node"

        # Send data to the client asking for an order
        self.sendData = True
        # Get the data and store it in the class
        self.get_sweet_command()
        # Wait for the data to be retrieved and processed
        while self.sendData == True:
            rospy.sleep(1)
        print self.request
        # Return the order as a response to the main shopkeeper's node
        return RequestCommandResponse(self.request)

# MAIN ROSPY LOOP
if __name__ == '__main__':
    # Create the server object
    server = Server()

    # Initialise the app server node
    rospy.init_node('app_server', anonymous = True)

    # Initialise a service to accept a request from the main shopkeeper node
    # connect this to the handle order function server side
    t = rospy.Service('get_command', RequestCommand, server.handle_order)
    print "initialised service"

    # Keep from exiting until this node is stopped
    rospy.spin()
