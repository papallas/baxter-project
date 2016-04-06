# server.py 
import socket                                         
import time

# create a socket object
serversocket = socket.socket(
	        socket.AF_INET, socket.SOCK_STREAM) 

# get local machine name
host = socket.gethostname()

port = 8000                                           

# bind to the port
serversocket.bind(('192.168.0.7', port))                                  

# queue up to 5 requests
serversocket.listen(1)
print "Server now listening"

#while True:
# establish a connection
conn, addr = serversocket.accept()      

print("Got a connection from %s" % str(addr))
time.sleep(4)
string = "Get choice\n"
conn.send(string.encode('ascii'))
print "Sent to client to start getting customer order"
conn.close()

conn, addr = serversocket.accept()

data = conn.recv(1024)
print data
print "\n"

conn.close()

serversocket.close()
