import socket

def Send(Message): #send string through udp
	UDP_IP = "127.0.0.1" #Replace with feather AP IP
	UDP_PORT = 5005
	#MESSAGE = "(10,0,1)" #(vel_d,theta_d,mode) #mode: 0=reset,1=normal

	print "UDP target IP:", UDP_IP
	print "UDP target port:", UDP_PORT
	print "message:", MESSAGE

	sock = socket.socket(socket.AF_INET, # Internet
						 socket.SOCK_DGRAM) # UDP
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

def Connect2Port(): #Connect to feather AP

	return

def parseInput(input): # get vel, theta, and mode from input
	
	
def main():

	Connect2Port()
    while 1:
	
		# get keyboard input, waits until enter pressed
		input = raw_input(">> ")
		
		v,theta,mode = parseInput(input)
		
		X = "(%f,%f,%d)" % (v, theta, mode)"
		
		Send(X)


if __name__ == '__main__':
    main()