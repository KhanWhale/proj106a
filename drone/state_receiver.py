import socket

localIP     = ''

statePort   = 8890

bufferSize  = 1024
 

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Bind to address and ip
UDPServerSocket.bind((localIP, statePort))
UDPServerSocket.sendto('command'.encode('utf-8'), ('192.168.10.1', 8889))

destinationSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

print("UDP server up and listening")

 

# Listen for incoming datagrams

while(True):
    print('hi')
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    message = bytesAddressPair[0]

    address = bytesAddressPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    destinationSocket.sendto(message, ('', 9001))
    
    print(clientMsg)
    print(clientIP)

   

