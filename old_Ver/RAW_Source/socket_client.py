import socket

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('192.168.137.1',8080))

client.send('I am CLIENT\n'.encode())

from_server = client.recv(4096)
#from_server = client.recv(4096).decode()
#from_server = client.recv(1024)

#strings = from_server('utf8')
#num = int(strings)

client.close()

print (from_server)
#print (num)
