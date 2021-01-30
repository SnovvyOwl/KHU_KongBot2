import socket

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('192.168.137.1',8080))


while True:
    client.send('I am CLIENT\n'.encode())
    from_server = client.recv(4096)
    print (from_server)





