import socket

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(('192.168.0.100', 8080))
serv.listen(5)
conn, addr = serv.accept()

while True:

    #num=123
        
    data = conn.recv(4096).decode()
        
    #send_data = input("message: ")
    #serv.send(send_data.encode())
    from_client = data
    print (from_client)
    conn.send("I am SERVER\n".encode())
        
    #conn.send(str(num).encode('utf-8'))

