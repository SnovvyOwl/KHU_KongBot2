import socket

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(('192.168.137.1', 8080))
serv.listen(5)

while True:
    conn, addr = serv.accept()
    from_client = ''

    while True:

        #num=123
        
        data = conn.recv(4096).decode()
        
        if not data: break
        from_client += data
        print (from_client)
        conn.send("I am SERVER\n".encode())
        
        #conn.send(str(num).encode('utf-8'))

    conn.close()
    print ('client disconnected')
