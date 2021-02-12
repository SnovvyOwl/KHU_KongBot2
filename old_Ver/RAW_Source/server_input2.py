import Tcpnetworks

server = Tcpnetworks.Tcpnetworks()
server.Serve('0.0.0.0', 8080)
server.Blocking(0)

while True:
    data = server.ReceiveStr2()
    from_client = data
    print (from_client)
    send_data = input('message : ')
    server.SendStr(send_data)
