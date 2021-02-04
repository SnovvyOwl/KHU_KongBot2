import Tcpnetworks

server = Tcpnetworks.Tcpnetworks()
server.Serve('0.0.0.0', 8080)

while True:
    data = server.ReceiveStr()
    from_client = data
    print (from_client)
    server.SendStr("I am server\n")
