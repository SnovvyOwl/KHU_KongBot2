import Tcpnetworks

client = Tcpnetworks.Tcpnetworks()
client.Client('192.168.0.100', 8080)

while True:
    client.SendStr('i am client\n')
    from_server = client.ReceiveStr()
    print (from_server)

