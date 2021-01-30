import Tcpnetworks

client = Tcpnetworks.Tcpnetworks()
client.Client('192.168.0.100', 8080)

while True:
    send_data = input('message :')
    client.SendStr(send_data)
    from_server = client.ReceiveStr()
    print (from_server)

