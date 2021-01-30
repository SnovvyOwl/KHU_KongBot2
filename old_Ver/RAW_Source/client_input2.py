import Tcpnetworks

client = Tcpnetworks.Tcpnetworks()
client.Client('192.168.0.100', 8080)
client.Blocking(0)

while True:
    send_data = input('message : ')
    client.SendStr(send_data)
    from_server = client.ReceiveStr2()
    print (from_server)