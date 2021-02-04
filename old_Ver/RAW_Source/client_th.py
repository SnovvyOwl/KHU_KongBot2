import Tcpnetworks
import threading
import time
#from multiprocessing import Process

client = Tcpnetworks.Tcpnetworks()
client.Client('192.168.0.142', 8080)

sender = threading.Thread(target=client.Send_th_data, args=(client.connection,))
#sender = Process(target=client.Send_th_data, args=(client.connection,))

sender.start()

while True:
    from_server = client.ReceiveStr()
    print (from_server)

