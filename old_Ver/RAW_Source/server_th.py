import Tcpnetworks
import threading
import time
#from multiprocessing import Process

server = Tcpnetworks.Tcpnetworks()
server.Serve('0.0.0.0', 8080)
#server.Blocking(0)

sender = threading.Thread(target=server.Send_th, args=(server.connection,))
receiver = threading.Thread(target=server.Receive_th, args=(server.connection,))
#receiver = threading.Thread(target=server.Receive2_th, args=(server.connection,))
#sender = Process(target=server.Send_th, args=(server.connection,))
#receiver = Process(target=server.Receive_th, args=(server.connection,))

sender.start()
receiver.start()

while True:
    time.sleep(1)
    pass