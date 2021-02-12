import socket
class Tcpnetworks:
    def __init__(self):
        self.com_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection = self.com_socket
    def Serve(self, IP, Port):
        self.com_socket.bind((IP, Port))
        self.com_socket.listen(5)
        self.connection, self.address =  self.com_socket.accept()
    def Client(self, IP, Port):
        self.com_socket.connect((IP,Port))
    def SendStr(self, str1):
        self.connection.send(str1.encode())
    def Send_th(self,send_data):
        while True:
            send_data = input('Command: ')
            self.connection.send(send_data.encode())
    def Send_th_data(self,send_data):  # need to fix
        while True:
            send_data = 'i am client\n'
            self.connection.send(send_data.encode())
    def ReceiveStr(self):
        return self.connection.recv(4096).decode()
    def ReceiveStr2(self):
        try:
            return self.connection.recv(4096).decode()
        except:
            return None
    def Receive_th(self,from_client):
        while True:
            from_client = self.connection.recv(4096).decode()
            print (from_client)
    def Receive2_th(self,from_client):
        while True: 
            try:
                from_client = self.connection.recv(4096).decode()
                print (from_client)
            except:
                from_client = None
                print (from_client)
    def Blocking(self, Tme):
        self.connection.setblocking(Tme)