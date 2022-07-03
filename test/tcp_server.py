# -*- coding: utf-8 -*-
import socket
from builtins import input
import threading
import time

class TcpServer:
    def __init__(self, port=3333):
        self.port = port
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.socket.settimeout(60.0)
        self.event = threading.Event()
        self.clients  = {}

    def __enter__(self):
        try:
            self.socket.bind(("0.0.0.0", self.port))
        except socket.error as e:
            print('Bind failed:{}'.format(e))
            raise

        self.socket.listen()
        print('Starting server on port={} ip={}'.format(self.port,"0.0.0.0"))
        # 启动线程接受连接
        self.server_thread = threading.Thread(target=self._accept,name='accept',daemon=True)
        self.server_thread.start()
        return self

    # 接收连接
    def _accept(self):
        while not self.event.is_set(): #多人连接
            conn,client = self.socket.accept()  #阻塞
            print("受到连接",conn,client)
            # f = conn.makefile(mode='rw')
            self.clients[client] = client
            # 受到连接开个线程来接收数据
            threading.Thread(target=self._recv, args=(conn, client), name='recv',daemon=True).start()

    def _recv(self,conn, client):
        frames = b''
        count = 0
        last_time = time.time()

        while not self.event.is_set():
            frames += conn.recv(128)
            first = frames.find(b'\x7D')
            last = frames.find(b'\x7E')
            if first != -1 and last != -1:
                if last < first:
                    frames = frames[last + 1:]
                else:
                    frame = frames[first:last + 1]
                    conn.send(frame)
                    count += 1
                    if int(time.time() - last_time) >= 1:
                        print("平均帧率 %s" % str(int(count / (time.time() - last_time))))
                        last_time = time.time()
                        count = 0

    def __exit__(self, exc_type, exc_value, traceback):
        self.server_thread.join()
        self.socket.close()


# 一个send,一个recv

if __name__ == '__main__':
    with TcpServer(3333) as s:
        print(input('Press Enter stop the server...'))
