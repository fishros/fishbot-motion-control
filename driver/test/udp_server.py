# -*- coding: utf-8 -*-
import socket
from builtins import input
from queue import Queue
import threading
import time
import struct

class UdpServer:
    def __init__(self,queue=Queue(),port=3333):
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.event = threading.Event()
        
        self.frame_queue = queue
        self.addr = None

        self.__enter__()

    def __enter__(self):
        try:
            self.socket.bind(('', self.port))
        except socket.error as e:
            print('Bind failed:{}'.format(e))
            raise
        # self.socket.listen()
        print('Starting server on port={} ip={}'.format(self.port,"0.0.0.0"))
        # 启动线程接受连接
        self.server_thread = threading.Thread(target=self.run_server,name='run_server')
        self.server_thread.start()
        return self

    def send_frame(self,frame):
        self.socket.sendto(frame,self.addr)

    def run_server(self):
        print("server start....")
        frames = b''
        count = 0
        last_time = time.time()
        start =  time.time()
        while not self.event.is_set():
            data, self.addr = self.socket.recvfrom(1024)
            if not data: continue
            frames += data
            first = frames.find(b'\x7D')
            last = frames.find(b'\x7E')
            if first != -1 and last != -1:
                if last < first:
                    frames = frames[last + 1:]
                else:
                    frame = frames[first:last + 1]
                    frames = frames[last + 1:]
                    self.frame_queue.put(frame)
                    count += 1
                    
                    # self.socket.sendto(frame,addr)
                    if int(time.time() - last_time) >= 10:
                        print("平均帧率 %s" % str(int(count / (time.time() - last_time))))
                        print(frame, " size:",len(frames))
                        print("运行：",time.time()-start)
                        last_time = time.time()
                        count = 0

    def __exit__(self, exc_type, exc_value, traceback):
        # self.server_thread.join()
        self.socket.close()



def uchar_checksum(data, byteorder='little'):
    '''
    char_checksum 按字节计算校验和。每个字节被翻译为无符号整数
    @param data: 字节串
    @param byteorder: 大/小端
    '''
    length = len(data)
    checksum = 0
    for i in range(0, length):
        checksum += int.from_bytes(data[i:i+1], byteorder, signed=False)
        checksum &= 0xFF # 强制截断
        
    return checksum


if __name__ == '__main__':
    queue = Queue()
    server = UdpServer(queue=queue,port=3333)
    while True:
        if queue.qsize()>0:
            frame = queue.get()
            print(frame)
            
            spped_left = frame[4:6]
            print("spped_left check sum:", spped_left)
            spped_left = struct.unpack("h",spped_left)[0]/1000.0
            
            print("unpanck:",spped_left)
