from socket import socket, AF_INET, SOCK_DGRAM

HOST = '133.68.108.96'   
PORT = 9000

# ソケットを用意
s = socket(AF_INET, SOCK_DGRAM)
# バインドしておく
s.bind((HOST, PORT))

while True:
    # 受信
    msg, address = s.recvfrom(8192)
    print(f"message: {msg}\nfrom: {address}")

# ソケットを閉じておく
s.close()
