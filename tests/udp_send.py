from socket import socket, AF_INET, SOCK_DGRAM

HOST = ''
PORT = 9000
ADDRESS = "133.68.108.109" 

s = socket(AF_INET, SOCK_DGRAM)
# ブロードキャストする場合は、ADDRESSを
# ブロードキャスト用に設定して、以下のコメントを外す
# s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

while True:
    msg = input("> ")
    # 送信
    s.sendto(msg.encode(), (ADDRESS, PORT))

s.close()