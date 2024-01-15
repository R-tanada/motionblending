from socket import *


## UDP送信クラス
class udpsend:
    def __init__(self):
        SrcIP = "127.0.0.1"  # 送信元IP
        SrcPort = 11111  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "127.0.0.1"  # 宛先IP
        DstPort = 22222  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)  # 送信元アドレスでバインド

    def send(self):
        data = "Hello"
        data = data.encode("utf-8")  # バイナリに変換

        self.udpClntSock.sendto(data, self.DstAddr)  # 宛先アドレスに送信


udp = udpsend()  # クラス呼び出し
udp.send()  # 関数実行
