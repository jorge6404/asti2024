from scapy.all import *

def packet_handler(packet):
    if packet.haslayer(TCP) and packet[TCP].dport == 8888:
        print(packet.summary())
        packet.show()

# Replace "wlo1" with your interface name
interface = "wlo1"

# Start sniffing packets
sniff(iface=interface, filter="tcp port 8888", prn=packet_handler, store=0)

