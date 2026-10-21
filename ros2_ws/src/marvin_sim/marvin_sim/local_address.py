# 枚举本机网卡并选择非回环 IPv4；显式传入的仿真 IP 也必须属于本机。
"""Select an assigned local IPv4: vendor SDK rejects every 127.* address."""
import fcntl
import ipaddress
import socket
import struct


def local_address(requested=''):
    addresses = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _, name in socket.if_nameindex():
            try:
                reply = fcntl.ioctl(sock.fileno(), 0x8915, struct.pack('256s', name.encode()[:15]))
                address = socket.inet_ntoa(reply[20:24])
                if not ipaddress.ip_address(address).is_loopback:
                    addresses.append(address)
            except OSError:
                continue
    if requested:
        if requested not in addresses:
            raise ValueError('sim_ip must be an IPv4 address assigned to this computer')
        return requested
    if not addresses:
        raise RuntimeError('SDK rejects loopback IPs; an assigned non-loopback IPv4 is required')
    return addresses[0]
