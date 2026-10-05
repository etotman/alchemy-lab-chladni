import os, sys, time, select, termios, zlib, struct

DEV = sys.argv[2] if len(sys.argv) > 2 else "/dev/cu.usbmodem307F366A34331"
PROTO = 0x01

def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0]); code_idx = 0; code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code; code_idx = len(out); out.append(0); code = 1
        else:
            out.append(b); code += 1
            if code == 255:
                out[code_idx] = code; code_idx = len(out); out.append(0); code = 1
    out[code_idx] = code
    return bytes(out)

def cobs_decode(data: bytes):
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]
        if code == 0: return None
        i += 1
        for _ in range(code - 1):
            if i >= len(data): return None
            out.append(data[i]); i += 1
        if code < 255 and i < len(data): out.append(0)
    return bytes(out)

def crc32(b): return zlib.crc32(b) & 0xFFFFFFFF

def build(type_, seq, body=b""):
    dec = bytearray(struct.pack("<BBHH", PROTO, type_, seq, len(body))) + body
    dec += struct.pack("<I", crc32(bytes(dec)))
    return cobs_encode(bytes(dec)) + b"\x00"

def open_port(dev):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    i_, o_, c_, l_, isp, osp, cc = a
    i_ &= ~(termios.IGNBRK|termios.BRKINT|termios.PARMRK|termios.ISTRIP|
            termios.INLCR|termios.IGNCR|termios.ICRNL|termios.IXON)
    o_ &= ~termios.OPOST
    l_ &= ~(termios.ECHO|termios.ECHONL|termios.ICANON|termios.ISIG|termios.IEXTEN)
    c_ &= ~(termios.CSIZE|termios.PARENB|termios.CSTOPB)
    c_ |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc = list(cc); cc[termios.VMIN] = 0; cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [i_, o_, c_, l_, termios.B115200, termios.B115200, cc])
    return fd

def read_frames(fd, timeout=1.5):
    acc = bytearray(); frames = []; end = time.time() + timeout
    while time.time() < end:
        if select.select([fd], [], [], 0.1)[0]:
            d = os.read(fd, 4096)
            if not d: continue
            for b in d:
                if b != 0: acc.append(b); continue
                if acc:
                    dec = cobs_decode(bytes(acc))
                    if dec and len(dec) >= 10:
                        blen = len(dec) - 10
                        ok = (dec[0] == PROTO
                              and struct.unpack("<H", dec[4:6])[0] == blen
                              and crc32(dec[:-4]) == struct.unpack("<I", dec[-4:])[0])
                        frames.append((dec[1], struct.unpack("<H", dec[2:4])[0], dec[6:6+blen], ok))
                acc = bytearray()
            if frames: return frames
    return frames

fd = open_port(DEV)
mode = sys.argv[1] if len(sys.argv) > 1 else "hello"

if mode == "hello":
    os.write(fd, build(0x01, 1))
    fr = read_frames(fd)
    if not fr:
        print("NO RESPONSE"); sys.exit(1)
    for t, seq, body, ok in fr:
        print(f"resp type=0x{t:02x} seq={seq} crc_ok={ok} len={len(body)}")
        print("  body:", body.hex())
        if t == 0x81 and ok and body:
            print("  status:", body[0])
elif mode == "reboot":
    os.write(fd, build(0x40, 2, bytes([1])))
    fr = read_frames(fd, 2.0)
    for t, seq, body, ok in fr:
        print(f"resp type=0x{t:02x} seq={seq} crc_ok={ok} status={body[0] if body else '?'}")
    if not fr: print("no ack (device may still have rebooted)")
os.close(fd)
