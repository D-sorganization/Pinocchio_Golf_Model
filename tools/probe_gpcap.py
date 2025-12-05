"""Probe .gpcap file structure."""
import struct
import sys

def probe(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()
    
    print(f"File size: {len(data)} bytes")
    
    # Identify strings (sequence of printable chars > 3)
    current_string = []
    
    for i, byte in enumerate(data):
        c = chr(byte)
        if c.isprintable():
            current_string.append(c)
        else:
            if len(current_string) > 3:
                s = "".join(current_string)
                print(f"Offset {i - len(s)}: {s}")
                
                # Check bytes immediately preceding/following for length indicators
                if i - len(s) >= 4:
                    pre_bytes = data[i - len(s) - 4 : i - len(s)]
                    val_int = struct.unpack('<I', pre_bytes)[0]
                    print(f"  Preceding 4 bytes (int): {val_int}")
                    
            current_string = []

if __name__ == '__main__':
    if len(sys.argv) > 1:
        probe(sys.argv[1])
    else:
        print("Usage: python probe_gpcap.py <filepath>")
