import serial
import struct
ser = serial.Serial('/dev/pts/1', 38400, timeout=1)
total = 0
while True:
    try:
        val = struct.unpack('>B',ser.read(1));
	total = (total + val[0]) & (2**8-1)
	#crc = struct.unpack('>B',total)[0] & 0x7F
        print "val: " + str(val[0]) + " total: " + str(total) + " checksum: " + str(total & 0x7F)
    except struct.error:
        total = 0
	print "-"
    except KeyboardInterrupt:
        print "closing port"
        ser.close()
        break
