#!/usr/bin/env python3
import struct
import zlib
import sys
import pandas as pd

# Binary format unpack string (68 bytes little-endian)
RECORD_FORMAT = "<I14fHBBf"  # Exactly 68 bytes: 1 uint32, 14 floats, 1 uint16, 2 uint8, 1 float
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)

def verify_and_unpack(filename):
    with open(filename, "rb") as f:
        data = f.read()

# 1. Unpack Header (14 bytes)
    magic, version, robot_id, maneuver_id, count, rec_size = struct.unpack("<4sHBB I H", data[:14])
    print(f"Header: Magic={magic.decode()}, Version={version}, RobotID={robot_id}, Records={count}, RecSize={rec_size}")
    assert magic == b"ANJM", "Invalid Header Magic!"
    assert rec_size == RECORD_SIZE, f"Record size mismatch! Expected {RECORD_SIZE}, got {rec_size}"

    # 2. Extract Payload & Footer
    payload = data[14:-8]
    footer_data = data[-8:]
    expected_crc, end_magic = struct.unpack("<I4s", footer_data)
    assert end_magic == b"MJNA", "Invalid Footer Magic!"

    computed_crc = zlib.crc32(payload)
    print(f"CRC Check: Expected=0x{expected_crc:08X}, Computed=0x{computed_crc:08X} -> {'PASS' if expected_crc == computed_crc else 'FAIL'}")
    assert expected_crc == computed_crc, "CRC32 Checksum Failed! File is corrupted."

    # 3. Unpack to Pandas DataFrame
    rows = [struct.unpack(RECORD_FORMAT, payload[i*rec_size:(i+1)*rec_size]) for i in range(count)]
    cols = ["t_ms", "x", "y", "heading", "v_cmd", "omega_cmd", "rpm_l", "rpm_r", "gyro_z",
            "vbat", "current_a", "uwb_raw", "uwb_clean", "uwb_rssi", "uwb_fp_power",
            "uwb_std_noise", "uwb_peer_id", "uwb_lde_err", "uwb_temp"]
    
    df = pd.DataFrame(rows, columns=cols)
    csv_name = filename.replace(".bin", ".csv")
    df.to_csv(csv_name, index=False)
    print(f"SUCCESS: Saved {len(df)} verified records to {csv_name}")
    print(df.head(5))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 verify_log.py <filename.bin>")
    else:
        verify_and_unpack(sys.argv[1])
