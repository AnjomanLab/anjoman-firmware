#!/usr/bin/env python3
"""
verify_log.py — Decode and verify Anjoman binary log files.

Supports schema version 1 (68 B) and version 2 (184 B, full mesh).
"""
import struct
import zlib
import sys
import pandas as pd


# ------------------------------------------------------------------------------
# Record formats (little-endian)
# ------------------------------------------------------------------------------
# v1: single UWB peer
RECORD_V1_FORMAT = "<I14fHBBf"   # 68 bytes
RECORD_V1_SIZE   = 68

# v2: full mesh, three UWB peer blocks
PEER_BLOCK_FORMAT = "BBHHHHfffff"   # 30 bytes
RECORD_V2_FORMAT = (
    "<I"               # t_ms
    "10f"              # x, y, heading, v_cmd, omega_cmd, rpm_l, rpm_r,
                       # gyro_z, imu_temp_c, imu_accel_x
    "4f"               # vbat, current_a, duty_l, duty_r
    "4f"               # target_rpm_l, target_rpm_r + 2 pad? no: 2f
    "2f"               # target_rpm_l, target_rpm_r
    + PEER_BLOCK_FORMAT * 3
    + "H"              # rxpacc
    + "4f"             # eskf_var_x/y/theta/bias
    + "I"              # tdma_frame_id
    + "4B"             # tdma_slot_index, tdma_sync_lost, maneuver_id, robot_state
)
RECORD_V2_SIZE = struct.calcsize(RECORD_V2_FORMAT)


# ------------------------------------------------------------------------------
# Column names
# ------------------------------------------------------------------------------
BASE_V1_COLS = [
    "t_ms", "x", "y", "heading", "v_cmd", "omega_cmd",
    "rpm_l", "rpm_r", "gyro_z", "vbat", "current_a",
    "uwb_raw", "uwb_clean", "uwb_rssi", "uwb_fp_power",
    "uwb_std_noise", "uwb_peer_id", "uwb_lde_err", "uwb_temp",
]


def v2_columns():
    cols = [
        "t_ms",
        "x", "y", "heading",
        "v_cmd", "omega_cmd",
        "rpm_l", "rpm_r",
        "gyro_z",
        "imu_temp_c", "imu_accel_x",
        "vbat", "current_a",
        "duty_l", "duty_r",
        "target_rpm_l", "target_rpm_r",
    ]
    for i in (1, 2, 3):
        cols += [
            f"uwb{i}_peer_id", f"uwb{i}_lde_err",
            f"uwb{i}_std_noise", f"uwb{i}_fp_ampl1",
            f"uwb{i}_fp_ampl2", f"uwb{i}_cir_pwr",
            f"uwb{i}_raw", f"uwb{i}_clean",
            f"uwb{i}_rssi", f"uwb{i}_fp_power",
            f"uwb{i}_resp_temp",
        ]
    cols += [
        "rxpacc",
        "eskf_var_x", "eskf_var_y",
        "eskf_var_theta", "eskf_var_bias",
        "tdma_frame_id",
        "tdma_slot_index", "tdma_sync_lost",
        "maneuver_id", "robot_state",
    ]
    return cols


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def verify_and_unpack(filename):
    with open(filename, "rb") as f:
        data = f.read()

    if len(data) < 22:
        print("ERROR: file too small")
        return

    # ---- Header (14 bytes) ----
    magic, version, robot_id, maneuver_id, count, rec_size = struct.unpack(
        "<4sHBB I H", data[:14]
    )
    print(f"Header: Magic={magic.decode()}, Version={version}, "
          f"RobotID={robot_id}, ManeuverID={maneuver_id}, "
          f"Records={count}, RecSize={rec_size}")
    assert magic == b"ANJM", "Invalid Header Magic!"

    # ---- Pick schema based on version ----
    if version == 1:
        expected_size = RECORD_V1_SIZE
        record_format = RECORD_V1_FORMAT
        cols = BASE_V1_COLS
    elif version == 2:
        expected_size = RECORD_V2_SIZE
        record_format = RECORD_V2_FORMAT
        cols = v2_columns()
    else:
        print(f"ERROR: unsupported schema version {version}")
        return

    assert rec_size == expected_size, (
        f"Record size mismatch: header says {rec_size}, "
        f"expected {expected_size} for version {version}"
    )

    # ---- Payload + footer ----
    payload     = data[14:-8]
    footer_data = data[-8:]
    expected_crc, end_magic = struct.unpack("<I4s", footer_data)
    assert end_magic == b"MJNA", "Invalid Footer Magic!"

    computed_crc = zlib.crc32(payload) & 0xFFFFFFFF
    ok = (expected_crc == computed_crc)
    print(f"CRC Check: Expected=0x{expected_crc:08X}, "
          f"Computed=0x{computed_crc:08X} -> "
          f"{'PASS' if ok else 'FAIL'}")
    assert ok, "CRC32 Checksum Failed! File is corrupted."

    # ---- Unpack ----
    rows = []
    for i in range(count):
        chunk = payload[i * rec_size : (i + 1) * rec_size]
        rows.append(struct.unpack(record_format, chunk))

    # ---- Sanity check: field count ----
    if len(rows[0]) != len(cols):
        print(f"WARNING: field count mismatch: "
              f"record has {len(rows[0])} fields, "
              f"columns defined {len(cols)}")
        n = min(len(rows[0]), len(cols))
        cols = cols[:n]
        rows = [r[:n] for r in rows]

    df = pd.DataFrame(rows, columns=cols)
    csv_name = filename.replace(".bin", ".csv")
    df.to_csv(csv_name, index=False)
    print(f"SUCCESS: Saved {len(df)} verified records to {csv_name}")
    print(df.head(3))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 verify_log.py <filename.bin>")
    else:
        verify_and_unpack(sys.argv[1])
