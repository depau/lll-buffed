import os
import subprocess

from serial.tools.list_ports_common import ListPortInfo

Import("env")

import time

from serial import Serial
from serial.tools import list_ports

DFU_VID = 0x0483
DFU_PID = 0xDF11

CDC_VID = 0x0483
CDC_PID = 0xF072

WAIT_TIMEOUT_S = 5.0
POLL_INTERVAL_S = 0.1


def is_dfu_device_connected(vid: int, pid: int) -> bool:
    p = subprocess.check_output(
        ["dfu-util", "-l"], text=True, stderr=subprocess.DEVNULL
    )
    return f"{vid:04x}:{pid:04x}" in p


def wait_for_dfu(vid: int, pid: int, timeout_s: float) -> bool:
    start_time = time.time()
    while time.time() - start_time < timeout_s:
        if is_dfu_device_connected(vid, pid):
            return True
        time.sleep(POLL_INTERVAL_S)
    return False


def find_serial_port(vid: int, pid: int) -> ListPortInfo | None:
    for port in list_ports.comports():
        if port.vid == vid and port.pid == pid:
            return port
    return None


def before_upload(source, target, env):
    os.system("which dfu-util")

    if is_dfu_device_connected(DFU_VID, DFU_PID):
        print(f"Found buffer in DFU mode, proceeding with upload")
        return

    cdc_port = find_serial_port(CDC_VID, CDC_PID)
    if cdc_port is None:
        raise RuntimeError(
            "Buffer not detected. Please connect the device in DFU mode by pressing the R button while holding the B button on the main board."
        )

    print("Found buffer in regular mode, rebooting into DFU mode...")
    with Serial(cdc_port.device, 115200, timeout=1, write_timeout=1) as ser:
        ser.write(b"\nreboot_dfu\n")
        ser.flush()

    dfu_port = wait_for_dfu(DFU_VID, DFU_PID, WAIT_TIMEOUT_S)
    if dfu_port is None:
        raise RuntimeError(
            "DFU device not detected. Please put the device in DFU mode manually by pressing the R button while holding the B button on the main board."
        )

    print(f"Found buffer in DFU mode, proceeding with upload")


env.AddPreAction("upload", before_upload)
