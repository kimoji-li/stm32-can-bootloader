# -*- coding: utf-8 -*-
"""CAN Bootloader 上位机

用法:
    python test_pcan.py <firmware.bin>    正常升级
    python test_pcan.py --fault           异常测试
"""
import can
import sys
import time

CAN_ID_CMD  = 0x100
CAN_ID_DATA = 0x101
CAN_ID_ACK  = 0x102

CMD_START  = 0x01
CMD_ERASE  = 0x02
CMD_FINISH = 0x03

ST_OK        = 0x00
ST_ERR_SEQ   = 0x01
ST_ERR_CMD   = 0x02
ST_ERR_FLASH = 0x03
ST_ERR_CRC   = 0x04
ST_ERR_SIZE  = 0x05

DATA_PER_FRAME = 6
FW_PATH = "D:\\Stm32cubemax\\App2\\App.bin"


# ---------------------------------------------------------------- helpers
def sstr(st):
    """None-safe status formatter"""
    return "None" if st is None else "0x%02X" % st


def crc32_mpeg(data):
    """CRC-32/MPEG-2: poly 0x04C11DB7, init 0xFFFFFFFF, no reflect, no final xor"""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= (b << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def flush(bus):
    """drop all pending messages in the rx buffer"""
    n = 0
    while bus.recv(timeout=0) is not None:
        n += 1
    return n


def wait_ack(bus, expect_seq, timeout=2.0):
    """wait for an ACK with matching sequence number"""
    end = time.time() + timeout
    while time.time() < end:
        rx = bus.recv(timeout=0.2)
        if rx is None:
            continue
        if rx.arbitration_id != CAN_ID_ACK:
            continue
        st = rx.data[0]
        sq = rx.data[1] | (rx.data[2] << 8)
        if st != 0x00:
            return st, sq
        if expect_seq is None or sq == expect_seq:
            return st, sq
    return None, None


def send_cmd(bus, cmd):
    bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=[cmd],
                         is_extended_id=False))


def send_data(bus, seq, payload):
    d = [seq & 0xFF, (seq >> 8) & 0xFF] + list(payload)
    bus.send(can.Message(arbitration_id=CAN_ID_DATA, data=d,
                         is_extended_id=False))


def send_finish(bus, crc):
    cmd = [CMD_FINISH,
           crc & 0xFF, (crc >> 8) & 0xFF, (crc >> 16) & 0xFF, (crc >> 24) & 0xFF,
           0, 0, 0]
    bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=cmd,
                         is_extended_id=False))


# ---------------------------------------------------------------- flow
def handshake(bus):
    print("[*] handshake: sending CMD_START, press RESET on STM32 if needed")
    flush(bus)
    for attempt in range(40):
        send_cmd(bus, CMD_START)
        st, sq = wait_ack(bus, None, timeout=0.5)
        if st == 0x00:
            print("    bootloader responded (attempt %d)" % (attempt + 1))
            time.sleep(1.0)
            print("    flushed %d stale messages" % flush(bus))
            return True
    print("    no response, exit")
    return False


def do_erase(bus):
    print("[*] CMD_ERASE ...")
    flush(bus)
    send_cmd(bus, CMD_ERASE)
    st, sq = wait_ack(bus, 0, timeout=20.0)
    print("    ACK: status=%s expect=%s" % (sstr(st), sq))
    return st == ST_OK


def load_fw(path):
    fw = bytearray(open(path, 'rb').read())
    pad = (DATA_PER_FRAME - (len(fw) % DATA_PER_FRAME)) % DATA_PER_FRAME
    if pad:
        fw += b'\xFF' * pad
    return fw


def send_frames(bus, fw, start, total):
    t0 = time.time()
    for i in range(start, total):
        chunk = fw[i * DATA_PER_FRAME:(i + 1) * DATA_PER_FRAME]
        ok = False
        for retry in range(10):
            flush(bus)
            send_data(bus, i, chunk)
            st, sq = wait_ack(bus, i + 1, timeout=1.5)
            if st == ST_OK and sq == i + 1:
                ok = True
                break
        if not ok:
            print("    frame %d FAILED (status=%s expect=%s)" % (i, sstr(st), sq))
            return False
        if (i + 1) % 200 == 0:
            print("    progress %d/%d (%d%%)" % (i + 1, total, (i + 1) * 100 // total))
    print("    all sent, %.1f sec" % (time.time() - t0))
    return True


# ---------------------------------------------------------------- normal upgrade
def normal_upgrade(bus, path):
    fw = load_fw(path)
    total = len(fw) // DATA_PER_FRAME
    crc = crc32_mpeg(fw)
    print("firmware : %s" % path)
    print("size     : %d bytes -> %d frames" % (len(fw), total))
    print("CRC32    : 0x%08X\n" % crc)

    if not handshake(bus):
        return
    if not do_erase(bus):
        return

    print("[*] sending firmware data ...")
    if not send_frames(bus, fw, 0, total):
        return

    print("[*] CMD_FINISH (with CRC) ...")
    flush(bus)
    send_finish(bus, crc)
    st, sq = wait_ack(bus, None, timeout=15.0)
    print("    ACK: status=%s expect=%s" % (sstr(st), sq))

    if st == ST_OK:
        print("\n>>> upgrade OK <<<")
    elif st == ST_ERR_CRC:
        print("\n>>> CRC mismatch, rejected <<<")
    elif st == ST_ERR_FLASH:
        print("\n>>> flash error <<<")
    else:
        print("\n>>> FAILED <<<")
    print("DONE")


# ---------------------------------------------------------------- fault test
def fault_test(bus):
    results = []

    fw = load_fw(FW_PATH)
    total = len(fw) // DATA_PER_FRAME
    crc = crc32_mpeg(fw)
    print("firmware : %s" % FW_PATH)
    print("size     : %d bytes -> %d frames" % (len(fw), total))
    print("CRC32    : 0x%08X\n" % crc)

    if not handshake(bus):
        return results
    if not do_erase(bus):
        return results

    # ---- TEST 1 : out-of-order ----
    print("\n===== TEST 1: out-of-order frame =====")
    print("  send seq=5 (STM32 expects 0)")
    flush(bus)
    send_data(bus, 5, [0xAA] * 6)
    st, sq = wait_ack(bus, None, timeout=2.0)
    print("  ACK: status=%s expect=%s" % (sstr(st), sq))
    ok = (st == ST_ERR_SEQ and sq == 0)
    print("  %s" % ("PASS" if ok else "FAIL"))
    results.append(("out-of-order rejected", ok))

    # ---- TEST 2 : duplicate ----
    print("\n===== TEST 2: duplicate frame =====")
    print("  send seq=0, then send seq=0 again")
    flush(bus)
    send_data(bus, 0, fw[0:6])
    st, sq = wait_ack(bus, 1, timeout=2.0)
    print("  1st ACK: status=%s expect=%s" % (sstr(st), sq))
    send_data(bus, 0, fw[0:6])
    st, sq = wait_ack(bus, None, timeout=2.0)
    print("  2nd ACK: status=%s expect=%s" % (sstr(st), sq))
    ok = (st == ST_ERR_SEQ and sq == 1)
    print("  %s" % ("PASS" if ok else "FAIL"))
    results.append(("duplicate detected", ok))

    # ---- TEST 3 : remaining frames ----
    print("\n===== TEST 3: send remaining %d frames =====" % (total - 1))
    ok = send_frames(bus, fw, 1, total)
    results.append(("remaining frames", ok))
    if not ok:
        return results

    # ---- TEST 4 : wrong CRC ----
    print("\n===== TEST 4: wrong CRC =====")
    bad = crc ^ 0x00000001
    print("  send bad CRC = 0x%08X (correct 0x%08X)" % (bad, crc))
    flush(bus)
    send_finish(bus, bad)
    st, sq = wait_ack(bus, None, timeout=15.0)
    print("  ACK: status=%s expect=%s" % (sstr(st), sq))
    ok = (st == ST_ERR_CRC)
    print("  %s" % ("PASS" if ok else "FAIL"))
    results.append(("wrong CRC rejected", ok))

    # ---- TEST 5 : correct CRC ----
    print("\n===== TEST 5: correct CRC =====")
    flush(bus)
    send_finish(bus, crc)
    st, sq = wait_ack(bus, None, timeout=15.0)
    print("  ACK: status=%s expect=%s" % (sstr(st), sq))
    ok = (st == ST_OK)
    print("  %s" % ("PASS" if ok else "FAIL"))
    results.append(("correct CRC accepted", ok))

    return results


# ---------------------------------------------------------------- main
def main():
    try:
        bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1',
                                bitrate=500000)
    except Exception as e:
        print("cannot open PCAN device: %s" % e)
        print("  -> close PCAN-View first, then re-plug the USB adapter")
        return

    print("PCAN-USB connected\n")

    try:
        if len(sys.argv) >= 2 and sys.argv[1] == '--fault':
            res = fault_test(bus)
            print("\n\n========== SUMMARY ==========")
            for name, ok in res:
                print("  [%s] %s" % ("PASS" if ok else "FAIL", name))
            return

        if len(sys.argv) < 2:
            print("usage:")
            print("  python test_pcan.py <firmware.bin>    normal upgrade")
            print("  python test_pcan.py --fault           fault test")
            return

        normal_upgrade(bus, sys.argv[1])
    finally:
        bus.shutdown()
        print("bus closed")


main()
