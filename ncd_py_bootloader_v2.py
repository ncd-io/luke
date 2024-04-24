# Syntax 
# $ python ncd_py_bootloader_v2.py <com port> <ncd update file> <Chunk size multiple of 16>
# Example: 
# $ python ncd_py_bootloader_v2.py COM4 ./Upgrade.ncd 240
from serial import Serial 
import sys 

UART_BR = 115200
READ_TIMEOUT = 5
INTER_BYTE_TIMEOUT = 0.05
MAX_CHUNK_SIZE = 240

STARTUP_MSG_LEN = 50
MANIFEST_LEN = 37

BL_CMD_RESET = 3
BL_CMD_STORE_MANIFEST = 1
BL_CMD_PROGRAM = 2

BL_CMD_CHECKSUM_LEN = 4
BL_CMD_LENGTH_LEN = 2

BL_CMD_LEN = 3
BL_PGM_CMD_LEN = 4

BL_CMD_RESP_LEN = 4
BL_MIN_CMD_RSP_LEN = BL_CMD_RESP_LEN + BL_CMD_CHECKSUM_LEN
BL_ERR_SUCCESS = 0
BL_CMD_RSP_STATUS_IDX = 3
BL_CMD_RSP_ID_IDX = 2

ser = None

def compute_checksum(data):
    crc = 0XFFFFFFFF
    for i in range(len(data)):
        crc = crc ^ data[i]
        for x in range(32):
            if(crc & 0x80000000):
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
    return crc

def wait_startup(max_img_sz):
    while True:
        msg_rcvd = False
        idx = 0
        startup_msg = ser.read(STARTUP_MSG_LEN+1)
        if STARTUP_MSG_LEN == len(startup_msg):
            msg_rcvd = True
        elif STARTUP_MSG_LEN+1 == len(startup_msg):
            idx = 1
            msg_rcvd = True
        if True == msg_rcvd:
            max_size = (startup_msg[5+idx] << 24) + (startup_msg[6+idx] << 16) + (startup_msg[7+idx] << 8) + startup_msg[8+idx]
            if max_size >= max_img_sz:
                print("Startup msg received!")
                break

def send_store_manifest(manifest):
    cmd = []
    cmd_len = BL_CMD_LEN + MANIFEST_LEN + BL_CMD_CHECKSUM_LEN - BL_CMD_LENGTH_LEN
    cmd.append(0xFF & (cmd_len >> 8))
    cmd.append(0xFF & cmd_len)
    cmd.append(BL_CMD_STORE_MANIFEST)
    cmd = cmd + list(manifest)
    cmd_crc = compute_checksum(cmd)
    cmd.append(0xFF & (cmd_crc >> 0))
    cmd.append(0xFF & (cmd_crc >> 8))
    cmd.append(0xFF & (cmd_crc >> 16))
    cmd.append(0xFF & (cmd_crc >> 24))
    ser.write(cmd)

def wait_manifest_ack():
    ret = False
    for i in range(0, 3):
        manifest_ack = ser.read(BL_MIN_CMD_RSP_LEN)
        if BL_MIN_CMD_RSP_LEN == len(manifest_ack):
            if (BL_ERR_SUCCESS == manifest_ack[BL_CMD_RSP_STATUS_IDX]) & (BL_CMD_STORE_MANIFEST == manifest_ack[BL_CMD_RSP_ID_IDX]):
                print("Manifest store success!")
                ret = True
                break
    return ret

def wait_pgm_pkt_ack():
    ret = False
    for i in range(0, 3):
        pgm_ack = ser.read(BL_MIN_CMD_RSP_LEN)
        if BL_MIN_CMD_RSP_LEN == len(pgm_ack):
            if (BL_ERR_SUCCESS == pgm_ack[BL_CMD_RSP_STATUS_IDX]) & (BL_CMD_PROGRAM == pgm_ack[BL_CMD_RSP_ID_IDX]):
                print("Pkt Pgm success")
                ret = True
                break
    return ret

def send_pgm_pkt(pkt, pkt_len, img_offset):
    cmd = []
    cmd_len = BL_CMD_LEN + BL_PGM_CMD_LEN + pkt_len + BL_CMD_CHECKSUM_LEN - BL_CMD_LENGTH_LEN
    cmd.append(0xFF & (cmd_len >> 8))
    cmd.append(0xFF & cmd_len)
    cmd.append(BL_CMD_PROGRAM)
    cmd.append(0xFF & (img_offset >> 24))
    cmd.append(0xFF & (img_offset >> 16))
    cmd.append(0xFF & (img_offset >> 8))
    cmd.append(0xFF & (img_offset >> 0))
    cmd = cmd + list(pkt)
    cmd_crc = compute_checksum(cmd)
    cmd.append(0xFF & (cmd_crc >> 0))
    cmd.append(0xFF & (cmd_crc >> 8))
    cmd.append(0xFF & (cmd_crc >> 16))
    cmd.append(0xFF & (cmd_crc >> 24))
    ser.write(cmd)

def send_reboot():
    cmd = []
    cmd_len = BL_CMD_LEN + BL_CMD_CHECKSUM_LEN - BL_CMD_LENGTH_LEN
    cmd.append(0xFF & (cmd_len >> 8))
    cmd.append(0xFF & cmd_len)
    cmd.append(BL_CMD_RESET)
    cmd_crc = compute_checksum(cmd)
    cmd.append(0xFF & (cmd_crc >> 0))
    cmd.append(0xFF & (cmd_crc >> 8))
    cmd.append(0xFF & (cmd_crc >> 16))
    cmd.append(0xFF & (cmd_crc >> 24))
    ser.write(cmd)

com_port = sys.argv[1] # Com port 
update_file = sys.argv[2] # file name
chunk_size = int(sys.argv[3]) # Chunk Size

ser = Serial(com_port, baudrate=UART_BR, timeout=READ_TIMEOUT, inter_byte_timeout=INTER_BYTE_TIMEOUT)

if chunk_size <= MAX_CHUNK_SIZE:
    if (chunk_size % 16 == 0):
        with open(update_file, mode="rb") as bin_file:
            fw_update = bin_file.read()
        if 0x01 == fw_update[0]:
            #Extract manifest
            manifest_length = (fw_update[1] << 24) + (fw_update[2] << 16) + (fw_update[3] << 8) + fw_update[4]
            manifest = fw_update[5 : 5 + manifest_length]
            max_img_size = (manifest[9] << 24) + (manifest[10] << 16) + (manifest[11] << 8) + manifest[12]

            #Extract Image
            image_offset = 5 + manifest_length
            if 0x02 == fw_update[image_offset]:
                image_length = (fw_update[image_offset + 1] << 24) + (fw_update[image_offset + 2] << 16) + (fw_update[image_offset + 3] << 8) + fw_update[image_offset + 4]
                image_offset = image_offset + 5
                image = fw_update[image_offset : image_offset + image_length]
                
                #Wait start up msg
                print("Waiting startup msg...")
                wait_startup(max_img_size)
                
                #Send manifest
                send_store_manifest(manifest)
                
                #Wait manifest Ack
                ret = wait_manifest_ack()
                
                if True == ret:
                    #Start programming image
                    image_idx = 0
                    while image_idx < image_length:
                        if (image_length - image_idx) > chunk_size:
                            bytes_to_write = chunk_size
                        else:
                            bytes_to_write = image_length - image_idx
                        pkt = image[image_idx : image_idx + bytes_to_write]
                        print("Pgm pkt @ ", hex(image_idx), " out of ", hex(image_length))
                        send_pgm_pkt(pkt, bytes_to_write, image_idx)
                        ret = wait_pgm_pkt_ack()
                        
                        if True == ret:
                            image_idx += bytes_to_write
                        else:
                            break

                    if image_idx == image_length:
                        # Image completed, Send Reboot
                        print("Sending Reboot")
                        send_reboot()
                    else:
                        print("Update failed.")
            else:
                print("Wrong File !!!")
        else:
            print("Wrong File !!!")
    else:
        print("Chunk Size not multiple of 16")
else:
    print("Chunk size should be less than 240")

if ser is not None:
    ser.close()