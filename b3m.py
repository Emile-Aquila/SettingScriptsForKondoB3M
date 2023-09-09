import serial
import time
import math
from dataclasses import dataclass

baudrate = 2000000
# baudrate = 1500000

port = "/dev/ttyUSB0"

b3m = serial.Serial(port, baudrate=baudrate, parity=serial.PARITY_NONE, timeout=0.5)


@dataclass
class RxDataNormal:
    command: int = 0
    status: int = 0
    id: int = 0


@dataclass
class RxDataSetPos:
    command: int = 0
    status: int = 0
    id: int = 0
    current_pos: float = 0.0


# WRITEコマンド
# メモリーマップ(RAM)のアドレス指定でデバイスのRAM上に書き込みます。
def B3M_Write_CMD(servo_id: int, tx_data: int, address: int, byte_num: int = 1):  # 送信コマンドを作成
    txCmd = [0x07 + byte_num, 0x04, 0x00, servo_id]
    for i in range(byte_num):
        txCmd.append((tx_data >> (i * 8)) & 0xff)
    txCmd += [address, 0x01]

    txCmd.append(sum(txCmd) & 0xff)  # リストの最後にチェックサムを挿入する
    b3m.write(txCmd)
    rxCmd = b3m.read(5)
    if len(rxCmd) == 0:
        return False, RxDataNormal()
    else:
        int_array = list(rxCmd)
        return True, RxDataNormal(int_array[1], int_array[2], int_array[3])


def B3M_setPos_CMD(servo_id: int, pos_f: float, move_time: int):  # 送信コマンドを作成
    pos = round(pos_f * 100.0)
    txCmd = [0x09, 0x06, 0x00, servo_id, pos & 0xff, (pos >> 8) & 0xff, move_time & 0xff, move_time >> 8 & 0xff]
    txCmd.append(sum(txCmd) & 0xff)  # リストの最後にチェックサムを挿入する
    b3m.write(txCmd)

    # サーボからの返事を受け取る
    rxCmd = b3m.read(7)

    if len(rxCmd) == 0:
        return False, RxDataSetPos()
    else:
        int_array = list(rxCmd)
        pos = float(byte_to_value(int_array[4:6], True)) / 100.0
        return True, RxDataSetPos(int_array[1], int_array[2], int_array[3], pos)


def B3M_Save_CMD(servo_id: int):  # 送信コマンドを作成
    txCmd = [0x05, 0x02, 0x00, servo_id]
    txCmd.append(sum(txCmd) & 0xff)  # リストの最後にチェックサムを挿入する
    b3m.write(txCmd)

    # サーボからの返事を受け取る
    rxCmd = b3m.read(5)
    if len(rxCmd) == 0:
        return False, RxDataNormal()
    else:
        int_array = list(rxCmd)
        return True, RxDataNormal(int_array[1], int_array[2], int_array[3])


def byte_to_value(byte_array: list[int], complement: bool = False) -> int:
    ans = 0
    for i in range(len(byte_array)):
        ans += ((byte_array[i] & 0xff) << (8 * i))
    if complement:
        if ans & (1 << (len(byte_array) * 8 - 1)) != 0:
            ans = -((ans - 1) ^ ((1 << (8 * len(byte_array))) - 1))
    return ans


def B3M_Read_CMD(servo_id: int, address: int, byte_num: int):  # 送信コマンドを作成
    txCmd = [0x07, 0x03, 0x00, servo_id, address, byte_num]
    txCmd.append(sum(txCmd) & 0xff)  # リストの最後にチェックサムを挿入する

    b3m.write(txCmd)
    rxCmd = b3m.read(5 + byte_num)
    if len(rxCmd) == 0:
        return False, RxDataNormal()
    else:
        int_array = list(rxCmd)
        return True, int_array[4:4 + byte_num]


def B3M_Read_Datas(servo_id: int):
    print("=====[INFORMATION]=====")
    flag, data = B3M_Read_CMD(servo_id, 0x00, 1)
    if flag:
        print("ID: {}".format(byte_to_value(data)))
    flag, data = B3M_Read_CMD(servo_id, 0x05, 2)  # min
    if flag:
        print("min angle: {} (deg)".format(float(byte_to_value(data, True)) / 100.0))
    flag, data = B3M_Read_CMD(servo_id, 0x07, 2)  # max
    if flag:
        print("max angle: {} (deg)".format(float(byte_to_value(data, True)) / 100.0))
    flag, data = B3M_Read_CMD(servo_id, 0x09, 2)  # offset
    if flag:
        print("offset: {} (deg)".format(float(byte_to_value(data, True)) / 100.0))
    flag, data = B3M_Read_CMD(servo_id, 0x1C, 2)  # 不感帯
    if flag:
        print("dead zone: {}".format(float(byte_to_value(data)) / 100.0))
    print("==================")


def Move_Test(servo_id: int):
    # B3Mサーボが動作するまでの準備
    # 動作モード：Free (動作モードと特性を設定するため、設定書き換え中の誤動作を防止するため脱力にしておく)
    flag, _ = B3M_Write_CMD(servo_id, 0x02, 0x28)

    if not flag:
        print("No response")

    # 位置制御モードに設定 (角度を指定して動作するモードです)
    _ = B3M_Write_CMD(servo_id, 0x02, 0x28)

    # 軌道生成タイプ：Even (直線補間タイプの位置制御を指定)
    _ = B3M_Write_CMD(servo_id, 0x01, 0x29)
    # 起動生成タイプ : Normal
    # reData = B3M_Write_CMD(servo_id, 0x00, 0x29)

    # ゲインプリセット：No.0 (PIDのプリセットゲインを位置制御モード用に設定)
    _ = B3M_Write_CMD(servo_id, 0x00, 0x5C)

    # 動作モード：Normal （Freeの状態からトルクオン）
    _ = B3M_Write_CMD(servo_id, 0x00, 0x28)

    # ID0、500msかけて5000(50度)の位置に移動する
    _ = B3M_setPos_CMD(servo_id, 50.00, 500)
    time.sleep(2.0)  # サーボが到達するまで次の指示を待つ

    # _ = B3M_setPos_CMD(servo_id, -50.00, 500)
    # time.sleep(2.0)  # サーボが到達するまで次の指示を待つ

    _ = B3M_setPos_CMD(servo_id, -180.0, 500)
    time.sleep(2.0)  # サーボが到達するまで次の指示を待つ


def Move_Test2(servo_id: int):
    # B3Mサーボが動作するまでの準備
    # 動作モード：Free (動作モードと特性を設定するため、設定書き換え中の誤動作を防止するため脱力にしておく)
    flag, _ = B3M_Write_CMD(servo_id, 0x02, 0x28)

    if not flag:
        print("No response")

    # 位置制御モードに設定 (角度を指定して動作するモードです)
    _ = B3M_Write_CMD(servo_id, 0x02, 0x28)

    # 軌道生成タイプ：Even (直線補間タイプの位置制御を指定)
    _ = B3M_Write_CMD(servo_id, 0x01, 0x29)
    # 起動生成タイプ : Normal
    # reData = B3M_Write_CMD(servo_id, 0x00, 0x29)

    # ゲインプリセット：No.0 (PIDのプリセットゲインを位置制御モード用に設定)
    _ = B3M_Write_CMD(servo_id, 0x00, 0x5C)

    # 動作モード：Normal （Freeの状態からトルクオン）
    _ = B3M_Write_CMD(servo_id, 0x00, 0x28)

    # ID0、500msかけて5000(50度)の位置に移動する
    _ = B3M_setPos_CMD(servo_id, 0.00, 500)
    time.sleep(2.0)  # サーボが到達するまで次の指示を待つ

    # _ = B3M_setPos_CMD(servo_id, -50.00, 500)
    # time.sleep(2.0)  # サーボが到達するまで次の指示を待つ

    _ = B3M_setPos_CMD(servo_id, -60.0, 500)
    time.sleep(2.0)  # サーボが到達するまで次の指示を待つ

    max_v = math.degrees(16 / 3 + math.pi / 4.0)
    print(max_v - 60.0)
    _ = B3M_setPos_CMD(servo_id, max_v, 500)
    time.sleep(2.0)  # サーボが到達するまで次の指示を待つ


def B3M_Change_Baudrate(servo_id: int, new_baudrate: int):
    _ = B3M_Write_CMD(servo_id, 0x02, 0x28)  # 動作モード: Free
    flag, _ = B3M_Write_CMD(servo_id, new_baudrate, 0x01, 4)
    if flag:
        print("Change Baudrate {} -> {}".format(baudrate, new_baudrate))
    else:
        print("Change Baudrate Failed!")
    flag, _ = B3M_Save_CMD(servo_id)
    if flag:
        print("New Baudrate Saved. Please Reboot B3M.")
    else:
        print("Save Failed!")


def B3m_Change_ID(servo_id: int, new_id: int):
    _ = B3M_Write_CMD(servo_id, 0x02, 0x28)  # 動作モード: Free
    flag, _ = B3M_Write_CMD(servo_id, new_id, 0x00)  # IDの設定を変更
    if flag:
        print("Change ID {} -> {}".format(servo_id, new_id))
    else:
        print("Change ID Failed!")
    flag, _ = B3M_Save_CMD(new_id)
    if flag:
        print("New ID Saved")
    else:
        print("Save Failed!")


def B3m_Set_Offset(servo_id: int, offset: float):
    flag, _ = B3M_Write_CMD(servo_id, 0x02, 0x28)  # 動作モード: Free
    offset_value = round(offset * 100.0)
    flag, _ = B3M_Write_CMD(servo_id, offset_value, 0x09, 2)  # 中央値offset
    if flag:
        print("Set Offset {}".format(offset))
    else:
        print("Set Offset Failed!")
    flag, _ = B3M_Save_CMD(servo_id)
    if flag:
        print("Offset Saved")
    else:
        print("Save Failed!")


def B3M_Set_MinAngle(servo_id: int, min_angle: float):
    flag, _ = B3M_Write_CMD(servo_id, 0x02, 0x28)  # 動作モード: Free
    min_angle_int = round(min_angle * 100.0)
    flag, _ = B3M_Write_CMD(servo_id, min_angle_int, 0x05, 2)  # 中央値offset
    if flag:
        print("Set Min Angle {}".format(min_angle_int))
    else:
        print("Set Min Angle Failed!")
    flag, _ = B3M_Save_CMD(servo_id)
    if flag:
        print("Min Angle Saved.")
    else:
        print("Save Failed!")


def B3M_Set_MaxAngle(servo_id: int, max_angle: float):
    flag, _ = B3M_Write_CMD(servo_id, 0x02, 0x28)  # 動作モード: Free
    max_angle_int = round(max_angle * 100.0)
    flag, _ = B3M_Write_CMD(servo_id, max_angle_int, 0x07, 2)  # 中央値offset
    if flag:
        print("Set Max Angle {}".format(max_angle_int))
    else:
        print("Set Max Angle Failed!")
    flag, _ = B3M_Save_CMD(servo_id)
    if flag:
        print("Max Angle Saved.")
    else:
        print("Save Failed!")


target_id = 5
new_id = 8

## movement test
# Move_Test(target_id)

## change ID and baudrate
# B3m_Change_ID(target_id, new_id)
# B3M_Change_Baudrate(new_id, 2000000)
# Move_Test(new_id)


## Set Min/Max Angle
B3M_Read_Datas(target_id)
B3M_Set_MaxAngle(target_id, 320.0)
B3M_Set_MinAngle(target_id, -320.0)
time.sleep(1.0)
B3M_Read_Datas(target_id)


## Set Offset
# time.sleep(2.0)
# B3m_Set_Offset(target_id, -60.0)
# time.sleep(2.0)

b3m.close()
