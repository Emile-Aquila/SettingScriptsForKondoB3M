import serial
import time

#baudrate = 2000000
baudrate = 1500000
b3m = serial.Serial('/dev/ttyUSB0', baudrate=baudrate, parity=serial.PARITY_NONE, timeout=0.5)
target_id = 0

#WRITEコマンド
#メモリーマップ(RAM)のアドレス指定でデバイスのRAM上に書き込みます。
def B3M_Write_CMD(servo_id, TxData, Address, byte_num=1): #送信コマンドを作成
    txCmd = [0x07+byte_num, 0x04,0x00,servo_id]
    for i in range(byte_num):
    	txCmd.append((TxData >> (i*8)) & 0xff)
    txCmd += [Address,0x01]
    
    checksum = 0
    for i in txCmd:
        checksum += i
    txCmd.append(checksum & 0xff)
    print(txCmd)
    
    b3m.write(txCmd)
    rxCmd = b3m.read(5)
    if len(rxCmd) == 0:
        return False

    return True


def B3M_setPos_CMD(servo_id, pos, MoveTime): #送信コマンドを作成
    txCmd = [0x09, 0x06,0x00, servo_id,pos & 0xff,pos >> 8 & 0xff,MoveTime & 0xff, MoveTime >> 8 & 0xff] #TIME_H

    checksum = 0
    for i in txCmd:
        checksum += i#リストの最後にチェックサムを挿入する
    txCmd.append(checksum & 0xff)
    b3m.write(txCmd)

    #サーボからの返事を受け取る
    rxCmd = b3m.read(7)

    if len(rxCmd) == 0:
        return False
    return True


def B3M_Save_CMD(servo_id): #送信コマンドを作成
    txCmd = [0x05, 0x02, 0x00, servo_id] 

    checksum = 0
    for i in txCmd:
        checksum += i#リストの最後にチェックサムを挿入する
    txCmd.append(checksum & 0xff)
    b3m.write(txCmd)

    #サーボからの返事を受け取る
    rxCmd = b3m.read(5)

    if len(rxCmd) == 0:
        return False
    return True


def move_test(servo_id):
	#B3Mサーボが動作するまでの準備
	#B3M_Write_CMD(servo_id, TxData, Address)

	#動作モード：Free (動作モードと特性を設定するため、設定書き換え中の誤動作を防止するため脱力にしておく)
	reData = B3M_Write_CMD(servo_id, 0x02, 0x28)
	if not reData:
		print("No response")

	#位置制御モードに設定 (角度を指定して動作するモードです)
	reData = B3M_Write_CMD(servo_id, 0x02, 0x28)

	#軌道生成タイプ：Even (直線補間タイプの位置制御を指定)
	reData = B3M_Write_CMD(servo_id, 0x01, 0x29)

    #ゲインプリセット：No.0 (PIDのプリセットゲインを位置制御モード用に設定)
	reData = B3M_Write_CMD(servo_id, 0x00, 0x5C)

    #動作モード：Normal （Freeの状態からトルクオン）
	reData = B3M_Write_CMD(servo_id, 0x00, 0x28)


    #ID0、500msかけて5000(50度)の位置に移動する
	reData = B3M_setPos_CMD(servo_id, 5000, 500)
	time.sleep(0.5) #サーボが到達するまで次の指示を待つ

	reData = B3M_setPos_CMD(servo_id, -5000, 500)
	time.sleep(0.5) #サーボが到達するまで次の指示を待つ

	reData = B3M_setPos_CMD(servo_id, 0, 500)
	time.sleep(0.5) #サーボが到達するまで次の指示を待つ
	
	b3m.close()
	

def change_baudrate(servo_id, new_baudrate):
	reData = B3M_Write_CMD(servo_id, 0x02, 0x28)
	reData = B3M_Write_CMD(servo_id, new_baudrate, 0x01, 4)
	if reData:
		print("Change Baudrate {} -> {}".format(baudrate, new_baudrate))
	else:
		print("Change Baudrate Failed!")
	reData = B3M_Save_CMD(servo_id)
	if reData:
		print("New Baudrate Saved")
	else:
		print("Save Failed!")
	


	
	
	


#change_baudrate(target_id, 2000000)
move_test(target_id)

b3m.close()





