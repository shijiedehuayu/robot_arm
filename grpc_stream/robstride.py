from enum import IntEnum
from ctypes import *
import time
import sys
import math
import struct
from collections import deque
from winreg import QueryReflectionKey
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
class Control_Type(IntEnum):
    MIT = 0
    POS=1
    POS_VEL = 5
    VEL = 2
    Torque_Pos = 3

class VCI_INIT_CONFIG(Structure):  
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]  
                
class VCI_CAN_OBJ(Structure):  
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)
                ] 
    
class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [('SIZE', c_uint16), ('STRUCT_ARRAY', POINTER(VCI_CAN_OBJ))]

    def __init__(self,num_of_structs):
                                                                 #这个括号不能少
        self.STRUCT_ARRAY = cast((VCI_CAN_OBJ * num_of_structs)(),POINTER(VCI_CAN_OBJ))#结构体数组
        self.SIZE = num_of_structs#结构体长度
        self.ADDR = self.STRUCT_ARRAY[0]#结构体数组地址  byref()转c地址

class Motor:
    def __init__(self, MotorType, SlaveID,inverse):
        """
        define Motor object 定义电机对象
        :param MotorType: Motor type 电机类型
        :param SlaveID: CANID 电机ID
        """
        self.Pd = float(0)
        self.Vd = float(0)
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)

        # 滤波参数（新增）
        self.alpha = 0.8  # 滤波系数
        # 平滑后的数据存储（新增）
        self.smooth_q = float(0)
        self.smooth_dq = float(0)
        self.smooth_tau = float(0)

        self.SlaveID = SlaveID
        self.MotorType = MotorType
        self.NowControlMode = Control_Type.MIT
        self.inverse=inverse

    def recv_data(self, q: float, dq: float, tau: float):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau

        # 一阶低通滤波计算平滑值：smooth = alpha * 新值 + (1 - alpha) * 旧平滑值
        self.smooth_q = self.alpha * q + (1 - self.alpha) * self.smooth_q
        self.smooth_dq = self.alpha * dq + (1 - self.alpha) * self.smooth_dq
        self.smooth_tau = self.alpha * tau + (1 - self.alpha) * self.smooth_tau

    def getPosition(self):
        """
        get the position of the motor 获取电机位置
        :return: the position of the motor 电机位置
        """
        return self.state_q

    def getVelocity(self):
        """
        get the velocity of the motor 获取电机速度
        :return: the velocity of the motor 电机速度
        """
        return self.state_dq

    def getTorque(self):
        """
        get the torque of the motor 获取电机力矩
        :return: the torque of the motor 电机力矩
        """
        return self.state_tau


class MotorControl:
    
    def __init__(self,can=1) -> None:
        self.control_dict={
            'GetID':0x0,'MIT':0x1,'Enable':0X3,'Stop':0X4,'SetZero':0X6,'SetID':0X7,
            'GetInfo':0X11,'SetInfo':0X12,'GetError':0X15,'SetBaud':0X16,'Save':0X18,'Server_ID':0xfd}
        #                   4310              4310_48            4340           4340_48
        self.Limit_Param = {'00':[12.57, 33, 14], '01':[12.57, 44, 17], '02':[12.57, 44, 17],
                            '03':[12.57, 20, 60], '04':[12.57, 15, 120], '05':[12.57, 50, 5.5], '06':[12.57, 50, 36]}
        self.motors_map={}
        CanDLLName = './ControlCAN.dll' #把DLL放到对应的目录下
        self.canDLL = windll.LoadLibrary(CanDLLName)
        self.VCI_USBCAN2 = 4
        self.STATUS_OK = 1
        self.can=can
        print('init robot communication')
    
    def opon_device(self):
        ret = self.canDLL.VCI_OpenDevice(self.VCI_USBCAN2, 0, 0)
        assert ret==self.STATUS_OK,'调用 VCI_OpenDevice出错\r\n'
        vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x14, 0)#波特率125k，正常模式
        ret = self.canDLL.VCI_InitCAN(self.VCI_USBCAN2, 0, self.can, byref(vci_initconfig))
        assert ret==self.STATUS_OK,'调用 VCI_InitCAN1出错\r\n'
        ret = self.canDLL.VCI_StartCAN(self.VCI_USBCAN2, 0, self.can)
        assert ret==self.STATUS_OK,'调用 VCI_StartCAN1 出错\r\n'
        print('open communication device1')
    
    def close_device(self):
        ret=self.canDLL.VCI_CloseDevice(self.VCI_USBCAN2, 0) 
        assert ret==self.STATUS_OK,'调用 VCI_CloseDevice 出错\r\n'
        print('close communication device1')

    def clear_buffer(self):
        ret = self.canDLL.VCI_ClearBuffer(self.VCI_USBCAN2, 0, 0)
        assert ret==self.STATUS_OK,'调用 VCI_ClearBuffer 出错\r\n'
        print('clear buffer1 ')

    def send_message(self,ID,DATA):
        ubyte_3array = c_ubyte*3
        recive = ubyte_3array(0, 0 , 0)
        vci_can_obj = VCI_CAN_OBJ(ID, 0, 0, 1, 0, 1,  8, DATA, recive)#单次发送
        ret = self.canDLL.VCI_Transmit(self.VCI_USBCAN2, 0, self.can, byref(vci_can_obj), 1)
        #print('send_message ID:',hex(ID),' Data: ',DATA[:])
        assert ret==self.STATUS_OK,'调用 VCI_Transmit 出错\r\n'

    def send_message_batch(self, id_list, data_list):
        """
        一次性发送多帧 CAN 消息
        :param id_list:  [id1, id2, ...]  每个 CAN 帧的 ID
        :param data_list: [data1, data2, ...]  每个 data 是长度为8的 bytes/bytearray/list
        """
        if len(id_list) != len(data_list):
            raise ValueError("id_list 和 data_list 长度必须一致")

        ubyte_3array = c_ubyte * 3
        count = len(id_list)

        # 创建 VCI_CAN_OBJ 数组
        VCI_CAN_OBJ_Array = VCI_CAN_OBJ * count
        objs = VCI_CAN_OBJ_Array()

        for i, (ID, DATA) in enumerate(zip(id_list, data_list)):
            if len(DATA) != 8:
                raise ValueError(f"第 {i} 个数据包长度不是 8 字节")

            recive = ubyte_3array(0, 0, 0)
            objs[i] = VCI_CAN_OBJ(
                ID,          # CAN ID
                0,           # TimeStamp
                0,           # TimeFlag
                1,           # SendType
                0,           # RemoteFlag
                1,           # ExternFlag
                8,           # DataLen
                (c_ubyte * 8)(*DATA),  # Data
                recive       # Reserved
            )

        # 一次性发送
        ret = self.canDLL.VCI_Transmit(
            self.VCI_USBCAN2,
            0,
            self.can,
            byref(objs[0]),
            1
        )

        assert ret == self.STATUS_OK, "调用 VCI_Transmit 出错"

    def Enable_all(self, id_list):
        """
        批量使能多个电机
        Args:
            id_list: 要使能的电机 ID 列表，例如 [1,2,3,4]
        Returns:
            无
        """
        ubyte_array = c_ubyte * 8

        can_id_list = []
        data_list = []

        for motor_id in id_list:
            ID_code = (self.control_dict['Enable'] << 24) | (self.control_dict['Server_ID'] << 8) | motor_id
            can_id_list.append(ID_code)
            data_list.append(ubyte_array())  # 空数据 8 字节

        # 批量下发
        self.send_message_batch(can_id_list, data_list)

        # 接收一次反馈并解码
        messages = self.recivice_message_once()
        print(messages)
        for msg in messages:
            self.decoder(msg)
        
        return None
            
    def recivice_message_once(self):
        rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2)#结构体数组
        recive_message=[]
        ret = self.canDLL.VCI_Receive(self.VCI_USBCAN2, 0, self.can, byref(rx_vci_can_obj.ADDR), 2, 0)
        for i in range(0,ret):
            ID=hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID)
            DataLen=hex(rx_vci_can_obj.STRUCT_ARRAY[i].DataLen)
            Data=list(rx_vci_can_obj.STRUCT_ARRAY[i].Data)
            recive_message.append({'ID':ID,'DataLen':DataLen,'Data':Data})
        return recive_message

    def addMotor(self, Motor):
        """
        add motor to the motor control object 添加电机到电机控制对象
        :param Motor: Motor object 电机对象
        """
        self.motors_map[Motor.SlaveID] = Motor
        return True

    def int_to_bytes_array(self, decimal_number, inverse=False):
        """
        将整数转换为字节数组（小端模式或大端模式）。
        Args:
            decimal_number (int): 要转换的整数，范围 0 到 65535。
            inverse (bool): 大端传输还是小端传输，false为大端，true为小端
        Returns:
            list[int]: 两个字节的数组，每个字节为 0-255。
        """
        if decimal_number < 0 or decimal_number > 65535:
            raise ValueError("Input must be in the range 0 to 65535.")

        # 小端模式 (低位字节在前，高位字节在后)
        if inverse:
            return [decimal_number & 0xFF, (decimal_number >> 8) & 0xFF]
        # 大端模式 (高位字节在前，低位字节在后)
        else:
            return [(decimal_number >> 8) & 0xFF, decimal_number & 0xFF]
    
    def Enable(self,id):
        """
        使能电机
        Args:
            id: 要使能的电机
        Returns:
            无。
        """
        ubyte_array = c_ubyte*8
        ID_code = (self.control_dict['Enable'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        self.send_message(ID_code,ubyte_array())
        message=self.recivice_message_once()
        for msg in message:
            self.decoder(msg)
        #print(message)
        return None

    def Disable(self,id,Clear=False):
        """
        暂停电机运行
        Args:
            id: 要暂停的电机
            Clear:要不要清除错误信息
        Returns:
            无。
        """
        ubyte_array = c_ubyte*8
        data=ubyte_array()
        ID_code = (self.control_dict['Stop'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        if not Clear:
            self.send_message(ID_code,data)
        else:
            data[0]=1
            self.send_message(ID_code,data)
        message=self.recivice_message_once()
        for msg in message:
            self.decoder(msg)
        return None
        
    def Set_Zero(self,id):
        """
        设置机械零位,标定的时候使用
        Args:
            id: 要保留设置零位的电机
        Returns:
            无。
        """
        ubyte_array = c_ubyte*8
        data=ubyte_array()
        ID_code = (self.control_dict['SetZero'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        data[0]=1
        self.send_message(ID_code,data)
        message=self.recivice_message_once()
        for msg in message:
            self.decoder(msg)
        print(message)
        return None

    def Set_Mode(self,id,mode=0):
        """
        设置机械零位,标定的时候使用
        Args:
            id: 要保留设置零位的电机
        Returns:
            无。
        """
        ubyte_array = c_ubyte*8
        data=ubyte_array()
        ID_code = (self.control_dict['SetInfo'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        data[:2]=self.int_to_bytes_array(0X7005,inverse=True)
        data[4]=mode
        self.motors_map[id].Control_Type=mode
        self.send_message(ID_code,data)
        message=self.recivice_message_once()
        # data_list = [f"0x{b:02X}" for b in data]
        # print(f"[Set_Mode] ID=0x{ID_code:X}, Bytes={data_list}")
        time.sleep(0.1)
        for msg in message:
            print(self.decoder(msg))
        return None

    def Set_zero_state(self,id,mode=0):
        """
        设置机械零位,标定的时候使用
        Args:
            id: 要保留设置零位的电机
        Returns:
            无。
        """
        ubyte_array = c_ubyte*8
        data=ubyte_array()
        ID_code = (self.control_dict['SetInfo'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        data[:2]=self.int_to_bytes_array(0X7005,inverse=True)
        data[4]=mode
        self.motors_map[id].Control_Type=mode
        self.send_message(ID_code,data)
        message=self.recivice_message_once()
        for msg in message:
            self.decoder(msg)
        return None

    def bytes_array_to_int(self, byte_array, inverse=False):
        """
        将两个字节的字节数组转换为整数（大端模式或小端模式）。
        Args:
            byte_array (list[int]): 两个字节的数组，每个字节为 0-255。
            inverse (bool): 大端传输还是小端传输，false为大端，true为小端
        Returns:
            int: 转换后的整数，范围 0 到 65535。
        """
        if len(byte_array) != 2:
            raise ValueError("Input must be an array of two bytes.")
        if not all(0 <= byte <= 255 for byte in byte_array):
            raise ValueError("Each byte must be in the range 0 to 255.")
    
        # 小端模式 (低位字节在前，高位字节在后)
        if inverse:
            return byte_array[0] | (byte_array[1] << 8)
        # 大端模式 (高位字节在前，低位字节在后)
        else:
            return (byte_array[0] << 8) | byte_array[1]

    def decoder(self,message):
        '''
        对所有信息进行解码，主要涵盖两种应答模式，
        一种是control_id为0，一种是control_id是2，
        两种模式的数据含义不一致，分开解码
        '''
        hex_number=int(message['ID'],16)
        Data=message['Data']
        control_ID=(hex_number >> 24) & 0xFF
        if control_ID==2:
            motor_can_id = (hex_number >> 8) & 0xFF
            host_can_id = hex_number & 0xFF

            # 提取故障信息 (Bit 16~21)
            fault_info = (hex_number >> 16) & 0x3F  # 6位故障信息
            faults = {
            "uncalibrated": bool(fault_info & (1 << 5)),  # Bit 21
            "hall_fault": bool(fault_info & (1 << 4)),    # Bit 20
            "mag_fault": bool(fault_info & (1 << 3)),     # Bit 19
            "over_temp": bool(fault_info & (1 << 2)),     # Bit 18
            "over_current": bool(fault_info & (1 << 1)),  # Bit 17
            "under_voltage": bool(fault_info & (1 << 0)), # Bit 16
            }
            #print(faults)
            angle=self.bytes_array_to_int(Data[:2])
            velocity=self.bytes_array_to_int(Data[2:4])
            torque=self.bytes_array_to_int(Data[4:6])
            temperature=self.bytes_array_to_int(Data[6:8])
            MotorType=self.motors_map[motor_can_id].MotorType
            motor_info={
                'angle':(angle/65535-0.5)*2*self.Limit_Param[MotorType][0],
                'velocity':(velocity/65535-0.5)*2*self.Limit_Param[MotorType][1],
                'torque':(torque/65535-0.5)*2*self.Limit_Param[MotorType][2],
                'temperature':temperature/10,
            }
            if self.motors_map[motor_can_id].inverse:
                motor_info['angle']=-motor_info['angle']
                motor_info['velocity']=-motor_info['velocity']
                motor_info['torque']=-motor_info['torque']
            #print(motor_info)
            self.motors_map[motor_can_id].recv_data(motor_info['angle'], motor_info['velocity'], motor_info['torque'])

            return motor_info
        else :
            print('un useful response')
    def clip(self,val, min_val, max_val):
        return max(min_val, min(max_val, val))
    
    def MIT(self,id,angle,velocity,torque,kp,kd):
        """
        Args:
            id: 要设置的电机
            angle: 目标角度,参考info_dict
            velocity: 目标速度,参考info_dict
            torque: 目标扭矩,参考info_dict
            kp: 目标kp,参考info_dict
            kd: 目标kd,参考info_dict
        Returns:
            无。
        """
        if self.motors_map[id].inverse:
            angle=-angle
            velocity=-velocity
            torque=-torque
        MotorType=self.motors_map[id].MotorType
        P_max,V_max,T_max=self.Limit_Param[MotorType]
        angle_int = int(((self.clip(angle, -P_max, P_max) + P_max) / (2 * P_max)) * 65535)
        velocity_int = int(((self.clip(velocity, -V_max, V_max) + V_max) / (2 * V_max)) * 65535)
        torque_int = int(((self.clip(torque, -T_max, T_max) + T_max) / (2 * T_max)) * 65535)
        kp_int = int((self.clip(kp, 0, 500) / 500.0) * 65535)
        kd_int = int((self.clip(kd, 0, 5) / 5.0) * 65535)
        ubyte_array = c_ubyte*8
        data=ubyte_array()
        ID_code = (self.control_dict['MIT'] << 24) | (torque_int << 8) | id
        data[:2]=self.int_to_bytes_array(angle_int)
        data[2:4]=self.int_to_bytes_array(velocity_int)
        data[4:6]=self.int_to_bytes_array(kp_int)
        data[6:8]=self.int_to_bytes_array(kd_int)
        self.send_message(ID_code,data)
        message=self.recivice_message_once()
        for msg in message:
            self.decoder(msg)

    def ChangeType(self, master_id: int, motor_id: int, type_id: int):
        """
        切换电机协议类型（扩展帧格式）
        Args:
            master_id: 主控制器的 CAN ID (bit23~8)
            motor_id:  目标电机的 CAN ID (bit7~0)
            type_id:   协议类型
                   0 -> 私有协议 (默认)
                   1 -> CANopen 协议
                   2 -> MIT 协议
        """
        if type_id not in (0, 1, 2):
            raise ValueError("协议类型必须是 0(私有) / 1(CANopen) / 2(MIT)")

        # 构造 29 位 CAN ID: 高 5 位固定 0x19
        can_id = (0x19 << 24) | ((master_id & 0xFFFF) << 8) | (motor_id & 0xFF)

        # 数据区：01 02 03 04 05 06 + type_id + 00
        ubyte_array = (c_ubyte * 8)(1, 2, 3, 4, 5, 6, type_id, 0x00)

        # 发送报文
        self.send_message(can_id, ubyte_array)  # extended=True 表示扩展帧
        time.sleep(0.1)
        # 等待反馈
        message = self.recivice_message_once()
        return None



    def POS_velocity(self, id, pos: float = 0.0, vel: float = 0.0):
        """
        设置电机参数（位置模式角度指令 / 速度限制）
        Args:
        id: 电机ID
        index: 参数索引 (0x7016=位置模式角度, 0x7017=速度限制)
        value: 写入的float数值
        Returns:
        None
        """
        # 构造 8 字节报文
        ubyte_array = c_ubyte * 8
        data = ubyte_array()

        # Byte0~1: index，小端序
        data[:2]=self.int_to_bytes_array(0x7017,inverse=True)
        # Byte2~3: 固定为 00 00
        data[2], data[3] = 0x00, 0x00
        # Byte4~7: 参数数据 float → 4字节，小端序
        value_bytes = struct.pack("<f", vel)   # 小端 float
        for i in range(4):
            data[4+i] = value_bytes[i]
        # CAN ID
        ID_code = (self.control_dict['SetInfo'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        self.send_message(ID_code, data)

        # Byte0~1: index，小端序
        data[:2]=self.int_to_bytes_array(0x7016,inverse=True)
        # Byte2~3: 固定为 00 00
        data[2], data[3] = 0x00, 0x00
        # Byte4~7: 参数数据 float → 4字节，小端序
        value_bytes = struct.pack("<f", pos)   # 小端 float
        for i in range(4):
            data[4+i] = value_bytes[i]
        # CAN ID
        ID_code = (self.control_dict['SetInfo'] << 24) | (self.control_dict['Server_ID'] << 8) | id
        self.send_message(ID_code, data)
        # 接收反馈
        message = self.recivice_message_once()
        for msg in message:
            print(self.decoder(msg))

        return None



t_list = []
angle_list = []
target_list = []
vel_list = []
target_vel_list = []
# 使用示例
if __name__ == "__main__":

    Motor1 = Motor('04', 1, True)
    Motor2 = Motor('04', 2, True)
    Motor3 = Motor('03', 3, False)
    Motor4 = Motor('00', 4, False)
    Motor5 = Motor('00', 5, False)
    Motor6 = Motor('00', 6, True)
    Motor7 = Motor('00', 7, True)


    comm = MotorControl(1)
    comm.opon_device()
    comm.clear_buffer()

    comm.addMotor(Motor1)
    comm.addMotor(Motor2)
    comm.addMotor(Motor3)
    comm.addMotor(Motor4)
    comm.addMotor(Motor5)
    comm.addMotor(Motor6)
    comm.addMotor(Motor7)

    
    # comm.Enable(1)
    # s=time.time()
    for i in range(1000):
        comm.Enable(1)
        comm.Enable(2)
        comm.Enable(3)
        comm.Enable(4)
        comm.Enable(5)
        comm.Enable(6)
        comm.Enable(7)
        # comm.Set_Zero(1)
        # comm.Set_Zero(2)
        # comm.Set_Zero(3)
        # comm.Set_Zero(4)
        # comm.Set_Zero(5)
        # comm.Set_Zero(6)
        # comm.Set_Zero(7)
        print(f"{Motor1.state_q},{Motor2.state_q},{Motor3.state_q},{Motor4.state_q},{Motor5.state_q},{Motor6.state_q},{Motor7.state_q}")
        time.sleep(0.1)
    # print(time.time()-s)

    
    # # #如果你的驱动需要显式切 MIT 协议，打开这行
    # # comm.ChangeType(master_id=0xfd, motor_id=motor_id, type_id=2)

    # comm.Enable(1)
    # comm.Enable(1)
    # comm.Enable(1)
    # comm.Enable(1)
    # q0 = Motor1.state_q
    # print(f"[INFO] Current position q0 = {q0:.4f} rad")
    # # ================== 正弦轨迹参数 ==================
    # amplitude = 0.3      # 正弦幅度 (rad)  小幅度很安全
    # freq = 0.5            # 频率 Hz
    # omega = 2 * math.pi * freq

    # kp = 3               # 偏软
    # kd = 0.08

    # duration = 2       # 运动时长 (s)
    # dt = 0.01             # 控制周期 10 ms
    # # ==================================================
    # t_start = time.time()

    # while True:
    #     t = time.time() - t_start
    #     if t > duration:
    #         break

    #     # 正弦叠加在当前位置
    #     # q_des = q0 + amplitude * math.sin(omega * t)
    #     # dq_des = amplitude * omega * math.cos(omega * t)
    #     # if dq_des > 0:
    #     #     torque = 0.4
    #     # else:
    #     #     torque = -0.4

    #     q_des = 0.2
    #     dq_des = 0
    #     torque = 0

    #     comm.MIT(
    #         1,
    #         angle=q_des,
    #         velocity=dq_des,
    #         torque=torque,
    #         kp=kp,
    #         kd=kd
    #     )
    #     t_list.append(t)
    #     angle_list.append(Motor1.state_q)
    #     target_list.append(q_des)
    #     vel_list.append(Motor1.state_dq)
    #     target_vel_list.append(dq_des)
    #     time.sleep(dt)

    # print("[INFO] Sin motion finished")
    # plt.figure(figsize=(10,5))

    # plt.subplot(2,1,1)
    # plt.plot(t_list, angle_list, label="Position0 (rad)")
    # plt.plot(t_list, target_list, label="Position1 (rad)")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Position (rad)")
    # plt.grid(True)
    # plt.legend()

    # plt.subplot(2,1,2)
    # plt.plot(t_list, vel_list, label="Vel0 (rad)")
    # plt.plot(t_list, target_vel_list, label="Vel1 (rad)")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Vel (rad/s)")
    # plt.grid(True)
    # plt.legend()

    # plt.tight_layout()
    # plt.show()

    comm.Disable(1)
    comm.Disable(2)
    comm.Disable(3)
    comm.Disable(4)
    comm.Disable(5)
    comm.Disable(6)
    comm.close_device()
