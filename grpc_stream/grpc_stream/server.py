import sys
import time
import grpc
from concurrent import futures
import robstride
# import trajectory
import Inv_Dyn
import robot_control_pb2
import robot_control_pb2_grpc
import threading

def command_generator(arm1_Motor_control,arm1_Motor_list):
    """
    主手控制循环
    每次 yield 一帧 7 关节
    """
    seq = 0
    while True:
        cmd = robot_control_pb2.MasterCommand(
            teleop_enable=True,
            control_mode=robot_control_pb2.POSITION,
            timestamp_ns=time.time_ns(),
            seq_id=seq,
        )

        # 示例：简单正弦运动
        arm1_theta_deg = [arm1_Motor_list[0].smooth_q, arm1_Motor_list[1].smooth_q, arm1_Motor_list[2].smooth_q, arm1_Motor_list[3].smooth_q,
                          arm1_Motor_list[4].smooth_q,arm1_Motor_list[5].smooth_q]      #六个关节对笛卡尔坐标系矫正后的角度
        arm1_theta_d_rad = [arm1_Motor_list[0].smooth_dq, arm1_Motor_list[1].smooth_dq, arm1_Motor_list[2].smooth_dq, arm1_Motor_list[3].smooth_dq,
                          arm1_Motor_list[4].smooth_dq,arm1_Motor_list[5].smooth_dq]     #六个关节的角速度
        arm1_theta_torque = [arm1_Motor_list[0].smooth_tau, arm1_Motor_list[1].smooth_tau, arm1_Motor_list[2].smooth_tau, arm1_Motor_list[3].smooth_tau,
                          arm1_Motor_list[4].smooth_tau,arm1_Motor_list[5].smooth_tau] 
        arm1_theta_dd_rad = [0,0,0,0,0,0]     #六个关节的角加速度
        arm1_torque=Inv_Dyn.Inv_Dyn2(arm1_theta_deg, arm1_theta_d_rad, arm1_theta_dd_rad)
        for idx in range(6):    #对主手各个关节下发控制指令
            arm1_Motor_control.MIT(idx+1,0,0,arm1_torque[idx],0,0)
        cmd.joints.position.extend(arm1_theta_deg+[arm1_Motor_list[6].smooth_q])
        cmd.joints.velocity.extend(arm1_theta_d_rad+[arm1_Motor_list[6].smooth_dq])
        cmd.joints.torque.extend(arm1_theta_torque+[arm1_Motor_list[6].smooth_tau])
        yield cmd
        seq += 1
        time.sleep(0.005)  # 100 Hz


def receive_thread(response_iterator):
    """
    接收从手状态
    """
    return None

def main():
    channel = grpc.insecure_channel("10.16.77.87:50051")
    stub = robot_control_pb2_grpc.RobotTeleopServiceStub(channel)
    arm1_Motor_Kp = [3,3,3,60,60,60]
    arm1_Motor_Kd = [0.08,0.08,0.08,4,4,4]
    arm1_Motor_1=robstride.Motor('03', 1, True)
    arm1_Motor_2=robstride.Motor('03', 2, False)
    arm1_Motor_3=robstride.Motor('06', 3, False)
    arm1_Motor_4=robstride.Motor('00', 4, False)
    arm1_Motor_5=robstride.Motor('00', 5, False)
    arm1_Motor_6=robstride.Motor('00', 6, True)
    arm1_Motor_7=robstride.Motor('05', 7, True)
    arm1_Motor_list=[arm1_Motor_1,arm1_Motor_2,arm1_Motor_3,arm1_Motor_4,arm1_Motor_5,arm1_Motor_6,arm1_Motor_7]
    arm1_Motor_control=robstride.MotorControl(0)
    arm1_Motor_control.opon_device()
    arm1_Motor_control.clear_buffer()
    for j,motor in enumerate(arm1_Motor_list):
        arm1_Motor_control.addMotor(motor)
        arm1_Motor_control.Enable(j+1)
        time.sleep(0.1)
    print('--------------')
    

    responses = stub.TeleopSession(command_generator(arm1_Motor_control,arm1_Motor_list))

    recv_thread = threading.Thread(
        target=receive_thread, args=(responses,), daemon=True
    )
    recv_thread.start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
