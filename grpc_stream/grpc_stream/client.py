import sys
import time
import grpc
from concurrent import futures
import robstride
# import trajectory
import Inv_Dyn
import Inv_Dyn_2
import robot_control_pb2
import robot_control_pb2_grpc


class RobotTeleopService(robot_control_pb2_grpc.RobotTeleopServiceServicer):
    def __init__(self):
        self.teleop_active = False
        self.last_cmd_time = time.time()
        self.arm2_Motor_Kp = [15,15,15,60,60,60]
        self.arm2_Motor_Kd = [0.2,0.2,0.2,4,4,4]
        self.arm2_Motor_1=robstride.Motor('04', 1, True)
        self.arm2_Motor_2=robstride.Motor('04', 2, True)
        self.arm2_Motor_3=robstride.Motor('03', 3, False)
        self.arm2_Motor_4=robstride.Motor('00', 4, False)
        self.arm2_Motor_5=robstride.Motor('00', 5, False)
        self.arm2_Motor_6=robstride.Motor('00', 6, True)
        self.arm2_Motor_list=[self.arm2_Motor_1,self.arm2_Motor_2,self.arm2_Motor_3,self.arm2_Motor_4,self.arm2_Motor_5,self.arm2_Motor_6]
        self.arm1_Motor_control=robstride.MotorControl(0)
        self.arm1_Motor_control.opon_device()
        self.arm1_Motor_control.clear_buffer()

        for j,motor in enumerate(self.arm2_Motor_list):
            self.arm1_Motor_control.addMotor(motor)
            self.arm1_Motor_control.Enable(j+1)
            self.arm1_Motor_control.Enable(j+1)
            self.arm1_Motor_control.Enable(j+1)
            self.arm1_Motor_control.Enable(j+1)
            time.sleep(0.1)
        

    def TeleopSession(self, request_iterator, context):
        """
        双向 stream:
        - request_iterator: 主手发来的 MasterCommand
        - yield SlaveState: 从手状态
        """
        print("Teleop session started")

        for cmd in request_iterator:
            self.last_cmd_time = time.time()

            # ===== 基本校验 =====
            if len(cmd.joints.position) != 7:
                print("Invalid joint size")
                continue

            self.teleop_active = cmd.teleop_enable

            # ===== 这里是你真正的控制逻辑 =====
            # 例如：
            q_des = list(cmd.joints.position)
            dq_des = list(cmd.joints.velocity)
            # send_to_robot_controller(q_des)
            arm1_theta_d_rad = [0,0,0,0,0,0]
            torque = Inv_Dyn_2.Inv_Dyn2(q_des[:6],arm1_theta_d_rad,arm1_theta_d_rad)
            for idx in range(6):
                if abs(self.arm2_Motor_list[idx].smooth_q - q_des[idx]) < 0.1:
                    self.arm1_Motor_control.MIT(idx+1,q_des[idx],dq_des[idx],torque[idx],self.arm2_Motor_Kp[idx],self.arm2_Motor_Kd[idx])
                    print(f"123")
                else:
                    angle = self.arm2_Motor_list[idx].smooth_q + (q_des[idx] - self.arm2_Motor_list[idx].smooth_q)*0.05
                    self.arm1_Motor_control.MIT(idx+1,angle,0,0,self.arm2_Motor_Kp[idx],self.arm2_Motor_Kd[idx])
                    print(f"456")

            

            # print(
            #     f"[Slave] seq={cmd.seq_id}, "
            #     f"enable={cmd.teleop_enable}, "
            #     f"q={q_des}"
            #     f"dq={dq_des}"
            #     f"tau={torque}"
            # )

            # ===== 构造反馈 =====
            state = robot_control_pb2.SlaveState(
                teleop_active=self.teleop_active,
                status=robot_control_pb2.OK,
                timestamp_ns=time.time_ns(),
            )

            # 示例：直接回传同样的关节（真实系统中是编码器值）
            state.joints.position.extend(q_des)
            state.joints.velocity.extend([0.0] * 7)
            state.joints.torque.extend([0.0] * 7)

            yield state

        print("Teleop session ended")


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    robot_control_pb2_grpc.add_RobotTeleopServiceServicer_to_server(
        RobotTeleopService(), server
    )

    server.add_insecure_port("[::]:50051")
    server.start()

    print("Slave server started on port 50051")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
