#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # 設定序列埠連接 OpenCR
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=57600,
                timeout=1
            )
            self.get_logger().info('已連接到 OpenCR')
            time.sleep(2)  # 等待 OpenCR 初始化
            
            # 清空緩衝區
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
        except Exception as e:
            self.get_logger().error(f'無法連接 OpenCR: {e}')
            return
        
        # 訂閱 cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('麥克納姆輪控制器已啟動')
        self.get_logger().info('直接控制 setVelocity(vx, vy, omega)')
        self.get_logger().info('等待 /cmd_vel 指令...')
    
    def cmd_vel_callback(self, msg):
        """
        接收 Twist 訊息並發送到 OpenCR
        msg.linear.x: 前進/後退速度 (m/s)
        msg.linear.y: 左/右平移速度 (m/s)
        msg.angular.z: 旋轉速度 (rad/s)
        """
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        omega = float(msg.angular.z)
        
        # 發送速度到 OpenCR
        self.send_velocity(vx, vy, omega)
        
        self.get_logger().info(f'發送速度: vx={vx:.3f}, vy={vy:.3f}, ω={omega:.3f}')
    
    def send_velocity(self, vx, vy, omega):
        """
        將速度打包成二進制數據發送到 OpenCR
        格式: [起始位元 0xFF] [vx(4 bytes)] [vy(4 bytes)] [omega(4 bytes)]
        """
        try:
            # 打包數據: 起始位元 + 三個浮點數
            data = struct.pack('<Bfff', 0xFF, vx, vy, omega)
            
            # 發送到 OpenCR
            self.serial_port.write(data)
            
        except Exception as e:
            self.get_logger().error(f'發送速度失敗: {e}')
    
    def __del__(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            # 停止所有馬達
            self.send_velocity(0.0, 0.0, 0.0)
            time.sleep(0.1)
            self.serial_port.close()
            self.get_logger().info('已關閉序列埠')

def main(args=None):
    rclpy.init(args=args)
    controller = MecanumController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
