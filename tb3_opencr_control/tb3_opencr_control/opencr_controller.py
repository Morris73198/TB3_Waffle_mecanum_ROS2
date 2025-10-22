#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

class OpenCRController(Node):
    def __init__(self):
        super().__init__('opencr_controller')
        
        self.serial_port = None
        self.is_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        
        # 初始化串口連接
        self.connect_serial()
        
        # 訂閱cmd_vel話題
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription
        
        # 創建定時器檢查連接狀態
        self.timer = self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info('OpenCR控制節點已啟動')
        
    def connect_serial(self):
        """建立串口連接"""
        serial_devices = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
        
        for device in serial_devices:
            try:
                self.get_logger().info(f'嘗試連接到 {device}...')
                
                # 關閉舊連接
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                    time.sleep(0.5)
                
                # 建立新連接
                self.serial_port = serial.Serial(
                    device, 
                    57600, 
                    timeout=2,
                    write_timeout=2,
                    dsrdtr=False,
                    rtscts=False
                )
                
                time.sleep(3)  # 等待OpenCR初始化
                
                # 測試連接
                if self.test_serial_connection():
                    self.is_connected = True
                    self.reconnect_attempts = 0
                    self.get_logger().info(f'OpenCR串口連接成功: {device}')
                    return True
                    
            except Exception as e:
                self.get_logger().warn(f'連接到 {device} 失敗: {e}')
                continue
                
        self.is_connected = False
        self.get_logger().error('無法連接到OpenCR')
        return False
    
    def test_serial_connection(self):
        """測試串口連接"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
            
        try:
            # 清空緩衝區
            self.serial_port.flush()
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # 發送停止命令測試
            self.serial_port.write(b'x')
            time.sleep(0.1)
            
            return True
        except Exception as e:
            self.get_logger().error(f'串口測試失敗: {e}')
            return False
    
    def check_connection(self):
        """定期檢查連接狀態"""
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('串口連接斷開，嘗試重新連接...')
            self.connect_serial()
    
    def cmd_vel_callback(self, msg):
        """處理Twist消息"""
        if not self.is_connected or not self.serial_port:
            self.get_logger().warn('OpenCR未連接，忽略命令')
            return
            
        linear_x = msg.linear.x
        linear_y = msg.linear.y  
        angular_z = msg.angular.z
        
        command = self.twist_to_opencr_command(linear_x, linear_y, angular_z)
        
        if command:
            success = self.send_command(command)
            if success:
                self.get_logger().info(f'發送命令: {command}')
            else:
                self.get_logger().error(f'命令發送失敗: {command}')
    
    def send_command(self, command, retry_count=3):
        """安全地發送命令到OpenCR"""
        for attempt in range(retry_count):
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write(command.encode())
                    self.serial_port.flush()
                    return True
                else:
                    self.get_logger().warn('串口未開啟，嘗試重新連接...')
                    if not self.connect_serial():
                        return False
                        
            except serial.SerialException as e:
                self.get_logger().error(f'串口錯誤 (嘗試 {attempt+1}/{retry_count}): {e}')
                self.is_connected = False
                
                if attempt < retry_count - 1:
                    time.sleep(0.5)
                    # 嘗試重新連接
                    self.connect_serial()
                    
            except Exception as e:
                self.get_logger().error(f'未預期錯誤: {e}')
                return False
                
        return False
    
    def twist_to_opencr_command(self, x, y, z):
        """將速度指令轉換為OpenCR字符命令"""
        # 設定死區，避免小幅度顫動
        if abs(x) < 0.1 and abs(y) < 0.1 and abs(z) < 0.1:
            return 'x'  # 停止
            
        if abs(z) > 0.2:  # 旋轉優先
            if z > 0:
                return 'q'  # 逆時針
            else:
                return 'e'  # 順時針
        elif abs(x) > abs(y):  # 前後移動
            if x > 0:
                return 'w'  # 前進
            else:
                return 's'  # 後退
        else:  # 左右移動
            if y > 0:
                return 'a'  # 左移
            else:
                return 'd'  # 右移
    
    def __del__(self):
        """清理資源"""
        if hasattr(self, 'serial_port') and self.serial_port:
            try:
                self.serial_port.write(b'x')  # 停止所有馬達
                self.serial_port.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    controller = OpenCRController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('收到中斷信號，正在關閉...')
    except Exception as e:
        controller.get_logger().error(f'節點運行錯誤: {e}')
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
