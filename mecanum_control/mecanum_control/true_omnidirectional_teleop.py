#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import math

class TrueOmnidirectionalTeleop(Node):
    def __init__(self):
        super().__init__('true_omnidirectional_teleop')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 速度設定
        self.linear_speed = 0.3
        self.angular_speed = 1.0
        self.speed_step = 0.05
        
        # 當前移動狀態
        self.current_angle = 0.0  # 移動方向角度（度）
        self.angle_step = 15.0    # 每次調整角度（度）
        self.is_moving = False
        self.is_rotating = False
        self.rotation_direction = 0  # 1=逆時針, -1=順時針
        
        self.get_logger().info('=== 360度全方位移動控制 ===')
        self.print_instructions()
        
        # 儲存終端機設定
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 建立定時器持續發送命令
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def print_instructions(self):
        msg = """
========================================
        360度全方位移動控制
========================================

移動方向控制:
---------------------------
數字鍵 1-8: 選擇8個方向
  7(315°)  8(0°)   9(45°)
     ↖      ↑      ↗
  4(270°)  5(停)  6(90°)
     ←      ●      →
  1(225°)  2(180°) 3(135°)
     ↙      ↓      ↗

方向微調:
---------------------------
, (逗號) : 逆時針調整方向 -15°
. (句號) : 順時針調整方向 +15°

旋轉控制:
---------------------------
q : 逆時針旋轉
e : 順時針旋轉

組合移動:
---------------------------
移動中按 q/e : 移動+旋轉組合

速度調整:
---------------------------
w : 加速 (+0.05 m/s)
s : 減速 (-0.05 m/s)
+ : 增加角速度
- : 減少角速度

其他:
---------------------------
space/x : 立即停止
c : 顯示當前狀態
h : 顯示說明
Ctrl+C : 退出

========================================
"""
        print(msg)
        self.print_status()
    
    def print_status(self):
        """顯示當前狀態"""
        print(f'\n當前狀態:')
        print(f'  移動方向: {self.current_angle:.0f}° ({self.angle_to_direction(self.current_angle)})')
        print(f'  線速度: {self.linear_speed:.2f} m/s')
        print(f'  角速度: {self.angular_speed:.2f} rad/s')
        print(f'  移動中: {"是" if self.is_moving else "否"}')
        print(f'  旋轉中: {"是" if self.is_rotating else "否"}')
        print()
    
    def angle_to_direction(self, angle):
        """將角度轉換為方向描述"""
        angle = angle % 360
        directions = [
            (0, "正前方 ↑"),
            (45, "右前方 ↗"),
            (90, "正右方 →"),
            (135, "右後方 ↘"),
            (180, "正後方 ↓"),
            (225, "左後方 ↙"),
            (270, "正左方 ←"),
            (315, "左前方 ↖")
        ]
        
        for deg, desc in directions:
            if abs(angle - deg) < 22.5:
                return desc
        return f"{angle:.0f}°"
    
    def get_key(self):
        """讀取單個按鍵"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def calculate_velocity(self):
        """根據當前狀態計算速度"""
        if not self.is_moving and not self.is_rotating:
            return 0.0, 0.0, 0.0
        
        vx = 0.0
        vy = 0.0
        omega = 0.0
        
        if self.is_moving:
            # 將角度轉換為弧度
            angle_rad = math.radians(self.current_angle)
            
            # 計算 x, y 方向的速度分量
            # 0° = 前進 (vx+), 90° = 右移 (vy-), 180° = 後退 (vx-), 270° = 左移 (vy+)
            vx = self.linear_speed * math.cos(angle_rad)
            vy = -self.linear_speed * math.sin(angle_rad)  # 負號因為ROS座標系
        
        if self.is_rotating:
            omega = self.rotation_direction * self.angular_speed
        
        return vx, vy, omega
    
    def timer_callback(self):
        """定時發送速度指令"""
        vx, vy, omega = self.calculate_velocity()
        
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.publisher.publish(twist)
    
    def run(self):
        """主控制迴圈"""
        try:
            while True:
                key = self.get_key()
                
                # 數字鍵 - 8方向移動
                if key == '8':
                    self.current_angle = 0.0
                    self.is_moving = True
                    print(f'向前移動 (0°)')
                elif key == '9':
                    self.current_angle = 45.0
                    self.is_moving = True
                    print(f'右前移動 (45°)')
                elif key == '6':
                    self.current_angle = 90.0
                    self.is_moving = True
                    print(f'向右移動 (90°)')
                elif key == '3':
                    self.current_angle = 135.0
                    self.is_moving = True
                    print(f'右後移動 (135°)')
                elif key == '2':
                    self.current_angle = 180.0
                    self.is_moving = True
                    print(f'向後移動 (180°)')
                elif key == '1':
                    self.current_angle = 225.0
                    self.is_moving = True
                    print(f'左後移動 (225°)')
                elif key == '4':
                    self.current_angle = 270.0
                    self.is_moving = True
                    print(f'向左移動 (270°)')
                elif key == '7':
                    self.current_angle = 315.0
                    self.is_moving = True
                    print(f'左前移動 (315°)')
                elif key == '5':
                    self.is_moving = False
                    print('停止移動')
                
                # 方向微調
                elif key == ',':
                    self.current_angle = (self.current_angle - self.angle_step) % 360
                    print(f'方向調整至: {self.current_angle:.0f}° ({self.angle_to_direction(self.current_angle)})')
                elif key == '.':
                    self.current_angle = (self.current_angle + self.angle_step) % 360
                    print(f'方向調整至: {self.current_angle:.0f}° ({self.angle_to_direction(self.current_angle)})')
                
                # 旋轉控制
                elif key == 'q':
                    self.is_rotating = True
                    self.rotation_direction = 1
                    print('開始逆時針旋轉')
                elif key == 'e':
                    self.is_rotating = True
                    self.rotation_direction = -1
                    print('開始順時針旋轉')
                
                # 速度調整
                elif key == 'w':
                    self.linear_speed += self.speed_step
                    print(f'線速度增加至: {self.linear_speed:.2f} m/s')
                elif key == 's':
                    self.linear_speed = max(0.0, self.linear_speed - self.speed_step)
                    print(f'線速度減少至: {self.linear_speed:.2f} m/s')
                elif key == '+' or key == '=':
                    self.angular_speed += 0.1
                    print(f'角速度增加至: {self.angular_speed:.2f} rad/s')
                elif key == '-' or key == '_':
                    self.angular_speed = max(0.0, self.angular_speed - 0.1)
                    print(f'角速度減少至: {self.angular_speed:.2f} rad/s')
                
                # 停止
                elif key == ' ' or key == 'x':
                    self.is_moving = False
                    self.is_rotating = False
                    print('完全停止')
                
                # 顯示狀態
                elif key == 'c':
                    self.print_status()
                
                # 顯示說明
                elif key == 'h':
                    self.print_instructions()
                
                # 退出
                elif key == '\x03':  # Ctrl+C
                    break
                
        except Exception as e:
            self.get_logger().error(f'錯誤: {e}')
        
        finally:
            # 停止機器人
            self.is_moving = False
            self.is_rotating = False
            twist = Twist()
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = TrueOmnidirectionalTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
