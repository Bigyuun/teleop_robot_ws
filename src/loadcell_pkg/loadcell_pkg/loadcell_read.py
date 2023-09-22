## ROS2
import rclpy
from rclpy.node import Node

# from std_msgs.msg import Float32MultiArray
from custom_interfaces.msg import LoadcellState

## Serial Comm
import serial
import threading
from parse import *

class LoadcellPublisher(Node):

    def __init__(self):
        super().__init__('loadcell_publisher')
        qos_depth = 10
        # self.loadcell_publisher_ = self.create_publisher(Float32MultiArray, 'loadcell_data', qos_depth)
        self.loadcell_publisher_ = self.create_publisher(LoadcellState, 'loadcell_data', qos_depth)
        print('loadcell_publisher is created!')

        # serial comm parameters
        self.serial_port = '/dev/ttyUSB0'  # 아두이노와 연결된 포트 이름으로 변경해야 합니다.
        self.baud_rate = 115200  # 아두이노와 동일한 전송 속도로 설정        
        self.loadcell_data= [0,0,0,0,0,0,0,0,0,0] # buffer
        self.loadcell_threshold = [0,0,0,0,0,0,0,0,0,0] # buffer

        ## initialize the threshold
        self.loadcell_threshold[0] = 1000
        self.loadcell_threshold[1] = 1000
        self.loadcell_threshold[2] = 1000
        self.loadcell_threshold[3] = 1000
        self.loadcell_threshold[4] = 1000
        self.loadcell_threshold[5] = 1000
        self.loadcell_threshold[6] = 1000
        self.loadcell_threshold[7] = 1000
        self.loadcell_threshold[8] = 1000
        self.loadcell_threshold[9] = 1000
        
        self.loadcell_pub_thread_ = threading.Thread(target=self.read_thread)
        self.loadcell_pub_thread_.start()

    def read_thread(self):
        # spin

        # 시리얼 통신 시작
        # while True:
        #     self.publishall()
        ser = serial.Serial(self.serial_port, self.baud_rate)
        print(f"Connected to {self.serial_port}")

        try:
            while True:
                # 시리얼 데이터 수신
                received_data = ser.readline().decode().strip().replace('\x00','')  # 수신한 데이터를 문자열로 변환
                #print("Received:", received_data)  # 수신한 데이터 출력
                if received_data:
                    parse_received_data = received_data.split(',')
                    #print("Parsed:", parse_received_data)  # 파싱된 데이터 출력
                    
                    for i, value in enumerate(parse_received_data):
                        self.loadcell_data[i] = value
                        # print(self.loadcell_data[i])
                
                self.publishall()
                
                
        except KeyboardInterrupt:
            print("Keyboard Interrupt: Exiting...")
        finally:
            ser.close()  # 시리얼 통신 종료
        
        self.loadcell_pub_thread_.join()
        

    def publishall(self):
        # msg = Float32MultiArray()
        msg = LoadcellState()
        msg.threshold = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        msg.data = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        for i in range(10):
            msg.threshold[i] = self.loadcell_threshold[i]
            msg.data[i] = float(self.loadcell_data[i])
        self.loadcell_publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)

    loadcell_publisher = LoadcellPublisher()
    rclpy.spin(loadcell_publisher)

    loadcell_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()