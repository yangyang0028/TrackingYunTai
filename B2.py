import sensor, image, time
from pyb import UART
import json


class PID:
    def __init__(self, limit, target, feedback, Kp, Ki, Kd, e_0, e_1, e_2):
        self.limit=limit
        self.target=target
        self.feedback=feedback
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.e_0=e_0
        self.e_1=e_1
        self.e_2=e_2

    def pid_calc(self):

        self.e_0 = self.target - self.feedback

        ep = self.e_0  - self.e_1
        ei = self.e_0
        ed = self.e_0 - 2*self.e_1 + self.e_2

        out = self.Kp*ep + self.Ki*ei + self.Kd*ed
        out = min(max(out, -self.limit), self.limit)
        self.e_2 = self.e_1
        self.e_1 = self.e_0
        return out



pid_x = PID(100, 0, 0,
    0.7, 0.3, 0,
    0, 0, 0)


pid_y = PID(100, 0, 0,
    -0.7, -0.3, 0,
    0, 0, 0)


sensor.reset() # 初始化摄像头
sensor.set_pixformat(sensor.RGB565) # 格式为 RGB565.
sensor.set_framesize(sensor.QQVGA) # 使用 QQVGA 速度快一些
sensor.set_auto_exposure(False,35000)
sensor.skip_frames(time = 2000) # 跳过2000s，使新设置生效,并自动调节白平衡
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)#关闭白平衡。白平衡是默认开启的，在颜色识别中，一定要关闭白平衡。
clock = time.clock() # 追踪帧率
uart = UART(3, 115200)

control_val_x=1350
control_val_y=1100

MIN_LIMIT_y=950
MAX_LIMIT_y=1250

MIN_LIMIT_x=1000
MAX_LIMIT_x=1700

ans_x=270
ans_y=219

aaa=28

while(True):

    quan=0;
    quan_id=-1

    img = sensor.snapshot(1.8) # 从感光芯片获得一张图像
    img.draw_rectangle([70,54,10,10])
    blobs = img.find_blobs([(32, 77, 23, 54, 2, 29)],pixels_threshold = 24,area_threshold = 5,merge = True)
    for b in blobs:
            if quan<b.pixels():
                quan=b.pixels()
                quan_id=b


    if quan > 0:
            x = quan_id[0]
            y = quan_id[1]
            width = quan_id[2]
            height = quan_id[3]
            img.draw_rectangle([x,y,width,height])
            img.draw_cross(quan_id[5], quan_id[6])

            pid_x.feedback=x
            pid_y.feedback=y

            pid_x.target=70
            pid_y.target=54

            control_val_x += int(pid_x.pid_calc())
            control_val_y += int(pid_y.pid_calc())

            control_val_x = max(min(control_val_x, MAX_LIMIT_x), MIN_LIMIT_x)
            control_val_y = max(min(control_val_y, MAX_LIMIT_y), MIN_LIMIT_y)

            print(control_val_x,control_val_y)
            uart.write(aaa.to_bytes(1,'int')+control_val_y.to_bytes(2,'int')+control_val_x.to_bytes(2,'int'))

'''
    for i in range(10):
        if control_val_y>1250 :
            control_val_y=950
        control_val_y+=1
        print(control_val_y)
        uart.write(aaa.to_bytes(1,'int')+control_val_y.to_bytes(2,'int')+control_val_x.to_bytes(2,'int'))
'''
