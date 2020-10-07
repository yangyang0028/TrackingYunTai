import sensor, image, time
from pyb import UART
import json

class PID: #增量式PID
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

class Kalman(object):
    def __init__(self, LastP, Now_P, out, Kg, Q, R):
        self.LastP = LastP
        self.Now_P = Now_P
        self.out = out
        self.Kg = Kg
        self.Q = Q
        self.R = R
    def kalmanFilter(self, input):
        #预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
        self.Now_P = self.LastP + self.Q
        #卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
        self.Kg = self.Now_P / (self.Now_P + self.R)
        #更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
        self.out = self.out + self.Kg * (input -self.out)#因为这一次的预测值就是上一次的输出值
        #更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
        self.LastP = (1-self.Kg) * self.Now_P
        return self.out


KfpX= Kalman(0.2,0,0,0,1,0.543)

KfpY= Kalman(0.2,0,0,0,1,0.543)

pid_x = PID(100, 0, 0,
    0.7, 0.2, 0.6,
    0, 0, 0)


pid_y = PID(100, 0, 0,
    -0.7, -0.2, 0.6,
    0, 0, 0)


sensor.reset() # 初始化摄像头
sensor.set_pixformat(sensor.RGB565) # 格式为 RGB565.
sensor.set_framesize(sensor.QVGA) # 使用 QQVGA 速度快一些
sensor.set_auto_exposure(False,35000) # 设置曝光时间
sensor.skip_frames(time = 2000) # 跳过2000s，使新设置生效,并自动调节白平衡
sensor.set_auto_gain(False)# 关闭自动增益
sensor.set_auto_whitebal(False)#关闭白平衡。白平衡是默认开启的，在颜色识别中，一定要关闭白平衡。
uart = UART(3, 115200)

control_val_x=1350#云台初始x位置
control_val_y=1100#云台初始y位置

MIN_LIMIT_x=1000#云台x最小值
MAX_LIMIT_x=1700#云台x最大值

MIN_LIMIT_y=950#云台y最小值
MAX_LIMIT_y=1250#云台y最大值

ans_x=146#弹道落点在图片上的x坐标
ans_y=129#弹道落点在图片上的y坐标

Roi=[0,0,320,240]

aaa=28#串口标志位

AX=0
AY=0
while(True):

    quan=0;#选取阈值后最大的目标区域
    quan_id=-1#选取阈值后最大的目标区域ID


    img = sensor.snapshot(1.8) # 从感光芯片获得一张图像
    blobs = img.find_blobs([(32, 77, 23, 54, 2, 29)],roi=Roi,pixels_threshold = 24,area_threshold = 5,merge = True)
    Roi=[0,0,320,240]
    img.draw_rectangle([ans_x-5,ans_y-5,10,10])
    for b in blobs:
        if quan<b.pixels():
            quan=b.pixels()
            quan_id=b
    if quan > 0:
        x = quan_id[0]
        y = quan_id[1]
        width = quan_id[2]
        height = quan_id[3]

        AX=int(KfpX.kalmanFilter(quan_id[5]))
        AY=int(KfpY.kalmanFilter(quan_id[6]))
        Roi=[quan_id[0]-30,quan_id[1]-30,quan_id[2]+60,quan_id[3]+60]
        img.draw_rectangle(Roi)
        img.draw_rectangle([x,y,width,height])
        img.draw_cross(AX, AY)

        #print(AX,AY)

        pid_x.feedback=AX
        pid_y.feedback=AY

        pid_x.target=ans_x
        pid_y.target=ans_y

        control_val_x += int(pid_x.pid_calc())
        control_val_y += int(pid_y.pid_calc())

        control_val_x = max(min(control_val_x, MAX_LIMIT_x), MIN_LIMIT_x)
        control_val_y = max(min(control_val_y, MAX_LIMIT_y), MIN_LIMIT_y)

        #print(control_val_x,control_val_y)
        uart.write(aaa.to_bytes(1,'int')+control_val_y.to_bytes(2,'int')+control_val_x.to_bytes(2,'int'))
