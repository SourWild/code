from machine import *
from smartcar import *
from seekfree import *
import gc
import time

#定义外设
ccd = TSL1401(1)
wireless = WIRELESS_UART(460800)
ticker_flag = False
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")
#舵机相关
angle = 104
pwm_servo_hz = 300
def duty_angle (freq, angle):
    return (65535.0 / (1000.0 / freq) * (0.5 + angle / 90.0))
duty=int(duty_angle(pwm_servo_hz,angle))
pwm_servo = PWM("C20", pwm_servo_hz, duty_u16 = duty)

# 回调函数
def time_pit_handler(time):
    global ticker_flag
    ticker_flag = True
    
#时钟
pit1 = ticker(1)# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多 四个
pit1.capture_list(ccd,encoder_l, encoder_r)# 关联采集接口 最少一个 最多八个

pit1.callback(time_pit_handler)
pit1.start(20)

#边界获取
def get_edge(ccd_data,edge):
    #print(ccd_data)
    checkw=4 #对比的像素宽度
    threshold=90 #对比度阈值
    edges=[edge[0],edge[1],0,0] #存放检测结果
    minl=10 #左边界最小值，找到的边界小于该值无效
    maxr=117 #右边界最大值，找到的边界大于该值无效
    mid=int((edge[1]+edge[0])/2) #当前小车位置
    #左
    for i in range(0,mid-checkw):
        a=ccd_data[mid-i]
        b=ccd_data[mid-i-checkw]
        #contrast=int(abs(a-b)*100/(a+b+1))
        contrast=abs(a-b)
        if(contrast>threshold):
            e=int((mid-i)-checkw/2)
            if(e>minl):
                edges[0]=e
                edges[2]=1
                break
    #右
    for i in range(mid,127-checkw):
        a=ccd_data[i]
        b=ccd_data[i+checkw]
        contrast=abs(a-b)
        if(contrast>threshold):
            e=int(i+checkw/2)
            if(e<maxr):
                edges[1]=e
                edges[3]=1
                break
    #print(edges)
    return edges

#代表左右边界以及左右边界数据是否改变，若没改变则代表丢线
edge1=[0,127,0,0]
edge2=[0,127,0,0]
wish_mid=127/2
weight=[1,0]#两个ccd的权重
P1=0.8#一次比例项系数
P2=0.05#二次比例项系数
D=0.7#微分项系数

servo_mid=104 #打直
servo_left=116 #左打死
servo_right=88 #右打死
err_last=0.0 #记录上一个误差值
straight_flag=0#直道标记
motor_duty=1500#电机占空比
while True:
    if (ticker_flag):
        # 获取ccd数据，搜边界
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        #print(ccd_data1)
        edge1=get_edge(ccd_data1,edge1)
        edge2=get_edge(ccd_data2,edge2)
        print(edge1,edge2)
        #wireless.send_ccd_image(WIRELESS_UART.CCD1_BUFFER_INDEX)
        wireless.send_ccd_image(WIRELESS_UART.CCD2_BUFFER_INDEX)
        
        # 计算误差和舵机目标角度
        mid1=(edge1[0]+edge1[1])/2
        mid2=(edge2[0]+edge2[1])/2
        print(mid1,mid2)
        err1=wish_mid-mid1
        output=P1*err1+D*(err1-err_last)+P2*err1*abs(err1)
        err_last=err1
        if(output>servo_left-servo_mid):
            output=servo_left-servo_mid
        if(output<servo_right-servo_mid):
            output=servo_right-servo_mid
        #控制舵机
        duty = int(duty_angle(pwm_servo_hz, servo_mid+output))
        pwm_servo.duty_u16(duty)
        
#         #速度控制
#         err2=wish_mid-mid2
#         err=weight[0]*(err1)+weight[1]*(err2)
#         #两个ccd都能搜到两条边界且远端误差很小，则认为是直道
#         if(edge1[2]==1 and edge1[3]==1 and edge2[2]==1 and edge2[3]==1 and err2>=-5 and err2<=5):
#             straight_flag=1
#         if(straight_flag):
#             motor_duty+=1000
#         else:
#             motor_duty-=1000
         
        motor_l.duty(1500)
        motor_r.duty(1500)
#         
#         #速度控制
#         encl_data = encoder_l.get()
#         encr_data = encoder_r.get()
#         print("enc ={:>6d}, {:>6d}\r\n".format(encl_data, encr_data))
        
        ticker_flag = False

    gc.collect()
