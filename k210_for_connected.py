# 检测黑框&绿点&红点&uart
#

import sensor, image, time, lcd

lcd.init()                          # Init lcd display
from machine import UART
from fpioa_manager import fm

red = (85, 100,-10,127,-18,127)#(76, 100, 30, 127, -128, 127)#(45, 100, 20, 127, 5, 127) #(87, 100, 30, 127, -128, 127)
red_black = (85, 100,-10,127,0,127)#(0,73,15,127,-18,127)#(40, 100, 30, 127, -128, 127)#(38, 80, 18, 26, -128, 127)#(30, 127, 15, 127, 15, 127)
green = (49, 100, -24, -5, -128, 127)
green_black = (22, 100, -128, -7, -128, 127)

roi_a = [10,10,160,110]#[40,10,140,60]
write_bytes = '#'
write_bytes1 = ''
write_bytes2 = ''
last_time = time.ticks_ms()

sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 灰度更快
sensor.set_framesize(sensor.QQVGA)
#sensor.set_auto_gain(False) # 颜色跟踪必须关闭自动增益
#sensor.set_auto_whitebal(False) # 颜色跟踪必须关闭白平衡
#sensor.set_brightness(5000)         # 设置亮度为3000
sensor.skip_frames(time = 2000)
clock = time.clock()

# binding UART2 IO:6->RX, 8->TX
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)

yb_uart = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

#try:
while(True):
    clock.tick()
    img = sensor.snapshot()



    # 下面的`threshold`应设置为足够高的值，以滤除在图像中检测到的具有
    # 低边缘幅度的噪声矩形。最适用与背景形成鲜明对比的矩形。

    for r in img.find_rects(threshold = 10000):
        #img.draw_rectangle(r.rect(), color = (0, 0, 0))
        i = 0

        write_bytes = '#'
        for p in r.corners():
          i = (i+1)%4
          print(p[0]),print(p[1])
          write_bytes = write_bytes + "r" + str(i)+ "x:"+ str(p[0]) + "y:" + str(p[1])

        for p in r.corners():
          i = (i+1)%4
          print(p[0]),print(p[1])
          write_bytes = write_bytes + "r" + str(i)+ "x:"+ str(p[0]) + "y:" + str(p[1])

        #write_bytes = write_bytes + "\n"
        print(write_bytes)
        #blob_green = img.find_blobs([green,green_black], roi=r.rect(), area_threshold=5, merge=True)


    blob_red = img.find_blobs([red,red_black], area_threshold=10,merge=True ,margin=30)

    if blob_red:                                            #如果找到了目标颜色
        for b in blob_red:
        #迭代找到的目标颜色区域
            img.draw_cross(b[5], b[6], color = (0, 0, 0))                  #画十字 cx,cy
            #print(b[5]) #返回色块的外框的中心x坐标（int），也可以通过blob[5]来获取。
            #print(b[6]) #返回色块的外框的中心y坐标（int），也可以通过blob[6]来获取。
            write_bytes2 = write_bytes +  "b_rx:" + str(b[5]) + "y:" + str(b[6])+ "END\n"
            print(write_bytes2)

    blob_green = img.find_blobs([green,green_black],area_threshold=5, merge=True)



    if blob_green:                                            #如果找到了目标颜色
        for d in blob_green:
        #迭代找到的目标颜色区域
            #img.draw_cross(d[5], d[6], color = (255, 255, 255))                  #画十字 cx,cy
            #print(d[5]) #返回色块的外框的中心x坐标（int），也可以通过blob[5]来获取。
            #print(d[6]) #返回色块的外框的中心y坐标（int），也可以通过blob[6]来获取。
            write_bytes1 = write_bytes + "b_gx:" + str(d[5]) + "y:" + str(d[6])+ "END\n"
            #print(write_bytes1)


    lcd.display(img)                # Display image on lcd.

    if time.ticks_ms() - last_time > 500:
         last_time = time.ticks_ms()
         #yb_uart.write(write_bytes)
         #yb_uart.write(write_bytes1)
         yb_uart.write(write_bytes2)



    #print("FPS %f" % clock.fps())
#except:
#    pass

yb_uart.deinit()
del yb_uart
