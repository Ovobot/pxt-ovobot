 # pxt-ovobot

A ovobot package for pxt-microbit

## 基本用法
```
//程序需要使用陀螺仪，超声波，巡线传感器，光敏传感器时，需要调用此初始化代码
//此代码执行需要3~4s的时间进行陀螺仪的校准，等待蜂鸣器滴的一声，初始化结束。
//此代码放置在
ovobot.initGyroSensor()

//点亮Bit机器人左右两个RGB三色灯
ovobot.setledGroupColor(Color.Red, Color.Red)
//熄灭Bit机器人左右两个RGB三色灯
ovobot.setledGroupColor(Color.Black, Color.Black)

//让Bit机器人向前或向后以某个速度运动一段时间（时间单位：ms，速度范围：0~255）
ovobot.move(MoveDir.Forward, 80, 3000)

//让Bit机器人停止运动
ovobot.stopMove()

//让Bit机器人以某个速度向前或向后一直运动（速度范围：0~255）
ovobot.moveAtSpeed(MoveDir.Forward, 80)

//让Bit机器人在某段时间内旋转一定的角度（时间单位：ms，速度范围：0~255）
ovobot.rotate(90, 1000)

//让Bit机器人以某个角速度一直旋转（速度范围：0~255）
ovobot.rotateAtSpeed(90)

//独立控制左右轮速度持续一段时间（时间单位：ms，速度范围：0~255）
ovobot.rawMotor(100, 100, 3000)

//独立控制左右轮速度
ovobot.rawMotorWithPwm(100, 100)

//获取与障碍物的距离 （长度单位：cm）
ovobot.readDistance()

//获取当前光照强度 （数值范围：0~255，数值越大，光照越强）
ovobot.readLightStrength()

//获取左右巡线传感器数值
ovobot.readLineSensorData(LineSensor.Left)
ovobot.readLineSensorData(LineSensor.Right)

```
## License

MIT

## Supported targets

* for PXT/microbit
(The metadata above is needed for package search.)

## 软件须知
使用本插件，需要购买Ovobot Bit 机器人。  
官方购买链接：[点击购买](https://www.ovobot.cn/zh-hans/catalogue/ovobot-bitzhi-neng-bian-cheng-ji-qi-ren-microbitzhi-neng-ji-qi-ren-tao-jian-stemjiao-yu-ji-qi-ren_4/)  
淘宝购买链接：[点击购买](https://item.taobao.com/item.htm?spm=a230r.1.14.1.1cca3b3cU2GvgV&id=584158203067&ns=1&abbucket=20#detail)

