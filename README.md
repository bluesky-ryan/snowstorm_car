# Snow Storm-Car（暴风雪战车）

## 一、介绍

Snow Storm car（暴风雪战车）是受邀国产RT-Thread操作系统举办的智能车比赛背景下，在RT-Thread系统上开发的一款学习智能车。主要是目的是学习了解麦克纳姆轮小车的控制、运动、建模、等相关知识，掌握智能车的控制开发。

## 二、原材料

#### 1.主控板

初次探索智能车本着节约成本和最低风险的原则，我们选用淘宝成品主控板（主控芯片STM32F103RCT6）

淘宝链接（鉴于老板比较热心我们为他打波广告）：

[]: https://item.taobao.com/item.htm?spm=a1z09.2.0.0.76402e8dPZHRxq&id=578738554719&_u=59ib8vj6170

图样：

![control](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/control.png)

#### 2.底座

麦克纳姆轮底座，某宝多的是自行选购

样图：

![car](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/car.png)

#### 3.电机

买底座基本都带电机，我们选用带AB编码器的1：30减速电机

样图：

![motor](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/motor.png)

#### 4.遥控

普通SONY PS2遥控30-40块钱

样图：

![ps2](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/ps2.png)

#### 5.电池

选用3S 11V航模电池

样图：

![batter](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/batter.png)



## 三、用到知识点

1.RT-Thread bsp移植。

2.STM32-CubeMXs使用。

3.RTOS使用。

4.PID控制理论。

5.麦克拉姆拉控制理论。

6.简单运动模型。

## 四、样机展示

1.样机实现了上、下、左、右、斜角平移、旋转等功能。

样图：

![snowstorm](https://github.com/bluesky-ryan/snowstorm_car/blob/master/Image/snowstorm.png)



## 五、期待

小车目前只实现了基础功能离智能还很遥远，我们还需学习新知识知识，为小车添砖加瓦。期待和大家一起打造一款属于我们自己的智能车。

​																														                                路漫漫其修远兮，吾将上下而求索---屈原

## 六、感谢

非常感谢RT-Thread举办的这次智能车比赛，让我们有机会和大家一起学习智能车知识，也很感谢群里们小伙伴们不厌其烦的讲解和帮助。一起学习的过程中认识了很多优秀热情的小伙伴，见识到了很多优秀的作品，感触良多受益匪浅，谢谢大家。
