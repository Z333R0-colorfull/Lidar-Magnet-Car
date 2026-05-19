# Lidar-Magnet-Car
Gcode based on insani-bot
一个超低成本激光雷达小车<br>
能够完成全向运动,雷达解算以及如果小车足够慢可以建图,磁传感器的读取和滤波功能<br>
成本:x2激光雷达35.8 (二手拆机件)<br>
&emsp; 3D打印及标准结构件(42个2*18mm钢销 9个3*20mm螺钉 6个3*30mm螺钉)约12元(不过前提是你或你朋友有打印机)<br>
&emsp; esp32c3主控8元 <br>
&emsp; l298n*2共12元 <br>
&emsp; 母对母杜邦线<40个记作3元  <br>
&emsp; 三个tt马达6元<br>
&emsp; 电池15元<br>
&emsp; 磁力计20元,如果焊接技术过硬参考pcb文件夹下的文件可以降到3元以下(磁力计本体才1.2元 我也不知道为什么成品pcb这么贵)<br>
&emsp; 磁汇聚器7.5元(如果你能捡到没人要的退火坡莫合金或者自己有坡莫合金会便宜很多,磁汇聚只看形状不看大小)<br>

需要注意的是必须调平重心否则运动极其不稳定,有条件的话上个编码器和陀螺仪可以显著降低漂移,再用热缩管把滚子套一层胶提高抓地力会好很多<br>
<br>
文件结构{由于传输有点问题第一级文件夹可以忽略}<br>
<br>
=========[结构件Gcode]====BAse17.12g1pcs.gcode 下底盘<br>
&emsp;&emsp;|&emsp;&emsp;|<br>
&emsp;&emsp;|&emsp;&emsp;|==LidarBASE14.12g1pcs.gcode 上底盘<br>
&emsp;|&emsp;&emsp;|<br>
&emsp;|&emsp;&emsp;|==tambo17.86g42pcs.gcode 滚子打印件<br>
&emsp;|&emsp;&emsp;|<br>
&emsp;|&emsp;&emsp;|==TTmotorAdapter13.87G9pcs.gcode 联轴器<br>
&emsp;|&emsp;&emsp;|<br>
&emsp;|&emsp;&emsp;|==WheelBAse33.88g6pcs.gcode  轮毂打印件<br>
&emsp;|<br>
&emsp;|[历史代码]====[nrf24l01小车]<br>
&emsp;|&emsp;|<br>
&emsp;|&emsp;|===[基站]<br>
&emsp;|<br>
&emsp;|[上位机代码]====MoveCtrl.py 运动控制与建图<br>
&emsp;|&emsp;|<br>
&emsp;|&emsp;|===5603.py 磁力计数据处理<br>
&emsp;|<br>
&emsp;|[小车代码]====proj3_rubbish.ino !?史?!<br>
