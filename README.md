<span style="color:#FF0000">#</span> <span style="color:#00FF00">Lidar</span><span style="color:#0000FF">-</span><span style="color:#FF00FF">Magnet</span><span style="color:#FFFF00">-</span><span style="color:#00FFFF">Car</span>

<span style="color:#FF4500">> 基于 insani-bot 的 Gcode 方案</span>  
<span style="color:#2E8B57">> **超低成本激光雷达小车**</span>  
<span style="color:#8A2BE2">> 实现全向运动、</span><span style="color:#FF1493">雷达解算、</span><span style="color:#00CED1">低速建图、</span><span style="color:#FFD700">磁传感器读取与滤波</span>

---

<span style="color:#DC143C">## 🧾 成本明细（参考）</span>

| <span style="color:#FF8C00">组件</span>               | <span style="color:#9932CC">数量</span> | <span style="color:#20B2AA">单价（元）</span> | <span style="color:#FF6347">总价（元）</span> | <span style="color:#4169E1">备注</span> |
|-------------------|------|------------|------------|------|
| <span style="color:#B22222">激光雷达（二手拆机）</span>| <span style="color:#FF69B4">2</span>    | <span style="color:#32CD32">17.9</span>       | <span style="color:#FFD700">35.8</span>       | <span style="color:#9400D3"> </span> |
| <span style="color:#CD5C5C">3D打印及标准结构件</span> | <span style="color:#7B68EE">1套</span>  | <span style="color:#3CB371">12</span>         | <span style="color:#FF4500">12</span>         | <span style="color:#DB7093">需自备打印机或朋友帮助</span> |
| <span style="color:#F08080">ESP32-C3 主控</span>      | <span style="color:#66CDAA">1</span>    | <span style="color:#EE82EE">8</span>          | <span style="color:#00FA9A">8</span>          | <span style="color:#FFB6C1"> </span> |
| <span style="color:#DA70D6">L298N 驱动板</span>       | <span style="color:#FFA07A">2</span>    | <span style="color:#87CEEB">6</span>          | <span style="color:#FFDAB9">12</span>         | <span style="color:#B0E0E6"> </span> |
| <span style="color:#FF69B4">母对母杜邦线</span>       | <span style="color:#CD5C5C">40根</span> | <span style="color:#00CED1">≈0.075</span>     | <span style="color:#FF8C00">3</span>          | <span style="color:#40E0D0"> </span> |
| <span style="color:#20B2AA">TT 马达</span>            | <span style="color:#FF4500">3</span>    | <span style="color:#9932CC">2</span>          | <span style="color:#DC143C">6</span>          | <span style="color:#7FFF00"> </span> |
| <span style="color:#B8860B">电池</span>               | <span style="color:#FF1493">1</span>    | <span style="color:#1E90FF">15</span>         | <span style="color:#FFD700">15</span>         | <span style="color:#ADFF2F"> </span> |
| <span style="color:#8B008B">磁力计</span>             | <span style="color:#FF6347">1</span>    | <span style="color:#00FA9A">20</span>         | <span style="color:#FF00FF">20</span>         | <span style="color:#F0E68C">若自制 PCB 可降至 3 元以下（芯片仅 1.2 元）</span> |
| <span style="color:#FFB6C1">磁汇聚器</span>           | <span style="color:#3CB371">1</span>    | <span style="color:#FFA500">7.5</span>        | <span style="color:#00BFFF">7.5</span>        | <span style="color:#FFC0CB">可用废旧坡莫合金替代</span> |

<span style="color:#C71585">> **总计约 119 元**（不含打印设备及可选传感器）</span>

---

<span style="color:#FF4500">## ⚠️ 重要提示</span>

- <span style="color:#DC143C">**必须调平重心**</span><span style="color:#32CD32">，否则运动极不稳定</span>  
- <span style="color:#FF8C00">推荐加装编码器 + 陀螺仪</span><span style="color:#1E90FF"> → 显著降低漂移</span>  
- <span style="color:#FF1493">用热缩管套住滚子</span><span style="color:#00CED1">可提高抓地力</span>

---

<span style="color:#8A2BE2">## 📁 文件结构</span>

<span style="color:#FFD700">由于传输原因，第一级文件夹可忽略。</span>

<span style="color:#2E8B57">.</span>
<span style="color:#B22222">├── 结构件Gcode</span>
<span style="color:#FF69B4">│   ├── BAse17.12g1pcs.gcode        # 下底盘</span>
<span style="color:#00FA9A">│   ├── LidarBASE14.12g1pcs.gcode   # 上底盘</span>
<span style="color:#FF4500">│   ├── tambo17.86g42pcs.gcode      # 滚子打印件</span>
<span style="color:#00BFFF">│   ├── TTmotorAdapter13.87G9pcs.gcode   # 联轴器</span>
<span style="color:#FF00FF">│   └── WheelBAse33.88g6pcs.gcode   # 轮毂打印件</span>
<span style="color:#FF8C00">│</span>
<span style="color:#9932CC">├── 历史代码</span>
<span style="color:#DC143C">│   └── nrf24l01小车</span>
<span style="color:#FFB6C1">│       └── 基站</span>
<span style="color:#32CD32">│</span>
<span style="color:#FFA500">├── 上位机代码</span>
<span style="color:#1E90FF">│   ├── MoveCtrl.py                 # 运动控制与建图</span>
<span style="color:#FF1493">│   └── 5603.py                     # 磁力计数据处理</span>
<span style="color:#00CED1">│</span>
<span style="color:#FF4500">└── 小车代码</span>
<span style="color:#9400D3">    └── proj3_rubbish.ino           # !?史?!</span>

---

<span style="color:#FF6347">## 🛠 零件清单（结构件）</span>

- <span style="color:#20B2AA">钢销：2×18mm × 42个</span>  
- <span style="color:#FFD700">螺钉：3×20mm × 9个</span>  
- <span style="color:#FF00FF">螺钉：3×30mm × 6个</span>

---

<span style="color:#4169E1">## 📌 备注</span>

- <span style="color:#FF4500">磁汇聚器：**只看形状不看大小**</span><span style="color:#32CD32">，可自行寻找低成本材料</span>  
- <span style="color:#FF1493">若想极致节约，可参考 `pcb` 文件夹自制磁力计电路板</span>  
- <span style="color:#00CED1">建议先调平重心、加固滚子</span><span style="color:#FF8C00">，再逐步加入编码器与陀螺仪</span>

--- 

<span style="color:#FF0000">有</span><span style="color:#00FF00">任</span><span style="color:#0000FF">何</span><span style="color:#FFFF00">问</span><span style="color:#FF00FF">题</span><span style="color:#00FFFF">欢</span><span style="color:#FF4500">迎</span><span style="color:#2E8B57">提</span><span style="color:#DC143C">出</span><span style="color:#FF69B4">改</span><span style="color:#8A2BE2">进</span><span style="color:#FFD700">。</span><span style="color:#9932CC">(</span><span style="color:#20B2AA">我</span><span style="color:#FF6347">去</span><span style="color:#4169E1">,</span><span style="color:#B22222">A</span><span style="color:#FF1493">I</span><span style="color:#00CED1">真</span><span style="color:#FF8C00">好</span><span style="color:#9400D3">用</span><span style="color:#7FFF00">,</span><span style="color:#FFB6C1">不</span><span style="color:#C71585">过</span><span style="color:#1E90FF">A</span><span style="color:#32CD32">I</span><span style="color:#FFA500">本</span><span style="color:#00FA9A">来</span><span style="color:#FF00FF">就</span><span style="color:#FF4500">是</span><span style="color:#87CEEB">输</span><span style="color:#FF69B4">出</span><span style="color:#00BFFF">m</span><span style="color:#FF6347">a</span><span style="color:#8A2BE2">r</span><span style="color:#FFD700">k</span><span style="color:#DC143C">d</span><span style="color:#00FFFF">o</span><span style="color:#FF1493">w</span><span style="color:#32CD32">n</span><span style="color:#FF8C00">格</span><span style="color:#4169E1">式</span><span style="color:#FF4500">的</span><span style="color:#20B2AA">)</span>
