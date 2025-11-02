# ROS1 快速啟動與連線建議

以下假設你的韌體（STM32 + rosserial）使用主題：
- 訂閱：`/target_deg`（`std_msgs/Float32MultiArray`，12 通道，單位：deg）
- 發佈：`/joint_states_deg`（`sensor_msgs/JointState`，position=deg）

## 1) 啟動 roscore
```bash
roscore
```

## 2) 啟動 rosbridge（另一個終端機）
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
預設 WebSocket 在 `ws://<ROS_HOST>:9090`。

## 3) 連上 rosserial（依你的連線埠調整）
- USB：
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=230400
```
- Windows（PowerShell）：
```powershell
rosrun rosserial_python serial_node.py _port:=COM5 _baud:=230400
```

確認 topics：
```bash
rostopic list
rostopic echo /joint_states_deg
```

## 4) 啟動前端
啟動簡單 HTTP 伺服器並開啟 `web_ui/index.html`（參見 `web_ui/README.md`）。

## 5) 驗證發布/回饋
- 在網頁拖動 slider 應看到：
```bash
rostopic echo /target_deg
```
有對應 12 維浮點陣列（度）。
- 裝置回饋：
```bash
rostopic echo /joint_states_deg
```

## 安全/限位建議
- 初次測試請小角度、低速度，在無負載情況
- 在韌體端加入角度軟限位、速度/加速度限制，避免突發大角度
- rosbridge 僅在可信網段開放；跨網段請用 VPN/反代（Nginx）並限制來源 IP
