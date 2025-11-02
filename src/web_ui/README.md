# 關節馬達即時調整面板（ROS1 + rosbridge + roslibjs）

本目錄提供最小可行的前端介面，透過 roslibjs 連線到 ROS1 的 rosbridge_server：
- 發布 `std_msgs/Float32MultiArray` 到 `/target_deg`（單位：度），索引順序：
  `[GIM1, RS2, RS3, RS4, R_ANKLE_ROLL, R_ANKLE_PITCH, GIM7, RS8, RS9, RS10, L_ANKLE_ROLL, L_ANKLE_PITCH]`
- 訂閱 `/joint_states_deg`（`sensor_msgs/JointState`）顯示各軸回饋（度數，僅 position）。

## 先決條件
- ROS1（Noetic/Melodic 皆可）
- 套件：`rosbridge_server`、`rosserial_python`
- 裝置端 firmware 已訂閱 `/target_deg` 並發佈 `/joint_states_deg`

## 啟動 rosbridge（ROS 主機上）
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
預設 WebSocket 端口為 `9090`，可在網頁 UI 內修改主機與 port。

## 啟動靜態檔伺服器
避免直接用 file:// 開啟，請啟動簡單 HTTP 伺服器：

- Python 3（跨平台，Windows 可用 `py -3 -m http.server 8000`）：
```bash
cd web_ui
python3 -m http.server 8000
```
瀏覽器打開 `http://localhost:8000/`，進入 `index.html`。

- 或 Node.js：
```bash
npm i -g http-server
cd web_ui
http-server -p 8000
```

## 使用步驟
1. 在 ROS 主機上啟動 rosbridge：`roslaunch rosbridge_server rosbridge_websocket.launch`
2. 確認 rosserial 已連線，且能看到 topics：`rostopic list`
3. 在本機或 ROS 主機上啟動靜態伺服器並開啟 `index.html`
4. 在頁面上填入 ROS 主機與 port（預設 `localhost:9090`），按「連線」
5. 拖動 12 個 slider（50 ms 節流）發布到 `/target_deg`
6. 右側「回饋」顯示 `/joint_states_deg` 的 position（deg）
7. 使用「全部歸零」一鍵發布 0 度

## 安全建議
- 先在無負載/懸空狀態下小角度測試，確認方向與限位
- 建議在 firmware 端限制最大速度/加速度與軟限位
- 若跨網段使用 rosbridge，請配置防火牆/反向代理並限制來源 IP
