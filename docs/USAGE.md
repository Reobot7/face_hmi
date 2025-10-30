# faceHMI 使用ガイド

## 基本的な使い方

### 1. ビルド

```bash
cd /path/to/face_hmi
colcon build
source install/setup.bash
```

### 2. 起動

#### 通常起動（フルスクリーン）

```bash
ros2 launch face_hmi face_hmi.launch.py
```

#### テストモード起動（ウィンドウモード + ランダム視線移動）

```bash
ros2 launch face_hmi test_face_hmi.launch.py
```

このモードでは、テストパブリッシャが自動的に起動し、目がランダムに動きます。

### 3. 手動でトピックを送信

別のターミナルから以下のコマンドでテスト可能です。

#### 視線を動かす

```bash
# 右上を見る
ros2 topic pub /hmi/attention geometry_msgs/Vector3 "{x: 0.5, y: 0.5, z: 0.0}" --once

# 左下を見る
ros2 topic pub /hmi/attention geometry_msgs/Vector3 "{x: -0.5, y: -0.5, z: 0.0}" --once

# 中央を見る
ros2 topic pub /hmi/attention geometry_msgs/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" --once
```

**座標系**:
- `x`: 横方向（-1.0 = 左端, 0.0 = 中央, 1.0 = 右端）
- `y`: 縦方向（-1.0 = 下端, 0.0 = 中央, 1.0 = 上端）

#### 稼働状態を変更

```bash
# アイドル状態
ros2 topic pub /hmi/activity std_msgs/String "data: 'idle'" --once

# 作業中
ros2 topic pub /hmi/activity std_msgs/String "data: 'working'" --once

# 移動開始の合図
ros2 topic pub /hmi/activity std_msgs/String "data: 'announce_move'" --once

# 移動中
ros2 topic pub /hmi/activity std_msgs/String "data: 'moving'" --once
```

#### 健康状態を送信

```bash
ros2 topic pub /hmi/health face_hmi_msgs/Health \
  "{battery_pct: 75.0, temp_c: 50.0, net_ok: true, charging: false}" --once
```

#### ラベルを表示

```bash
ros2 topic pub /hmi/attention_label std_msgs/String "data: 'Hello World'" --once
```

## テストパブリッシャを単独で起動

```bash
ros2 run face_hmi test_publisher
```

このノードは以下の動作をします：

- **3秒ごと**: ランダムな方向に視線を変更
- **10秒ごと**: 稼働状態を順番に変更（idle → working → announce_move → moving → ...）
- **1秒ごと**: 健康状態を更新（バッテリー減少、温度変動をシミュレート）

## パラメータのカスタマイズ

launchファイルでパラメータを指定できます。

```bash
ros2 launch face_hmi face_hmi.launch.py \
    camera_frame:=my_camera_link \
    fov_x_deg:=85.0 \
    fov_y_deg:=55.0 \
    fullscreen:=true \
    fps:=30
```

### 利用可能なパラメータ

| パラメータ | 型 | デフォルト値 | 説明 |
|-----------|-----|------------|------|
| `camera_frame` | string | `camera_optical_frame` | カメラのTFフレーム名 |
| `fov_x_deg` | double | `90.0` | 水平視野角（度） |
| `fov_y_deg` | double | `60.0` | 垂直視野角（度） |
| `fullscreen` | bool | `true` | フルスクリーンモード |
| `fps` | int | `60` | フレームレート |

## 終了方法

- **ESCキー**または**Qキー**を押す
- または`Ctrl+C`でターミナルから終了

## トラブルシューティング

### 画面が表示されない

- ディスプレイが正しく接続されているか確認
- `DISPLAY`環境変数が設定されているか確認:
  ```bash
  echo $DISPLAY
  ```

### 目が動かない

- トピックが正しくパブリッシュされているか確認:
  ```bash
  ros2 topic echo /hmi/attention
  ```
- テストパブリッシャを起動して確認:
  ```bash
  ros2 run face_hmi test_publisher
  ```

### 動きがカクカクする

- `smoothing_factor`を小さくする（`face_hmi_node.py`内）
- FPSを上げる（launchパラメータ`fps:=120`など）

### Raspberry Piで動作が重い

- FPSを下げる（`fps:=30`など）
- フルスクリーンモードを使用
- 不要なバックグラウンドプロセスを停止

## 次のステップ

- [カスタマイズガイド](CUSTOMIZATION.md)で見た目を調整
- [ROADMAP](ROADMAP.md)で今後の機能を確認
