# faceHMI 感情表現プリセット

faceHMIには、様々な感情を表現するための目のプリセットが用意されています。

## 利用可能なプリセット

### 1. Sleepy（眠そう）

**ファイル**: `config/sleepy.yaml`

**特徴**:
- 細長い目（aspect_ratio: 0.6）
- ゆっくりとした動き（smoothing: 0.08）
- 頻繁で遅い瞬き（2-4秒間隔、0.25秒持続）
- 小さめのハイライト

**用途**: 低電力モード、待機状態、リラックスした雰囲気

**起動方法**:
```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/sleepy.yaml
```

---

### 2. Surprised（驚き）

**ファイル**: `config/surprised.yaml`

**特徴**:
- 大きく開いた目（width_divisor: 2.5, aspect_ratio: 1.5）
- 素早い動き（smoothing: 0.35）
- まれな瞬き（8-15秒間隔、0.1秒持続）
- 広い移動範囲（margin: 3）
- サッカード有効

**用途**: 予期しないイベント検出、新しい物体発見、緊急事態

**起動方法**:
```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/surprised.yaml
```

---

### 3. Angry（怒り）

**ファイル**: `config/angry.yaml`

**特徴**:
- 細めの目（width_divisor: 5, aspect_ratio: 0.8）
- 鋭い動き（smoothing: 0.28）
- まれな瞬き（6-12秒間隔）- 睨みつける
- 暗いグラデーション（outer_color: 20）
- サッカード有効（速い視線移動）

**用途**: 警告表示、禁止エリア侵入検知、エラー状態

**起動方法**:
```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/angry.yaml
```

---

### 4. Gentle（優しい）

**ファイル**: `config/gentle.yaml`

**特徴**:
- 柔らかい丸い目（aspect_ratio: 1.3）
- 非常にスムーズな動き（smoothing: 0.1）
- 通常の瞬き（3.5-6秒間隔、0.18秒持続）
- 多層グラデーション（layers: 7）
- サッカード無効

**用途**: 接客モード、子供との対話、安心感の提供

**起動方法**:
```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/gentle.yaml
```

---

### 5. Alert（警戒）

**ファイル**: `config/alert.yaml`

**特徴**:
- 中サイズの目（width_divisor: 3.8）
- 速い反応（smoothing: 0.25）
- 効率的な瞬き（4-7秒間隔、0.13秒持続）
- 広い移動範囲（margin: 5）- スキャン動作
- サッカード有効

**用途**: セキュリティモード、環境監視、パトロール中

**起動方法**:
```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/alert.yaml
```

---

## プリセット比較表

| プリセット | 目のサイズ | 動きの速さ | 瞬き頻度 | 移動範囲 | サッカード |
|-----------|----------|----------|---------|---------|-----------|
| **Sleepy** | 標準 | 非常に遅い | 高い | 狭い | 無効 |
| **Surprised** | 非常に大きい | 速い | 非常に低い | 広い | 有効 |
| **Angry** | 小さい | やや速い | 低い | 中程度 | 有効 |
| **Gentle** | やや大きい | 非常に遅い | 標準 | 中程度 | 無効 |
| **Alert** | 標準 | 速い | 標準 | 広い | 有効 |

## パラメータ詳細比較

### 目の形状

| プリセット | width_divisor | aspect_ratio | 説明 |
|-----------|--------------|--------------|------|
| Sleepy | 4 | 0.6 | 細長い、半分閉じた |
| Surprised | 2.5 | 1.5 | 大きく開いた、縦長 |
| Angry | 5 | 0.8 | 細い、睨む |
| Gentle | 3.5 | 1.3 | 柔らかい丸み |
| Alert | 3.8 | 1.4 | 開いた、注意深い |

### 動きの特性

| プリセット | smoothing_factor | margin_x/y | 特徴 |
|-----------|-----------------|-----------|------|
| Sleepy | 0.08 | 15 | ゆっくり、制限的 |
| Surprised | 0.35 | 3 | 速い、広範囲 |
| Angry | 0.28 | 8 | 鋭い、直接的 |
| Gentle | 0.1 | 12 | 滑らか、穏やか |
| Alert | 0.25 | 5 | 機敏、スキャン |

### 瞬きの特性

| プリセット | interval (s) | duration (s) | 特徴 |
|-----------|-------------|--------------|------|
| Sleepy | 2.0 - 4.0 | 0.25 | 頻繁、遅い |
| Surprised | 8.0 - 15.0 | 0.1 | まれ、速い |
| Angry | 6.0 - 12.0 | 0.12 | まれ、鋭い |
| Gentle | 3.5 - 6.0 | 0.18 | 標準、柔らかい |
| Alert | 4.0 - 7.0 | 0.13 | 標準、効率的 |

## 使用シナリオ例

### ロボット受付

```bash
# 通常時: 優しい表情
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/gentle.yaml

# 人が近づいた: 警戒モード
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/alert.yaml
```

### セキュリティロボット

```bash
# パトロール中: 警戒モード
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/alert.yaml

# 侵入者検知: 怒りモード
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/angry.yaml
```

### 家庭用ロボット

```bash
# 待機中: 眠そう
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/sleepy.yaml

# 呼ばれた: 驚き
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/surprised.yaml

# 対話中: 優しい
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/gentle.yaml
```

## カスタマイズ

プリセットをベースに、独自の感情表現を作成できます：

```bash
# プリセットをコピー
cp $(ros2 pkg prefix face_hmi)/share/face_hmi/config/gentle.yaml my_emotion.yaml

# 編集して独自の感情を作成
nano my_emotion.yaml

# 使用
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=/path/to/my_emotion.yaml
```

## 動的な感情切り替え

実行中にプリセットを切り替えるには、ノードを再起動する必要があります。将来的には、トピック経由での動的切り替えも検討されています（ROADMAP参照）。

## 次のステップ

- [設定ガイド](CONFIGURATION.md)で各パラメータの詳細を確認
- [使用ガイド](USAGE.md)で基本的な使い方を確認
- [ROADMAP](ROADMAP.md)で今後の機能を確認
