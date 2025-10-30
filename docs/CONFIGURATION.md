# faceHMI 設定ファイルガイド

faceHMIでは、目のアピアランス（外観）とモーション（動き）を設定ファイル（YAML）で管理できます。

## 設定ファイルの種類

### 1. デフォルト設定

**ファイル**: `config/default.yaml`

すべてのパラメータを含む基本設定です。カスタマイズの出発点として使用できます。

### 2. アピアランス専用設定

**ファイル**: `config/eye_appearance.yaml`

目の形状、サイズ、配置に関する設定のみを含みます。

### 3. モーション専用設定

**ファイル**: `config/eye_motion.yaml`

目の動きのスムーズさなど、モーションに関する設定のみを含みます。

### 4. カスタム設定例

- `config/large_eyes.yaml`: 大きな目の設定例
- `config/quick_motion.yaml`: 素早い動きの設定例

## 使用方法

### デフォルト設定で起動

```bash
ros2 launch face_hmi face_hmi_with_config.launch.py
```

### カスタム設定ファイルを指定

```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=/path/to/your_config.yaml
```

### プリセット設定を使用

```bash
# 大きな目
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/large_eyes.yaml

# 素早い動き
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=$(ros2 pkg prefix face_hmi)/share/face_hmi/config/quick_motion.yaml
```

## パラメータリファレンス

### アピアランス設定

#### 目の形状とサイズ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `eye.width_divisor` | int | 4 | 画面サイズを割る値。小さいほど大きい目（3=大, 6=小） |
| `eye.aspect_ratio` | float | 1.2 | 縦横比（1.0=円, 1.5=縦長楕円） |
| `eye.spacing_divisor` | int | 3 | 目の間隔。小さいほど近い |
| `eye.vertical_position` | float | 0.5 | 縦位置（0.5=中央, 0.3=上, 0.7=下） |

#### 瞳孔の設定

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `pupil.size_divisor` | float | 2.0 | 目の幅を割る値。大きいほど小さい瞳孔 |
| `pupil.aspect_ratio` | float | 1.3 | 瞳孔の縦横比 |
| `pupil.margin_x` | int | 10 | 横方向の移動制限（ピクセル）。大きいほど狭い範囲 |
| `pupil.margin_y` | int | 10 | 縦方向の移動制限（ピクセル） |

#### ハイライトの設定

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `highlight.size_divisor` | int | 5 | 瞳孔幅を割る値。ハイライトのサイズ |
| `highlight.offset_x_divisor` | int | 6 | ハイライトの横位置オフセット |
| `highlight.offset_y_divisor` | int | 6 | ハイライトの縦位置オフセット |

#### グラデーションの設定

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `gradient.layers` | int | 5 | グラデーションのレイヤー数。多いほど滑らか |
| `gradient.outer_color` | int | 40 | 外側の色（0-255、グレースケール） |
| `gradient.inner_color` | int | 0 | 内側の色（通常は0=黒） |

### モーション設定

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `motion.smoothing_factor` | float | 0.15 | 補間係数（0.05=滑らか, 0.3=速い, 1.0=即座） |
| `blink.enabled` | bool | false | まばたき機能（将来実装予定） |
| `saccade.enabled` | bool | false | サッカード（急速眼球運動）機能（将来実装予定） |

### カメラとディスプレイ設定

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|-----------|------|
| `camera_frame` | string | camera_optical_frame | カメラのTFフレーム |
| `fov_x_deg` | float | 90.0 | 水平視野角（度） |
| `fov_y_deg` | float | 60.0 | 垂直視野角（度） |
| `fullscreen` | bool | true | フルスクリーンモード |
| `fps` | int | 60 | フレームレート |

## カスタム設定ファイルの作成

### 1. テンプレートをコピー

```bash
cp $(ros2 pkg prefix face_hmi)/share/face_hmi/config/default.yaml my_config.yaml
```

### 2. パラメータを編集

```yaml
/**:
  ros__parameters:
    # 必要な部分だけ変更
    eye:
      width_divisor: 3  # 大きな目
    
    motion:
      smoothing_factor: 0.1  # ゆっくりした動き
```

### 3. 設定ファイルを使用

```bash
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=/path/to/my_config.yaml
```

## よくある設定例

### 大きくて表情豊かな目

```yaml
eye:
  width_divisor: 3
  aspect_ratio: 1.4
pupil:
  size_divisor: 1.8
  margin_x: 5
  margin_y: 5
motion:
  smoothing_factor: 0.12
```

### 小さくて素早い目

```yaml
eye:
  width_divisor: 6
  aspect_ratio: 1.1
pupil:
  size_divisor: 2.5
motion:
  smoothing_factor: 0.3
```

### ゆったりとした動き

```yaml
motion:
  smoothing_factor: 0.08
pupil:
  margin_x: 15
  margin_y: 15
```

### 広範囲を見渡す

```yaml
pupil:
  margin_x: 3
  margin_y: 3
motion:
  smoothing_factor: 0.2
```

## 設定の組み合わせ

アピアランスとモーションの設定を別々に管理できます：

```bash
# アピアランスのみカスタマイズ
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=my_appearance.yaml

# モーションのみカスタマイズ
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=my_motion.yaml

# 両方を組み合わせた完全な設定
ros2 launch face_hmi face_hmi_with_config.launch.py \
    config_file:=my_complete_config.yaml
```

## トラブルシューティング

### 設定が反映されない

- パッケージを再ビルドしたか確認:
  ```bash
  colcon build --packages-select face_hmi
  source install/setup.bash
  ```

### パラメータのタイプエラー

- YAMLファイルの型が正しいか確認（int, float, bool, string）
- インデントが正しいか確認（スペース2個）

### 設定ファイルが見つからない

- パスが正しいか確認:
  ```bash
  ros2 pkg prefix face_hmi
  ls $(ros2 pkg prefix face_hmi)/share/face_hmi/config/
  ```

## 次のステップ

- [使用ガイド](USAGE.md)で基本的な使い方を確認
- [ROADMAP](ROADMAP.md)で今後の機能を確認
