# faceHMI カスタマイズガイド

このドキュメントでは、faceHMIの見た目や動作をカスタマイズする方法を説明します。

## 目のサイズ調整

目のサイズは`face_hmi_node.py`の`draw_eye()`メソッド内で調整できます。

### 調整箇所

```python
def draw_eye(self, center_x: int, center_y: int, is_left: bool):
    """Draw a single eye with moving pupil"""
    # Eye dimensions (oval shape)
    eye_width = min(self.width, self.height) // 4  # ← ここを変更
    eye_height = int(eye_width * 1.2)  # ← 縦横比を変更
```

### パラメータ説明

| パラメータ | 説明 | デフォルト値 | 推奨範囲 |
|-----------|------|------------|---------|
| `eye_width` | 目の横幅 | `画面サイズ // 4` | `// 3` ~ `// 6` |
| `eye_height` | 目の縦幅 | `eye_width * 1.2` | `* 1.0` ~ `* 1.5` |

### 調整例

**大きな目にする場合**:
```python
eye_width = min(self.width, self.height) // 3  # より大きく
eye_height = int(eye_width * 1.3)
```

**小さな目にする場合**:
```python
eye_width = min(self.width, self.height) // 6  # より小さく
eye_height = int(eye_width * 1.1)
```

**真円に近い目にする場合**:
```python
eye_width = min(self.width, self.height) // 4
eye_height = int(eye_width * 1.0)  # 縦横比1:1
```

## 瞳孔のサイズ調整

瞳孔のサイズも同じ`draw_eye()`メソッド内で調整できます。

### 調整箇所

```python
# Pupil dimensions
pupil_width = eye_width // 2  # ← ここを変更
pupil_height = int(pupil_width * 1.3)  # ← 縦横比を変更
```

### パラメータ説明

| パラメータ | 説明 | デフォルト値 | 推奨範囲 |
|-----------|------|------------|---------|
| `pupil_width` | 瞳孔の横幅 | `eye_width // 2` | `// 1.5` ~ `// 3` |
| `pupil_height` | 瞳孔の縦幅 | `pupil_width * 1.3` | `* 1.0` ~ `* 1.5` |

### 調整例

**大きな瞳孔**:
```python
pupil_width = eye_width // 1.5  # より大きく
pupil_height = int(pupil_width * 1.4)
```

**小さな瞳孔**:
```python
pupil_width = eye_width // 3  # より小さく
pupil_height = int(pupil_width * 1.2)
```

## 瞳孔の移動範囲調整

瞳孔がどこまで移動できるかを調整できます。

### 調整箇所

```python
# Calculate pupil position based on attention
# Limit movement to stay within the white part
max_offset_x = (eye_width - pupil_width) // 2 - 10  # ← 最後の数値を変更
max_offset_y = (eye_height - pupil_height) // 2 - 10  # ← 最後の数値を変更
```

### パラメータ説明

| パラメータ | 説明 | デフォルト値 | 効果 |
|-----------|------|------------|------|
| `- 10` (X方向) | 横方向の余白 | `10` | 大きいほど移動範囲が狭い |
| `- 10` (Y方向) | 縦方向の余白 | `10` | 大きいほど移動範囲が狭い |

### 調整例

**より広い範囲で動かす**:
```python
max_offset_x = (eye_width - pupil_width) // 2 - 5  # 余白を小さく
max_offset_y = (eye_height - pupil_height) // 2 - 5
```

**より狭い範囲で動かす**:
```python
max_offset_x = (eye_width - pupil_width) // 2 - 20  # 余白を大きく
max_offset_y = (eye_height - pupil_height) // 2 - 20
```

## 目の間隔調整

2つの目の間隔を調整できます。

### 調整箇所

`render()`メソッド内:

```python
def render(self):
    # ...
    # Calculate eye positions
    eye_spacing = self.width // 3  # ← ここを変更
    eye_y = self.height // 2
    left_eye_x = self.width // 2 - eye_spacing // 2
    right_eye_x = self.width // 2 + eye_spacing // 2
```

### パラメータ説明

| パラメータ | 説明 | デフォルト値 | 推奨範囲 |
|-----------|------|------------|---------|
| `eye_spacing` | 目の中心間の距離 | `画面幅 // 3` | `// 2.5` ~ `// 4` |

### 調整例

**目を近づける**:
```python
eye_spacing = self.width // 4  # より近く
```

**目を離す**:
```python
eye_spacing = self.width // 2.5  # より離す
```

## スムーズさ（補間速度）の調整

目の動きのスムーズさを調整できます。

### 調整箇所

`__init__()`メソッド内:

```python
# State variables
self.target_attention_x = 0.0
self.target_attention_y = 0.0
self.current_attention_x = 0.0
self.current_attention_y = 0.0
self.smoothing_factor = 0.15  # ← ここを変更
```

### パラメータ説明

| 値 | 効果 | 用途 |
|----|------|------|
| `0.05` | 非常にゆっくり、滑らか | リラックスした雰囲気 |
| `0.15` | 適度にスムーズ（デフォルト） | 一般的な用途 |
| `0.3` | やや速い | キビキビした動き |
| `0.5` | 速い | 素早い反応が必要な場合 |
| `1.0` | 即座に移動（補間なし） | デバッグ用 |

### 調整例

**よりゆっくり動かす**:
```python
self.smoothing_factor = 0.08  # ゆったりとした動き
```

**より速く動かす**:
```python
self.smoothing_factor = 0.25  # キビキビした動き
```

## ハイライトの調整

瞳孔内の白いハイライト（光の反射）を調整できます。

### 調整箇所

```python
# Highlight dimensions
highlight_radius = pupil_width // 5  # ← サイズを変更

# Draw highlight (white spot)
highlight_offset_x = -pupil_width // 6  # ← 位置を変更
highlight_offset_y = -pupil_height // 6  # ← 位置を変更
```

### 調整例

**大きなハイライト**:
```python
highlight_radius = pupil_width // 4
```

**小さなハイライト**:
```python
highlight_radius = pupil_width // 8
```

**ハイライトの位置を変更**:
```python
highlight_offset_x = -pupil_width // 4  # より左に
highlight_offset_y = -pupil_height // 4  # より上に
```

## クイックリファレンス

よく使う調整のまとめ：

```python
# face_hmi_node.py の draw_eye() メソッド内

# 目のサイズ
eye_width = min(self.width, self.height) // 4  # 大きく: //3, 小さく: //6
eye_height = int(eye_width * 1.2)  # 縦長: *1.4, 横長: *1.0

# 瞳孔のサイズ
pupil_width = eye_width // 2  # 大きく: //1.5, 小さく: //3
pupil_height = int(pupil_width * 1.3)

# 移動範囲
max_offset_x = (eye_width - pupil_width) // 2 - 10  # 広く: -5, 狭く: -20
max_offset_y = (eye_height - pupil_height) // 2 - 10

# __init__() メソッド内
self.smoothing_factor = 0.15  # ゆっくり: 0.08, 速く: 0.25

# render() メソッド内
eye_spacing = self.width // 3  # 近く: //4, 遠く: //2.5
```

## 変更の反映方法

1. `face_hmi_node.py`を編集
2. パッケージを再ビルド:
   ```bash
   cd /path/to/face_hmi
   colcon build --packages-select face_hmi
   source install/setup.bash
   ```
3. ノードを再起動:
   ```bash
   ros2 launch face_hmi face_hmi.launch.py
   ```
