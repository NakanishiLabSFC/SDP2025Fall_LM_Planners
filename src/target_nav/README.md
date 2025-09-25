# target_nav

ROS2 + Nav2環境で動作するナビゲーションノードパッケージです。オブジェクト名による指定、座標直指定、高レベル日本語命令に対応した多様なナビゲーション機能を提供します。

## 特徴

- **3つのインターフェース**: 名前指定、座標直指定、高レベル命令
- **Nav2統合**: NavigateToPoseアクションを使用した確実なナビゲーション
- **日本語対応**: 「ミャクミャクの椅子へ向かう」などの自然言語命令をサポート
- **拡張可能設計**: ロボットアーム制御等の将来機能追加に対応
- **エラーハンドリング**: リトライ機能と詳細なステータス通知

## 前提条件

- ROS2 Humble 以降
- Nav2 navigation stack
- Python 3.8+
- 依存パッケージ: `nav2_msgs`, `geometry_msgs`, `std_msgs`, `tf_transformations`

## インストール・ビルド

```bash
# ワークスペースのsrcディレクトリにクローン
cd ~/your_workspace/src
git clone <repository_url> target_nav

# 依存パッケージをインストール
rosdep install --from-paths . --ignore-src -r -y

# ビルド
cd ~/your_workspace
colcon build --packages-select target_nav

# セットアップ
source install/setup.bash
```

## 使用方法

### 1. 起動



```bash
# Nav2を先に起動
cd navigation_utility
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch src/nav2_bringup/launch/bringup_launch.py map:=src/nav2_bringup/maps/sdp20250925_map.yaml

#rviz2も先に起動
cd navigation_utility
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# target_navノードを起動
source install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch target_nav target_nav.launch.py
```

### 2. ナビゲーション実行

#### A) 名前指定ナビゲーション
```bash
ros2 topic pub /go_to_name std_msgs/String "data: 'ミャクミャクの椅子'"
ros2 topic pub /go_to_name std_msgs/String "data: '抹茶の箱'"
ros2 topic pub /go_to_name std_msgs/String "data: '3Dプリンター'"
```

#### B) 座標直指定ナビゲーション
```bash
ros2 topic pub /go_to_pose geometry_msgs/PoseStamped "
header:
  frame_id: map
pose:
  position: {x: 0.4557, y: 0.4359, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
"
```

#### C) 高レベル日本語命令
```bash
ros2 topic pub /robot_command std_msgs/String "data: 'ミャクミャクの椅子へ向かう'"
ros2 topic pub /robot_command std_msgs/String "data: '抹茶の箱へ向かう'"
ros2 topic pub /robot_command std_msgs/String "data: '3Dプリンターへ向かう'"
ros2 topic pub /robot_command std_msgs/String "data: '行動終了'"
```

### 3. 結果の確認
```bash
ros2 topic echo /nav_result
```

## 設定ファイル

### config/object_map.yaml
オブジェクト名と座標のマッピング設定:
```yaml
objects:
  "ミャクミャクの椅子":
    frame_id: map
    x: -0.4068
    y: 0.0938
    yaw: 1.57
  "抹茶の箱":
    frame_id: map
    x: 0.5555
    y: -0.6994
    yaw: 3.14
  "3Dプリンター":
    frame_id: map
    x: 0.4557
    y: 0.4359
    yaw: 0.00
```

### config/command_map.yaml
高レベル命令のマッピング設定:
```yaml
navigation_commands:
  "ミャクミャクの椅子へ向かう": "ミャクミャクの椅子"
  "抹茶の箱へ向かう": "抹茶の箱"
  "3Dプリンターへ向かう": "3Dプリンター"

unsupported_patterns:
  - "にあるアイテムを探す"
  - "にあるクッキーを掴む"
  - "にある皿を掴む"
  - "にある遊び道具を掴む"
  - "でアイテムを手放す"

special_commands:
  "行動終了": "END_ACTION"
```

## 結果通知

`/nav_result` トピックで以下の形式で結果を通知:

### ナビゲーション結果
- `SUCCEEDED:name=ミャクミャクの椅子` - 名前指定成功
- `NAVIGATION:SUCCEEDED:command=ミャクミャクの椅子へ向かう` - 高レベル命令成功
- `FAILED:name=抹茶の箱 code=5` - 失敗（エラーコード付き）
- `CANCELED:pose=(x,y,yaw)` - キャンセル

### その他の結果
- `UNSUPPORTED:command=ミャクミャクの椅子にあるアイテムを探す` - 非対応命令
- `SPECIAL:END_ACTION:command=行動終了` - 特殊命令
- `UNKNOWN:command=未知の命令` - 未定義命令

## 対応命令一覧

### ✅ 対応済み
- `[場所名]へ向かう` → ナビゲーション実行
- `行動終了` → 終了アクション

### ⏸️ 未対応（将来拡張予定）
- `[場所名]にあるアイテムを探す` → 物体認識機能
- `[場所名]にある[アイテム]を掴む` → ロボットアーム制御
- `[場所名]でアイテムを手放す` → ロボットアーム制御

## パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `object_map_path` | `config/object_map.yaml` | オブジェクトマップファイルパス |
| `goal_timeout_sec` | `120.0` | ナビゲーションタイムアウト（秒） |
| `max_retries` | `1` | 失敗時の最大リトライ回数 |
| `approach_offset` | `0.0` | 目標からのオフセット距離（m） |

## 起動時パラメータ指定例

```bash
ros2 launch target_nav target_nav.launch.py \
  object_map_path:=/custom/path/to/object_map.yaml \
  goal_timeout_sec:=180.0 \
  max_retries:=3
```

## トラブルシューティング

### NavigateToPose action server not available
Nav2が起動していない場合に発生。先にNav2を起動してください:
```bash
ros2 launch nav2_bringup bringup_launch.py map:=your_map.yaml
```

### Object not found in map
`object_map.yaml`に該当するオブジェクトが定義されていません。設定ファイルを確認してください。

### Navigation failed
- ロボットの位置推定が不安定な可能性があります
- 目標地点が到達不可能な場合があります
- パラメータ `max_retries` を増やして再試行してください

## 開発・拡張

### 新しいオブジェクト追加
1. `config/object_map.yaml` にオブジェクト定義を追加
2. `config/command_map.yaml` に対応する命令を追加
3. パッケージを再ビルド

### 新機能追加
`target_nav/node.py` の `on_robot_command_callback` 関数を拡張して、新しい命令タイプに対応できます。

## ライセンス

Apache-2.0

## 作成者

ROS Developer

---

このパッケージはNav2を使った自律移動ロボットのナビゲーション制御を簡単にするために開発されました。ロボットアーム制御などの追加機能も将来的に統合予定です。
