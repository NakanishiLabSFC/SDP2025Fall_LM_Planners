# ROS2 + Nav2で日本語コマンドによるシーケンシャルナビゲーションシステムの実装

## はじめに

ロボットナビゲーションにおいて、「ミャクミャクの椅子へ向かう」「3Dプリンターに行って、その後抹茶の箱に向かう」といった自然言語でのコマンド実行は、ロボットの使いやすさを大幅に向上させます。

本記事では、ROS2 HumbleとNavigation2を使用して、日本語自然言語コマンドによる**シーケンシャルナビゲーションシステム**を実装した過程を紹介します。実際の開発で遭遇した技術的課題とその解決方法も含めて解説します。

## システム概要

### 主要機能

実装したシステムは以下の4つのインターフェースを提供します：

1. **名前指定ナビゲーション**: `/go_to_name` - オブジェクト名で目的地指定
2. **座標直指定ナビゲーション**: `/go_to_pose` - 座標値での目的地指定  
3. **単一日本語コマンド**: `/robot_command_input` - 自然言語での単発命令
4. **シーケンス実行**: `/robot_command_sequence` - 複数コマンドの順次自動実行 ⭐️**今回の主要実装**

### システムアーキテクチャ

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ 日本語コマンド   │───▶│  target_nav_node │───▶│ NavigateToPose  │
│ "ミャク,抹茶"   │    │                  │    │  Action Server  │
└─────────────────┘    │ ・コマンド解析    │    │    (Nav2)       │
                      │ ・シーケンス管理  │    └─────────────────┘
┌─────────────────┐    │ ・エラー処理      │    ┌─────────────────┐
│ 設定ファイル     │───▶│                  │───▶│ ステータス通知   │  
│ object_map.yaml │    └──────────────────┘    │ /sequence_status│
│ command_map.yaml│                           └─────────────────┘
└─────────────────┘
```

## 環境構築・前提条件

**前提条件:**
- Ubuntu 22.04 + ROS2 Humble Desktop
- Navigation2 (`ros-humble-navigation2`)
- Cartographer (`ros-humble-cartographer ros-humble-cartographer-ros`)

### 3.1 Cartographerでのマップ作成

ナビゲーションシステムには環境地図が必要です。Cartographerを使用してSLAMによるマップ作成を行います。

```bash
# 環境セットアップ
source /opt/ros/humble/setup.bash

# Cartographerでのマッピング（TurtleBot3の例）
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_cartographer cartographer.launch.py

# 別ターミナルでロボット操作
ros2 run turtlebot3_teleop teleop_keyboard

# マップ完成後、保存
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

作成された`my_map.pgm`と`my_map.yaml`がナビゲーションで使用されます。

### 3.2 RViz2のPublish Pointによる座標取得

オブジェクトの正確な座標を取得するために、RViz2の「Publish Point」機能を使用します。

```bash
# Nav2環境の起動
ros2 launch nav2_bringup navigation_launch.py map:=~/my_map.yaml

# 別ターミナルでRViz2を起動
ros2 launch nav2_bringup rviz_launch.py

# 座標取得用のトピックを監視
ros2 topic echo /clicked_point
```

RViz2上で：
1. 上部ツールバーの「Publish Point」ボタンをクリック
2. マップ上の目的地をクリック
3. ターミナルに表示される座標をメモ

出力例：
```yaml
header:
  stamp:
    sec: 1758863901
    nanosec: 575727333
  frame_id: map
point:
  x: 0.5882173180580139
  y: 0.9352026581764221
  z: 0.002471923828125
```

## パッケージ構成と設計

### 4.1 ディレクトリ構造

```
target_nav/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   ├── object_map.yaml      # オブジェクト座標管理
│   └── command_map.yaml     # 日本語コマンドマッピング
├── launch/
│   └── target_nav.launch.py # 起動設定
└── target_nav/
    ├── __init__.py
    ├── node.py              # メインノード実装
    └── utils.py             # ユーティリティ関数
```

### 4.2 設定ファイル設計

**object_map.yaml** - オブジェクトの座標情報
```yaml
objects:
  "ミャクミャクの椅子":
    frame_id: map
    x: -1.0763
    y: 0.1095
    yaw: 1.57
  "抹茶の箱":
    frame_id: map
    x: 0.6696
    y: -0.5724
    yaw: 3.14
  "3Dプリンター":
    frame_id: map
    x: 0.5882
    y: 0.9352
    yaw: -1.57
```

**command_map.yaml** - 日本語コマンドのマッピング
```yaml
navigation_commands:
  "ミャクミャクの椅子へ向かう": "ミャクミャクの椅子"
  "抹茶の箱へ向かう": "抹茶の箱"
  "3Dプリンターへ向かう": "3Dプリンター"

special_commands:
  "行動終了": "END_ACTION"

unsupported_patterns:
  - "にあるアイテムを探す"
  - "にあるクッキーを掴む"
```

## 実装詳細

### 5.1 基本ナビゲーションノード

メインのナビゲーションノードは`NavigateToPose` action clientを使用してNav2との通信を行います。

```python
#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class TargetNavNode(Node):
    def __init__(self):
        super().__init__('target_nav')
        
        # Create callback group for concurrent operations
        callback_group = ReentrantCallbackGroup()
        
        # Create action client for NavigateToPose
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=callback_group
        )
        
        # Load configuration files
        self.objects = self.load_object_map()
        self.command_map = self.load_command_map()
        
        # Initialize sequence management state
        self.action_queue = []
        self.current_action_index = 0
        self.sequence_active = False
        self.sequence_id = 0
```

### 5.2 日本語コマンド処理

自然言語コマンドを座標情報に変換する処理です。

```python
def on_robot_command_callback(self, msg):
    """Handle high-level robot command"""
    command = msg.data.strip()
    self.get_logger().info(f'Received robot command: "{command}"')
    
    # Check if it's a navigation command
    nav_commands = self.command_map.get('navigation_commands', {})
    if command in nav_commands:
        target_name = nav_commands[command]
        
        # Get object coordinates
        if target_name in self.objects:
            obj_data = self.objects[target_name]
            x = float(obj_data['x'])
            y = float(obj_data['y'])
            yaw = float(obj_data['yaw'])
            frame_id = obj_data.get('frame_id', 'map')
            
            # Create pose and navigate
            pose = create_pose_stamped(x, y, yaw, frame_id)
            tag = f"command={command}"
            self.go_to(pose, tag)
```

### 5.3 シーケンス機能実装

複数コマンドの順次実行機能の核心部分です。

```python
def on_sequence_callback(self, msg):
    """Handle sequence of comma-separated commands"""
    # Parse comma-separated command string
    command_string = msg.data.strip()
    self.action_queue = [cmd.strip() for cmd in command_string.split(',') if cmd.strip()]
    self.current_action_index = 0
    self.sequence_active = True
    self.sequence_id += 1
    
    self.get_logger().info(f'Starting sequence {self.sequence_id} with {len(self.action_queue)} actions')
    
    # Start the first action
    self.execute_next_action()

def execute_next_action(self):
    """Execute the next action in the sequence"""
    if not self.sequence_active or self.current_action_index >= len(self.action_queue):
        self.complete_sequence()
        return
    
    current_action = self.action_queue[self.current_action_index]
    
    # Publish sequence status
    status_msg = f'SEQUENCE:{self.sequence_id}:EXECUTING:{self.current_action_index + 1}/{len(self.action_queue)}:"{current_action}"'
    self.publish_sequence_status(status_msg)
    
    # Process the command
    self.process_command_for_sequence(current_action)

def handle_sequence_progress(self, result_str):
    """Handle progress in sequence execution"""
    if 'SUCCEEDED' in result_str or 'SKIPPED' in result_str or 'SPECIAL' in result_str:
        # Move to next action
        self.current_action_index += 1
        self.execute_next_action()
    elif 'FAILED' in result_str:
        # Continue with next action even if one fails
        self.get_logger().warn(f'Action failed, continuing with next action: {result_str}')
        self.current_action_index += 1
        self.execute_next_action()
```

### 5.4 エラーハンドリング機構

システムの安定性を確保するための重要な機能です。

```python
def retry_or_fail(self, error_code):
    """Handle retry logic or final failure"""
    self.retry_count += 1
    
    if self.retry_count <= self.max_retries:
        # For sequence mode, don't retry - just fail and move to next action
        if self.sequence_active:
            fail_msg = f'SEQUENCE:FAILED:{self.current_tag} code={error_code}'
            self.get_logger().warn(f'Sequence navigation failed, moving to next action: {self.current_tag}')
            self.publish_result(fail_msg)
        else:
            # Handle non-sequence navigation failure
            fail_msg = f'FAILED:{self.current_tag} code={error_code}'
            self.publish_result(fail_msg)
```

## 使用方法とデモ

### 6.1 インストールとビルド

```bash
# リポジトリのクローン
git clone https://github.com/NakanishiLabSFC/SDP2025Fall_LM_Planners.git
cd SDP2025Fall_LM_Planners

# 依存関係のインストール
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --packages-select target_nav

# 環境セットアップ
source install/setup.bash
```

### 6.2 座標設定手順

RViz2のPublish Point機能で取得した座標を設定ファイルに反映します。

```bash
# 1. Nav2環境を起動
ros2 launch nav2_bringup navigation_launch.py map:=~/my_map.yaml

# 2. RViz2で座標取得
ros2 launch nav2_bringup rviz_launch.py

# 3. 別ターミナルで座標監視
ros2 topic echo /clicked_point

# 4. RViz2でPublish Pointを使用してクリック
# 5. 取得した座標をobject_map.yamlに記録
```

### 6.3 基本的な使用例

```bash
# target_navノードを起動
ros2 launch target_nav target_nav.launch.py

# 【単一コマンド実行】
ros2 topic pub /robot_command_input std_msgs/String "data: 'ミャクミャクの椅子へ向かう'"

# 【シーケンス実行】複数コマンドを順次実行
ros2 topic pub /robot_command_sequence std_msgs/String "data: 'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう, 抹茶の箱へ向かう'" --once
```

### 6.4 ステータス監視

```bash
# シーケンスの実行状況を監視
ros2 topic echo /sequence_status

# ナビゲーション結果を監視
ros2 topic echo /nav_result
```

実行時の出力例：
```
SEQUENCE:1:STARTED:3 actions
SEQUENCE:1:EXECUTING:1/3:"ミャクミャクの椅子へ向かう"
SEQUENCE:1:EXECUTING:2/3:"3Dプリンターへ向かう"
SEQUENCE:1:EXECUTING:3/3:"抹茶の箱へ向かう"
SEQUENCE:1:COMPLETED:ALL:3 actions processed
```

## トラブルシューティング

開発過程で実際に遭遇した問題と解決方法を紹介します。

### 7.1 StringMultiArrayインポートエラー

**問題**: ROS2 Humbleで`StringMultiArray`がインポートできない
```python
ImportError: cannot import name 'StringMultiArray' from 'std_msgs.msg'
```

**解決方法**: `std_msgs/String`を使用してカンマ区切り文字列で実装
```python
# ❌ 問題のあるコード
from std_msgs.msg import StringMultiArray

# ✅ 修正後
from std_msgs.msg import String

def on_sequence_callback(self, msg):
    # カンマ区切り文字列を配列に変換
    command_string = msg.data.strip()
    self.action_queue = [cmd.strip() for cmd in command_string.split(',') if cmd.strip()]
```

### 7.2 --onceフラグ使用時の無限ループ問題

**問題**: コマンドに改行が含まれると`--once`が正しく認識されない
```bash
# ❌ 問題のあるコマンド（改行が原因）
ros2 topic pub /robot_command_sequence std_msgs/String "data: 
  'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう'"
  --once
```

**解決方法**: コマンドを1行で実行
```bash
# ✅ 修正後（1行で実行）
ros2 topic pub /robot_command_sequence std_msgs/String "data: 'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう'" --once
```

### 7.3 シーケンス実行中の重複コマンド問題

**問題**: 新しいシーケンスが開始されると前のシーケンスが中途半端に残る

**解決方法**: シーケンス開始時に前のシーケンスを適切に中断
```python
def on_sequence_callback(self, msg):
    # Check if a sequence is already active
    if self.sequence_active:
        self.get_logger().warn(f'New sequence received while sequence {self.sequence_id} is active. Interrupting current sequence.')
        # Cancel current navigation if any
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except:
                pass
```

### 7.4 ROS_DOMAIN_ID不一致問題

**問題**: ノード間でメッセージが届かない

**解決方法**: 統一したドメインIDの使用
```bash
# 全てのターミナルで同一のドメインIDを設定
export ROS_DOMAIN_ID=30
```

## 応用・拡張例

### 8.1 ロボットアーム制御との連携

現在のシステムはナビゲーション特化ですが、将来的にはロボットアーム制御も統合予定です。

```yaml
# 拡張予定のcommand_map.yaml
manipulation_commands:
  "ミャクミャクの椅子にあるアイテムを掴む": 
    navigation: "ミャクミャクの椅子"
    manipulation: "PICK_OBJECT"
  "抹茶の箱でアイテムを手放す":
    navigation: "抹茶の箱" 
    manipulation: "PLACE_OBJECT"
```

### 8.2 音声認識との統合

音声認識システムとの統合により、より自然なHRIを実現可能です。

```bash
# 音声認識パイプライン（構想）
音声入力 → Speech-to-Text → 自然言語処理 → target_nav → ナビゲーション実行
```

## おわりに

本記事では、ROS2 + Nav2を使用した日本語コマンドによるシーケンシャルナビゲーションシステムの実装を紹介しました。実際の開発で遭遇した技術的課題とその解決過程も含めて解説することで、読者の皆様が同様のシステムを構築する際の参考になることを目指しました。

### GitHubリポジトリ

実装コードは以下のリポジトリで公開しています：

🔗 **[SDP2025Fall_LM_Planners](https://github.com/NakanishiLabSFC/SDP2025Fall_LM_Planners)**

### 今後の開発予定

- ロボットアーム制御との統合
- 物体認識機能の追加
- 音声認識システムとの連携
- Web UIによる直感的な操作インターフェース

### 謝辞

本システムの開発にあたり、Nav2環境構築では[高木大先輩のnavigation_utilityリポジトリ](https://github.com/TKDRYU104/navigation_utility)を参考にさせていただきました。

---

**タグ**: #ROS2 #Navigation2 #自然言語処理 #ロボティクス #Python #シーケンシャル制御