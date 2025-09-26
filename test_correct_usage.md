# 正しいコマンド実行方法

## 問題のあるコマンド実行
```bash
# ❌ 間違い - 改行があるため--onceが別コマンドとして認識される
ros2 topic pub /robot_command_sequence std_msgs/String "data: 
  'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう, 抹茶の箱へ向かう'"
  --once
```

## 正しいコマンド実行

### 方法1: 1行で実行 (推奨)
```bash
export ROS_DOMAIN_ID=30
ros2 topic pub /robot_command_sequence std_msgs/String "data: 'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう, 抹茶の箱へ向かう'" --once
```

### 方法2: バックスラッシュで継続
```bash
export ROS_DOMAIN_ID=30
ros2 topic pub /robot_command_sequence std_msgs/String \
  "data: 'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう, 抹茶の箱へ向かう'" --once
```

### 方法3: Pythonスクリプトを使用
```bash
export ROS_DOMAIN_ID=30
python3 test_sequence.py
```

## 実行結果の確認

### シーケンス状況の監視
```bash
export ROS_DOMAIN_ID=30
ros2 topic echo /sequence_status
```

### ナビゲーション結果の監視  
```bash
export ROS_DOMAIN_ID=30
ros2 topic echo /nav_result
```

## 期待されるログ出力
```
[target_nav]: Starting sequence 1 with 3 actions
[target_nav]: Commands: ['ミャクミャクの椅子へ向かう', '3Dプリンターへ向かう', '抹茶の箱へ向かう']
[target_nav]: Executing action 1/3: "ミャクミャクの椅子へ向かう"
[target_nav]: Navigation goal accepted: sequence=1:action=1:command=ミャクミャクの椅子へ向かう
[target_nav]: Navigation succeeded: sequence=1:action=1:command=ミャクミャクの椅子へ向かう
[target_nav]: Executing action 2/3: "3Dプリンターへ向かう"
...
[target_nav]: Sequence 1 completed
```