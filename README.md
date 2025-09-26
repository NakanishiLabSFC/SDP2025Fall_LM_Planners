# SDP2025Fall_LM_Planners

ROS2 + Nav2 環境で動作するナビゲーションノードパッケージ集です。

## 概要

このリポジトリは慶應義塾大学 SFC 中西研究室の 2025 年秋学期 SDP（Senior Design Project）で開発された LM（Language Model）ベースのプランナーコンポーネントを含んでいます。

## パッケージ構成

### 🚀 target_nav（アクティブ）

ROS2 + Nav2 環境で動作する多機能ナビゲーションノードです。

**特徴:**

- **4 つのインターフェース**: 名前指定、座標直指定、単一高レベル命令、シーケンス実行
- **Nav2 統合**: NavigateToPose アクションを使用した確実なナビゲーション
- **日本語対応**: 「ミャクミャクの椅子へ向かう」などの自然言語命令をサポート
- **シーケンス実行**: カンマ区切りで複数コマンドを順次自動実行
- **拡張可能設計**: ロボットアーム制御等の将来機能追加に対応
- **エラーハンドリング**: 失敗時も次のアクションに継続、詳細なステータス通知

**使用方法:**

nav2環境は高木大先輩のnavgation_utilityのリポジトリで動かしてください。
https://github.com/TKDRYU104/navigation_utility

```bash
# target_navノードを起動
ros2 launch target_nav target_nav.launch.py

# 単一コマンド実行の例
ros2 topic pub /robot_command_input std_msgs/String "data: 'ミャクミャクの椅子へ向かう'"
ros2 topic pub /robot_command_input std_msgs/String "data: '抹茶の箱へ向かう'"
ros2 topic pub /robot_command_input std_msgs/String "data: '3Dプリンターへ向かう'"

# シーケンス実行の例（複数コマンドを順次実行）
ros2 topic pub /robot_command_sequence std_msgs/String "data: 'ミャクミャクの椅子へ向かう, 3Dプリンターへ向かう, 抹茶の箱へ向かう'" --once

# シーケンス状況の確認
ros2 topic echo /sequence_status
```

詳細なドキュメントは [src/target_nav/README.md](src/target_nav/README.md) を参照してください。

### 📦 LM_planer（非アクティブ）

> **注意**: このパッケージは現在使用されていません。開発の初期段階で作成されたスケルトンパッケージですが、実装は `target_nav` に統合されました。

## インストール・ビルド

```bash
# リポジトリをクローン
git clone https://github.com/NakanishiLabSFC/SDP2025Fall_LM_Planners.git
cd SDP2025Fall_LM_Planners

# 依存パッケージをインストール
rosdep install --from-paths src --ignore-src -r -y

# ビルド（target_navのみ）
colcon build --packages-select target_nav

# セットアップ
source install/setup.bash
```

## 前提条件

- ROS2 Humble 以降
- Nav2 navigation stack
- Python 3.8+

## 対応命令例

### ナビゲーション命令（実装済み）

- `"ミャクミャクの椅子へ向かう"` → 指定位置へ移動
- `"抹茶の箱へ向かう"` → 指定位置へ移動
- `"3Dプリンターへ向かう"` → 指定位置へ移動
- `"行動終了"` → 終了アクション

### 将来拡張予定

- `"[場所]にあるアイテムを探す"` → 物体認識機能
- `"[場所]にある[アイテム]を掴む"` → ロボットアーム制御
- `"[場所]でアイテムを手放す"` → ロボットアーム制御

## ライセンス

Apache-2.0

## 開発者

慶應義塾大学 SFC 中西研究室
