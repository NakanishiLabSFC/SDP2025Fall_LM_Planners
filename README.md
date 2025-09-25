# SDP2025Fall_LM_Planners

ROS2 + Nav2環境で動作するナビゲーションノードパッケージ集です。

## 概要

このリポジトリは慶應義塾大学SFC中西研究室の2025年秋学期SDP（Senior Design Project）で開発されたLM（Language Model）ベースのプランナーコンポーネントを含んでいます。

## パッケージ構成

### 🚀 target_nav（アクティブ）
ROS2 + Nav2環境で動作する多機能ナビゲーションノードです。

**特徴:**
- **3つのインターフェース**: 名前指定、座標直指定、高レベル命令
- **Nav2統合**: NavigateToPoseアクションを使用した確実なナビゲーション
- **日本語対応**: 「ミャクミャクの椅子へ向かう」などの自然言語命令をサポート
- **拡張可能設計**: ロボットアーム制御等の将来機能追加に対応
- **エラーハンドリング**: リトライ機能と詳細なステータス通知

**使用方法:**

nav2環境は高木大先輩のnavgation_utilityのリポジトリで動かしてください。
https://github.com/TKDRYU104/navigation_utility

```bash
# target_navノードを起動
ros2 launch target_nav target_nav.launch.py

# 高レベル日本語命令の例
ros2 topic pub /robot_command std_msgs/String "data: 'ミャクミャクの椅子へ向かう'"
ros2 topic pub /robot_command std_msgs/String "data: '抹茶の箱へ向かう'"
ros2 topic pub /robot_command std_msgs/String "data: '3Dプリンターへ向かう'"
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

慶應義塾大学SFC中西研究室
