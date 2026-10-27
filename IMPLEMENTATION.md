# 実装と再現ガイド

## 最初に見るもの

1. `REPORT.md`：問い、比較、結果、限界。
2. `outputs/final_demo.mp4`：字幕入りの紹介動画。
3. `outputs/reproduction/summary.csv`：一括再実行の結果。成功させる方式だけでなく、失敗する基準方式も含む。
4. `LEARNING_GUIDE.md`：本人が理解・確認するための短い実習。

## Macで環境を作る

Apple Silicon Mac、Python 3.12を使用。以下はこのフォルダで実行する。

```sh
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python check_assets.py
python run_experiments.py --quick
```

`requirements-lock-macos.txt`は実験時の全依存ライブラリ。より厳密な再現にはrequirements.txtの代わりに使う。異なるOS・CPUでの再現性は未確認。Macの描画にはログイン中の画面セッションが必要。通常の物理計算はウィンドウを開かない。

```sh
python run_experiments.py
python verify_integrated.py --pause-start 8 --render --output integrated_demo
```

一括実行は数十本の短いシミュレーションを順番に実行する。出力先は`outputs/reproduction/`。同名の再現結果は上書きされる。配布された結果を残す場合は先に別フォルダへコピーする。

## 段階別の実装と確認方法

| 段階 | コード・実行例 | 見る点 |
|---|---|---|
| モデル確認 | `python check_assets.py` | 固定コミット由来のモデルと重みのSHA-256が一致する |
| 学習済み歩行 | `python verify_go2.py --stance-width .25 --stance-length .4 --output walking_check` | 足のトルク駆動で前進する。速度・旋回指令への追従は完全ではない |
| 位置補正 | `python verify_navigation.py --seeds 0 1 2` | 直進、旋回、目標付近での保持。位置誤差と転倒を読む |
| 直線での同行 | `python verify_person_follow.py --seeds 0 1 2` | 人が止まる、ロボットが待つ、再開する |
| 人の高さの考慮 | `python verify_clearance.py --mode robot_only` と `--mode person_aware` | 同じ出入口で経路が変わる。robot_onlyの人接触は意図した比較結果 |
| 幾何判定 | `python check_clearance_geometry.py` | 人形状と障害物距離を別計算で照合 |
| 統合 | `python verify_integrated.py --pause-start 20 --seeds 0 1 2` | 迂回中の人停止から復帰して両者が到着する |
| 待機なし比較 | `python verify_integrated.py --waiting disabled` | 到着だけでは同行品質を評価できないことを確認 |

## システムの流れ

既知の障害物地図と必要高さ → A*で経路生成 → 位置・向きのフィードバック → 学習済み歩行モデルのCPU推論 → 関節目標角 → PD制御 → MuJoCoの物理演算。

歩行モデルへの入力には姿勢、関節状態、速度指令、履歴などを用いる。物理刻みは0.002秒、歩行モデルと案内判断は0.02秒ごと。ロボット位置を経路上へ書き換えることはしない。

人モデルは実ロボットの記録軌跡に沿う。軌跡上で約0.9m後方を目標に、最大速度0.22m/s、加速度上限0.5m/s²で位置を更新する。人の歩行筋力、転倒、リードの張力は計算しない。

`guide_supervisor.py`は人の観測速度と中心間距離だけで待機・再開を決める。人が止まる時刻は実験側のみが保持し、判断器へ渡さない。距離1.08m超かつ速度0.025m/s未満が0.30秒続く、または距離1.35m超で待機。速度0.035m/s超かつ距離1.25m未満で再開。これらは本実験で決めた値であり実機用の安全基準ではない。

## 結果の読み方

- `mission_passed`：両者が目的地に入り、接触・転倒なし。
- `guidance_passed`：上記に加え、距離1.45m未満、停止条件では待機と再開があり、待機に入った後の停止期間内移動が10cm未満。
- `wait_motion_during_pause_m`：待機開始後の変位。人の停止指示からの総移動ではない。
- `min_person_clearance_m`：人のカプセル・頭球と障害物箱との符号付き距離。負になると接触として試行を終了する。
- `samples`：0.04秒ごとの時系列。接触・転倒はより細かい物理刻みで検査する。
- `events`：ウェイポイント更新、待機、再開、終了の時刻。
- `seed`：初期向き・関節角・人の開始位置の小さな変更。新しい地図や人の特性を表す番号ではない。

## 制約・失敗時の調べ方

`ModuleNotFoundError`なら使用中のPythonが`.venv/bin/python`か確認する。描画だけ失敗する場合は`--render`なしで物理計算を先に確認する。実験の`False`はPython実行の異常とは限らない。接触名、転倒、到着、待機履歴の順にJSONを見る。

重み・ロボット形状の配布元とライセンスは`THIRD_PARTY.md`。元データの出所とハッシュはmanifest群。課題原本は今回アクセスできず、提出形式の最終照合は別途必要。

## 動画・レポートを作り直す

```sh
python -m pip install -r requirements-report.txt
python verify_clearance.py --mode robot_only --render --output clearance_demo
python verify_clearance.py --mode person_aware --render --output clearance_demo
python make_clearance_comparison.py
python verify_integrated.py --render --output integrated_demo
python make_final_video.py
python make_report.py
python verify_deliverables.py
```

動画のフォントはmacOSのヒラギノ、PDFはArial Unicodeを使用。別環境では日本語フォントのパスを`--font`で指定する。フォント自体は配布しない。動画は字幕のみ、比較部分2倍速・統合部分実時間。再生速度と条件を画面にも表示する。

簡易確認（`--quick`）の出力は`outputs/quick/`。全22試行の記録を上書きしません。
