# SO-101 3Dシミュレーター (第1段階: 関節角度入力版)

ターミナルに各軸の角度[deg]を入力すると、matplotlibの3Dビューア上で
SO-101アームがその姿勢に動きます。

## セットアップ

仮想環境を作成して依存パッケージをインストールします。

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

(既に `venv` フォルダが用意されている場合は `python -m venv venv` は不要です)

## 実行

```powershell
.\venv\Scripts\Activate.ps1
python simulator.py
```

ウィンドウが開いたら、ターミナル側のプロンプト `>>` に6軸分の角度を
スペースまたはカンマ区切りで入力してください。

```
>> 0 0 0 0 0 0
>> 30 -20 40 0 90 45
```

軸の順番:

1. `shoulder_pan`  : ベース回転
2. `shoulder_lift` : 肩の上下
3. `elbow_flex`    : 肘の曲げ
4. `wrist_flex`    : 手首の曲げ
5. `wrist_roll`    : 手首の回転
6. `gripper`       : グリッパー開閉 (約-10°=閉, 約100°=全開)

その他のコマンド:

- `help`  : 使い方と可動範囲を再表示
- `reset` : 全軸を0度に戻す
- `quit`  : 終了

## 補足

- `so101_kinematics.py` の各関節のオフセット(`JOINT_PARAMS`の`xyz`/`rpy`)と
  可動範囲(`limits_rad`)は、公式リポジトリ
  [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
  の `Simulation/SO101/so101_new_calib.urdf` から取得した値をそのまま
  使用しており、実機の3Dオフセットを忠実に再現しています。
- グリッパー指の見た目(`FINGER_LENGTH`)のみ概算値です。
- 次のステップとして、エンド位置・グリッパー位置を指定して逆運動学(IK)で
  各軸角度を自動計算する機能を追加する予定です。
