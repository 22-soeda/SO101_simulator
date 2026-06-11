# SO-101 3Dシミュレーター

PyVista(VTKベース)の3Dビューア上でSO-101アームの姿勢を確認できる
シミュレーターです。左ドラッグで視点を回転(常にZ軸を上に保つ
ターンテーブル操作)、スクロールでズームできます。
2種類の操作方法があります。

- `simulator.py`          : ターミナルに各軸の角度[deg]を入力して動かす版
- `simulator_ik.py`       : スライダーで手先位置・姿勢を指定し、逆運動学(IK)で
  動かす版
- `simulator_ik_sync.py`  : `simulator_ik.py`に加えて、実機のFEETECHサーボへ
  関節角度をリアルタイムに送信する版

## セットアップ

リポジトリをクローンし、仮想環境を作成して依存パッケージを
インストールします。

```powershell
git clone <このリポジトリのURL>
cd SO101_simulator
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

C++版IK拡張(任意・高速化)をビルドする場合は、続けて
[C++版IK拡張](#c版ik拡張-任意高速化)の手順も実行してください。
拡張をビルドしなくても、純粋なPython実装で動作します。

## 実行 (関節角度入力版)

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

## 実行 (IK・スライダー操作版)

```powershell
.\venv\Scripts\Activate.ps1
python simulator_ik.py
```

ウィンドウ右側の6本のスライダーを動かすと、アームがその通りに動きます。

- `X [m]` / `Y [m]` / `Z [m]` : グリッパー先端(TCP)の目標位置
- `Pitch [deg]`                : グリッパー先端が向く方向の、水平面に対する角度
- `wrist_roll [deg]`           : 手首の回転 (IKの対象外、直接指定)
- `gripper [deg]`               : グリッパーの開閉 (IKの対象外、直接指定)

`X`/`Y`/`Z`/`Pitch` を変更すると、数値IK(ヤコビアン+反復法)で
`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex` の4関節角度が
自動計算されます。目標が可動範囲外などで到達できない場合は、最も近い
姿勢になります。

## 実行 (実機サーボ同期版)

`simulator_ik.py`と同じ操作に加えて、接続されているFEETECHサーボへ
関節角度をリアルタイムに送信します。

### 接続設定

`servo_config.py` でポート・ボーレート・各関節とサーボIDの対応を設定します
(デフォルト: `COM12`, 1Mbps, `shoulder_pan`=ID1 〜 `gripper`=ID6)。

### キャリブレーション

実機の角度とシミュレーターの論理角度を合わせるため、初回は
キャリブレーションを行います。

1. 各サーボを、シミュレーターの基準姿勢(全軸0度)に対応する実機の
   姿勢に手で動かしておく。
2. 以下を実行する。

   ```powershell
   .\venv\Scripts\Activate.ps1
   python calibrate_servos.py
   ```

3. 接続されているサーボごとに、回転方向(`1`または`-1`)と現在の論理角度
   (基準姿勢なら`0`)を入力する。`Enter`のみでデフォルト値を使用できる。

設定値は `servo_calibration.json` (gitignore対象、実機ごとに異なる) に
保存され、各関節の以下の値が含まれる:

- `home_position` : 論理角度0度に対応するサーボ位置
- `direction`     : サーボ位置の増加方向と論理角度の+方向の関係 (`1`/`-1`)
- `position_min` / `position_max` : `so101_kinematics.py`の可動範囲
  (`JOINT_LIMITS_DEG`)から計算される、サーボに送信してよい位置の範囲
  (ソフトリミット)

### 実行

```powershell
.\venv\Scripts\Activate.ps1
python simulator_ik_sync.py
```

起動時に接続されているサーボIDが表示される。`servo_config.py`の
`JOINT_TO_SERVO_ID`に登録されているIDのうち、実際に接続されている
関節のみ実機に角度が送信される(未接続の関節は無視されシミュレーターの
表示のみ更新される)。

## 補足

- `so101_kinematics.py` の各関節のオフセット(`JOINT_PARAMS`の`xyz`/`rpy`)と
  可動範囲(`limits_rad`)は、公式リポジトリ
  [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
  の `Simulation/SO101/so101_new_calib.urdf` から取得した値をそのまま
  使用しており、実機の3Dオフセットを忠実に再現しています。
- グリッパー指の見た目(`FINGER_LENGTH`)のみ概算値です。
- IKは `so101_ik.py` でヤコビアンの数値微分+疑似逆行列によるGauss-Newton法で
  解いています。特異姿勢付近では振動することがあるため、反復中で最も
  誤差が小さかった角度を採用しています。

## C++版IK拡張 (任意・高速化)

`so101_ik.py`のIK計算は、`cpp_ik/`にビルド済みのC++拡張
(`so101_ik_cpp`)があれば自動的にそちらを使う(同じアルゴリズムで
約100倍高速・計算中はGILを解放)。拡張が無くても純粋なPython実装に
自動でフォールバックするため、ビルドは必須ではない。

ビルドするには、Visual StudioのMSVC C++ビルドツールが必要:

```powershell
.\venv\Scripts\Activate.ps1
pip install pybind11 setuptools
.\cpp_ik\build.ps1
```

成功すると `cpp_ik\so101_ik_cpp*.pyd` が生成される(gitignore対象、
Pythonバージョン・環境ごとに再ビルドが必要)。
