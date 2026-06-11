# SO-101 3Dシミュレーター

PyVista(VTKベース)の3Dビューア上でSO-101アームの姿勢を確認できる
シミュレーターです。左ドラッグで視点を回転(常にZ軸を上に保つ
ターンテーブル操作)、スクロールでズームできます。
3種類の操作方法があります。

- `simulator.py`          : ターミナルに各軸の角度[deg]を入力して動かす版
- `simulator_ik.py`       : スライダーで手先位置・姿勢を指定し、逆運動学(IK)で
  動かす版
- `simulator_ik_sync.py`  : `simulator_ik.py`に加えて、実機のFEETECHサーボへ
  関節角度をリアルタイムに送信する版

いずれも起動時の姿勢は `home_position.json` に保存された
**ホームポジション**です(ファイルが無ければ全軸0度)。実機を使う場合の
キャリブレーション・ホームポジションの設定方法は
[キャリブレーションとホームポジション](#キャリブレーションとホームポジション)
を参照してください。

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
- `reset` : 全軸を0度(初期姿勢)に戻す。ホームポジションとは別の姿勢なので
  注意(起動時の姿勢に戻すにはスクリプトを再起動する)。
- `quit`  : 終了

## 実行 (IK・スライダー操作版)

```powershell
.\venv\Scripts\Activate.ps1
python simulator_ik.py
```

ウィンドウ右側の6本のスライダーを動かすと、アームがその通りに動きます。
スライダーの初期値は、ホームポジションに対応する手先位置・ピッチ角・
`wrist_roll`/`gripper`角度になります。

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
関節角度をリアルタイムに送信します。実機を使う前に
[キャリブレーションとホームポジション](#キャリブレーションとホームポジション)
の設定を行ってください。

```powershell
.\venv\Scripts\Activate.ps1
python simulator_ik_sync.py
```

起動時に接続されているサーボIDが表示される。`servo_config.py`の
`JOINT_TO_SERVO_ID`に登録されているIDのうち、実際に接続されている
関節のみ実機に角度が送信される(未接続の関節は無視されシミュレーターの
表示のみ更新される)。

## キャリブレーションとホームポジション

実機のFEETECHサーボを使う場合、2つの設定ファイルを用意します。
どちらもgitignore対象で、実機(個体)ごとに異なる値を持ちます。

| ファイル | 内容 | 設定するスクリプト |
| --- | --- | --- |
| `servo_calibration.json` | 各関節の**初期姿勢**(論理角度0度)に対応するサーボの生位置・回転方向・可動範囲 | `calibrate_servos.py` / `calibrate_servos_visual.py` |
| `home_position.json` | シミュレーター起動時の姿勢(**ホームポジション**)。初期姿勢(0度)から各関節を何度動かした姿勢かを論理角度[deg]で保存 | `calibrate_servos_visual.py`(終了時に自動保存) / `set_home_position.py` |

### 接続設定

`servo_config.py` でポート・ボーレート・各関節とサーボIDの対応を設定します
(デフォルト: `COM12`, 1Mbps, `shoulder_pan`=ID1 〜 `gripper`=ID6)。

### 1. 初期姿勢のキャリブレーション

#### 対話版 (推奨): calibrate_servos_visual.py

3Dシミュレーターを見ながら、回転方向・初期姿勢(0度)・可動範囲を
まとめて設定できます。

```powershell
.\venv\Scripts\Activate.ps1
python calibrate_servos_visual.py
```

1. 起動すると接続中のサーボのトルクがOFFになり、アームを手で動かせる
   状態のまま各関節の現在角度を読み取り続ける。アームを動かすと、
   3D上の対応する軸もリアルタイムに連動する。
2. 画面左の赤いチェックボックス(上から `shoulder_pan` 〜 `gripper` の順)
   は各関節の回転方向。チェックを入り切りすると回転方向の符号が反転し、
   3D上の回転方向も反転するので、実機とシミュレーターの回転方向が
   一致するように調整する。
3. 実機を、3D上の論理角度0度(初期姿勢)に対応する姿勢に手で動かし、
   緑のチェックボックスをクリックする。その時点のサーボ位置が論理角度
   0度のオフセット(`home_position`)として、回転方向とあわせて
   `servo_calibration.json` に保存される。
4. 各関節を可動範囲の両端まで手で動かすと、画面に観測した角度範囲が
   `range=[min, max]` として表示される。一通り動かし終えたら青の
   チェックボックスをクリックすると、観測範囲が`position_min`/
   `position_max`として保存される(5度未満しか動かしていない関節は
   既存の値のまま変更されない)。橙のチェックボックスで範囲計測を
   リセットできる。
5. ウィンドウを閉じると、その時点の姿勢が**ホームポジション**として
   `home_position.json` に自動保存される(後述)。

#### コマンドライン版: calibrate_servos.py

3D表示なしの、数値入力のみの簡易版です。

```powershell
.\venv\Scripts\Activate.ps1
python calibrate_servos.py
```

接続されているサーボごとに、回転方向(`1`または`-1`)と現在の論理角度
(初期姿勢なら`0`)を入力します。`Enter`のみでデフォルト値を使用できます。
`position_min`/`position_max`は`so101_kinematics.py`の可動範囲
(`JOINT_LIMITS_DEG`)から自動計算されます。

`servo_calibration.json`に保存される値:

- `home_position` : 論理角度0度に対応するサーボ位置
- `direction`     : サーボ位置の増加方向と論理角度の+方向の関係 (`1`/`-1`)
- `position_min` / `position_max` : サーボに送信してよい位置の範囲
  (ソフトリミット)

### 2. ホームポジションの設定

`home_position.json`には、各関節の論理角度[deg]として「初期姿勢(0度)から
何度動かした姿勢か」を保存します。`simulator.py` / `simulator_ik.py` /
`simulator_ik_sync.py`は、起動時にこの姿勢から始まります
(ファイルが無ければ全軸0度)。

`calibrate_servos_visual.py`のウィンドウを閉じると自動的に保存されます
(上記の手順5)。3D表示なしで設定し直したい場合は、次のスクリプトを
使います。

```powershell
.\venv\Scripts\Activate.ps1
python set_home_position.py
```

接続中のサーボのトルクをOFFにし、各関節の論理角度を読み取り続けます。
アームを手でホームポジションにしたい姿勢へ動かしてから`Ctrl+C`を押すと、
その時点の姿勢が`home_position.json`に保存されます。

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
