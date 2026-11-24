# プロジェクト概要：BLE受信 ➔ USB HIDキーボード入力中継デバイス

## 1. 目的とシステム構成
パスワードマネージャなどで生成・取得したパスワードを使って、PCのログインやアカウント初期設定を行うため、入力元PCから出力先PC/スマホへキーボード入力を行うデバイスを構成する。

## 登場するデバイス
- 入力先デバイス: USB キーボードの形式で文字入力を受け入れるデバイス。デバイスの種類は Windows, Mac, Android に対応。 USB HID Bridge とはUSBケーブルで接続。
- 入力元デバイス: 入力先デバイスに入力するキーイベントに対応する文字列を入力して、USB HID Bridgeに送出するデバイス。 Google Chromeが動く デバイス種類は Windows, Mac, Linux PC。
- USB HID Bridge: 入力元デバイスから送出された文字列を受け取り、入力先デバイスのUSB HID Keyboardプロトコルでキーイベントを送出。入力元デバイスとは Bluetoothで通信し、入力先デバイスにはUSB OTGポートからUSB HID Keyboardとしてキーイベントを送出。

入力元PCではWebブラウザのUIを用いたツールからキーボードで入力する内容を指定する。入力先PC/スマホにはUSB接続を行い USB キーボード(USB HID)として振る舞う。

```text
[ 送信側PC (Chrome拡張機能 / Web Bluetooth API) ]
    ↓  BLE GATT 書き込み (無線)
[ ESP32-S3 (NimBLE受信 ➔ TinyUSB HID変換) ]
    ↓  USB OTGケーブル 1本 (有線)
[ 入力先ターゲットPC (標準USBキーボードとして認識) ]
```

## 実装言語、フレームワーク
- ESP-IDF を利用
- C言語で実装

## VS Code の IntelliSense
C/C++ 拡張機能は `build/compile_commands.json` を参照する設定になっています。
ESP-IDF の環境を有効にして `idf.py build` を実行すると、このファイルが生成されます。
ビルドディレクトリを変更した場合は、VS Code の `C_Cpp.default.compileCommands` も変更してください。

## 各パートでの要件
- 入力元PCからESP32-S3 へのBluetooth通信には BLEペアリングを利用して暗号化する
- デバイスの機体を識別できるようにするために、Bluetoothデバイス名にMACアドレスを含める。
- USB HID として mount されていないなどのエラーを利用者に通知するために、戻り値をWeb UI で通知するようにする。



