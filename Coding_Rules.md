# コーディングルール

このドキュメントは、PFLIGHT_26プロジェクトにおけるコーディング規約を定義します。
主にStateManagerのコードから抽出されたルールをベースに、一貫性のある開発を実現するための指針となります。

## 目次

1. [命名規則](#命名規則)
2. [ファイルとヘッダー構成](#ファイルとヘッダー構成)
3. [コメントとドキュメンテーション](#コメントとドキュメンテーション)
4. [フォーマットとインデント](#フォーマットとインデント)
5. [C++言語機能の使用](#c言語機能の使用)
6. [デザインパターン](#デザインパターン)
7. [エラーハンドリング](#エラーハンドリング)
8. [構造体とクラス設計](#構造体とクラス設計)

---

## 命名規則

### クラス名
**PascalCase** を使用

```cpp
// Good
class StateManager { };
class InitState { };
class MotorController { };

// Bad
class state_manager { };
class initState { };
```

### メンバ変数
**snake_case + 末尾アンダースコア** を使用

```cpp
class StateManager {
private:
    bool is_first_execution_;      // Good
    StateID init_state_id_;        // Good
    StateContext state_context_;   // Good

    // Bad
    bool isFirstExecution;
    StateID initStateId;
    bool is_first_execution;  // 末尾アンダースコアなし
};
```

### 関数名・メソッド名
**camelCase** を使用

```cpp
// Good
void update();
StateContext& getContext();
void changeState(StateID state_id);

// Bad
void Update();
StateContext& get_context();
void ChangeState(StateID state_id);
```

### 列挙型
**enum class** を使用し、名前は **PascalCase**、値は **SCREAMING_SNAKE_CASE**

```cpp
// Good
enum class StateID : uint8_t {
    INIT_STATE = 1,
    PRE_FLIGHT_STATE = 2,
    MANUAL_FLIGHT_STATE = 3,
    INVALID_STATE = 255
};

// Bad
enum StateID {  // enum classを使用すべき
    initState = 1,
    preFlightState = 2
};
```

### 構造体名
**PascalCase** を使用

```cpp
// Good
struct StateContext { };
struct SensorData { };
struct ControlOutput { };

// Bad
struct state_context { };
struct sensor_data { };
```

### 定数
**SCREAMING_SNAKE_CASE** を使用（列挙型の値も含む）

```cpp
// Good
constexpr uint32_t DEFAULT_LOOP_TIME_US = 10000;
const float MAX_MOTOR_PWM = 100.0f;

// 列挙型内
enum class StateID : uint8_t {
    INIT_STATE = 1,
    EMERGENCY_STATE = 13
};
```

### ファイル名
**snake_case** を使用、拡張子は `.hpp` / `.cpp`

```
// Good
state_manager.hpp
state_manager.cpp
state_interface.hpp

// Bad
StateManager.hpp
stateManager.cpp
state_interface.h  // .hではなく.hppを使用
```

---

## ファイルとヘッダー構成

### インクルードガード
`#ifndef` / `#define` / `#endif` 形式を使用し、末尾にヘッダー名をコメント

```cpp
#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

// ヘッダーの内容

#endif // STATE_MANAGER_HPP
```

### インクルード順序
1. 対応するヘッダー（.cppファイルの場合）
2. プロジェクト内ヘッダー
3. サードパーティライブラリ
4. 標準ライブラリ

```cpp
// state_manager.cpp
#include "StateManager/state_manager.hpp"        // 1. 対応するヘッダー
#include "StateFactory/state_factory.hpp"        // 2. プロジェクト内
#include "isr_manager.hpp"                       // 2. プロジェクト内

// context.hpp
#include <cstdint>                               // 4. 標準ライブラリ
#include <array>                                 // 4. 標準ライブラリ
#include <optional>                              // 4. 標準ライブラリ
#include "usart.h"                               // 2. プロジェクト内（HAL）
#include "Vector3f.hpp"                          // 2. プロジェクト内
#include "1DoF_PID/PID.h"                        // 3. サードパーティ
```

### 前方宣言の活用
不要なインクルードを避けるため、可能な限り前方宣言を使用

```cpp
// Good - state_manager.hpp
// 前方宣言
class IMUSensor;
class MagnetometerSensor;

class StateManager {
private:
    // ポインタや参照のみの場合は前方宣言で十分
    std::unique_ptr<StateInterface> current_state_;
};

// Bad - 不要なインクルード
#include "imu_sensor.hpp"
#include "magnetometer_sensor.hpp"
```

---

## コメントとドキュメンテーション

### 日本語コメントの使用
このプロジェクトでは **日本語コメント** を積極的に使用

```cpp
// 状態管理クラス
class StateManager {

    // 初期化処理
    void init();

    // 状態遷移（StateIDを受け取りFactoryで状態を生成）
    void changeState(StateID state_id);

    // 初回実行フラグ
    bool is_first_execution_ = true;
};
```

### セクション区切り
大きなセクションの区切りには `// =====` を使用

```cpp
// ===== InitStateBase =====
// 初期化状態の共通処理を実装する基底クラス
class InitStateBase : public StateInterface {
    // ...
};

// ===== PreFlightStateBase =====
// 飛行前の共通処理を実装する基底クラス
class PreFlightStateBase : public StateInterface {
    // ...
};
```

### 関数内コメント
処理のステップを番号付きコメントで明示

```cpp
void StateManager::init() {

    // 1. 初回実行フラグをクリア
    is_first_execution_ = false;

    // 2. 使用するインスタンスの初期化

    // 2-1 センサーモジュールの初期化
    state_context_.instances.imu_sensor.emplace(/*...*/);

    // 2-2 モーターインスタンスの初期化
    state_context_.instances.left_motor.emplace(/*...*/);

    // 3. 初期状態を生成
    current_state_ = StateFactory::createState(init_state_id_);
}
```

### TODO・FIXMEコメント
将来実装する内容を明示

```cpp
// あとでLidarからのデータを受け取る部分を実装する
//
//
```

---

## フォーマットとインデント

### インデント
**タブ** または **4スペース** を使用（プロジェクトで統一）

### 中括弧の配置
**Allman スタイル** (K&Rではない)：中括弧は改行して配置

```cpp
// Good - Allman スタイル
if (condition) {

    doSomething();
}

void function() {

    // 処理
}

// Bad - K&R スタイル（使用しない）
if (condition) {
    doSomething();
}
```

### 空行の使用
- 関数やメソッドの間に1行の空行
- ブロックの開始直後に1行の空行を入れる
- 論理的なまとまりごとに空行で区切る

```cpp
void StateManager::update() {

    // 初回実行時は初期化を行う
    if (is_first_execution_) {

        init();
        return;
    }

    // SBUSデータの更新
    if (state_context_.instances.sbus_receiver.has_value()) {

        nokolat::SBUS& sbus = state_context_.instances.sbus_receiver.value();
        const nokolat::SBUS_DATA& sbus_data = sbus.getData();

        state_context_.control_input.data = sbus_data.data;
        state_context_.control_input.failsafe = sbus_data.failsafe;
    }
}
```

### 行の長さ
- 可読性を優先し、適切に改行
- 長い式や関数呼び出しは読みやすく分割

```cpp
// Good
state_context_.instances.angle_roll_pid.emplace(
    state_context_.pid_gains.angle_kp,
    state_context_.pid_gains.angle_ki,
    state_context_.pid_gains.angle_kd,
    state_context_.loop_time_us / 1000000.0f
);

// Bad - 1行が長すぎる
state_context_.instances.angle_roll_pid.emplace(state_context_.pid_gains.angle_kp, state_context_.pid_gains.angle_ki, state_context_.pid_gains.angle_kd, state_context_.loop_time_us / 1000000.0f);
```

---

## C++言語機能の使用

### std::optionalの使用
null許容なインスタンスには `std::optional<T>` を使用

```cpp
// Good
struct Instances {
    std::optional<ICM42688P_HAL_I2C> imu_sensor;
    std::optional<BMM350> mag_sensor;
    std::optional<MotorController> left_motor;
};

// 使用時
if (context.instances.imu_sensor.has_value()) {

    // .value() または -> でアクセス
    nokolat::SBUS& sbus = context.instances.sbus_receiver.value();
    context.instances.imu_sensor->Connection();
}

// Bad - 生ポインタを使用
ICM42688P_HAL_I2C* imu_sensor = nullptr;
if (imu_sensor != nullptr) {
    // ...
}
```

### スマートポインタ
動的メモリ管理には **スマートポインタ** を使用

```cpp
// Good - unique_ptr
std::unique_ptr<StateInterface> current_state_;

// 所有権の移譲
current_state_ = std::move(new_state);

// Factory での生成
static std::unique_ptr<StateInterface> createState(StateID state_id) {

    switch (state_id) {
        case StateID::INIT_STATE:
            return std::make_unique<InitState>();
        default:
            return nullptr;
    }
}

// Bad - 生ポインタ
StateInterface* current_state_;
current_state_ = new InitState();  // メモリリークの危険性
```

### enum classの使用
**enum class** を使用し、基底型を明示

```cpp
// Good
enum class StateID : uint8_t {
    INIT_STATE = 1,
    PRE_FLIGHT_STATE = 2,
    INVALID_STATE = 255
};

// 使用時は型をキャスト
printf("State: %d\n", static_cast<int>(state_id));

// Bad - 古い enum
enum StateID {
    INIT_STATE = 1,
    PRE_FLIGHT_STATE = 2
};
```

### デフォルト値の設定
メンバ変数には可能な限りデフォルト値を設定

```cpp
// Good
struct StateContext {
    uint32_t loop_time_us = 0;
};

class StateManager {
private:
    bool is_first_execution_ = true;
};

// Bad - 初期化されていない
struct StateContext {
    uint32_t loop_time_us;  // 未初期化
};
```

### 参照の活用
コピーを避けるため、適切に参照を使用

```cpp
// Good - 参照で取得
StateContext& StateManager::getContext() {
    return state_context_;
}

const nokolat::SBUS_DATA& sbus_data = sbus.getData();

// Good - const参照で受け取る
virtual StateResult update(StateContext& context) = 0;
virtual void enter(StateContext& context) = 0;
```

---

## デザインパターン

### Stateパターン
状態遷移ロジックにはStateパターンを使用

```cpp
// 状態インターフェース
class StateInterface {
public:
    virtual void enter(StateContext& context) = 0;
    virtual StateResult update(StateContext& context) = 0;
    virtual void exit(StateContext& context) = 0;
    virtual StateID getStateID() const = 0;
};

// 具体的な状態クラス
class InitState : public StateInterface {
public:
    void enter(StateContext& context) override { /* ... */ }
    StateResult update(StateContext& context) override { /* ... */ }
    void exit(StateContext& context) override { /* ... */ }
};

// 状態管理クラス
class StateManager {
private:
    std::unique_ptr<StateInterface> current_state_;

    void changeState(StateID state_id) {
        if (current_state_) {
            current_state_->exit(state_context_);
        }
        current_state_ = StateFactory::createState(state_id);
        if (current_state_) {
            current_state_->enter(state_context_);
        }
    }
};
```

### Factoryパターン
オブジェクト生成にはFactoryパターンを使用

```cpp
class StateFactory {
public:
    static std::unique_ptr<StateInterface> createState(StateID state_id) {

        switch (state_id) {
            case StateID::INIT_STATE:
                return std::make_unique<InitState>();
            case StateID::PRE_FLIGHT_STATE:
                return std::make_unique<PreFlightState>();
            default:
                return nullptr;
        }
    }
};
```

### Template Methodパターン
共通処理と個別処理を分離

```cpp
// 基底クラス（共通処理を実装）
class InitStateBase : public StateInterface {
protected:
    // 派生クラスで実装すべき処理
    virtual void onEnter(StateContext& context) = 0;
    virtual void onExit(StateContext& context) = 0;
    virtual StateResult onUpdate(StateContext& context) = 0;

public:
    // 共通処理を含むテンプレートメソッド
    void enter(StateContext& context) override {
        // 共通の初期化処理
        // ...

        // 派生クラス固有の処理
        onEnter(context);
    }

    StateResult update(StateContext& context) override {
        // 共通の更新処理
        // ...

        // 派生クラス固有の処理
        return onUpdate(context);
    }
};

// 具体クラス（個別処理を実装）
class ManualFlightState : public ManualFlightStateBase {
private:
    void onEnter(StateContext& context) override {
        // ManualFlightState固有の初期化
    }

    StateResult onUpdate(StateContext& context) override {
        // ManualFlightState固有の更新処理
        return {true, false, StateID::MANUAL_FLIGHT_STATE};
    }
};
```

---

## エラーハンドリング

### 早期リターン
エラー条件は早期にチェックして即座にリターン

```cpp
void StateManager::changeState(StateID state_id) {

    // Factoryを使用して新しい状態を生成
    std::unique_ptr<StateInterface> new_state = StateFactory::createState(state_id);

    // ポインタの安全性を確認（早期リターン）
    if (!new_state) {

        printf("[StateManager::changeState] Failed to create state (ID: %d)\n",
               static_cast<int>(state_id));
        return;
    }

    // 正常処理を続ける
    if (current_state_) {
        current_state_->exit(state_context_);
    }
    current_state_ = std::move(new_state);
}
```

### nullチェックの徹底
`std::optional` や ポインタは必ずnullチェック

```cpp
// std::optional のチェック
if (!context.instances.imu_sensor.has_value()) {

    printf("Error: IMU sensor instance is not initialized.\n");
    return {false, false, StateID::INIT_STATE};
}

// ポインタのチェック
if (!current_state_) {

    printf("[StateManager::update] StateInstance Error\n");
    changeState(StateID::EMERGENCY_STATE);
    return;
}
```

### エラーメッセージの出力
デバッグのため、エラー時は分かりやすいメッセージを出力

```cpp
// Good - 詳細な情報を含む
printf("[StateManager::changeState] Failed to create state (ID: %d)\n",
       static_cast<int>(state_id));

printf("Error: IMU sensor connection failed.\n");

// Bad - 情報が不足
printf("Error\n");
```

### StateResultによるエラー伝播
処理結果は `StateResult` 構造体で返す

```cpp
struct StateResult {
    bool success;               // 処理の成否
    bool should_transition;     // 状態遷移の有無
    StateID next_state_id;      // 次の状態ID
    const char* error_message = nullptr;
};

// 使用例
StateResult InitState::update(StateContext& context) {

    if (!context.instances.imu_sensor->Connection()) {

        printf("Error: IMU sensor connection failed.\n");
        return {false, false, StateID::INIT_STATE};
    }

    // 成功時
    return {true, true, StateID::CALIBRATION_STATE};
}

// 呼び出し側でのチェック
StateResult result = current_state_->update(state_context_);
if (!result.success) {

    printf("[StateManager::update] State Update Failed\n");
    changeState(StateID::EMERGENCY_STATE);
    return;
}
```

---

## 構造体とクラス設計

### 構造体の使用
データコンテナには構造体を使用

```cpp
// センサーデータを格納する構造体
struct SensorData {

    // IMU (ICM42688P)
    Vector3f accel;      // 加速度 [m/s^2]
    Vector3f gyro;       // 角速度 [rad/s]

    // 磁気センサー (BMM350)
    Vector3f mag;        // 磁気 [uT]

    // 計算データ
    Vector3f angle;     // 角度 [deg]
    float altitude;       // 高度 [m]
};
```

### 単位の明示
物理量には単位をコメントで明示

```cpp
struct AttitudeState {

    float roll;           // ロール角 [deg]
    float pitch;          // ピッチ角 [deg]
    float yaw;            // ヨー角 [deg]
    float roll_rate;      // ロール角速度 [deg/s]
    float pitch_rate;     // ピッチ角速度 [deg/s]
    float yaw_rate;       // ヨー角速度 [deg/s]
};
```

### 配列の使用
固定長配列には `std::array` を使用

```cpp
// Good
struct ControlOutput {
    std::array<float, 2> motor_pwm; // 2つのモーターの PWM 値 [0-100] %
    std::array<float, 4> servo_pwm; // 4つのサーボの 角度 [-90 ~ 90] deg
};

// Bad - C配列
struct ControlOutput {
    float motor_pwm[2];
    float servo_pwm[4];
};
```

### コンストラクタとデストラクタ
明示的なデフォルト/削除を使用

```cpp
class StateManager {
public:
    // コンストラクタ（明示的に定義）
    StateManager(StateID init_state_id, uint32_t loop_time_us);

    // デストラクタ（デフォルト実装を使用）
    virtual ~StateManager() = default;
};

class StateInterface {
public:
    virtual ~StateInterface() = default;
};
```

### 仮想関数のオーバーライド
`override` キーワードを必ず使用

```cpp
// Good
class InitState : public StateInterface {
public:
    void enter(StateContext& context) override;
    StateResult update(StateContext& context) override;
    void exit(StateContext& context) override;
    StateID getStateID() const override;
};

// Bad - override なし（タイポに気づけない）
class InitState : public StateInterface {
public:
    void enter(StateContext& context);  // オーバーライドされない可能性
};
```

---

## 一般化されたコード例

### 新しい状態の追加方法

#### 1. StateIDに追加

```cpp
// StateInterface/state_id.hpp
enum class StateID : uint8_t {

    // 既存の状態...
    MANUAL_FLIGHT_STATE = 4,

    // 新しい状態を追加
    NEW_CUSTOM_STATE = 14,

    INVALID_STATE = 255
};
```

#### 2. ヘッダーファイルで宣言

```cpp
// States/StateHeaders.hpp

// 新しいカスタム状態を実装するクラス
class NewCustomState : public ManualFlightStateBase {
public:
    virtual ~NewCustomState() = default;
    virtual StateID getStateID() const override;
private:
    virtual void onEnter(StateContext& context) override;
    virtual void onExit(StateContext& context) override;
    virtual StateResult onUpdate(StateContext& context) override;
};
```

#### 3. 実装ファイルを作成

```cpp
// States/ManualFlightStateBase/new_custom_state.cpp
#include "../StateHeaders.hpp"
#include "../../StateContext/context.hpp"


void NewCustomState::onEnter(StateContext& context) {

    // 状態に入る際の初期化処理
    printf("[NewCustomState] Entering state\n");
}


StateResult NewCustomState::onUpdate(StateContext& context) {

    // 定期的に呼ばれる処理

    // センサーデータの取得
    if (context.instances.imu_sensor.has_value()) {

        // センサーから値を取得
    }

    // 状態遷移の条件チェック
    bool should_transition = false;  // 遷移条件を判定

    if (should_transition) {

        return {true, true, StateID::NEXT_STATE};
    }

    // 状態を継続
    return {true, false, StateID::NEW_CUSTOM_STATE};
}


void NewCustomState::onExit(StateContext& context) {

    // 状態から出る際のクリーンアップ処理
    printf("[NewCustomState] Exiting state\n");
}


StateID NewCustomState::getStateID() const {

    return StateID::NEW_CUSTOM_STATE;
}
```

#### 4. Factoryに登録

```cpp
// StateFactory/state_factory.hpp
static std::unique_ptr<StateInterface> createState(StateID state_id) {

    switch (state_id) {

        // 既存の状態...

        // 新しい状態を追加
        case StateID::NEW_CUSTOM_STATE:
            return std::make_unique<NewCustomState>();

        default:
            return nullptr;
    }
}
```

---

## まとめ

このコーディングルールに従うことで、以下の利点が得られます：

- **一貫性**: コードベース全体で統一されたスタイル
- **可読性**: 日本語コメントと明確な命名による理解のしやすさ
- **保守性**: 明確なパターンによる変更の容易さ
- **安全性**: スマートポインタとエラーハンドリングによるバグの削減

新しいコードを書く際は、このドキュメントを参照し、既存のコードと一貫性を保つようにしてください。
