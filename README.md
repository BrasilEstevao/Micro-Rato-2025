Here is the **clean documentation** of your robot control system based on the provided source code structure.

---

# 📁 **Project Documentation: Embedded Robot Control System**

---

## 🔧 **Config.h**

Defines configuration constants:

```cpp
#define START_BUTTON 6
#define RESET_BUTTON 26
```

* `START_BUTTON`: GPIO pin used to start the robot.
* `RESET_BUTTON`: GPIO pin used to reset the robot or state.

---

## 🧠 **State\_machine.h**

### 🕒 **Timer Structure**

```cpp
typedef struct {
    unsigned char on;
    unsigned int time;
} timerBlock;
```

* `on`: Indicates if the timer is active.
* `time`: Timer count value.

---

### 🔄 **State Machines**

#### **Main FSM States (`StateNamesMain`)**

```cpp
IDLE_MAIN = 0
MAP = 1
READY = 2
SOLVE = 3
SOLVED = 4
```

#### **Mapping FSM States (`StateNamesMap`)**

```cpp
IDLE_MAP = 0
FOLLOW_LINE_MAP = 1
U_TURN = 2
LEFT_TURN_MAP = 3
RIGHT_TURN_MAP = 4
REVERSE = 5
SMALL_FORWARD = 6
FORWARD_MAP = 7
END = 8
```

#### **Solving FSM States (`StateNamesSolve`)**

```cpp
IDLE_SOLVE = 0
FOLLOW_LINE_SOLVE = 1
GET_INSTRUCTION = 2
LEFT_TURN_SOLVE = 3
RIGHT_TURN_SOLVE = 4
FORWARD_SOLVE = 5
FINISH = 6
```

#### **Debug/Test FSM States (`StateNamesTest`)**

```cpp
FORWARD_TEST = 0
RIGHT_TURN_TEST = 1
LEFT_TURN_TEST = 2
BACKWARD_TEST = 3
STOP_TEST = 4
```

---

### ⚙️ **Global FSM Variables**

```cpp
extern StateNamesMain currentStateMain;
extern StateNamesMap currentStateMap;
extern StateNamesSolve currentStateSolve;
extern StateNamesTest currentStateTeste; // Only for DEBUG
extern bool END_MAP;
extern bool END_SOLVE;
```

---

### 🧩 **FSM Functions**

```cpp
void edge_detection();
void update_timers();
void start_timer(timerBlock* t);
void stop_timer(timerBlock* t);
void init_ST();

void Main_FSM_Handler();
void Map_FSM_Handler();
void Solve_FSM_Handler();
void Test_FSM_Handler();
```

---

## 🤖 **Robot.h**

### 🔢 **Constants**

```cpp
#define NUM_WHEELS 2
```

---

### 🔁 **Enums**

```cpp
enum control_mode_t {
    cm_pwm,
    cm_pid
};
```

* `cm_pwm`: Direct PWM control.
* `cm_pid`: PID velocity control.

---

### 🧱 **robot\_t Class Definition**

#### ⚙️ **Encoders & Sensors**

```cpp
int enc1, enc2, Senc1, Senc2;
float tof_dist, prev_tof_dist;
int TouchSwitch, LastTouchSwitch;
```

#### 🧭 **Odometry & Kinematics**

```cpp
float w1e, w2e, v1e, v2e; // wheel velocities
float ve, we;             // linear and angular velocity
float ds, dstheta;
float rel_s, rel_theta;
float xe, ye, theta;      // position and orientation
```

#### 🔋 **Motor and Control**

```cpp
int PWM_1, PWM_2;
int PWM_1_req, PWM_2_req;
control_mode_t control_mode;
float v, w, v_req, w_req;
float dv_max, dw_max;
float wheel_radius, wheel_dist;
float viref, v2ref, w1ref, w2ref;
```

#### 🧮 **PID Parameters**

```cpp
double IRkp = 8, IRki = 0.02, IRkd = 6;
double lastIRkp, lastIRki, lastIRkd;
double error, prevError, integral, derivative;
```

#### 🔋 **Others**

```cpp
float battery_voltage;
int button_state;
int solenoid_PWM;
float right_v, left_v, right_w, left_w;
float follow_v, follow_k;
```

---

### 🔧 **robot\_t Methods**

```cpp
robot_t();                            // Constructor
void setState(byte new_state);       // Set robot state
void odometry();                     // Update position estimate
void setRobotVW(float Vnom, float Wnom); // Set target linear and angular velocities
void accelerationLimit();            // Limit acceleration
void VWToMotorsVoltage();            // Convert velocities to motor voltages

// Line following and motion primitives
void followLineRight(float Vnom, float K);
void followLineLeft(float Vnom, float K);
void stop();
void followLine();
void left_turn();
void right_turn();
void u_turn();
void reverse(int leftPWM, int rightPWM);
void forward(int leftPWM, int rightPWM);

int IR_sum(); // Sum of IR sensor readings
```

---

### 🌐 **External Robot Instance**

```cpp
extern robot_t robot;
```

---

## 🧪 **DEBUG Mode**

When `DEBUG == 1`, additional debug state and FSM handler (`Test_FSM_Handler()`) are enabled for controlled testing of specific behaviors like turns, forward/reverse motions, and stop.

---

Let me know if you need class diagrams, flowcharts, or code comments added!
