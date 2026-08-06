### 3.1.2 键盘控制实现

#### 1.源文件 {#1源文件}

功能包go2\_teleop\_ctrl\_keyboard的go2\_teleop\_ctrl\_keyboard目录下，新建Python文件go2\_teleop\_ctrl\_keyboard.py，并编辑文件，输入如下内容：

```py
import sys
import threading

from unitree_api.msg import Request
import rclpy
import json

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as unitree_api/msg/Request messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   q    w    e
   a    x    d
   z    s    c

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   Q    W    E
   A    X    D
   Z    S    C


anything else : stop

r/t : increase/decrease max speeds by 10%
f/g : increase/decrease only linear speed by 10%
v/b : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

# 定义常量字典
ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,                    # 阻尼控制
    "BALANCESTAND": 1002,            # 平衡站立
    "STOPMOVE": 1003,                # 停止运动
    "STANDUP": 1004,                 # 站立
    "STANDDOWN": 1005,               # 站立下降
    "RECOVERYSTAND": 1006,           # 恢复站立
    "EULER": 1007,                   # 欧拉角控制
    "MOVE": 1008,                    # 移动
    "SIT": 1009,                     # 坐下
    "RISESIT": 1010,                 # 从坐下恢复站立
    "SWITCHGAIT": 1011,              # 切换步态
    "TRIGGER": 1012,                 # 触发
    "BODYHEIGHT": 1013,              # 身体高度调整
    "FOOTRAISEHEIGHT": 1014,         # 脚部抬起高度调整
    "SPEEDLEVEL": 1015,              # 速度级别调整
    "HELLO": 1016,                   # 打招呼
    "STRETCH": 1017,                 # 伸展
    "TRAJECTORYFOLLOW": 1018,        # 轨迹跟随
    "CONTINUOUSGAIT": 1019,          # 连续步态
    "CONTENT": 1020,                 # 内容
    "WALLOW": 1021,                  # 打滚
    "DANCE1": 1022,                  # 舞蹈1
    "DANCE2": 1023,                  # 舞蹈2
    "GETBODYHEIGHT": 1024,           # 获取身体高度
    "GETFOOTRAISEHEIGHT": 1025,      # 获取脚部抬起高度
    "GETSPEEDLEVEL": 1026,           # 获取速度级别
    "SWITCHJOYSTICK": 1027,          # 切换操纵杆
    "POSE": 1028,                    # 姿态
    "SCRAPE": 1029,                  # 刮擦
    "FRONTFLIP": 1030,               # 前空翻
    "FRONTJUMP": 1031,               # 前跳
    "FRONTPOUNCE": 1032              # 前扑
}

moveBindings = {
    'w': (1, 0, 0, 0),
    'e': (1, 0, 0, -1),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'q': (1, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'c': (-1, 0, 0, 1),
    'z': (-1, 0, 0, -1),
    'E': (1, -1, 0, 0),
    'W': (1, 0, 0, 0),
    'A': (0, 1, 0, 0),
    'D': (0, -1, 0, 0),
    'Q': (1, 1, 0, 0),
    'S': (-1, 0, 0, 0),
    'C': (-1, -1, 0, 0),
    'Z': (-1, 1, 0, 0),
}

speedBindings = {
    'r': (1.1, 1.1),
    't': (.9, .9),
    'f': (1.1, 1),
    'g': (.9, 1),
    'v': (1, 1.1),
    'b': (1, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('go2_teleop_ctrl_keyboard')

    pub = node.create_publisher(Request, '/api/sport/request', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.2
    turn = 0.4
    x = 0.0
    y = 0.0
    api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
    th = 0.0
    status = 0.0

    req = Request()

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                th = 0.0
                api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
                if (key == '\x03'):
                    break

            js = {
                "x": x * speed,
                "y": y * speed,
                "z": th * turn 
            }
            req.parameter = json.dumps(js)
            req.header.identity.api_id = api_id
            pub.publish(req)

    except Exception as e:
        print(e)

    finally:

        js = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0 
        }
        req.header.identity.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        req.parameter = json.dumps(js)

        pub.publish(req)

        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
```

#### 2.编辑配置文件 {#2编辑配置文件}

##### 1.package.xml {#1packagexml}

在创建功能包时，所依赖的功能包已经自动配置了，配置内容如下：

```
<depend>rclpy</depend>
<depend>unitree_api</depend>
```

##### 2.setup.py {#2setuppy}

setup.py文件中`entry_points`字段的`console_scripts`中添加如下内容：

```
entry_points={
    'console_scripts': [
        'go2_teleop_ctrl_keyboard = go2_teleop_ctrl_keyboard.go2_teleop_ctrl_keyboard:main'
    ],
},
```

#### 3.编译 {#3编译}

终端中进入当前工作空间，编译功能包：

```
colcon build --packages-select go2_teleop_ctrl_keyboard
```

#### 4.执行 {#4执行}

当前工作空间下，启动终端，并输入如下指令：

```
ros2 run go2_teleop_ctrl_keyboard go2_teleop_ctrl_keyboard
```

各个功能按键的作用如下：

**移动按键**

* `w`: 向前移动。
* `e`: 向后并向左转移动（在普通模式下）。
* `a`: 向左转。
* `d`: 向右转。
* `q`: 向前并向右转移动（在普通模式下）。
* `s`: 向后移动。
* `c`: 向后并向左转移动（在全向模式下，与`e`相对）。
* `z`: 向前并向右转移动（在全向模式下，与`q`相对）。

当按下`Shift`键时，机器人进入全向模式（Holonomic mode），允许它进行侧向移动（strafing）：

* `W`: 向前移动（全向模式与`w`相同）。
* `E`: 向左移动。
* `A`: 向后并向左移动。
* `D`: 向后并向右移动。
* `Q`: 向右移动。
* `S`: 停止移动。
* `C`: 向前并向左移动。
* `Z`: 向前并向右移动。

**速度调整按键**

* `r`: 增加最大速度和转向速度10%。
* `t`: 减少最大速度和转向速度10%。
* `f`: 仅增加线性速度10%（不影响转向速度）。
* `g`: 仅减少线性速度10%。
* `v`: 仅增加转向速度10%（不影响线性速度）。
* `b`: 仅减少转向速度10%。



