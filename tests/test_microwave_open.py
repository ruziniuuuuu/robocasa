import mujoco
import glfw
import numpy as np
import mujoco.viewer as v

import mujoco
import numpy as np
from mujoco import viewer as v

"""
    脚本描述：测试微波炉开关门

    1. 需要检查微波炉xml的按钮是否满足以下格式
        <body name="start_button" pos="0.41 -0.22 -0.13">
          <body name="start_button_body" pos="0 0 0">
              <joint name="start_button_joint" type="slide" ...../>
              <geom name="start_button_geom" class="collision" ...../>
          </body>
        </body
        .......
    2. 需要检查微波炉xml的门关节是否含有以下actuator (增加到xml末尾处即可)        
        <actuator name="door_actuator" joint="microjoint" kp="500" kv="50"/>

    3. 运行脚本开始测试，在查看器中拉拽按钮，观察结果
"""



def test_microwave(model_path):
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 获取组件ID
    door_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'door_actuator')
    door_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'microjoint')
    open_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'start_button_joint')
    close_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'close_button_joint')

    viewer = mujoco.viewer.launch_passive(model, data)

    # 初始化基准位置（前200帧稳定模型）
    open_origin = close_origin = 0.0
    for _ in range(200):
        mujoco.mj_step(model, data)
        open_origin += data.jnt(open_joint_id).qpos[0]
        close_origin += data.jnt(close_joint_id).qpos[0]
    open_origin /= 200
    close_origin /= 200

    # 控制参数
    PRESS_THRESHOLD = 0.002  # 2mm触发阈值
    DOOR_OPEN = -1.57        # 全开角度(-90°)
    DOOR_CLOSE = 0.0         # 全关角度

    # 新增时间相关参数
    DOOR_SPEED = 0.05  # 门运动速度

    current_target = DOOR_CLOSE

    while True:
        # 计算实时位移
        delta_open = data.jnt(open_joint_id).qpos[0] - open_origin
        delta_close = data.jnt(close_joint_id).qpos[0] - close_origin

        # 开按钮触发
        if delta_open > PRESS_THRESHOLD:
            current_target = DOOR_OPEN

        elif delta_close > PRESS_THRESHOLD:
            current_target = DOOR_CLOSE

        print(f"当前目标：{current_target}")

        current_angle = data.jnt(door_joint_id).qpos[0]

        # 直接设置速度
        if current_target == DOOR_OPEN:
            data.ctrl[door_actuator_id] = max(current_angle - DOOR_SPEED, current_target)
        elif current_target == DOOR_CLOSE:
            data.ctrl[door_actuator_id] = min(current_angle + DOOR_SPEED, current_target)
    

        # 实时计算控制量（比例控制）
        current_angle = data.jnt(door_joint_id).qpos[0]
        error = current_target - current_angle
        print(f"当前角度：{current_angle}, 目标角度：{current_target}, 误差：{error}")

                # 到达目标后停止
        if abs(error) < 0.01:
            data.ctrl[door_actuator_id] = 0
            print("到达目标")
        
        print(f"当前控制量：{data.ctrl[door_actuator_id]}")

        mujoco.mj_step(model, data)
        viewer.sync()


if __name__ == "__main__":

    test_microwave("./robocasa/models/assets/fixtures/microwaves/gray/model.xml")   
