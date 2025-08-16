'''
This demo shows the communication interface of MR813 motion control board based on Lcm for crawl gait
- robot_control_cmd_lcmt.py
- file_send_lcmt.py
- Gait_Def_crawl.toml
- Gait_Params_crawl.toml
- Usergait_List_crawl.toml

匍匐前进步态演示程序
流程：检查机器人状态 -> 安全启动 -> 恢复正常姿态 -> 匍匐前进 -> 恢复到正常行走姿态
'''
import lcm
import sys
import time
import toml
import copy
import math
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from file_send_lcmt import file_send_lcmt

robot_cmd = {
    'mode':0, 'gait_id':0, 'contact':0, 'life_count':0,
    'vel_des':[0.0, 0.0, 0.0],
    'rpy_des':[0.0, 0.0, 0.0],
    'pos_des':[0.0, 0.0, 0.0],
    'acc_des':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point':[0.0, 0.0, 0.0],
    'foot_pose':[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'step_height':[0.0, 0.0],
    'value':0,  'duration':0
    }

def check_robot_status(lcm_cmd, cmd_msg):
    """检查机器人当前状态并确保安全"""
    print("🔍 检查机器人状态...")
    
    # 首先发送纯阻尼模式，确保机器人处于安全状态
    print("🛡️  进入安全模式（纯阻尼）...")
    cmd_msg.mode = 7  # PureDamper
    cmd_msg.gait_id = 0
    cmd_msg.duration = 0
    cmd_msg.life_count += 1
    
    # 发送多次确保命令被接收
    for i in range(5):
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
        time.sleep(0.2)
    
    print("✅ 机器人已进入安全模式")
    time.sleep(1.0)  # 等待机器人稳定

def safe_startup_sequence(lcm_cmd, cmd_msg):
    """安全启动序列"""
    print("🚀 执行安全启动序列...")
    
    # 步骤1：进入位置插值控制模式
    print("   步骤1：进入位置插值控制模式...")
    cmd_msg.mode = 12  # Position interpolation control
    cmd_msg.gait_id = 0
    cmd_msg.duration = 2000  # 2秒
    cmd_msg.life_count += 1
    cmd_msg.vel_des = [0.0, 0.0, 0.0]
    cmd_msg.rpy_des = [0.0, 0.0, 0.0]
    cmd_msg.pos_des = [0.0, 0.0, 0.0]  # 确保机器人处于正常高度
    
    lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
    time.sleep(2.5)  # 等待执行完成
    
    # 步骤2：再次进入纯阻尼模式，准备下一步
    print("   步骤2：进入纯阻尼模式...")
    cmd_msg.mode = 7  # PureDamper
    cmd_msg.gait_id = 0
    cmd_msg.duration = 0
    cmd_msg.life_count += 1
    
    lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
    time.sleep(1.0)
    
    print("✅ 安全启动序列完成")

def main():
    print("🤖 开始匍匐前进步态演示...")
    print("📋 完整流程：状态检查 -> 安全启动 -> 恢复正常姿态 -> 匍匐前进 -> 恢复到正常行走姿态")
    
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    usergait_msg = file_send_lcmt()
    cmd_msg = robot_control_cmd_lcmt()
    
    try:
        # 第一步：检查机器人状态并进入安全模式
        check_robot_status(lcm_cmd, cmd_msg)
        
        # 第二步：执行安全启动序列
        safe_startup_sequence(lcm_cmd, cmd_msg)
        
        # 第三步：加载匍匐前进步态参数
        print("📁 加载匍匐前进步态参数...")
        steps = toml.load("Gait_Params_crawl.toml")
        full_steps = {'step':[robot_cmd]}
        k = 0
        
        # 处理步态参数
        for i in steps['step']:
            cmd = copy.deepcopy(robot_cmd)
            cmd['duration'] = i['duration']
            if i['type'] == 'usergait':                
                cmd['mode'] = 11 # LOCOMOTION
                cmd['gait_id'] = 110 # USERGAIT - 确保这个ID与步态定义文件匹配
                cmd['vel_des'] = i['body_vel_des']
                cmd['rpy_des'] = i['body_pos_des'][0:3]
                cmd['pos_des'] = i['body_pos_des'][3:6]
                cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                cmd['acc_des'] = i['weight']
                cmd['value'] = i['use_mpc_traj']
                cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                cmd['ctrl_point'][2] =  i['mu']
            if k == 0:
                full_steps['step'] = [cmd]
            else:
                full_steps['step'].append(cmd)
            k = k + 1
            
        # 保存处理后的步态参数
        f = open("Gait_Params_crawl_full.toml", 'w')
        f.write("# Gait Params for crawl\n")
        f.writelines(toml.dumps(full_steps))
        f.close()
        print(f"✅ 步态参数处理完成，共 {k} 个步骤")

        # 第四步：发送步态定义文件
        print("📤 发送步态定义文件...")
        file_obj_gait_def = open("Gait_Def_crawl.toml",'r')
        file_obj_gait_params = open("Gait_Params_crawl_full.toml",'r')
        
        # 发送步态定义
        usergait_msg.data = file_obj_gait_def.read()
        lcm_usergait.publish("user_gait_file",usergait_msg.encode())
        print("   ✅ 步态定义文件已发送")
        time.sleep(0.5)
        
        # 发送步态参数
        usergait_msg.data = file_obj_gait_params.read()
        lcm_usergait.publish("user_gait_file",usergait_msg.encode())
        print("   ✅ 步态参数文件已发送")
        time.sleep(0.5)  # 增加等待时间确保文件被正确接收
        
        file_obj_gait_def.close()
        file_obj_gait_params.close()

        # 第五步：执行匍匐前进步态序列
        print("🎯 执行匍匐前进步态序列...")
        user_gait_list = open("Usergait_List_crawl.toml",'r')
        steps = toml.load(user_gait_list)
        
        print(f"📊 步态列表包含 {len(steps['step'])} 个阶段")
        
        for step_idx, step in enumerate(steps['step']):
            if step_idx == 0:
                print("🔄 第一阶段：恢复正常姿态...")
            elif step_idx == 1:
                print("🦎 第二阶段：执行匍匐前进...")
            elif step_idx == 2:
                print("📈 第三阶段：从匍匐姿态恢复到正常高度...")
            elif step_idx == 3:
                print("🦵 第四阶段：调整腿部位置到正常行走姿态...")
            elif step_idx == 4:
                print("🚶 第五阶段：激活正常行走步态...")
            elif step_idx == 5:
                print("🏁 第六阶段：恢复正常状态...")
                
            print(f"   执行步骤 {step_idx + 1}，模式: {step['mode']}，持续时间: {step['duration']}ms")
            
            cmd_msg.mode = step['mode']
            cmd_msg.value = step['value']
            cmd_msg.contact = step['contact']
            cmd_msg.gait_id = step['gait_id']
            cmd_msg.duration = step['duration']
            cmd_msg.life_count += 1
            
            for i in range(3):
                cmd_msg.vel_des[i] = step['vel_des'][i]
                cmd_msg.rpy_des[i] = step['rpy_des'][i]
                cmd_msg.pos_des[i] = step['pos_des'][i]
                cmd_msg.acc_des[i] = step['acc_des'][i]
                cmd_msg.acc_des[i+3] = step['acc_des'][i+3]
                cmd_msg.foot_pose[i] = step['foot_pose'][i]
                cmd_msg.ctrl_point[i] = step['ctrl_point'][i]
            for i in range(2):
                cmd_msg.step_height[i] = step['step_height'][i]
                
            lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
            time.sleep(0.1)
            
        # 维持心跳
        print("💓 维持心跳信号...")
        for i in range(100): # 15s心跳，用于在life_count不更新时维持心跳
            lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
            time.sleep(0.2)
            
        print("🎉 匍匐前进步态演示完成！")
        
    except KeyboardInterrupt:
        print("⚠️  用户中断，切换到纯阻尼模式...")
        cmd_msg.mode = 7 # PureDamper before KeyboardInterrupt
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
        pass
    except Exception as e:
        print(f"❌ 发生错误: {e}")
        # 紧急停止，切换到纯阻尼模式
        cmd_msg.mode = 7
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
    
    sys.exit()

if __name__ == '__main__':
    main() 