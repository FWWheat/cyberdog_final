'''
This demo shows the communication interface of MR813 motion control board based on Lcm for crawl gait
- robot_control_cmd_lcmt.py
- file_send_lcmt.py
- Gait_Def_crawl.toml
- Gait_Params_crawl.toml
- Usergait_List_crawl.toml

匍匐前进步态演示程序
流程：恢复正常姿态 -> 匍匐前进 -> 恢复正常状态
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

def main():
    print("开始匍匐前进步态演示...")
    print("流程：恢复正常姿态 -> 匍匐前进 -> 恢复正常状态")
    
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    usergait_msg = file_send_lcmt()
    cmd_msg = robot_control_cmd_lcmt()
    
    try:
        # 加载匍匐前进步态参数
        print("加载匍匐前进步态参数...")
        steps = toml.load("Gait_Params_crawl.toml")
        full_steps = {'step':[robot_cmd]}
        k = 0
        
        # 处理步态参数
        for i in steps['step']:
            cmd = copy.deepcopy(robot_cmd)
            cmd['duration'] = i['duration']
            if i['type'] == 'usergait':                
                cmd['mode'] = 11 # LOCOMOTION
                cmd['gait_id'] = 110 # USERGAIT
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

        # 发送步态定义文件
        print("发送步态定义文件...")
        file_obj_gait_def = open("Gait_Def_crawl.toml",'r')
        file_obj_gait_params = open("Gait_Params_crawl_full.toml",'r')
        usergait_msg.data = file_obj_gait_def.read()
        lcm_usergait.publish("user_gait_file",usergait_msg.encode())
        time.sleep(0.5)
        usergait_msg.data = file_obj_gait_params.read()
        lcm_usergait.publish("user_gait_file",usergait_msg.encode())
        time.sleep(0.1)
        file_obj_gait_def.close()
        file_obj_gait_params.close()

        # 执行匍匐前进步态序列
        print("执行匍匐前进步态序列...")
        user_gait_list = open("Usergait_List_crawl.toml",'r')
        steps = toml.load(user_gait_list)
        
        for step_idx, step in enumerate(steps['step']):
            if step_idx == 0:
                print("第一阶段：恢复正常姿态...")
            elif step_idx == 1:
                print("第二阶段：执行匍匐前进...")
            elif step_idx == 2:
                print("第三阶段：恢复正常状态...")
                
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
        print("维持心跳信号...")
        for i in range(100): # 15s心跳，用于在life_count不更新时维持心跳
            lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
            time.sleep(0.2)
            
        print("匍匐前进步态演示完成！")
        
    except KeyboardInterrupt:
        print("用户中断，切换到纯阻尼模式...")
        cmd_msg.mode = 7 # PureDamper before KeyboardInterrupt
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
        pass
    except Exception as e:
        print(f"发生错误: {e}")
        # 紧急停止，切换到纯阻尼模式
        cmd_msg.mode = 7
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd",cmd_msg.encode())
    
    sys.exit()

if __name__ == '__main__':
    main() 