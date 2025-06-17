import subprocess
import time

def start_script(script_name):
    return subprocess.Popen(['python3', script_name])

def main():

    # 启动代理脚本
    agent1_process = start_script('agent1_dynmic.py')
    agent2_process = start_script('agent2_dynmic.py')
    agent3_process = start_script('agent3_dynmic.py')
    agent4_process = start_script('agent4_dynmic.py')
    agent5_process = start_script('agent5_dynmic.py')
    agent6_process = start_script('agent6_dynmic.py')
    
    # 确保所有代理脚本都已启动
    time.sleep(1)  # 适当等待确保所有代理都已启动

    # 启动拍卖协调者脚本
    auction_process = start_script('Coordinator_Dynamic.py')

    # 等待所有进程完成
    try:
        auction_process.wait()
        agent1_process.wait()
        agent2_process.wait()
        agent3_process.wait()
        agent4_process.wait()
        agent5_process.wait()
        agent6_process.wait()
    except KeyboardInterrupt:
        auction_process.terminate()
        agent1_process.terminate()
        agent2_process.terminate()
        agent3_process.terminate()
        agent4_process.terminate()
        agent5_process.terminate()
        agent6_process.terminate()


if __name__ == '__main__':
    main()
