import subprocess
import time

def start_script(script_name):
    return subprocess.Popen(['python3', script_name])

def main():

    # 启动代理脚本
    agent1_process = start_script('a1.py')
    agent2_process = start_script('a2.py')
    agent3_process = start_script('a3.py')

    # 确保所有代理脚本都已启动
    time.sleep(2)  # 适当等待确保所有代理都已启动

    # 启动拍卖协调者脚本
    auction_process = start_script('coordi.py')

    # 等待所有进程完成
    try:
        auction_process.wait()
        agent1_process.wait()
        agent2_process.wait()
        agent3_process.wait()
    except KeyboardInterrupt:

        auction_process.terminate()
        agent1_process.terminate()
        agent2_process.terminate()
        agent3_process.terminate()

if __name__ == '__main__':
    main()
