import socket
import json

# 示例目标位置
targets = [{'x': 2.5, 'y': 1.0}, {'x': -6.0, 'y': -0.5}, {'x': 5.0, 'y': 1.0}]
agents = ['localhost:8001', 'localhost:8002', 'localhost:8003']  # 代理地址

def send_data(host, port, data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(json.dumps(data).encode())
        response = s.recv(1024).decode()
    return json.loads(response)

# 初始化未分配的目标和活跃的代理
unassigned_targets = targets[:]
active_agents = agents[:]
winners = {}

# 进行多轮竞拍，直到所有目标都被分配
while unassigned_targets and active_agents:
    bids = []

    # 向所有活跃代理发送所有未分配的目标并收集出价
    for target in unassigned_targets:
        for agent in active_agents:
            host, port = agent.split(':')
            bid = send_data(host, int(port), target)
            bids.append({'agent': agent, 'bid': bid['cost'], 'target': target})

    # 确定每个目标的最低出价
    current_round_winners = {}
    for target in unassigned_targets:
        relevant_bids = [b for b in bids if b['target'] == target]
        if relevant_bids:
            min_bid = min(relevant_bids, key=lambda x: x['bid'])
            current_round_winners[min_bid['agent']] = target

    # 通知当前轮次的获胜者，并更新未分配的目标和活跃的代理
    for winner, target in current_round_winners.items():
        host, port = winner.split(':')
        send_data(host, int(port), {'win': True, 'target': target})
        if winner in active_agents:
            active_agents.remove(winner)
        if target in unassigned_targets:
            unassigned_targets.remove(target)

    winners.update(current_round_winners)

# 拍卖完成
print("所有目标已分配")
