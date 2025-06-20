import random
if __name__ == '__main__':
    consumer_goals ={
        "101": (4.742, -0.752, -90),
        "102": (2.135, -0.714, -90),
        "103": (-0.305, -0.737, -90),
        "104": (-0.291, 0.543, 90),
        "105": (1.865, 0.540, 90),       
    }
    goal_list = [("原点", (0,0,0))]
    des, goal = goal_list[0]
    print("Initial goal:", des, goal)
    goal = random.choice(list(consumer_goals.items()))
    print("Consumer goals:", goal)