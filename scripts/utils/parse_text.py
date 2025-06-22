import re

# 相似字段映射表
SIMILAR_DISH_MAP = {
    "有 那": "牛奶",
    "有 买": "牛奶",
    "有 哪": "牛奶",
    "而": "二",
    "因": "一",
}

def chinese2digits(chinese_num):
    cn_num = {'零':0, '一':1, '二':2, '两':2, '三':3, '四':4, '五':5, '六':6, '七':7, '八':8, '九':9}
    return cn_num.get(chinese_num[-1], 1)  # 只取最后一位，默认1

def replace_similar_fields(text, mapping):
    for k, v in mapping.items():
        text = re.sub(k, v, text)
    return text

def parse_order_text(order_text, dish_list):
    order_text = replace_similar_fields(order_text, SIMILAR_DISH_MAP)
    # 去掉“房间”及其前面的内容
    order_text = re.sub(r'^.*房间', '', order_text)
    result = {dish: 0 for dish in dish_list if dish}
    for dish in result:
        idx = order_text.find(dish)
        if idx != -1:
            # 向前查找最近的中文数字
            prev_text = order_text[:idx]
            match = re.findall(r'[零一二两三四五六七八九]', prev_text)
            if match:
                count = chinese2digits(match[-1])
                result[dish] = count
            else:
                result[dish] = 1  # 没有数字默认1
        else:
            result[dish] = 0
    return result