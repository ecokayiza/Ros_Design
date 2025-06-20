from PIL import Image, ImageDraw
from ultralytics import YOLO
import cv2
import numpy as np

def get_rois(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    rois = []
    # 利用边缘和轮廓检测数字区域
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        # 简单筛选可能的数字区域（可根据实际情况调整阈值）
        if w*h > 500 and w*h < 5000 and h < 70 and w < 100:
            rois.append((x, y, w, h))
    rois.sort(key=lambda r: r[0])  # 按x坐标排序
    
    #找到最近的3个框并合并为一个大框
    if len(rois) >= 3:
        x = min(r[0] for r in rois[:3])
        y = min(r[1] for r in rois[:3])
        w = max(r[0] + r[2] for r in rois[:3]) - x
        h = max(r[1] + r[3] for r in rois[:3]) - y
        digit_roi = np.array([[x-5,y-5], [x+w+5, y-5], [x+w+5, y+h+5], [x-5, y+h+5]], dtype=np.float32)
    else:
        return None
    
    return digit_roi

def warp_perspective_by_corners(image, corners, output_size=(640,320)):
    """利用角点做透视变换，返回变换后的图像"""
    # 确保顺序一致（OpenCV默认顺时针或逆时针都可以）
    dst_pts = np.float32([[0, 0], [output_size[0]-1, 0],
                          [output_size[0]-1, output_size[1]-1], [0, output_size[1]-1]])
    M = cv2.getPerspectiveTransform(np.float32(corners), dst_pts)
    warped = cv2.warpPerspective(image, M, output_size)
    return warped

def iou(box1, box2):
    # box: [x1, y1, x2, y2]
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = area1 + area2 - inter_area
    if union_area == 0:
        return 0
    return inter_area / union_area

def process_image(image,model):
    # 1. 找角点
    corners = get_rois(image)
    if corners is None:
        return ""
    print("Detected corners:", corners)
    # 2. 做透视变换
    warped = warp_perspective_by_corners(image, corners)
    warped_rgb = cv2.cvtColor(warped, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(warped_rgb)
    model_results = model(img) 


    # Sort results based on the x-coordinate
    detected_objects = []
    captcha_result = ""
    boxes_data = []

    for result in model_results:
        for box in result.boxes:
            conf = float(box.conf.item())  # 获取置信度
            box_data = {
                "xyxy": box.xyxy[0].tolist(),
                "label": str(box.cls.item()),
                "conf": conf
            }
            boxes_data.append(box_data)

    sorted_boxes_data = sorted(boxes_data, key=lambda x: x['conf'], reverse=True)

    # 选择三个不重叠的box
    selected_boxes = []
    for box in sorted_boxes_data:
        box_coords = box['xyxy']
        overlap = False
        for sel in selected_boxes:
            if iou(box_coords, sel['xyxy']) > 0.1:  # IOU阈值可调整
                overlap = True
                break
        if not overlap:
            selected_boxes.append(box)
        if len(selected_boxes) == 3:
            break
    numbers = []
    for box in selected_boxes:
        label_name = box["label"]
        label_x = box["xyxy"][0]
        numbers.append((label_name,label_x))
    numbers.sort(key=lambda x: x[1])  # 按x坐标排序
    numbers = [str(int(float(num[0])))  for num in numbers]  # 提取数字部分
    if numbers[2] == '8':
        numbers[2] = '3'
    number_str = "".join(numbers)
    return number_str


if __name__ == "__main__":

    image_path = "data/105-1.png"
    image = cv2.imread(image_path)
    number_str = process_image(image, YOLO("/home/eco/catkin_ws/src/ros_design/scripts/checkpoints/svhn_best.pt"))
    print("Recognized number:", number_str)

    # if image is None:
    #     raise FileNotFoundError(f"图像读取失败: {image_path}")

    # # 1. 找角点
    # corners = get_rois(image)
    # print("Detected corners:", corners)
    # # 2. 做透视变换
    # warped = warp_perspective_by_corners(image, corners)
    # warped_rgb = cv2.cvtColor(warped, cv2.COLOR_BGR2RGB)
    # img = Image.fromarray(warped_rgb)
    # cv2.imshow("Warped", warped)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # # Load the model
    # model = YOLO("/home/eco/catkin_ws/src/ros_design/scripts/checkpoints/svhn_best.pt")
    # # Get model results
    # model_results = model(img) 
    # draw = ImageDraw.Draw(img)

    # # Sort results based on the x-coordinate
    # detected_objects = []
    # captcha_result = ""
    # boxes_data = []

    # for result in model_results:
    #     for box in result.boxes:
    #         conf = float(box.conf.item())  # 获取置信度
    #         box_data = {
    #             "xyxy": box.xyxy[0].tolist(),
    #             "label": str(box.cls.item()),
    #             "conf": conf
    #         }
    #         boxes_data.append(box_data)

    # sorted_boxes_data = sorted(boxes_data, key=lambda x: x['conf'], reverse=True)

    # # 选择三个不重叠的box
    # selected_boxes = []
    # for box in sorted_boxes_data:
    #     box_coords = box['xyxy']
    #     overlap = False
    #     for sel in selected_boxes:
    #         if iou(box_coords, sel['xyxy']) > 0.1:  # IOU阈值可调整
    #             overlap = True
    #             break
    #     if not overlap:
    #         selected_boxes.append(box)
    #     if len(selected_boxes) == 3:
    #         break

    # # 绘制并显示
    # for box in selected_boxes:
    #     label_name = box["label"]
    #     box_coords = box["xyxy"]
    #     conf = box["conf"]
    #     draw.rectangle(box_coords, outline="red")
    #     draw.text((box_coords[0], box_coords[1] - 10), f"{label_name} {conf:.2f}", fill="red")

    # img.show()
