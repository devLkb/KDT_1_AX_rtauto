"""
Roboflow 등에서 라벨링 후 내보낸 YOLOv8 포맷 데이터셋으로 YOLO를 파인튜닝하는 스크립트.

사용법:
    python train_custom_yolo.py --data path/to/dataset/data.yaml

학습이 끝나면 가장 좋은 가중치가 다음 경로에 생성됨:
    runs/detect/train/weights/best.pt
이 파일 경로를 zed_sender.py의 YOLO_WEIGHTS에 넣으면 됨.
"""

import argparse

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True, help="dataset의 data.yaml 경로")
    parser.add_argument("--base", default="yolov8s.pt", help="파인튜닝 시작점 가중치")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    opt = parser.parse_args()

    model = YOLO(opt.base)
    model.train(data=opt.data, epochs=opt.epochs, imgsz=opt.imgsz)


if __name__ == "__main__":
    main()
