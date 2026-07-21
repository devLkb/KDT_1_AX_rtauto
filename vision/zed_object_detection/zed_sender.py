"""
ZED 2i로 물체를 감지하고 3D 좌표를 UDP로 Unity에 전송하는 스크립트.

2D 물체 감지는 YOLO(ultralytics, COCO 사전학습)로 하고, 그 결과를
ZED SDK의 Custom Object Detection에 넣어서 3D 좌표/트래킹을 얻는다.
ZED 내장 감지 모델(MULTI_CLASS_BOX_ACCURATE)은 사람/차량/가방/동물/전자기기/
과일채소/스포츠용품 클래스만 있어서 텀블러, 음료수 같은 물체를 잡지 못한다.
COCO 클래스에는 cup/bottle이 포함돼 있어 YOLO 쪽이 더 잘 잡는다.

사전 설치:
    pip install pyzed ultralytics

실행 전 확인:
    - UNITY_IP: Unity가 돌아가는 PC의 IP (같은 PC면 "127.0.0.1")
    - UNITY_PORT: Unity 쪽 리시버가 열어둔 포트와 동일해야 함
"""

import json
import socket
import time

import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO

UNITY_IP = "127.0.0.1"                                    #여기에 아이피 주소 확인 ( 유니티 돌아가는 PC IP)
UNITY_PORT = 5007
SEND_INTERVAL_SEC = 0.05  # 초당 약 20회 전송

CAMERA_WINDOW = "ZED Camera"
COORDS_WINDOW = "Object Coordinates"

YOLO_WEIGHTS = "yolov8m.pt"  # COCO 사전학습 (80개 일반 사물 클래스), 처음 실행 시 자동 다운로드됨
YOLO_IMG_SIZE = 640
YOLO_CONF_THRES = 0.5


def xywh_to_corners(xywh):
    x_min = max(0, xywh[0] - 0.5 * xywh[2])
    x_max = xywh[0] + 0.5 * xywh[2]
    y_min = max(0, xywh[1] - 0.5 * xywh[3])
    y_max = xywh[1] + 0.5 * xywh[3]
    return np.array([[x_min, y_min], [x_max, y_min], [x_max, y_max], [x_min, y_max]])


def detections_to_custom_box(boxes):
    output = []
    for det in boxes:
        obj = sl.CustomBoxObjectData()
        obj.bounding_box_2d = xywh_to_corners(det.xywh[0])
        obj.label = int(det.cls.item())
        obj.probability = det.conf.item()
        obj.is_grounded = False
        output.append(obj)
    return output


def main():
    print("YOLO 모델 로딩 중...")
    model = YOLO(YOLO_WEIGHTS)
    class_names = model.names  # {class_id: name}

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    # Unity(왼손 좌표계, Y-up)와 동일한 좌표계로 바로 받기
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"카메라 열기 실패: {status}")
        return

    zed.enable_positional_tracking(sl.PositionalTrackingParameters())

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Object Detection 활성화 실패: {err}")
        zed.close()
        return

    obj_runtime_param = sl.CustomObjectDetectionRuntimeParameters()
    obj_runtime_param.object_detection_properties.detection_confidence_threshold = 40
    objects = sl.Objects()
    image_zed = sl.Mat()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cv2.namedWindow(CAMERA_WINDOW)
    cv2.namedWindow(COORDS_WINDOW)

    print(f"전송 시작: {UNITY_IP}:{UNITY_PORT} (카메라 창에서 q 또는 ESC로 종료)")

    try:
        last_sent = 0.0
        while True:
            if zed.grab() != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            frame_rgb = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2RGB)

            results = model.predict(
                frame_rgb, imgsz=YOLO_IMG_SIZE, conf=YOLO_CONF_THRES, verbose=False
            )[0].cpu().numpy()

            zed.ingest_custom_box_objects(detections_to_custom_box(results.boxes))
            zed.retrieve_custom_objects(objects, obj_runtime_param)

            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            detections = []
            for obj in objects.object_list:
                if obj.tracking_state != sl.OBJECT_TRACKING_STATE.OK:
                    continue
                x, y, z = obj.position
                label = class_names.get(obj.raw_label, str(obj.raw_label))
                detections.append(
                    {
                        "id": int(obj.id),
                        "label": label,
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                    }
                )

                corners = np.array(obj.bounding_box_2d, dtype=np.int32)
                if corners.shape[0] >= 4:
                    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                    top_left = corners[0]
                    cv2.putText(
                        frame,
                        f"{label} #{obj.id} z={z:.2f}m",
                        (int(top_left[0]), max(int(top_left[1]) - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

            coord_img = np.zeros((max(len(detections), 1) * 30 + 20, 420, 3), dtype=np.uint8)
            for i, d in enumerate(detections):
                line = f"#{d['id']} {d['label']}: x={d['x']:.2f} y={d['y']:.2f} z={d['z']:.2f}"
                cv2.putText(
                    coord_img,
                    line,
                    (10, 25 + i * 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

            cv2.imshow(CAMERA_WINDOW, frame)
            cv2.imshow(COORDS_WINDOW, coord_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

            now = time.time()
            if now - last_sent < SEND_INTERVAL_SEC:
                continue
            last_sent = now

            if detections:
                payload = json.dumps({"objects": detections}).encode("utf-8")
                sock.sendto(payload, (UNITY_IP, UNITY_PORT))

                summary = ", ".join(
                    f"{d['label']}(x={d['x']:.2f}, y={d['y']:.2f}, z={d['z']:.2f})"
                    for d in detections
                )
                print(f"[{time.strftime('%H:%M:%S')}] {len(detections)}개 감지: {summary}")

    except KeyboardInterrupt:
        print("종료합니다.")
    finally:
        cv2.destroyAllWindows()
        zed.disable_object_detection()
        zed.disable_positional_tracking()
        zed.close()
        sock.close()


if __name__ == "__main__":
    main()
