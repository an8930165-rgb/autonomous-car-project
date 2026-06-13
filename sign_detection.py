import cv2
import numpy as np
import torch
from ultralytics import YOLO

# Ép PyTorch chỉ dùng 1 luồng CPU
# Tránh tranh tài nguyên với lane_detection
torch.set_num_threads(1)

class SignDetector:
    def __init__(self, model_path="Traffic_signss_v7.pt", base_speed=80):
        print(f"[INFO] Đang nạp mô hình AI: {model_path}...")
        self.model = YOLO(model_path)

        # Tốc độ nền do Web Server gửi xuống
        self.base_speed = base_speed

        # Tốc độ THỰC TẾ của xe (Sẽ bị thay đổi khi gặp biển báo)
        self.current_speed = base_speed

        # =====================================================
        # Cache kernel sharpen — tính 1 lần khi khởi động
        # Dùng mãi, không tính lại mỗi frame như code cũ
        # =====================================================
        self._sharpen_kernel = np.array([
            [ 0, -1,  0],
            [-1,  5, -1],
            [ 0, -1,  0]
        ], dtype=np.float32)

    # =====================================================
    # THUẬT TOÁN UNSHARP MASK (Kích nét chống Camera Blur)
    # Dùng filter2D 1 bước thay vì GaussianBlur + addWeighted
    # Kết quả tương đương nhưng nhanh hơn ~30%
    # =====================================================
    def _sharpen_frame(self, frame):
        return cv2.filter2D(frame, -1, self._sharpen_kernel)

    def process(self, frame):
        """
        Nhận frame RGB 640x480 từ camera (do main.py gửi vào).
        Không convert màu — main.py đã xử lý đúng RGB/BGR rồi.
        """
        # =====================================================
        # BƯỚC 1: THU NHỎ FRAME ĐỂ AI CHẠY NHANH HƠN
        # Bỏ ROI cắt vùng vì biển nằm ở rìa trái/phải
        # → Cắt ROI dễ mất biển với camera top-down
        # Thu nhỏ toàn frame 640x480 → 320x240 (giảm 4x khối lượng)
        # =====================================================
        scale_factor = 2
        small_frame = cv2.resize(frame, (640 // scale_factor, 480 // scale_factor))

        # =====================================================
        # BƯỚC 2: TIỀN XỬ LÝ VÀ CHẠY MÔ HÌNH YOLO
        #
        # Chỉ sharpen khi speed <= 70 (xe đứng im hoặc chậm)
        # Lúc này blur do camera rung → sharpen có tác dụng ✅
        #
        # Khi speed > 70 (tức 80 PWM — nhanh nhất):
        # Blur do chuyển động có hướng → sharpen vô tác dụng
        # Bỏ sharpen để tiết kiệm CPU cho lane_detection ✅
        # =====================================================
        if self.current_speed > 70:
            processed_frame = small_frame
        else:
            processed_frame = self._sharpen_frame(small_frame)

        results = self.model(processed_frame, conf=0.10, verbose=False)

        sign_detected = False
        detected_sign_name = "None"

        # Vẽ dải viền mờ màu xám báo hiệu vùng AI đang quét
        cv2.line(frame, (0, 0), (640, 0), (100, 100, 100), 2)
        cv2.putText(frame, "AI SCAN AREA", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

        # =====================================================
        # BƯỚC 3: LỌC VÀ ÁNH XẠ NGƯỢC TỌA ĐỘ
        # =====================================================
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            temp_sign_name = self.model.names[class_id].upper()
            score = float(box.conf[0])

            # -----------------------------------------------------
            # THRESHOLD THEO TỐC ĐỘ THỰC TẾ
            #
            # STOP khi speed >= 80: 0.50 (không để quá thấp —
            # xe đang chạy nhanh nhất mà dừng đột ngột rất nguy hiểm)
            #
            # 30/60: 0.40 — cân bằng giữa không miss và không nhầm
            # Nếu vẫn nhầm 30↔60 → tăng dần lên 0.45 → 0.50 → 0.55
            # -----------------------------------------------------
            if self.current_speed >= 80:
                if "STOP" in temp_sign_name:
                    required_conf = 0.50
                elif "30" in temp_sign_name:
                    required_conf = 0.40
                elif "60" in temp_sign_name:
                    required_conf = 0.40
                else:
                    required_conf = 0.70
            else:
                if "STOP" in temp_sign_name:
                    required_conf = 0.75
                elif "30" in temp_sign_name:
                    required_conf = 0.40
                elif "60" in temp_sign_name:
                    required_conf = 0.40
                else:
                    required_conf = 0.70

            if score < required_conf:
                continue

            # -----------------------------------------------------
            # ÁNH XẠ NGƯỢC TỌA ĐỘ từ ảnh nhỏ 320x240 về ảnh gốc 640x480
            # Không cộng y_offset vì đã bỏ ROI
            # -----------------------------------------------------
            x1_small, y1_small, x2_small, y2_small = map(int, box.xyxy[0])

            x1 = x1_small * scale_factor
            y1 = y1_small * scale_factor
            x2 = x2_small * scale_factor
            y2 = y2_small * scale_factor

            box_area = (x2 - x1) * (y2 - y1)

            cv2.putText(frame, f"Area: {box_area}", (x1, y1 - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # =======================================================
            # MÀNG LỌC VẬT LÝ: Chỉ chốt khi biển đủ gần (Area >= 300)
            # =======================================================
            if box_area < 300:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                continue

            detected_sign_name = temp_sign_name
            sign_detected = True

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{detected_sign_name} {int(score*100)}%"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Khóa mục tiêu đầu tiên, không xét tiếp
            break

        # =====================================================
        # BƯỚC 4: LOGIC GHI NHỚ VÀ ĐIỀU TỐC
        # =====================================================
        if sign_detected:
            if "STOP" in detected_sign_name:
                self.current_speed = 0
                cv2.putText(frame, "DA CHOT TOC DO: 0 (STOP)",
                            (30, 200), cv2.FONT_HERSHEY_SIMPLEX,
                            1.2, (0, 0, 255), 3)
            elif "30" in detected_sign_name:
                self.current_speed = 60
                cv2.putText(frame, "DA CHOT TOC DO: 60",
                            (30, 200), cv2.FONT_HERSHEY_SIMPLEX,
                            1.2, (0, 255, 255), 3)
            elif "60" in detected_sign_name:
                self.current_speed = 80
                cv2.putText(frame, "DA CHOT TOC DO: 80",
                            (30, 200), cv2.FONT_HERSHEY_SIMPLEX,
                            1.2, (0, 255, 0), 3)
        else:
            cv2.putText(frame, f"DANG GIU TOC DO: {self.current_speed}",
                        (30, 200), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (255, 255, 255), 2)

        return self.current_speed, frame, detected_sign_name