import cv2
import time
import serial
import os
import threading
import socket

from picamera2 import Picamera2
from lane_detection import LaneDetector
from sign_detection import SignDetector

# Nhúng Web Server
import web_server

os.environ['QT_QPA_PLATFORM'] = 'xcb'
os.environ["OMP_NUM_THREADS"] = "1"
cv2.setNumThreads(1)

# ==========================================
# BIẾN TOÀN CỤC CHO GIAO TIẾP ĐA LUỒNG
# ==========================================
current_camera_frame_rgb = None  # Luôn lưu RGB cho AI
ai_target_speed = 80
ai_sign_view_rgb = None          # Kết quả trả về từ AI cũng là RGB
ai_detected_sign = "None"

# ==========================================
# BIẾN TOÀN CỤC CHO LUỒNG UART (40Hz)
# ==========================================
uart_final_speed = 0
uart_final_steer = 0
uart_auto_flag   = 0
uart_avoid_flag  = 0  # [MỚI] 1: Bật lách, 0: Tắt lách

# ==========================================
# HÀM TỰ ĐỘNG LẤY IP WIFI CỦA RASPBERRY PI
# ==========================================
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)) 
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

# ==========================================
# 1. KHỞI TẠO HỆ THỐNG
# ==========================================
picam2 = Picamera2()
# Định dạng gốc là RGB, rất tiện cho AI và Web
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

try:
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    print("✅ Mở UART thành công")
except:
    ser = None
    print("❌ Cảnh báo: Không thể mở UART")

lane_system = LaneDetector()
sign_system = SignDetector(model_path="Traffic_signss_v7.pt", base_speed=80)

# ==========================================
# LUỒNG UART CỐ ĐỊNH 40Hz — Bắn dữ liệu liên tục không độ trễ
# ==========================================
def uart_control_thread():
    global uart_final_speed, uart_final_steer, uart_auto_flag, uart_avoid_flag
    while True:
        t_start = time.perf_counter()
        if ser and ser.is_open:
            try:
                # [MỚI] Bắn 4 thông số xuống ATmega: Speed : Steer : Auto_Mode : Avoid_Enable
                cmd = f"D:{int(uart_final_speed)}:{int(uart_final_steer)}:{uart_auto_flag}:{uart_avoid_flag}\n"
                ser.write(cmd.encode('utf-8'))
            except Exception:
                pass
        
        # Cân bằng thời gian ngủ để đạt đúng 40 vòng/giây (0.025s)
        elapsed = time.perf_counter() - t_start
        remaining = 0.025 - elapsed
        if remaining > 0:
            time.sleep(remaining)

# ==========================================
# LUỒNG CHẠY NGẦM: NHẬN DIỆN BIỂN BÁO (AI)
# ==========================================
def ai_worker_thread():
    global current_camera_frame_rgb, ai_target_speed, ai_sign_view_rgb, ai_detected_sign
    
    # Ép AI chạy ở mức ưu tiên thấp hơn luồng dò làn đường (Tùy chọn)
    try:
        os.nice(10)
    except Exception:
        pass

    while True:
        if current_camera_frame_rgb is not None:
            # Copy frame RGB để AI xử lý độc lập
            frame_to_process = current_camera_frame_rgb.copy()
            
            # Đưa RGB vào cho AI quét
            speed, view_rgb, sign = sign_system.process(frame_to_process)
            
            # Cập nhật kết quả ra biến toàn cục (view_rgb đang là RGB)
            ai_target_speed = speed
            ai_sign_view_rgb = view_rgb
            ai_detected_sign = sign
            
        time.sleep(0.2)

# ==========================================
# 2. VÒNG LẶP XỬ LÝ CHÍNH
# ==========================================
def main_loop():
    global current_camera_frame_rgb, uart_final_speed, uart_final_steer, uart_auto_flag, uart_avoid_flag
    
    web_server.car_telemetry['mode'] = 'MANUAL'
    last_web_speed = web_server.car_settings['base_speed']
    
    while True:
        # 1. Cập nhật cài đặt từ Web
        lane_system.hsv = web_server.car_settings['hsv']
        lane_system.k_heading = web_server.car_settings['k_heading']
        lane_system.k_offset = web_server.car_settings['k_offset']

        current_web_speed = web_server.car_settings['base_speed']
        if current_web_speed != last_web_speed:
            sign_system.base_speed = current_web_speed
            sign_system.current_speed = current_web_speed 
            last_web_speed = current_web_speed

        # 2. LẤY ẢNH TỪ CAMERA (Mặc định cấu hình đang là RGB)
        frame_rgb = picam2.capture_array()
        
        # Đẩy ngay ảnh RGB này cho luồng AI ngầm
        current_camera_frame_rgb = frame_rgb

        # 3. DÒ LÀN ĐƯỜNG (CHUYỂN SANG BGR ĐỂ OPENCV XỬ LÝ CHUẨN)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        steering_angle, lane_overlay_bgr, debug_mask_bgr = lane_system.process(frame_bgr)

        # 4. GỘP HÌNH ẢNH TRÊN NỀN KHUNG CANVAS RGB
        # Lấy base ảnh từ AI (RGB) hoặc lấy ảnh gốc (RGB) nếu AI chưa chạy xong
        base_view_rgb = ai_sign_view_rgb if ai_sign_view_rgb is not None else frame_rgb
        
        # Chuyển overlay vạch kẻ đường từ BGR sang RGB để trộn
        lane_overlay_rgb = cv2.cvtColor(lane_overlay_bgr, cv2.COLOR_BGR2RGB)
        
        # Trộn 2 ảnh RGB lại với nhau
        dashboard_rgb = cv2.addWeighted(base_view_rgb, 1, lane_overlay_rgb, 0.1, 0)
        
        # Ghi text
        cv2.putText(dashboard_rgb, f"STEER: {steering_angle:.1f}", (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 5. XỬ LÝ ẢNH PiP (Picture-in-Picture)
        pip_h, pip_w = 180, 240
        debug_pip_bgr = cv2.resize(debug_mask_bgr, (pip_w, pip_h))
        # Đổi PiP sang RGB trước khi dán lên khung RGB
        debug_pip_rgb = cv2.cvtColor(debug_pip_bgr, cv2.COLOR_BGR2RGB)
        
        margin = 10
        y1, y2 = margin, margin + pip_h
        x1, x2 = 640 - pip_w - margin, 640 - margin
        
        # Vẽ viền màu Vàng 
        cv2.rectangle(dashboard_rgb, (x1-2, y1-2), (x2+2, y2+2), (255, 255, 0), 2)
        cv2.putText(dashboard_rgb, "Sliding Windows", (x1, y1-5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Dán ảnh nhỏ lên ảnh lớn
        dashboard_rgb[y1:y2, x1:x2] = debug_pip_rgb

        # 6. ĐẨY ẢNH RGB LÊN WEB
        web_server.global_frame = dashboard_rgb.copy()
        
        # -----------------------------------------------------
        # 7. LOGIC ĐIỀU KHIỂN & ĐẨY XUỐNG UART THREAD
        # -----------------------------------------------------
        current_mode = web_server.car_telemetry['mode']
        
        final_speed = ai_target_speed
        final_steer = steering_angle

        # [ĐÃ SỬA] Cờ AUTO trả về đúng 1
        uart_auto_flag = 1 if current_mode == 'AUTO' else 0
        
        # [MỚI] Đọc cờ Tránh vật cản từ Web (Mặc định là 1 nếu chưa bấm)
        uart_avoid_flag = web_server.car_telemetry.get('avoid_enable', 1)

        if current_mode == 'MANUAL':
            cmd = web_server.manual_command
            if cmd == 'UP': final_speed, final_steer = ai_target_speed, 0
            elif cmd == 'DOWN': final_speed, final_steer = -ai_target_speed, 0
            elif cmd == 'LEFT': final_speed, final_steer = ai_target_speed, -45
            elif cmd == 'RIGHT': final_speed, final_steer = ai_target_speed, 45
            else: 
                final_speed, final_steer = 0, 0

        # Lưu vào biến toàn cục để UART Thread đọc và gửi đi
        uart_final_speed = final_speed
        uart_final_steer = final_steer

        # Cập nhật Telemetry lên Web
        web_server.car_telemetry['speed'] = int(final_speed)
        web_server.car_telemetry['steer'] = int(final_steer)
        web_server.car_telemetry['sign'] = ai_detected_sign

# ==========================================
# 3. KHỞI CHẠY ĐA LUỒNG
# ==========================================
if __name__ == '__main__':
    pi_ip = get_local_ip()
    
    web_thread = threading.Thread(target=web_server.start_server, daemon=True)
    web_thread.start()

    ai_thread = threading.Thread(target=ai_worker_thread, daemon=True)
    ai_thread.start()
    
    # [MỚI] Khởi chạy luồng UART
    uart_thread = threading.Thread(target=uart_control_thread, daemon=True)
    uart_thread.start()
    
    print(f"🌐 Web Server đã mở! Truy cập ngay: http://{pi_ip}:5000")
    print("🚀 Hệ thống xử lý đa luồng đã sẵn sàng!")

    try:
        main_loop()
    except KeyboardInterrupt:
        print("\nĐã dừng khẩn cấp bằng phím Ctrl+C")
    finally:
        picam2.stop()
        picam2.close()
        if ser: ser.close()
        cv2.destroyAllWindows()