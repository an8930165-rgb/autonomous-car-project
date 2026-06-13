from flask import Flask, render_template, Response, jsonify, request
import cv2
import time
import logging

app = Flask(__name__)


# ==========================================
# TẮT LOG RÁC CỦA FLASK
# ==========================================
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Chỉ in ra Terminal khi máy chủ web bị lỗi

# ==========================================
# CÁC BIẾN TOÀN CỤC (Dùng chung với main.py)
# ==========================================
global_frame = None
car_telemetry = {
    'mode': 'MANUAL',
    'speed': 0,
    'steer': 0,
    'sign': 'None',
    'avoid_enable': 0  # 1: Bật (Mặc định), 0: Tắt
}
manual_command = "STOP" # Trạng thái phím điều khiển (UP, DOWN, LEFT, RIGHT, STOP)

# THÊM BIẾN NÀY ĐỂ LƯU CÀI ĐẶT
car_settings = {
    "base_speed": 80,
    "hsv": [0, 0, 230, 255, 50, 255], # L-H, L-S, L-V, U-H, U-S, U-V
    "k_heading": 1.6, 
    "k_offset": 1.5
}

# ==========================================
# CÁC ROUTE CỦA WEB SERVER
# ==========================================
@app.route('/')
def index():
    """Trang chủ giao diện"""
    return render_template('index.html')

def gen_frames():
    """Hàm Generator liên tục chuyển đổi khung hình ảnh thành luồng Video (MJPEG)"""
    global global_frame
    while True:
        if global_frame is not None:
            # Mã hóa ảnh BGR sang định dạng JPEG
            ret, buffer = cv2.imencode('.jpg', global_frame)
            frame_bytes = buffer.tobytes()
            # Trả về từng khung hình cho trình duyệt
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    """Endpoint truyền video"""
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
def telemetry():
    """API gửi dữ liệu tốc độ, góc lái lên Web (Trình duyệt gọi mỗi 200ms)"""
    return jsonify(car_telemetry)

@app.route('/control', methods=['POST'])
def control():
    """API nhận lệnh điều khiển từ Web gửi xuống"""
    global car_telemetry, manual_command
    data = request.json
    
    if 'mode' in data:
        car_telemetry['mode'] = data['mode']
        print(f"[WEB] Chuyển chế độ: {data['mode']}")
        
    if 'command' in data:
        manual_command = data['command']
        # Chỉ in ra log nếu đang ở chế độ MANUAL
        if car_telemetry['mode'] == 'MANUAL':
            print(f"[WEB] Lệnh tay: {manual_command}")
            
    return jsonify({"status": "success"})

@app.route('/settings', methods=['POST'])
def settings():
    """API nhận thông số cài đặt từ các thanh trượt (Trackbars)"""
    global car_settings
    data = request.json
    
    if 'base_speed' in data:
        car_settings['base_speed'] = int(data['base_speed'])
    if 'hsv' in data:
        car_settings['hsv'] = data['hsv']

    # Ép kiểu float (số thập phân) cho 2 hệ số K
    if 'k_heading' in data: car_settings['k_heading'] = float(data['k_heading'])
    if 'k_offset' in data: car_settings['k_offset'] = float(data['k_offset'])
        
    return jsonify({"status": "success"})

@app.route('/api/toggle_avoid', methods=['POST'])
def toggle_avoid():
    data = request.json
    car_telemetry['avoid_enable'] = 1 if data['enable'] else 0
    return jsonify({"status": "success", "avoid_enable": car_telemetry['avoid_enable']})
# ==========================================
# HÀM KHỞI CHẠY (Gọi từ main.py)
# ==========================================
def start_server():
    # debug=False, use_reloader=False RẤT QUAN TRỌNG để chạy trong đa luồng
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)