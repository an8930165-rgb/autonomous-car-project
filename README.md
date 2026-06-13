# 🚗 Autonomous Car Project - Mô Hình Xe Tự Hành (Level 2)

Dự án xe tự lái quy mô nhỏ tích hợp công nghệ Thị giác máy tính (Computer Vision), Trí tuệ nhân tạo (Edge AI) và Điều khiển nhúng thời gian thực. Hệ thống có khả năng tự động bám làn đường, nhận diện biển báo giao thông để điều tốc và hỗ trợ giám sát/điều khiển từ xa thông qua Web Dashboard.

## ✨ Tính năng nổi bật

* **🛣️ Dò làn đường thời gian thực (Lane Detection):** Áp dụng kỹ thuật biến đổi phối cảnh (Bird's-eye View) và thuật toán Cửa sổ trượt (Sliding Windows) kết hợp OpenCV để lọc nhiễu và bám vạch kẻ đường chính xác.
* **🧠 Nhận diện biển báo giao thông (Traffic Sign Detection):** Tích hợp mô hình học sâu YOLO (được tối ưu hóa bằng framework NCNN) chạy độc lập trên luồng ngầm để phát hiện và phản hồi với các biển báo (STOP, 30, 60).
* **⚙️ Điều khiển quỹ đạo (Trajectory Tracking):** Sử dụng thuật toán Stanley Controller với hệ số PID thích ứng theo vận tốc ($K_p$ thay đổi tự động) giúp xe ôm cua mượt mà, không bị văng đuôi.
* **🛑 Lách vật cản tĩnh (Obstacle Avoidance):** Xử lý tín hiệu cảm biến siêu âm SRF05 bằng ngắt ngoài (External Interrupt) trên vi điều khiển để tự động kích hoạt chuỗi lệnh phanh và lách chướng ngại vật.
* **🌐 Giao diện Web Dashboard:** Ứng dụng Flask Web Server cho phép người dùng giám sát camera trực tiếp, theo dõi thông số viễn trắc (Telemetry) và tinh chỉnh thông số AI/PID từ xa.
* **⚡ Kiến trúc đa luồng (Multi-threading):** Luồng điều khiển UART (40Hz) được tách biệt hoàn toàn khỏi luồng xử lý ảnh và AI, đảm bảo vi điều khiển luôn nhận lệnh liên tục với độ trễ bằng 0 (Zero-latency control).

## 🛠️ Cấu trúc Hệ thống

### 1. Phần cứng (Hardware)
* **Máy tính nhúng (High-level Control):** Raspberry Pi 4 Model B.
* **Vi điều khiển (Low-level Control):** ATmega16 (Giao tiếp UART với Raspberry Pi qua mạch phân áp 3.3V).
* **Camera:** Pi Camera Module / USB Webcam.
* **Cảm biến:** Cảm biến khoảng cách siêu âm SRF05.
* **Cơ cấu truyền động:** * Module điều khiển động cơ L298N.
    * 2 Động cơ DC (Truyền động vi sai).
    * 1 Động cơ Servo (Cơ cấu đánh lái).

### 2. Ngôn ngữ & Thư viện (Software Stack)
* **Python 3:** Ngôn ngữ lập trình chính trên Raspberry Pi.
* **OpenCV:** Xử lý ảnh và thị giác máy tính.
* **PyTorch & Ultralytics (YOLO):** Huấn luyện và chạy suy luận nhận diện biển báo.
* **Flask:** Xây dựng máy chủ Web Dashboard.
* **C/C++ (AVR):** Firmware nạp trên vi điều khiển ATmega16.

## 🚀 Hướng dẫn Cài đặt & Khởi chạy

### Bước 1: Chuẩn bị môi trường
Đảm bảo Raspberry Pi của bạn đã được cài đặt Python 3 và các thư viện cần thiết:
```bash
# Cập nhật hệ thống
sudo apt update && sudo apt upgrade -y

# Cài đặt các thư viện yêu cầu
pip install opencv-python numpy flask pyserial torch torchvision ultralytics
Bước 2: Khởi động hệ thống
Kết nối nguồn cho vi điều khiển ATmega16 và mạch động lực L298N.

Kết nối Raspberry Pi. Tại terminal của Pi, chạy file thực thi chính:

Bash
python main.py
Lưu ý: Ngay khi chạy code, xe sẽ ở trạng thái khóa an toàn (MANUAL Mode - Tốc độ 0).

Bước 3: Giám sát qua Web Dashboard
Mở trình duyệt web trên máy tính/điện thoại dùng chung mạng Wi-Fi với Raspberry Pi.

Truy cập vào địa chỉ IP của Pi với cổng 5000 (Ví dụ: http://192.168.1.100:5000).

Chuyển sang chế độ AUTO để xe bắt đầu tự hành, hoặc dùng các nút điều hướng để lái xe bằng tay.

📁 Cấu trúc Thư mục
Plaintext
autonomous-car-project/
├── main.py                 # File thực thi chính, khởi tạo đa luồng
├── lane_detection.py       # Thuật toán xử lý ảnh và bám làn
├── sign_detection.py       # Tích hợp AI (YOLO) nhận diện biển báo
├── web_server.py           # Máy chủ Flask & API giao tiếp
├── templates/
│   └── index.html          # Giao diện Web Dashboard
├── firmware/
│   └── ATmega16_code.c     # Mã nguồn C nạp cho vi điều khiển
├── models/
│   └── Traffic_signss.pt   # Trọng số mô hình AI đã huấn luyện
└── README.md
👨‍💻 Tác giả
Phan Nhật Anh, Nguyễn Việt Anh - Thiết kế kiến trúc, phát triển phần mềm và phần cứng.

Dự án này được phát triển nhằm mục đích nghiên cứu ứng dụng thị giác máy tính và hệ thống nhúng thời gian thực.
