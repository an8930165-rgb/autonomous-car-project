import cv2
import numpy as np
import time  # [MỚI] Thêm thư viện để xử lý thời gian thực

class LaneDetector:
    def __init__(self):
        # Thông số cấu hình Toán học
        self.LANE_WIDTH_PIXELS = 560.0 
        self.REAL_LANE_WIDTH_M = 0.25 
        self.xm_per_pix = self.REAL_LANE_WIDTH_M / self.LANE_WIDTH_PIXELS 
        self.LOOK_AHEAD_M = 0.5  
        self.k_heading = 1.6     
        self.k_offset = 1.5    
        
        # Cấu hình HSV mặc định (Sẽ được Web Server ghi đè liên tục)
        self.hsv = [0, 0, 230, 255, 50, 255]

        # --- 1. Cấu hình nhận diện ngã tư ---
        self.TURN_THRESHOLD = 280         # Ngưỡng lệch X (pixel) để nhận diện góc vuông
        self.TURN_FRAMES_REQUIRED = 2     # Cần 2 khung hình liên tiếp để xác nhận
        self.turn_frame_count = 0         
        self.current_turn_type = None     

        # --- 2. CẤU HÌNH RẼ VÒNG KÍN (MỚI) ---
        self.is_turning = False           # Cờ đánh dấu xe ĐANG rẽ ngã tư
        self.turn_direction = 0           # 1: Rẽ phải, -1: Rẽ trái
        self.turn_start_time = 0.0        # [MỚI] Biến lưu mốc thời gian bắt đầu rẽ

        # --- 3. Cấu hình mất làn (Trí nhớ ngắn hạn) ---
        self.last_steering_angle = 0.0    
        self.lost_lane_counter = 0        
        self.MAX_LOST_FRAMES = 15         # Tối đa 15 frames (~0.5s) nhắm mắt chạy mù

    def process(self, frame):
        # ==========================================
        # 1. TIỀN XỬ LÝ ẢNH & LỌC MÀU
        # ==========================================
        tl, bl, tr, br = (80,300), (0,480), (520,300), (640,480)
        pts1 = np.float32([tl, bl, tr, br]) 
        pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]]) 
        
        matrix = cv2.getPerspectiveTransform(pts1, pts2) 
        transformed_frame = cv2.warpPerspective(frame, matrix, (640, 480))

        hsv_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
        l_h, l_s, l_v, u_h, u_s, u_v = self.hsv
        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv_frame, lower, upper)

        # ==========================================
        # 2. KHỞI TẠO CỬA SỔ TRƯỢT
        # ==========================================
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0]/2)
        min_peak = 1000 

        left_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > min_peak else 100 
        right_base = np.argmax(histogram[midpoint:]) + midpoint if np.max(histogram[midpoint:]) > min_peak else 540 

        y = 472
        lx, rx = [], []
        msk_bgr = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)

        # ==========================================
        # 3. QUÉT CỬA SỔ LỌC NHIỄU
        # ==========================================
        while y > 0:
            start_x_left = max(0, left_base - 50) 
            end_x_left = min(640, left_base + 50)
            img_left = mask[y-40:y, start_x_left : end_x_left]
            contours_l, _ = cv2.findContours(img_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours_l:
                c_left = max(contours_l, key=cv2.contourArea)
                M = cv2.moments(c_left)
                if M["m00"] > 10: 
                    cx = int(M["m10"]/M["m00"])
                    real_cx = start_x_left + cx 
                    lx.append(real_cx)
                    left_base = real_cx
                    cv2.rectangle(msk_bgr, (start_x_left, y), (end_x_left, y-40), (0, 255, 0), 2)
            
            start_x_right = max(0, right_base - 50)
            end_x_right = min(640, right_base + 50)
            img_right = mask[y-40:y, start_x_right : end_x_right]
            contours_r, _ = cv2.findContours(img_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours_r:
                c_right = max(contours_r, key=cv2.contourArea)
                M = cv2.moments(c_right)
                if M["m00"] > 10:
                    cx = int(M["m10"]/M["m00"])
                    real_cx = start_x_right + cx
                    rx.append(real_cx)
                    right_base = real_cx
                    cv2.rectangle(msk_bgr, (start_x_right, y), (end_x_right, y-40), (0, 0, 255), 2)
            
            y -= 40

        # ==========================================
        # 4. TRÍ TUỆ NHÂN TẠO: MÁY TRẠNG THÁI LÁI XE
        # ==========================================
        steering_angle = 0.0
        has_left, has_right = len(lx) > 2, len(rx) > 2
        
        try:
            # ----------------------------------------------------
            # TRẠNG THÁI 1: XE ĐANG TRONG QUÁ TRÌNH RẼ NGÃ TƯ
            # ----------------------------------------------------
            if self.is_turning:
                # Tiếp tục ôm cua gắt theo hướng đã định
                steering_angle = 45.0 * self.turn_direction
                
                # Tìm tâm của vạch kẻ đường mới (ở sát gầm xe y=480)
                current_center = -1
                if has_left and has_right:
                    current_center = (lx[0] + rx[0]) / 2
                elif has_left:
                    current_center = lx[0] + (self.LANE_WIDTH_PIXELS/2)
                elif has_right:
                    current_center = rx[0] - (self.LANE_WIDTH_PIXELS/2)

                # [MỚI] TÍNH THỜI GIAN ĐÃ RẼ ĐƯỢC BAO LÂU
                elapsed_time = time.time() - self.turn_start_time

                # [MỚI] KIỂM TRA ĐIỀU KIỆN THOÁT LẠI LÀN HOẶC TIMEOUT
                # Điều kiện 1: Đã rẽ tối thiểu 0.1s VÀ tâm đường lọt vào giữa camera
                centered_condition = (elapsed_time > 0.1) and (160 < current_center < 480)
                
                # Điều kiện 2: Failsafe thoát gắt nếu quá 0.3s
                timeout_condition = (elapsed_time >= 0.25)

                if centered_condition or timeout_condition:
                    self.is_turning = False     # Kết thúc rẽ
                    self.turn_direction = 0
                    self.lost_lane_counter = 0  # Reset bộ đếm mất làn để chạy thẳng

                # (Nếu chưa thỏa mãn, code sẽ bỏ qua phần dưới và tiếp tục ôm cua 45 độ)

            # ----------------------------------------------------
            # TRẠNG THÁI 2: XE ĐANG CHẠY BÌNH THƯỜNG / TÌM NGÃ TƯ
            # ----------------------------------------------------
            else:
                if has_left or has_right:
                    self.lost_lane_counter = 0 # Thấy đường -> Reset mất làn

                    # A. Quét tìm ngã rẽ vuông góc (Bộ đếm 4 khung hình)
                    detected_turn_this_frame = None
                    if has_left and (lx[0] - lx[-1] > self.TURN_THRESHOLD):
                        detected_turn_this_frame = 'LEFT'
                    elif has_right and (rx[-1] - rx[0] > self.TURN_THRESHOLD):
                        detected_turn_this_frame = 'RIGHT'

                    if detected_turn_this_frame:
                        if self.current_turn_type == detected_turn_this_frame:
                            self.turn_frame_count += 1
                        else:
                            self.current_turn_type = detected_turn_this_frame
                            self.turn_frame_count = 1
                    else:
                        self.turn_frame_count = 0
                        self.current_turn_type = None

                    # B. KÍCH HOẠT RẼ NGÃ TƯ KHI ĐỦ ĐIỀU KIỆN
                    if self.turn_frame_count >= self.TURN_FRAMES_REQUIRED:
                        self.is_turning = True       # Khóa hệ thống vào trạng thái rẽ
                        self.turn_frame_count = 0    # Dọn dẹp cho lần ngã tư tiếp theo
                        self.turn_start_time = time.time() # [MỚI] Bấm giờ bắt đầu ôm cua
                        
                        if self.current_turn_type == 'LEFT':
                            self.turn_direction = -1
                            steering_angle = -45.0
                        elif self.current_turn_type == 'RIGHT':
                            self.turn_direction = 1
                            steering_angle = 45.0

                    # C. TOÁN HỌC STANLEY (Chỉ chạy khi không có ngã tư)
                    # [QUAN TRỌNG] Đã sửa đổi thành elif để loại bỏ lỗi ghi đè steering_angle
                    elif not self.is_turning:
                        y_offset_eval = 480  
                        y_heading_eval = 350 
                        
                        avg_slope, lane_center_x = 0.0, 320
                        
                        if has_left and has_right:
                            left_fit = np.polyfit([472 - i*40 for i in range(len(lx))], lx, 2)
                            right_fit = np.polyfit([472 - i*40 for i in range(len(rx))], rx, 2)
                            avg_slope = ((2*left_fit[0]*y_heading_eval + left_fit[1]) + (2*right_fit[0]*y_heading_eval + right_fit[1])) / 2
                            lane_center_x = ( (left_fit[0]*y_offset_eval**2 + left_fit[1]*y_offset_eval + left_fit[2]) + 
                                              (right_fit[0]*y_offset_eval**2 + right_fit[1]*y_offset_eval + right_fit[2]) ) / 2
                        elif has_left and not has_right:
                            left_fit = np.polyfit([472 - i*40 for i in range(len(lx))], lx, 2)
                            avg_slope = 2 * left_fit[0] * y_heading_eval + left_fit[1]
                            lane_center_x = (left_fit[0]*y_offset_eval**2 + left_fit[1]*y_offset_eval + left_fit[2]) + (self.LANE_WIDTH_PIXELS/2) 
                        elif has_right and not has_left:
                            right_fit = np.polyfit([472 - i*40 for i in range(len(rx))], rx, 2)
                            avg_slope = 2 * right_fit[0] * y_heading_eval + right_fit[1]
                            lane_center_x = (right_fit[0]*y_offset_eval**2 + right_fit[1]*y_offset_eval + right_fit[2]) - (self.LANE_WIDTH_PIXELS/2) 

                        heading_error_deg = np.arctan(-avg_slope) * 180 / np.pi
                        lane_offset = (lane_center_x - 320) * self.xm_per_pix  
                        offset_angle_deg = np.arctan(lane_offset / self.LOOK_AHEAD_M) * 180 / np.pi
                        
                        steering_angle = (self.k_heading * heading_error_deg) + (self.k_offset * offset_angle_deg)
                        
                    # Lưu trữ bộ nhớ
                    self.last_steering_angle = steering_angle

                # ----------------------------------------------------
                # TRẠNG THÁI 3: MẤT LÀN ĐƯỜNG -> TRÍ NHỚ NGẮN HẠN
                # ----------------------------------------------------
                else:
                    self.lost_lane_counter += 1
                    if self.lost_lane_counter < self.MAX_LOST_FRAMES:
                        steering_angle = self.last_steering_angle
                    else:
                        steering_angle = 0.0

        except Exception as e:
            pass 

        final_angle = np.clip(steering_angle, -45, 45)

        # ==========================================
        # 5. VẼ LÀN ĐƯỜNG ẢO (GREEN OVERLAY)
        # ==========================================
        color_warp = np.zeros((480, 640, 3), dtype=np.uint8)
        
        try:
            ploty_l = [472 - i*40 for i in range(len(lx))]
            ploty_r = [472 - i*40 for i in range(len(rx))]

            if has_left and has_right:
                pts_left = np.array([np.transpose(np.vstack([lx, ploty_l]))])
                pts_right = np.array([np.flipud(np.transpose(np.vstack([rx, ploty_r])))])
                pts = np.hstack((pts_left, pts_right))
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
                
            elif has_left and not has_right:
                fake_rx = [x + self.LANE_WIDTH_PIXELS for x in lx]
                pts_left = np.array([np.transpose(np.vstack([lx, ploty_l]))])
                pts_right = np.array([np.flipud(np.transpose(np.vstack([fake_rx, ploty_l])))])
                pts = np.hstack((pts_left, pts_right))
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
                
            elif has_right and not has_left:
                fake_lx = [x - self.LANE_WIDTH_PIXELS for x in rx]
                pts_left = np.array([np.transpose(np.vstack([fake_lx, ploty_r]))])
                pts_right = np.array([np.flipud(np.transpose(np.vstack([rx, ploty_r])))])
                pts = np.hstack((pts_left, pts_right))
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
                
        except Exception as e:
            pass 

        inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
        lane_overlay = cv2.warpPerspective(color_warp, inv_matrix, (640, 480))
        
        return final_angle, lane_overlay, msk_bgr