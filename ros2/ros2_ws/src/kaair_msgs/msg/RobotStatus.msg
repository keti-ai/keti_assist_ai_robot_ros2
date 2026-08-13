std_msgs/Header header

# 로봇 전체 동작 상태
uint8 IDLE    = 0
uint8 MOVING  = 1
uint8 ERROR   = 2
uint8 state

# 현재 조인트 위치 (joint1 ~ joint7)
float64[] joint_positions

# 배터리 잔량 (0.0 ~ 100.0 %)
float32 battery_percent

# 오류 메시지 (state == ERROR 일 때 사용)
string error_message
