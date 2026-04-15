std_msgs/Header header
bool estop
bool safety_ok
bool motor_cmd
bool motor_contactor_fb     # true = คอนแทคเตอร์ปิดอยู่ (มีไฟไปไดรฟ์)
bool driver_enable_fb
bool driver_fault
float32 bus_voltage         # โวลต์บัสที่วัดได้ (ADC)
