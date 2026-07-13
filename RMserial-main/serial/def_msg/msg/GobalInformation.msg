## 用于serial->系统
# received heart_beat
std_msgs/Header header
float32 battery #剩下电量     0--100
float32 life_extra #剩下生命  0--100
float32 color#装甲板颜色
float32 bullet_extra #剩余子弹 0-255
float32 fault_flag #故障指示电控方面
# additional data
bool launch
bool arm
float32 base_hp_our
float32 base_hp_enemy
float32 judge_warning
# game_data
float32 stage_remain_time
float32 game_progress

#下发上，暂时没用
