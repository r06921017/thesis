int8 id             # instruction index
float64 r           # initial reward
float64 b           # decade factor
bool type           # 0:human 1:robot
int8 duration       # time spend for completing, unit: step_t
string source       # person who ask for request
int8 status         # -1: None, human: emotion(0~3), robot: human action(0~11)
int8 function       # 0:NOP, 1:chat, 2: encourage, 3:remind object, 4:remind schedule, 5:check human, 6:charge, 7: play videos, 8:play game, 9:emergency
string target       # person or object that robot interact to
int8 destination    # -1: None, 0:office, 1:bedroom, 2:charge, 3:alley, 4:living, 5:dining, 6: greeting, 7: emergency