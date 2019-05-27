int32 id            # instruction index
float64 r           # initial reward, 1: robot, 2:positive, 3:neutral, 4:negative, 5:physical
float64 b           # decade factor
bool type           # 0:human 1:robot
int8 duration       # time spend for completing, unit: step_t
string source       # person who ask for request
int8 status         # -1: None, human: emotion(0~3), robot: human action(0~11)
int8 function       # 0:NOP, 1:chat(encourage), 2:remind object, 3:remind schedule, 4:check human, 5:charge, 6: play videos, 7:play game, 8:emergency, 9:report to source
string target       # person or object that robot interact to
int8 destination    # -1: None, 0:office, 1:bedroom, 2:charge, 3:alley, 4:living, 5:dining, 6: greeting, 7: emergency