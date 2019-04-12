int8 id             # instruction index
float64 r           # initial reward
float64 b           # decade factor
bool type           # 0:human 1:robot
int8 duration       # time spend for completing
int8 location       # 0:office, 1:bedroom, 2:charge, 3:alley, 4:living, 5:dining
string source       # person who ask for request
int8 status         # human: emotion(), robot: human action
int8 function       # 0:emergency, 1:chat, 2:remind object, 3:remind schedule, 4:check human, 5:charge, 6: play videos, 7:play game
string target
int8 destination