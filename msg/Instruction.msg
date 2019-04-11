int8 id             # instruction index
float64 r           # initial reward
float64 b           # decade factor
bool type           # 0:human 1:robot
int8 duration       # time spend for completing
int8 location       # 0:office, 1:bedroom, 2:charge, 3:alley, 4:living, 5:dining
string source       # person who ask for request
int8 status         # human: emotion(), robot: action
int8 function       # 0:
string target
int8 destination