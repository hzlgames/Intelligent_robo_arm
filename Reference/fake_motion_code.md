新的视觉跟踪运动思路：

``` 伪代码

//均以画面左上角为零点，x正方向向右，y正方向向左, 先使用底座J1旋转、手臂J3和腕部J4调俯仰来追寻、对准目标，调整位置的距离差值直到小于允许误差范围（追寻算法可复用这一套逻辑）
void FindObject(object){    //寻找目标的代码（只是让镜头对准目标）
    do{
        GoalSightCenter = find(TerminalObject); 
            J1.theta += StepSize(diff(GoalSightCenter.x, ActualSightCenter.x)); //底座舵机J1向角度正方向（逆时针）移动一定的步长，若过了则角度改变量为负值（随目标距离动态变化，允许用户设置像素距离-实际角度的映射关系，下同）
        if(J4.theta < boundary){    //此处边界值允许用户设置
            J4.theta += min(StepSize(diff(GoalSightCenter.y, ActualSightCenter.y)), boundary - J4.theta) ;  //腕部舵机J4向角度正方向（俯视方向）移动一定的步长（在临界角以内）
        }
        else{
            J3.theta += max(StepSize(diff(GoalSightCenter.y, ActualSightCenter.y))), ;  //手臂舵机J3向角度正方向（俯视方向）移动一定的步长
        }
    }while(Diff_L2(GoalSightCenter, ActualSightCenter) >= disc) 实际上只要在误差允许范围内即可
    return;
}

/*反复尝试接近，直到抵达可抓取范围再尝试抓取，若抓取失败则向前一个小步长重试
先移动到抓取范围内
运动逻辑：先使用J2和J3两个舵机边靠近物体边调整角度，当J3调整到和J2共线后，不再调整J3，转而变成J2和J4调整角度，如果J2超出安全范围或J4超出安全范围则判定为不可达*/
bool ApproachObject(object){
    while(OutOfRange(object) == true){    //还没到达抓取范围，即目标物体在画面中还不够近
        GoalSightCenter = find(TerminalObject); 
        J1.theta += StepSize(diff(GoalSightCenter.x, ActualSightCenter.x)); //为避免横向扰动，底座舵机也可能需要微调
        if(J2.theta在边界范围内){
            J2.theta += ConstStepSize; //先让肩部舵机固定转一个角度（为了接近物体），此时物体的中心会在画面中心的上方
        }
        else{
            return False;
        }
        while(GoalSightCenter - find(TerminalObject) > 误差范围){   //
            if(abs(J3.theta) > 角度扰动){    //先调整手臂舵机，一旦伸直和肩部共线就改为调整J4
                J4.theta += min(StepSize(diff(GoalSightCenter.y, ActualSightCenter.y)), boundary - J4.theta) ;  //腕部舵机J4向角度正方向（俯视方向）移动一定的步长（在临界角以内）
            }
            else if(J4.theta < boundary){    //必须要保证J4在安全范围内，避免物体在太近的位置导致过弯折而损坏
                向目标方向移动J4
            }
            else{
                return False;
            }
        }
    }
    return True;
}

/*起始阶段*/
do{
    FindObject(手)  //其实这里只要保证手大概在画面中间即可
}while(使用手势确定了抓取目标)  //由于手会一直动所以要一直跟随目标

/*记录目标位置*/
FindObject(锁定的物体goal_object)

initial_pos = read(现在的各个舵机角度)

/*记录终点*/
do{
    FindObject(red_dot)  //桌面上会贴一红点来标记放置的位置，此处红点可能不会直接在画面中，需要J1舵机边遍历安全范围边查找是否有红点，发现后再放到视线中心。一个问题是视觉会无时无刻不在画面中识别最可能的像素集合作为红点位置，需要设置一个识别阈值
}while(使用手势确定了终点)

terminal_pos = read(现在的各个舵机角度)

/*接近阶段*/
舵机位置 = initial_pos;
ApproachObject(goal_object) //此处为了避免其他物体干扰，可以在画面中心附近寻找物体（因为没办法记录物体）
FetchObject()   //此处逻辑请你补充实现，大致思路是通过步进夹取，每步进行完之后读取实际夹爪姿态，如果偏差很大说明夹中了
舵机位置 = terminal_pos;    //此处要先动肩部再动手臂最后动腕部，避免机械臂撞地
ApproachObject(red_dot) //接近红点
placeObject()   //此处只要简单地开爪即可
舵机回到原位（开机时的位置）

```