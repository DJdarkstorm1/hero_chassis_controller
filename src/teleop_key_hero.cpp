//
// Created by dj on 2026/11/3.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

#include <dynamic_reconfigure/Reconfigure.h>

//动态参数的修改
bool ChangeMode(const std::string &host_node,
                  const std::string &param_name,bool value) {
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter bp;
    bp.name  = param_name;
    bp.value = value;
    srv.request.config.bools.push_back(bp);

    if (!ros::service::call(host_node + "/set_parameters", srv)) {
        //printf("\rcall %s/set_parameters failed", host_node.c_str());
        return false;
    } else {
        // printf("\rset %s.%s = %s", host_node.c_str(),
        //          param_name.c_str(), value ? "true" : "false");
        return true;
    }
}

//printf("\r 模式: %s      ", mode.enable ? "全局模式" : "底盘模式");
//控制按键映射
std::map<char, std::vector<float>> movekeys
{//linear:vx,vy;angular:wz
    {'w', {1,0,0}},//前
    {'s', {-1,0,0}},//后
    {'a', {0,1,0}},//左
    {'d', {0,-1,0}},//右
    {'q', {1,1,0}},//左前
    {'e', {1,-1,0}},//右前
    {'z', {-1,1,0}},//左后
    {'c', {-1,-1,0}},//右后
    {'j', {0,0,1}},//左转
    {'l', {0,0,-1}},//右转
    {'u', {0,0,3}},//左小陀螺
    {'o', {0,0,-3}},//右小陀螺
};

//速度按键映射
std::map<char, std::vector<float>> speedkeys
{
    {'i', {1,1.1}},//旋转加速
    {'k', {1,0.9}},//旋转减速
    {'g', {1.1,1}},//直线加速
    {'h', {0.9,1}}//直线减速

};
//提示信息
const char* msg = R"(

键盘控制发布速度消息
---------------------------
平移运动:
   q   w   e
   a   x   d
   z   s   c

----------------------------
旋转运动：
   u    _    o    小陀螺
   j    _    l    转向

-----------------------------
速度调节：
        i   增加10%角速度
        k   减少10%角速度
        g   增加10%线速度
        h   减少10%线速度
其他按键 : 停止

CTRL-C to quit

)";

//初始化变量
float speed(0.5); // 线速度 (m/s)
float turn(1.0); // 角速度 (rad/s)
float x(0), y(0), th(0); // 前进/后退/旋转
char key(' ');//输入按键
bool global_mode(true);
//非阻塞式按键输入
int getch(void) {
    int ch;
    struct termios oldt, newt;
    //保存上一次的按键，新的用于修改
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // 关闭规范模式和回显
    newt.c_lflag &= ~(ICANON | ECHO);

    // 忽略 break 条件
    newt.c_iflag |= IGNBRK;

    // 关闭输入时的 CR/NL 转换和软件流控
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);

    // 再关一批：规范模式、回显、信号字符、扩展输入
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);

    // 读至少 1 个字符，无超时（阻塞读）
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);  // 应用设置

    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);   // 恢复原终端设置
    return ch;
}

int main (int argc, char **argv)
{
    //1.包含头文件
    //2.初始化ros节点
    if(argc < 2)
        return 1;
    const std::string host_node = argv[1];
    const std::string param_name = argv[2];
    setlocale(LC_ALL, "");
    ros::init(argc,argv,"teleop_key_hero");
    //3.创建节点句柄
    ros::NodeHandle nh;
    //4.创建发布者/订阅者对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    //5.编写发布逻辑，发布数据
    //    5.1.创建被发布的数据
    geometry_msgs::Twist twist;

    printf("%s", msg);
    printf("\r当前线速度： %f\t旋转角速度： %f 控制模式：%s| 等待输入...\r", speed, turn, (global_mode ? "全局" : "底盘"));


    //    5.2.设置发布频率
    while (true)
    {
        key = getch();
        if (movekeys.count(key)==1)//平移
        {
            x = movekeys[key][0];
            y = movekeys[key][1];
            th = movekeys[key][2];
            printf("\r当前线速度： %f\t旋转角速度： %f 控制模式：%s| 输入命令： %c   ", speed, turn, (global_mode ? "全局" : "底盘"), key);
        }
        else if (speedkeys.count(key)==1)//旋转
        {
            speed = speedkeys[key][0] * speed;
            turn = speedkeys[key][1] * turn;
            printf("\r当前线速度： %f\t旋转角速度： %f 控制模式：%s| 输入命令： %c   ", speed, turn, (global_mode ? "全局" : "底盘"), key);

        }
        else if (key=='f') {
            global_mode = !global_mode;
            if (global_mode) {
                if (ChangeMode(host_node,param_name,global_mode))
                {
                    printf("\r已切换为全局模式                                                          ");
                }
                else {
                    printf("\r切换全局模式失败                                                          ");
                }

            }
            else {
                if (ChangeMode(host_node,param_name,global_mode))
                {
                    printf("\r已切换为底盘模式                                                          ");
                }
                else {
                    printf("\r切换底盘模式失败                                                          ");
                }
            }
        }
        else//其他按键无效
        {
            x=y=th=0;//停车
            if (key=='\x03') {
                printf("\r已退出                                                                  \n");
                break;//ctrl+c退出
            }

            printf("\r当前线速度： %f\t旋转角速度： %f 控制模式：%s| 无效输入! %c", speed, turn, (global_mode ? "全局" : "底盘"), key);
        }
        //    5.3.循环发布数据
        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = 0.0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        pub.publish(twist);
        ros::spinOnce();
    }

    return 0;
}













