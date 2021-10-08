#include"DataStruct.h"
#include "math.h"
#include "plane.h"
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <windows.h>
#include <QDebug>
#define SIGN(x) ((x < 0 ) ? 1 : 0 )
#define Pai 3.1415926

//角度坐标系转换
inline double angleConver(double angle)
{
    return -90 - angle;
}

//抽取向量点乘
inline double PointMul(double x0, double y0, double x1, double y1)
{
    return x0 * x1 + y0 * y1;
}

//抽取向量取模
inline double vecMod(double x, double y)
{
    return sqrt(x * x + y * y);
}


/*
 * 找到最容易射中的球
 */

int FindBall(DataStruct *data){
    for(int i=data->ball_size - 1;i>=0;i--)
    {
        //求得飞机与球之间的直线方程
        double px=data->plane1.x;
        double py=data->plane1.y;
        double bx=data->ball[i].x;
        double by=data->ball[i].y;
        double kb=(py-by)/(px-bx);
        double kp=angleConver(data->plane1.angle);
        if(kb > kp && data->ball[i].v_x >0 || kb<kp &&data->ball[i].v_x<0)
            return i;
    }

    float vm=0;
    int num;
    for(int i=0;i<data->ball_size;i++)
    {
        float vx=data->ball->v_x;
        float vy=data->ball->v_y;
        float v=vecMod(vx,vy);
        if(v>vm)
        {
            vm=v;
            num=i;
        }
    }
    return num;
}



/*
 * 同时躲避两个球
 */
int Hideball(DataStruct *data, BallStruct &ball,KeyStruct*key)
{

    //获取当前球的坐标和速度
    float ball_x = ball.x, ball_y = ball.y, ball_v_x = ball.v_x, ball_v_y = ball.v_y;

    float ballx=data->ball->x;
    float bally=data->ball->y;
    for(int i=0;i<data->ball_size;i++)
    {
        if(data->ball[i].r<10)
        {
            continue;
        }
        else {
            key->shoot=true;
            if(key->shield)
            {
                int w=data->ball_size-1;
                return w;
            }
        }
        float vx=data->ball[i].v_x;
        float vy=data->ball[i].v_y;
        return vecMod(vx,vy);
    }

    for(int x=0;x<10;x++)
    {
        int a=FindBall(data);
        if(a)
        {
            if(data->ball[a].r<10)
            {
                continue;
            }
            else {
                key->shoot=true;
                if(key->shield)
                {
                    int w=data->ball_size-1;
                    return w;
                }
            }
            float vx=data->ball[a].v_x;
            float vy=data->ball[a].v_y;
        }

    }

}



/*
 * 瞄准某个球
 * 参数：data：游戏数据，ball_x、ball_y：球当前位置，ball_v_x、ball_v_y：球当前速度，leftward、rightward：返回动作
 * 返回值：0：瞄准成功，-1：瞄准失败
 */
int ShootBall(DataStruct *data, BallStruct &ball, int &leftward, int &rightward)
{
    float ball_x = ball.x, ball_y = ball.y, ball_v_x = ball.v_x, ball_v_y = ball.v_y;
    //瞄准的角度
    double angleTo, angleDiff;
    //球运动方向和飞机与球位置向量的夹角余弦
    double cosPosV;
    //两向量点乘除以两向量的模
    cosPosV = PointMul(ball_v_x, ball_v_y, data->plane1.x - ball_x, data->plane1.y - ball_y)
            / vecMod(ball_v_x, ball_v_y) / vecMod(data->plane1.x - ball_x, data->plane1.y - ball_y);
    //根据正弦定理（a/sinA=b/sinB）求出sin值再求得所需度数
    angleTo = angleConver(        //BETA
                (asin(sqrt(1 - cosPosV * cosPosV) * vecMod(ball_v_x, ball_v_y) / 2000)  //ALPHA

                 + atan2(ball_y - data->plane1.y, ball_x - data->plane1.x)) * 180 / Pai);

    //计算飞机朝向与该角度之差
    angleDiff = fmod(fmod(data->plane1.angle - angleTo, 360) + 360, 360);




    //根据角度差选择更优旋转方向
    if (angleDiff < 3.6 || angleDiff > 356.4)
        {
            return 0;
        }
        else if (angleDiff < 180 )
        {
            leftward = false;
            rightward = true;
            return -1;
        }
        else
        {
            leftward = true;
            rightward = false;
            return 1;
        }
        return 0;
}


int HideBall(DataStruct *data, BallStruct &ball, int &leftward, int &rightward)
{
    float des_x,des_y,cross_x,cross_y,ball_y;
    float ball_v_x,ball_v_y;
    data->ball->x=des_x;
    data->ball->y=des_y;
    des_y=data->plane1.y;
    if(ball_v_x*ball_v_y>0)
    {
       des_y=data->plane1.y;
       if(ball_y>cross_y)
       {
           des_x+=data->ball->r+data->plane1.r;
       }
       else
       {
           des_x-=data->ball->r+data->plane1.r;
       }
    }
    else
    {
        des_y=data->plane1.y;
        if(ball_y>cross_y)
        {
            des_x-=data->ball->r+data->plane1.r;
        }
        else
        {
            des_x+=data->ball->r+data->plane1.r;
        }
    }
}

/*
 * 预测飞机位置
 * 参数：data：游戏数据，x、y：返回位置，time：给定时间
 * 返回值：0：预测成功，-1：预测失败
 */
int AftPlanePos(DataStruct *data, int time, float &x, float &y)
{
    double v_x0, v_y0, v_x1, v_y1, a_x, a_y;
    //获取初速度
    v_x0 = data->plane1.v_x;
    v_y0 = data->plane1.v_y;
    //无速度则无需预测
    if (v_x0 == 0 && v_y0 == 0)
    {
        x = data->plane1.x;
        y = data->plane1.y;
        return 0;
    }
    //计算加速度
    a_x = -v_x0 / vecMod(v_x0, v_y0) * 4000;
    a_y = -v_y0 / vecMod(v_x0, v_y0) * 4000;
    //计算末速度
    v_x1 = v_x0 + a_x * time / 100;
    if (SIGN(v_x1) != SIGN(v_x0))
    {
        v_x1 = 0;
    }
    v_y1 = v_y0 + a_y * time / 100;
    if (SIGN(v_y1) != SIGN(v_y0))
    {
        v_y1 = 0;
    }
    //计算位置
    x = data->plane1.x + (v_x1 * v_x1 - v_x0 * v_x0) / 2 / a_x;
    y = data->plane1.y + (v_y1 * v_y1 - v_y0 * v_y0) / 2 / a_y;
    return 0;
}



/*
 * 转向某点
 * 参数：data：游戏数据，x、y：目标点，leftward、rightward：返回动作
 * 返回值：0：正在转向，1：完成转向，-1：转向失败
 */
int turnAt(DataStruct *data, float x, float y, int &leftward, int &rightward)
{
    //飞机到目的地的角度
    double angleTo, angleDiff;

    //计算飞机到目的地的角度并改变坐标系
    angleTo = angleConver(atan2(y - data->plane1.y, x - data->plane1.x) * 180 / Pai);
    //计算飞机朝向与该角度之差
    angleDiff = fmod(fmod(data->plane1.angle - angleTo, 360) + 360, 360);

    //根据角度差选择更优旋转方向
    if (angleDiff < 3.6 || angleDiff > 356.4)
    {
        return 1;
    }
    else if (angleDiff < 180)
    {
        leftward = false;
        rightward = true;
    }
    else
    {
        leftward = true;
        rightward = false;
    }

    return 0;
}





/*
 * 移动至某点
 * 参数：data：游戏数据，x、y：目标点，forward、leftward、rightward：返回动作
 * 返回值：0：正在移动，1：完成移动，-1：移动失败
 */

int moveAt(DataStruct *data, float x, float y, int &forward, int &leftward, int &rightward, int precision = 1)
{
    //计算当前点到终点距离
    double dis = vecMod(y - data->plane1.y, x - data->plane1.x);
    //已到达目标点则终止动作
    if (dis < precision)
    {
        return 1;
    }

    //预测飞机位置
    float pre_x, pre_y;
    AftPlanePos(data, 1000, pre_x, pre_y);
    //正在转向则不加速
    if (!turnAt(data, data->plane1.x + x - pre_x, data->plane1.y + y - pre_y, leftward, rightward))
    {
        forward = false;
        return 0;
    }

    dis = vecMod(y - pre_x, x - pre_y);
    //停下时未到达目标点
    if (dis >= precision)
    {
        forward = true;
        leftward = false;
        rightward = false;
    }

    return 0;
}



void move(time_t lastFetch,time_t fetch)
{
    //飞机在xy方向的速度位移正负量，角度的正负量
    double v_x_plus=0;
    double v_x_minus=0;
    double x_plus=0;
    double x_minus=0;
    double v_y_plus=0;
    double v_y_minus=0;
    double y_plus=0;
    double y_minus=0;
    double v_y_slowDown=0;
    double y_slownDown=0;
    double angle_plus=0;
    double angle_minus=0;
}


void resetKeyPressed()
{
    double a[10];

    for(int i=RAND_MAX;i>=0;i--)
    {
        a[i]=false;
    }
    return ;

}




/*
 * 判断是否需要移动
 * 参数：data：游戏数据、des_x：目标点横坐标、des_y：目标点纵坐标
 * 返回值：0：无需移动，1：需要移动
 */
int needmove(DataStruct *data, float &des_x, float &des_y, KeyStruct*key,int preTime = 200)
{
    float min_t = INT_MAX;
    //预测每一个球的移动
    for (int i = 0; i < data->ball_size; i++)
    {
        //获取当前球的引用
        BallStruct &ball = data->ball[i];

        //获取当前球的坐标和速度
        float ball_x = ball.x, ball_y = ball.y, ball_v_x = ball.v_x, ball_v_y = ball.v_y;

        float v=vecMod(ball_v_x,ball_v_y);
//        qDebug() <<v;

        //补充
        //计算出2.5s后球的位置
        double ball_x_aft = ball_x + ball_v_x * 500;
        double ball_y_aft = ball_y + ball_v_y * 500;
        int rectflagxup = 0;
        int rectflagxbo = 0;
        int rectflagyri = 0;
        int rectflagyle = 0;
        int rect = 0;
        if(ball_y_aft < 0 ) rectflagxup = 1;
        if(ball_x_aft < 0 ) rectflagyle= 1;
        if(ball_y_aft > 1500 )rectflagxbo = 1;
        if(ball_x_aft > 2000 )rectflagyri = 1;


        float a;
        float b;
        float c;


        //计算球运动直线方程
        float A, B, C;
        A = -1;                                        //Y前的系数
        B = ball_v_y / ball_v_x;                       //X前的系数
        C = ball_y - ball_x * ball_v_y / ball_v_x;     //常数项

        int k =0;
        //x向下越界
        if(rectflagxbo)
        {
            rect =1;
            float a = A;
            float b = -B;
            float c = -C + 3000 - 2 * data->ball->r;
        }
        //x方向向上越界
        if(rectflagxup)
        {
            rect = 1;
            float a = A;
            float b = -B;
            float c = -C;
        }

        //y向左越界
        if(rectflagyle)
        {
            rect = 1;
            float a = A;
            float b = -B;
            float c =C + 4000 * B;

        }
        //y向右越界
        if(rectflagyri)
        {
            rect = 1;
            float a = A;
            float b = -B;
            float c = C;

        }

    stop1:
        if(k)
        {
            if(rectflagxup)
            {
                ball_x =  (A*data->ball->r - C) / B;
                ball_y = data->ball->r;
                ball_v_y = - ball_v_y;
            }
            if(rectflagyle)
            {
                ball_x = ball.r;
                ball_y = (B*ball.r + C) / A;
                ball_x = -ball_v_x;
            }
            if(rectflagyri)
            {
                ball_x = 2000 - ball.r;
                ball_y = C + (2000 - data->ball->r) * B;
                ball_v_x = -ball_v_x;
            }
            if(rectflagxbo)
            {
                ball_y = 1500 - data->ball->r;
                ball_x = (1500 - data->ball->r - C) / B;
                ball_v_y = - ball_v_y;
            }

        }
        //计算飞机到直线距离
        float dis = fabs((A * data->plane1.y + B * data->plane1.x + C) / vecMod(A, B));

        //计算垂点坐标
        float cross_x = (data->plane1.y - ball_y + ball_v_y / ball_v_x * ball_x + ball_v_x / ball_v_y * data->plane1.x)
                / (ball_v_x / ball_v_y + ball_v_y / ball_v_x);
        float cross_y = (ball_v_y / ball_v_x) * (cross_x - ball_x) + ball_y;
        //计算到垂点的时间
        float t = (cross_x - ball_x) / ball_v_x * 100;
//        if(k) {
//            t-=30;
//            qDebug() <<t;
//            goto stop2;
//        }
        //反向运动或时间过久则忽略该球
        if (t < 0 || t > preTime)
        {
            continue;
        }

    stop2:
//        if(t < aftmin_t) aftmin_t = t;

        //判断该球是否有威胁
        if (dis < data->plane1.r + ball.r + 10 && t < min_t)
        {
            //设置最紧迫威胁
            min_t = t;
            if(ball_v_y > 0)
            {
                des_y = data->plane1.y -  ball.r * qAbs(data->plane1.y - cross_y)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_y < 50)
                {
                    des_y = data->plane1.y +  ball.r * qAbs(data->plane1.y - cross_y)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            else {
                des_y = data->plane1.y +  ball.r * qAbs(data->plane1.y - cross_y)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_y > 1400)
                {
                    des_y = data->plane1.y -  ball.r * qAbs(data->plane1.y - cross_y)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            if(ball_v_x > 0)
            {
                des_x = data->plane1.x +  ball.r * qAbs(data->plane1.x - cross_x)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_x > 1400)
                {
                    des_x = data->plane1.x -  ball.r * qAbs(data->plane1.x - cross_x)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            else {
                des_x = data->plane1.x -  ball.r * qAbs(data->plane1.x - cross_x)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_x < 100)
                {
                    des_x = data->plane1.x +  ball.r * qAbs(data->plane1.x - cross_x)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            if(key->shield)
            {
                if(ball_v_x>0)
                {
                    des_x+=3 * data->ball->r+data->plane1.r;
                }
                else
                {
                    des_x-=3 * data->ball->r+data->plane1.r;
                }


//                des_y=data->plane1.y;
//                if(ball_v_x*ball_v_y>0)
//                {
//                   des_y=data->plane1.y;
//                   if(ball_y>cross_y)
//                   {
//                       des_x+=data->ball->r+data->plane1.r;
//                   }
//                   else
//                   {
//                       des_x-=data->ball->r+data->plane1.r;
//                   }
//                }
//                else
//                {
//                    des_y=data->plane1.y;
//                    if(ball_y>cross_y)
//                    {
//                        des_x-=data->ball->r+data->plane1.r;
//                    }
//                    else
//                    {
//                        des_x+=data->ball->r+data->plane1.r;
//                    }
//                }
            }




//            if (des_x < data->plane1.r || des_x > 1500 - data->plane1.r)
//            {
//                des_x = data->plane1.x;
//                if (data->plane1.y - cross_y > 0)
//                    des_y = cross_y + ball.r + data->plane1.r;
//                else
//                    des_y = cross_y - ball.r - data->plane1.r;
//            }
//            //如果y越界
//            if (des_y < data->plane1.r || des_y > 1500 - data->plane1.r)
//            {
//                des_y = data->plane1.y;
//                if (data->plane1.x - cross_x > 0)
//                    des_x = cross_x + ball.r + data->plane1.r;
//                else
//                    des_x = cross_x - ball.r - data->plane1.r;
//            }
//            //向相反方向移动球半径的距离
//            des_x = data->plane1.x +  ball.r * qAbs(data->plane1.x - cross_x)
//                    / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
//            des_y = data->plane1.y +  ball.r * qAbs(data->plane1.y - cross_y)
//                    / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
//            //如果x越界
//            if (des_x < data->plane1.r || des_x > 1500 - data->plane1.r)
//            {
//                des_x = data->plane1.x;
//                if (data->plane1.y - cross_y > 0)
//                    des_y = cross_y + ball.r + data->plane1.r;
//                else
//                    des_y = cross_y - ball.r - data->plane1.r;
//            }
//            //如果y越界
//            if (des_y < data->plane1.r || des_y > 1500 - data->plane1.r)
//            {
//                des_y = data->plane1.y;
//                if (data->plane1.x - cross_x > 0)
//                    des_x = cross_x + ball.r + data->plane1.r;
//                else
//                    des_x = cross_x - ball.r - data->plane1.r;
//            }
        }


        if(!rect)continue;
        else{
            rect = 0;
            A = a;
            B = b;
            C = c;
            k = 1;
            goto stop1;
        }
    }

    for(int i=0;i<data->bullet2_size;i++)
    {
        BulletStruct &bullet=data->bullet2[i];
        float bullet_v_x=bullet.v_x,bullet_v_y=bullet_v_y,bullet_x=bullet.x,bullet_y=bullet.y;


        //计算子弹运动直线方程
        float A, B, C;
        A = -1;       //Y前的系数
        B = bullet_v_y / bullet_v_x;   //X前的系数
        C = bullet_y - bullet_x * bullet_v_y / bullet_v_x;     //常数项

        //计算飞机到直线距离
        float dis = fabs((A * data->plane1.y + B * data->plane1.x + C) / vecMod(A, B));

        //计算垂点坐标
        float cross_x = (data->plane1.y - bullet_y + bullet_v_y / bullet_v_x * bullet_x + bullet_v_x / bullet_v_y * data->plane1.x)
                / (bullet_v_x / bullet_v_y + bullet_v_y / bullet_v_x);
        float cross_y = (bullet_v_y / bullet_v_x) * (cross_x - bullet_x) + bullet_y;

        //计算到垂点的时间
        float t = (cross_x - bullet_x) / bullet_v_x * 100;
        //反向运动或时间过久则忽略该球
        if (t < 0 || t > preTime)
        {
            continue;
        }
        //判断该子弹是否有威胁
        if (dis < data->plane1.r + bullet.r + 5 && t < min_t)
        {
            //设置最紧迫威胁
            min_t = t;
            if(bullet_v_y > 0)
            {
                des_y = data->plane1.y -   qAbs(data->plane1.y - cross_y)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_y < 50)
                {
                    des_y = data->plane1.y +  bullet.r * qAbs(data->plane1.y - cross_y)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            else {
                des_y = data->plane1.y +  bullet.r * qAbs(data->plane1.y - cross_y)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_y > 1400)
                {
                    des_y = data->plane1.y -  bullet.r * qAbs(data->plane1.y - cross_y)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            if(bullet_v_x > 0)
            {
                des_x = data->plane1.x +  bullet.r * qAbs(data->plane1.x - cross_x)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_x > 1900)
                {
                    des_x = data->plane1.x -  bullet.r * qAbs(data->plane1.x - cross_x)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
            else {
                des_x = data->plane1.x -  bullet.r * qAbs(data->plane1.x - cross_x)
                        / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                if(des_x < 50)
                {
                    des_x = data->plane1.x +  bullet.r * qAbs(data->plane1.x - cross_x)
                            / vecMod(data->plane1.x - cross_x, data->plane1.y - cross_y);
                }
            }
        }
    }



    if (min_t != INT_MAX)
    {
        return 1;
    }

    //给定时间内无危险
    return 0;
}

void ai_pve(DataStruct*data,KeyStruct*key){
    //默认不移动且恒发射子弹
    key->forward = false;
    key->rotate_left = false;
    key->rotate_right = false;
    key->shoot = true;

    static int cnt = 0;
    for(int i=0;i<data->ball_size;i++)
    {
        if(qAbs(data->ball->x - data->plane1.x) < 200 && qAbs(data->ball->y - data->plane1.y) < 300)
            cnt++;
    }
    if(cnt >= 3)
    {
        cnt = 0;

        key->shield = true;
    }
    else {
        key->shield = false;
    }

    //创建静态移动标志
    static float moveFlag = 1, des_x, des_y;
    //标志为0则正在移动或检查发现需要移动
    if (needmove(data, des_x, des_y,key) || moveFlag == 0)
    {
        //进行移动并返回移动结果
        moveFlag = moveAt(data, des_x, des_y, key->forward, key->rotate_left, key->rotate_right);
        //返回标志为-1则发生错误
        if (moveFlag == -1)
        {
            std::cout << "移动发生错误" << std::endl;
        }
        //结束函数执行
        return;
    }

    //如果球个数不为零
    if (data->ball_size)
    {
        ShootBall(data, data->ball[FindBall(data)], key->rotate_left, key->rotate_right);
    }
}
