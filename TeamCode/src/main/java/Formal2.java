/**
 * 这个类是参加2019FTC的手动正式程序，主要用手柄控制机器人的移动自转，以及机器人上的各种功能。
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Formal2", group="Iterative Opmode")
//为这段程序归类，方便寻找
//“@TeleOp”将这段程序归类为手动程序，自动程序为“@Autonomous”
//“name=”后是这段程序在与手柄连接的手机（DriveStation / DS）
//"group="后是这段程序的组别
public class Formal2 extends OpMode {
    //region 定义各变量
    /**
     * 这里是定义控制我们机器人所需的变量，每个变量都对应着一个对象
     * “private”是声明这个变量是一个私有变量，无法在其它类里使用
     * “DcMotor”、“double”、“BNO055IMU”等都是变量类型，后面的motor_zuoqian等都是为了方便而给变量起的名字
     * 编程时要首先声明一个对象，然后再给这个对象一些行为
     */

    //region 定义时间
    private ElapsedTime runtime = new ElapsedTime();

//ElapsedTime是一个声明时间的变量，此变量的具体的实现方式已被我都不知道的dalao们编写好了，刚开始编程时很多时候都不需要纠结一个东西的具体实现方式，只要按照方法来实现它即可，“能用不就行了”
    //endregion

    //region 定义电机
    private DcMotor motor_zuoqian;
    DcMotor motor_zuohou;
    DcMotor motor_youqian;
    DcMotor motor_youhou;
    DcMotor motor_xuanzhuan;
    DcMotor motor_lashen;
    DcMotor motor_xuangua;
    DcMotor motor_tian;

    // motor_zuoqian指左前轮上的电机，motor_youqian指右前轮上的电机，以此类推
    //motor_xuanzhuan    旋转机械臂          所使用的电机
    //motor_lashen       伸长或收缩机械臂    所使用的电机
    //motor_xuangua      抬升机器人          所使用的电机
    //motor_tian         机械臂采矿装置的旋转，我们戏称之为“舔狗”
//DcMotor的声明对象是电机，Servo声明的声明对象是伺服舵机。
//电机的主要作用是往一个方向一直旋转，伺服舵机的作用主要是旋转到一定角度，各有各的优点
// 机器人的大部分动作都能通过电机和伺服舵机的旋转来操控，我们这一届参赛的队员们都比较硬核，整个机器人都通过电机来操控
    //endregion

    // region 定义功率
    double power_zuoqian;
    double power_youqian;
    double power_zuohou;
    double power_youhou;
    double power_xuanzhuan;
    double power_lashen;
    double power_xuangua;
    double power_tian;
//power_zuoqian的声明对象是左前轮电机的输出功率，以此类推
//任何事物都可被称为是一个对象。电机可以是对象，电机的输出功率也可以是对象
    //endregion

    //region 定义手柄上的摇杆上的拨动时的输出的数值
    double p1lx;
    double p1rx;
    double p1ly;

//手柄上的摇杆可视为一个上下颠倒的坐标轴，上是y轴负方向，下是y轴正方向，左是x轴负方向，右是x轴正方向
//此变量是优化时所使用，我也不知道能不能优化程序但是感觉上好像优化了
    //endregion

    //endregion

    //region 初始化
    @Override
    public void init() {
        // region 初始化电机
        //region 配对电机
        motor_zuoqian = hardwareMap.get(DcMotor.class, "motor_zuoqian");
        motor_youqian = hardwareMap.get(DcMotor.class, "motor_youqian");
        motor_zuohou = hardwareMap.get(DcMotor.class, "motor_zuohou");
        motor_youhou = hardwareMap.get(DcMotor.class, "motor_youhou");
        motor_xuanzhuan = hardwareMap.get(DcMotor.class, "motor_xuanzhuan");
        motor_xuangua = hardwareMap.get(DcMotor.class, "motor_xuangua");
        motor_lashen = hardwareMap.get(DcMotor.class, "motor_lashen");
        motor_tian = hardwareMap.get(DcMotor.class, "motor_tian");
        //“=”左侧的motor_zuoqian是变量，右侧的motor_zuoqian是配对时需要填写的名字
// 这段程序叫为电机注入灵魂，原来的 DcMotor _zuoqian只是一个变量，它有了自己的名字，名字叫motor_zuoqian。但是它不知道自己控制的是左前轮上的电机，这段程序就是配对变量motor_zuoqian与左前轮上的电机
        // endregion
        //region 设置电机的转动方向
        motor_zuoqian.setDirection(DcMotor.Direction.FORWARD);
        motor_zuohou.setDirection(DcMotor.Direction.FORWARD);
        motor_youqian.setDirection(DcMotor.Direction.REVERSE);
        motor_youhou.setDirection(DcMotor.Direction.REVERSE);
// 电机的正转反转，我们不知道电机怎么样是正转，最好的办法是接上电机试一试
        //endregion
        // region 设置零功率刹车
        motor_lashen.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_xuanzhuan.getController().setMotorZeroPowerBehavior(0, DcMotor.ZeroPowerBehavior.BRAKE);
        motor_xuangua.getController().setMotorZeroPowerBehavior(2, DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion
        //endregion
        //region 输出“初始化已完成”
        telemetry.addData("Status", "Initialized");
        telemetry.update();
/* “telemetry”可以将一些信息传递到DS上，方便读取程序进行到了哪里
    上面这段程序输出的内容是“Status: Initalized”
   “telemetry.addData（"xxx"，"xxx"）;将一些信息储存起来还未传送
   “telemetry.update();”将储存起来的信息发送至DS
 */
        //endregion
    }
//按下“INIT”后只执行一次的程序

    @Override
    public void init_loop() {

    }
//按下“INIT”后循环执行的程序
    //endregion

    //region 开始
    @Override
    public void start() {
        runtime.reset();
        //重置机器人运行时间
    }
//按下“START”后执行一次的程序

    //region 主循环
    @Override
    public void loop() {
        //region 底盘手动程序
        //region 读取手柄上的信息
        p1lx = gamepad1.left_stick_x;
        p1ly = gamepad1.left_stick_y;
        p1rx = Range.clip(gamepad1.right_stick_x, -0.6, 0.6);
//打出“gamepad1.”可以调出手柄的库，gamepad1指手柄1，后面的left_stick_y指左侧摇杆的y轴
//“Range.clip(数值变量，最小值，最大值)”是一个为数值变量限定范围的函数
        //endregion
        //region 计算得到底盘各电机要输出的功率
        power_zuoqian = Range.clip(p1ly + p1lx + p1rx, -1, 1);
        power_youqian = Range.clip(p1ly - p1lx - p1rx, -1, 1);
        power_zuohou = Range.clip(p1ly - p1lx + p1rx, -1, 1);
        power_youhou = Range.clip(p1ly + p1lx - p1rx, -1, 1);
/* 此为底盘全向平移+自转的计算公式
 * 底盘四个电机驱动四个麦克纳姆轮（麦轮），地盘上斜向对立的两个麦轮控制一个斜向的运动，四个麦轮可以进行全向平移；
 * 将斜向的里根据机器人的朝向所在的直线与垂直于机器人朝向所在的直线进行正交分解。
 * 前后平移：当四个电机功率全部设置为1（四个麦轮都向前转）时，左右两侧的分力相互抵消，机器人向前平移；反之，当四个电机功率全部设置为-1时，机器人向后平移
 * 左右平移：按照机器人社常年的麦轮装配方式，当左前电机向前，左后电机向后，右前电机向后，右后电机向前时，机器人向左平移；四个电机设置相反功率，机器人向右平移
 * 全向平移（简易版）：运用矢量的合成，将前后平移与左右平移合成，可进行全向平移
 * 全向平移（增强版）：每两个斜向上的麦轮控制一个斜向的平移，运用矢量的合成将斜向的矢量合成向各个方向平移
 * 这是历代程序都要思考的问题，找个时间拿着四个麦轮发一会呆，列一列草稿，多算算，就能够理解了
 * 麦轮的装配方式可以不同，具体问题具体分析
 *
 * 麦轮的自转：左侧两个麦轮向前旋转，右侧两个麦轮向后旋转，因为斜向两个麦轮的转动方向相反，左右两侧的分力相互抵消，从而使机器人顺时针旋转；四个电机设置相反功率，机器人逆时针旋转
 */
        //endregion
        //endregion
        //region 完成机器人各功能的程序
        if (gamepad2.left_bumper) {
            power_xuangua = 0.5;
            telemetry.addData("时间", runtime.milliseconds());
        } else if (gamepad2.right_bumper) {
            power_xuangua = -0.5;
        } else {
            power_xuangua = 0;
        }
        //机器人的抬升
        if (gamepad2.x) {
            power_tian = -1;
        } else if (gamepad2.y) {
            power_tian = 0;
        }
        //机器人的收集
        power_xuanzhuan = Range.clip(-gamepad2.right_stick_y, -0.8, 0.8);
        //机械臂的旋转
        power_lashen = Range.clip(-gamepad2.left_stick_y, -0.8, 0.8);
        //机械臂的伸缩
        //endregion
        //region 为各电机输出功率
        motor_zuoqian.setPower(power_zuoqian);
        motor_youqian.setPower(power_youqian);
        motor_zuohou.setPower(power_zuohou);
        motor_youhou.setPower(power_youhou);
        motor_xuanzhuan.getController().setMotorPower(0, power_xuanzhuan);
        motor_xuangua.getController().setMotorPower(2, power_xuangua);
        motor_lashen.setPower(power_lashen);
        motor_tian.setPower(power_tian);
/* “DcMotor.setPower(数值);”可以为机器人输出功率
 *  前面的程序所计算的是机器人的功率，但是没有把这个功率输出在各电机上，这段程序就是为机器人提供我们已经计算好了的功率
 */
        //endregion
        // region 输出信息
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "zuoqian (%.2f), youqian (%.2f), zuohou(%.2f), youhou(%.2f),xuanzhuan(%.2f),lashen(%2.2f),xuangua(%.2f),tian(%.2f)", power_zuoqian, power_youqian, power_zuohou, power_youhou, power_xuanzhuan, power_lashen, power_xuangua, power_tian);
        //endregion
    }
//这一段是机器人控制时运行的核心程序，程序的运行方式是不断地循环运行，称之为主循环
    //endregion
    //endregion
    @Override
    public void stop() {
    }
//按下“STOP”后执行一次的程序
}