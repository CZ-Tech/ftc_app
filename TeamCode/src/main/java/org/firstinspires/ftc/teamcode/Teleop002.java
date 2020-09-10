package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Teleop002", group="Iterative Opmode")


public class Teleop002 extends OpMode
{
//region 定义各变量


    //region 定义时间
    private ElapsedTime runtime = new ElapsedTime();
    //endregion

    //region 定义电机
    private DcMotor zuoqian;
    private DcMotor zuohou;
    private DcMotor youqian;
    private DcMotor youhou;
    private DcMotor tisheng;
    private Servo shangzhua;
    private Servo xiazhua;
    private Servo guding;
    //endregion

    // region 定义功率
    double power_zuoqian;
    double power_youqian;
    double power_zuohou;
    double power_youhou;
    double power_tisheng;
    //endregion

    //region 定义手柄上的摇杆上的拨动时的输出的数值
    double p1lx;
    double p1rx;
    double p1ly;
    double p2ly;
    //endregion

    //region 定义舵机
    double highest_position;
    double lowest_position;
    //endregion

//endregion

//region 初始化

    @Override
    public void init()
    {
        // region 初始化电机

        //region 配对电机
        zuoqian = hardwareMap.get(DcMotor.class, "motor_zuoqian");
        youqian = hardwareMap.get(DcMotor.class, "motor_youqian");
        zuohou = hardwareMap.get(DcMotor.class, "motor_zuohou");
        youhou = hardwareMap.get(DcMotor.class, "motor_youhou");
        // endregion

        //region 配对伺服电机
        tisheng = hardwareMap.get(DcMotor.class,"tisheng");
        shangzhua = hardwareMap.get(Servo.class,"shangzhua");
        xiazhua = hardwareMap.get(Servo.class,"xiazhua");
        guding = hardwareMap.get(Servo.class,"guding");
        //endregion

        //region 设置电机的转动方向
        zuoqian.setDirection(DcMotor.Direction.FORWARD);
        zuohou.setDirection(DcMotor.Direction.FORWARD);
        youqian.setDirection(DcMotor.Direction.REVERSE);
        youhou.setDirection(DcMotor.Direction.REVERSE);
        tisheng.setDirection(DcMotor.Direction.FORWARD);
        //endregion

        //region 设置伺服电机的初始位置
        lowest_position = 1;
        //endregion

        //region 输出“初始化已完成”
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion
    }

    @Override
    public void init_loop()
    {

    }
    //endregion

//region 开始


    //region重置机器人运行时间
    @Override
    public void start()//
    {
        runtime.reset();

    }
    //endregion

    //region 主循环


    @Override
    public void loop()
    {
        //region 底盘手动程序


        //region 读取手柄上的信息
        p1lx = gamepad1.left_stick_x;
        p1ly = -gamepad1.left_stick_y;
        p1rx = Range.clip(gamepad1.right_stick_x, -0.6, 0.6);
        //endregion

        //region 计算得到底盘各电机要输出的功率
        power_zuoqian = Range.clip(p1ly + p1lx + p1rx, -0.7, 0.7);
        power_youqian = Range.clip(p1ly - p1lx - p1rx, -0.7, 0.7);
        power_zuohou = Range.clip(p1ly - p1lx + p1rx, -0.7, 0.7);
        power_youhou = Range.clip(p1ly + p1lx - p1rx, -0.7, 0.7);
        //endregion

        //region 为各电机输出功率
        zuoqian.setPower(power_zuoqian);
        youqian.setPower(power_youqian);
        zuohou.setPower(power_zuohou);
        youhou.setPower(power_youhou);
        //endregion


        //region 完成机器人各功能的程序 TODO:测试伺服电机的旋转

        //region 完成提升装置的升降
        p2ly = -gamepad2.left_stick_y;
        power_tisheng = Range.clip(p2ly,-1,1);
        tisheng.setPower(power_tisheng);
        //endregion

        //region上爪的前后伸缩
        if (gamepad2.dpad_right)
        {
            shangzhua.setPosition(highest_position += 0.1);
        }
        if (gamepad2.dpad_left)
        {
            shangzhua.setPosition(highest_position -= 0.1);
        }
        //endregion

        //region 完成机械掌的开合
        if (gamepad2.left_bumper)
        {
            xiazhua.setPosition(1);
        }
        if (gamepad2.right_bumper)
        {
            xiazhua.setPosition(0);
        }
        //endregion

        //region 底盘勾爪的伸缩
        if (gamepad2.x)
        {
            guding.setPosition(0.3);
        }
        if (gamepad2.y)
        {
            guding.setPosition(1);
        }
        //endregion

        //endregion



        // region 输出信息
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "zuoqian (%.2f), youqian (%.2f), zuohou(%.2f), youhou(%.2f)", power_zuoqian, power_youqian, power_zuohou, power_youhou);
        //endregion

    }
    //endregion


//endregion

    //region 结束
    @Override
    public void stop()
    {

    }
//endregion
}
