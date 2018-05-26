/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EYPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.YuvImage;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTelemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "fantastic", group = "Linear Opmode")
//@Disabled
public class fantasticTETeleop extends TurningEchoHardware {
    private ElapsedTime runtime = new ElapsedTime();//计时

    boolean catchBlockCase = false;

    int count = 0;

    public void runOpMode() {
        TurningEchoHardwareConfigure();
        Thread1 shift = new Thread1("shift");
        Thread2 initIMU_ALL = new Thread2(("initIMU_ALL"));
        telemetry.addData("Hardware", "Initialized");
        telemetry.addData("parameters", "Initialized");
        telemetry.addData("IMU", "Initialized");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        releaseBlock12();
        releaseBlock34();

        servoKickBall_2.setPosition(0.44);

        tripodHead.setPosition(tripodHeadPosition);

        powerMode = 1;

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

//        PowerFL = 0;
//        PowerFR = 0;
//        PowerBL = 0;
//        PowerBR = 0;

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();

            powerMode = switchPowerMode();//powerMode变量——SwitchpowerMode方法，powerMode变量将有1（正常速度）、2.5（慢速）的返回值

            frameControl();

            /*if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_down) {//dpad的底盘中速运行模式
                if (gamepad1.dpad_up) {
                    moveFix(0.7,moveStatus.xF);
                } else if (gamepad1.dpad_down) {
                    moveFix(0.7,moveStatus.xB);
                } else if (gamepad1.dpad_left) {
                    moveFix(1,moveStatus.rL);
                } else if (gamepad1.dpad_right) {
                    moveFix(1,moveStatus.rR);
                }
            }*/

//            if (!catchBlockCase){
//                if (count == 0){
//                    servoCatchBlock(0.38, 0.36);
//                    count++;
//                }
//                else if (count == 1){
//                    servoCatchBlock(0.37, 0.37);
//                    count--;
//                }
//            }
            if (gamepad1.left_bumper || gamepad1.right_bumper) {//左、右平移
                if (gamepad1.left_bumper) {
                    moveFix(1, moveStatus.xL);
                } else if (gamepad1.right_bumper) {
                    moveFix(1, moveStatus.xR);
                }
            }

//            if (gamepad2.start){
//                motorShift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            if (gamepad2.right_stick_button) {
                shift.start();
                while (gamepad2.right_stick_button) {
                    idle();
                }
            }

            else if (gamepad2.left_stick_button){
                if (shiftReversed){
                    motorShift.setPower(0.2);
                }
                else {
                    motorShift.setPower(-0.2);
                }
                while (gamepad2.left_stick_button){
                    idle();
                }
                motorShift.setPower(0);
            }

            if (gamepad2.right_stick_y<=-0.5) {
                if (!shiftReversed) {
                    if (!block12Catched) {
                        catchBlock12();
                        block12Catched = true;
                    } else if (block12Catched) {
                        releaseBlock12();
                        block12Catched = false;
                    }
                } else if (shiftReversed) {
                    if (!block34Catched) {
                        catchBlock34();
                        block34Catched = true;
                    } else if (block34Catched) {
                        releaseBlock34();
                        block34Catched = false;
                    }
                }
                while (gamepad2.right_stick_y<=-0.5) {
                    idle();
                }
            } else if (gamepad2.right_stick_y>=0.5) {
                if (shiftReversed) {
                    if (!block12Catched) {
                        catchBlock12();
                        block12Catched = true;
                    } else if (block12Catched) {
                        releaseBlock12();
                        block12Catched = false;
                    }
                } else if (!shiftReversed) {
                    if (!block34Catched) {
                        catchBlock34();
                        block34Catched = true;
                    } else if (block34Catched) {
                        releaseBlock34();
                        block34Catched = false;
                    }
                }
                while (gamepad2.right_stick_y>=0.5) {
                    idle();
                }
            }

            if (gamepad2.left_stick_y != 0) {
                lift(-gamepad2.left_stick_y);
            }
            //ARM!ARM!ARM!ARM!ARM!ARM!ARM!ARM!ARM!ARM!ARM!ARM!
            if (gamepad1.dpad_up) {
                servoKickBall_1.setPosition(0.15);
            }
            if (gamepad1.dpad_down) {
                servoKickBall_1.setPosition(0.82);
            }

            if (gamepad1.dpad_left && servoBallPosition_2 <= 1) {
                servoBallPosition_2 = servoBallPosition_2 + 0.02;
                sleep(20);
            }

            if (gamepad1.dpad_right && servoBallPosition_2 >= 0) {
                servoBallPosition_2 = servoBallPosition_2 - 0.02;
                sleep(20);
            }

            servoKickBall_2.setPosition(servoBallPosition_2);

            if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1){
                armCase = true;
                while (gamepad2.left_trigger >0 || gamepad2.right_trigger > 0){
                    idle();
                }
            }

            if (armCase){
                motorArm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            }

//            if (gamepad1.right_stick_x > 0.4) {
//                tripodHeadPosition = tripodHeadPosition + 0.02;
//                tripodHead.setPosition(tripodHeadPosition);
//                sleep(15);
//            } else if (gamepad1.right_stick_x < -0.4) {
//                tripodHeadPosition = tripodHeadPosition - 0.02;
//                tripodHead.setPosition(tripodHeadPosition);
//                sleep(15);
//            }

            if (gamepad1.x && gamepad1.y && gamepad1.a && gamepad1.b) {
                break;
            }

            if (gamepad1.left_stick_button) {
                initIMU_ALL.start();
                while (gamepad1.left_stick_button){
                    idle();
                }
            }

            if (gamepad1.start) {//当一操start被按下
//                moveFix(1,moveStatus.xF);
//                sleep(200);
                double R;//自转角度
                double rPower;//自转功率
                while (true) {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//获得IMU角度
                    gravity = imu.getGravity();//获得IMU重力传感器数据
                    R = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));//
                    rPower = Range.clip(Math.abs(R / 45), 0.15, 1);//自转功率取绝对值，最低为0.15（太慢转不动），最高为1
                    telemetry.addData("rPower = ", rPower);//打印rPower的值
                    telemetry.update();
                    if (R >= -0.7 && R <= 0.7) {//在+-0.7的角度内停止自转，已足够精确
                        break;
                    } else if (R < -0.7) {
                        moveFix(rPower, moveStatus.rL);//向左旋转
                    } else if (R > 0.7) {
                        moveFix(rPower, moveStatus.rR);//向右旋转
                    } else idle();

                    if (!gamepad1.start) {//一操start键松开
                        frameStop();//停止
                        break;//跳出循环
                    }
                }
            }

            if (gamepad1.right_stick_button) {//如果一操右摇杆按钮被按下
                double Y;//y轴姿态角
                double X;//x轴姿态角
                double yPower;//y轴功率
                double xPower;
                while (true) {
                    double Battery = getBatteryVoltage();//获得电池电量
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//当前陀螺仪（三轴姿态角）数据赋给angles
                    gravity = imu.getGravity();//当前加速度数据赋给gravity
//                    Y = Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle)) + 1.6;//Y轴姿态角（车体平放时y轴姿态角是-1.6左右）
//                    X = Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));//x轴姿态角
                    Y = Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle)) - yError + 0.2;
                    X = Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle)) - xError;
                    if (Y > 10) {//Y轴角度控制在-10到10之间（上板和下板的最大倾斜角）
                        Y = 10;
                    } else if (Y < -10) {
                        Y = -10;
                    }
                    if (X > 5) {//X轴角度控制在-5°到5°之间
                        X = 5;
                    } else if (X < -5) {
                        X = -5;
                    }
                    //x、y轴功率与x、y轴姿态角函数关系式
                    yPower = +0.000002652391975309918 * Y * Y * Y * Y * Y - 0.000002411265432094396 * Y * Y * Y * Y - 0.00033661265432107044 * Y * Y * Y + 0.0005806327160490544 * Y * Y + 0.05630401234567932 * Y - 0.042283950617282684;
                    xPower = +0.0016666666666666741 * X * X * X + 1.850371707708594e-17 * X * X - 0.10166666666666667 * X - 1.850371707708594e-17;
                    moveVar(yPower, xPower, 0, 1);//移动函数，输入x、y轴功率值
                    telemetry.addData("blankY", Y);//打印y轴姿态角数据
                    telemetry.addData("yPower", yPower);//打印y轴功率值
                    telemetry.addData("blankX", X);//打印x轴姿态角数据
                    telemetry.addData("xPower", xPower);//打印x轴功率值
                    telemetry.addData("Battery", Battery);//打印电池电量
                    //打印底盘四个电机功率
                    //telemetry.addData("Motors", "zuoqian (%.2f), youqian (%.2f),zuohou (%.2f),youhou (%.2f)", PowerFL, PowerFR, PowerBL, PowerBR);
                    telemetry.update();

                    if (!gamepad1.right_stick_button || gamepad1.start) {//如果一操右摇杆按钮被松开
                        frameStop();//停车
                        break;//退出循环
                    }
                }
            }

//            if (gamepad1.b) {
//                double Y;
//                double X;
//                double yPower;
//                double xPower;
//                while (true) {
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//获得IMU角度
//                    gravity = imu.getGravity();
//                    Y = gravity.yAccel;
//                    X = gravity.xAccel;
//                }
//
//            }
//            PowerFL = motorFL.getPower();
//            PowerFR = motorFR.getPower();
//            PowerBL = motorBL.getPower();
//            PowerBR = motorBR.getPower();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("roll", formatAngle(angles.angleUnit, angles.secondAngle));
            telemetry.addData("pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
//            telemetry.addData("gravityX", gravity.xAccel);
//            telemetry.addData("gravityY", gravity.yAccel);
//            telemetry.addData("gravityZ", gravity.zAccel);
//            telemetry.addData("shiftCurrentPosition",motorShift.getCurrentPosition());
//            telemetry.addData("shiftTargetPosition",motorShift.getTargetPosition());
            telemetry.addData("shiftPower",motorShift.getPower());
            //telemetry.addData("Motors", "zuoqian (%.2f), youqian (%.2f),zuohou (%.2f),youhou (%.2f)", PowerFL, PowerFR, PowerBL, PowerBR);
            telemetry.update();
        }
    }
}