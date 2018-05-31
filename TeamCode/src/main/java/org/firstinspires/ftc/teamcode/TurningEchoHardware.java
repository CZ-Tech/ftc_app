package org.firstinspires.ftc.teamcode;

import android.view.KeyEvent;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

/**
 * Created by DeanNoreen on 2018/4/20.
 */

/**
 * {@link TurningEchoIMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */

public class TurningEchoHardware extends BasicOpMode_Linear {
    private ElapsedTime runtime = new ElapsedTime();//计时

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    ColorSensor sensorColour1;
    ColorSensor sensorColour2;
    DistanceSensor sensorDistance1;
    DistanceSensor sensorDistance2;

    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;//定义底盘电机
    //motorFL 左前电机
    //motorFR 右前电机
    //motorBL 左后电机
    //motorBR 右后电机

    public DcMotor motorShift = null;

    DcMotor motorArm =null;

    public Servo servoKickBall_1 = null;
    public Servo servoKickBall_2 = null;

    public Servo servoCatchBlock_1 = null;
    public Servo servoCatchBlock_2 = null;
    public Servo servoCatchBlock_3 = null;
    public Servo servoCatchBlock_4 = null;//定义夹持方块的舵机

    public DcMotor motorLift = null;//定义抬升滑轨的电机

    public Servo tripodHead = null;

    Servo watcher = null;

    Servo servoBaby_1 = null;
    Servo servoBaby_2 = null;

    double servoBallPosition_2 = 0.59;
    double servoBallPosition_1 = 0;

    double tripodHeadPosition = Range.clip(0.5, 0, 1);

    boolean autoBlankBalance = false;
    //boolean robot_case_1 = false;
    //boolean robot_case_2 = false;
    final double POWER_MODE_SLOW = 3;
    final double POWER_MODE_FAST = 1;
    double powerMode = Range.clip(1.48, 1,2);//切换快/慢速模式

    boolean armCase = false;

    public boolean shiftReversed = false;

    public boolean block12Catched = false;
    public boolean block34Catched = false;

    public double servoBlockPosition_1_tight = 0.69;
    public double servoBlockPosition_2_tight = 0;
    public double servoBlockPosition_3_tight = 0.3;
    public double servoBlockPosition_4_tight = 0.9;
    public double servoBlockPosition_1_release = 0.35;
    public double servoBlockPosition_2_release = 0.31;
    public double servoBlockPosition_3_release = 0.61;
    public double servoBlockPosition_4_release = 0.61;

    double servoBabyPosition_1_down = 1;
    double servoBabyPosition_1_up = 0;
    double servoBabyPosition_2_tight = 0.3;
    double servoBabyPosition_2_release = 0;

    double yError = 0;
    double xError = 0;

    boolean servoBaby_1_case = false;
    boolean servoBaby_2_case = false;

    boolean shiftCount = false;

    final double errorIMU = 0.8;

    double shiftPosition = 0;

//    double PowerFL = motorFL.getPower();
//    double PowerFR = motorFR.getPower();
//    double PowerBL = motorBL.getPower();
//    double PowerBR = motorBR.getPower();

    public void TurningEchoHardwareConfigure() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        servoCatchBlock_1 = hardwareMap.servo.get("servoCatchBlock_1");
        servoCatchBlock_2 = hardwareMap.servo.get("servoCatchBlock_2");
        servoCatchBlock_3 = hardwareMap.servo.get("servoCatchBlock_3");
        servoCatchBlock_4 = hardwareMap.servo.get("servoCatchBlock_4");

        motorLift = hardwareMap.dcMotor.get("motorLift");

        motorShift = hardwareMap.dcMotor.get("motorShift");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        servoKickBall_1 = hardwareMap.get(Servo.class, "servoKickBall_1");
        servoKickBall_2 = hardwareMap.get(Servo.class, "servoKickBall_2");

        tripodHead = hardwareMap.get(Servo.class, "tripodHead");
        watcher = hardwareMap.get(Servo.class, "watcher");

        servoBaby_1 = hardwareMap.servo.get("servoBaby_1");
        servoBaby_2 = hardwareMap.servo.get("servoBaby_2");

        sensorColour1 = hardwareMap.get(ColorSensor.class, "sensorColourDistance1");
        sensorColour2 = hardwareMap.get(ColorSensor.class, "sensorColourDistance2");

        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "sensorColourDistance1");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensorColourDistance2");

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Hardware", "Initialized");
        telemetry.update();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.addData("Hardware", "Initialized");
        telemetry.addData("parameters", "Initialized");
        telemetry.update();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Hardware", "Initialized");
        telemetry.addData("parameters", "Initialized");
        telemetry.addData("IMU", "Initialized");
        telemetry.update();

        // Set up our telemetry dashboard
        //composeTelemetry();
    }

    public class Thread1 extends Thread {
        public Thread1(String name) {
            this.setName(name);
        }

        public void run() {
            telemetry.addData("shift", "started");//名为“shift”的线程
            telemetry.update();
            try {
                shiftReversed = !shiftReversed;
                if (shiftReversed) {//如果电机（shift）已反转
                    motorShift.setPower(0.63);
                    sleep(150, 0);
                    motorShift.setPower(0.43);
                    sleep(350, 0);
                    motorShift.setPower(0.23);
                    sleep(600, 0);
                }

                else {
                    motorShift.setPower(-0.63);
                    sleep(150, 0);
                    motorShift.setPower(-0.43);
                    sleep(350, 0);
                    motorShift.setPower(-0.23);
                    sleep(600, 0);
                }
                motorShift.setPower(0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public class Thread2 extends Thread{
        public Thread2(String name) {
            this.setName(name);
        }

        public void run() {
            telemetry.addData("initIMU", "started");
            telemetry.update();
            try {
                imu.initialize(parameters);//初始化IMU参数
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);//初始化IMU的陀螺仪角度
                sleep(800);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                yError = Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle));
                xError = Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void initIMU() {
        imu.initialize(parameters);//初始化IMU参数
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);//初始化IMU的陀螺仪角度
    }

    public void autoTurnLocation(double degree) {
        runtime.reset();
        double R;//自转角度
        double rPower;//自转功率
        double target;//目标旋转角度
        while (true) {
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//获得IMU角度
            gravity = imu.getGravity();//获得IMU重力传感器
            R = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            target = R - degree;//目标旋转角度为此时IMU所测角减去设定角度数
            rPower = Range.clip(Math.abs(target / 75), 0.24, 1);//自转功率取绝对值，最低为0.2（太慢无效率），最高为1
            telemetry.addData("rPower = ", rPower);//打印rPower的值
//            telemetry.update();
            if (rPower<0.23){
                rPower=0.23;
            }
            if (target >= -0.4 && target <= 0.4 && getRuntime()>=1.2) {//在+-0.4的角度内停止自转，已足够精确
                break;
            }
//            else if (getRuntime()>=4){
//                break;
//            }
            if (target < -0.4) {
                moveFix(rPower, moveStatus.rL);//向左旋转
            } else if (target > 0.4) {
                moveFix(rPower, moveStatus.rR);//向右旋转
            }
        }
    }

    enum moveStatus {
        yF, yB, xR, xL, rL, rR, S
//      yF = forward 前进
//         yB = back 后退
//            xR = right 右平移
//               xL = left 左平移
//                   rL = 左转
//                      rR = 右转
//                          S = stop 停止
    }

    public void moveFix(double power, moveStatus moveStatus) {//0 <= power <= 1
        switch (moveStatus) {
            case yF:
                motorFL.setPower(power / powerMode);
                motorFR.setPower(power / powerMode);
                motorBL.setPower(power / powerMode);
                motorBR.setPower(power / powerMode);
                break;

            case yB:
                motorFL.setPower(-power / powerMode);
                motorFR.setPower(-power / powerMode);
                motorBL.setPower(-power / powerMode);
                motorBR.setPower(-power / powerMode);
                break;

            case xL:
                motorFL.setPower(-power);
                motorFR.setPower(power);
                motorBL.setPower(power);
                motorBR.setPower(-power);
                break;

            case xR:
                motorFL.setPower(power);
                motorFR.setPower(-power);
                motorBL.setPower(-power);
                motorBR.setPower(power);
                break;

            case rL:
                if (powerMode == 2){
                    motorFL.setPower(-power / (powerMode+1.3));
                    motorFR.setPower(power / (powerMode+1.3));
                    motorBL.setPower(-power / (powerMode+1.3));
                    motorBR.setPower(power / (powerMode+1.3));
                }

                else if (powerMode == 1.48 || powerMode == 1){
                    motorFL.setPower(-power / powerMode);
                    motorFR.setPower(power / powerMode);
                    motorBL.setPower(-power / powerMode);
                    motorBR.setPower(power / powerMode);
                }
                break;

            case rR:
                if (powerMode == 2){
                    motorFL.setPower(power / (powerMode+1.3));
                    motorFR.setPower(-power / (powerMode+1.3));
                    motorBL.setPower(power / (powerMode+1.3));
                    motorBR.setPower(-power / (powerMode+1.3));
                }

                else if (powerMode == 1.48 || powerMode == 1){
                    motorFL.setPower(power / powerMode);
                    motorFR.setPower(-power / powerMode);
                    motorBL.setPower(power / powerMode);
                    motorBR.setPower(-power / powerMode);
                }
                break;

            case S:
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                break;
        }
    }

    public void moveVar(double yPower, double xPower, double powerMode) {
        //                       yPower = y轴功率（前后平移方向上的功率）
        //                                      xPower = x轴功率（左右平移方向上的功率）
        //                                                     rPower = 自转功率（左右转向的功率）
        double FinalPower1 = Range.clip((yPower + xPower) / powerMode, -1, 1);
        double FinalPower2 = Range.clip((yPower - xPower) / powerMode, -1, 1);
        double FinalPower3 = Range.clip((yPower - xPower) / powerMode, -1, 1);
        double FinalPower4 = Range.clip((yPower + xPower) / powerMode, -1, 1);

        motorFL.setPower(FinalPower1);
        motorFR.setPower(FinalPower2);
        motorBL.setPower(FinalPower3);
        motorBR.setPower(FinalPower4);
    }

    public void frameStop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        moveVar(0, 0, 1);

        moveFix(0, moveStatus.yF);
    }

    public void servoKickBall(double servoBallPosition_1, double servoBallPosition_2) {
        servoKickBall_1.setPosition(servoBallPosition_1);
        servoKickBall_2.setPosition(servoBallPosition_2);
    }

    public void catchBlock12() {
        servoCatchBlock_1.setPosition(servoBlockPosition_1_tight);
        servoCatchBlock_2.setPosition(servoBlockPosition_2_tight);
    }

    public void releaseBlock12() {
        servoCatchBlock_1.setPosition(servoBlockPosition_1_release);
        servoCatchBlock_2.setPosition(servoBlockPosition_2_release);
    }

    public void catchBlock34() {
        servoCatchBlock_3.setPosition(servoBlockPosition_3_tight);
        servoCatchBlock_4.setPosition(servoBlockPosition_4_tight);
    }

    public void releaseBlock34() {
        servoCatchBlock_3.setPosition(servoBlockPosition_3_release);
        servoCatchBlock_4.setPosition(servoBlockPosition_4_release);
    }


    public void lift(double powerLift) {//滑轨抬升函数
        motorLift.setPower(powerLift);
    }

    public void shift(double powerShift) {//滑轨抬升函数
        motorLift.setPower(powerShift);
    }

    public double switchPowerMode() {//切换低/高速模式
        if (gamepad1.b){
            return 1;
        }

        else if (gamepad1.x){
            return 2;
        }

        else if (gamepad1.a){
            return 1.48;
        }

        else return powerMode;
//        if (gamepad1.right_stick_y != 0) {
//            powerMode = Range.clip(0.75 * gamepad1.right_stick_y + 1.75, POWER_MODE_FAST, POWER_MODE_SLOW);
//        }
//        return powerMode;

//        if (gamepad1.right_stick_y > 0.4) {
//            return (POWER_MODE_SLOW);
//        } else if (gamepad1.right_stick_y < -0.4) {
//            return (POWER_MODE_FAST);
//        } else {
//            return (powerMode);
//        }
    }

    ////////////////////////////////////////////////////////////
    public void catchBlock() {
        idle();
    }

    public void releaseBlock() {
        idle();
    }

    ////////////////////////////////////////////////////////////////
    public void shakeHead(double range) {
        tripodHeadPosition = tripodHeadPosition + range;
        tripodHead.setPosition(tripodHeadPosition);
        sleep(500);
        tripodHeadPosition = tripodHeadPosition - 2 * range;
        tripodHead.setPosition(tripodHeadPosition);
        sleep(500);
    }

//    public void frameControl() {
//        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {//底盘平移
//            moveVar(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger, powerMode);
//        } else {
//            moveVar(0, 0, 0, 0);
//        }
//    }

    public void frameControl() {
        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {//底盘平移
            moveVar(-gamepad1.left_stick_y, gamepad1.left_stick_x, powerMode);
        }
    }

//    public boolean isDoubleClick(String key) {
//        if (key.equals("dpad_up")) {
//            if (gamepad1.dpad_up) {
//                runtime.reset();
//                while (gamepad1.dpad_up) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.dpad_up) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("dpad_down")) {
//            if (gamepad1.dpad_down) {
//                runtime.reset();
//                while (gamepad1.dpad_down) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.dpad_down) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("dpad_right")) {
//            if (gamepad1.dpad_right) {
//                runtime.reset();
//                while (gamepad1.dpad_right) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.dpad_right) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("dpad_left")) {
//            if (gamepad1.dpad_left) {
//                runtime.reset();
//                while (gamepad1.dpad_left) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.dpad_left) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("a")) {
//            if (gamepad1.a) {
//                runtime.reset();
//                while (gamepad1.a) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.a) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("b")) {
//            if (gamepad1.b) {
//                runtime.reset();
//                while (gamepad1.b) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.b) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("x")) {
//            if (gamepad1.x) {
//                runtime.reset();
//                while (gamepad1.x) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.x) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("y")) {
//            if (gamepad1.y) {
//                runtime.reset();
//                while (gamepad1.y) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.y) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("right_bumper")) {
//            if (gamepad1.right_bumper) {
//                runtime.reset();
//                while (gamepad1.right_bumper) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.right_bumper) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("left_bumper")) {
//            if (gamepad1.left_bumper) {
//                runtime.reset();
//                while (gamepad1.left_bumper) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.left_bumper) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        } else if (key.equals("left_stick_button")) {
//            int count = 0;
//            while (gamepad1.left_stick_button && count <= 5) {
//                count++;
//                sleep(100);
//            }
//            if (count > 4) {
//                return true;
//            } else return false;
//        } else if (key.equals("right_stick_button")) {
//            if (gamepad1.right_stick_button) {
//                runtime.reset();
//                while (gamepad1.right_stick_button) {
//                    idle();
//                }
//                while (getRuntime() < 0.8) {
//                    if (gamepad1.right_stick_button) {
//                        return true;
//                    } else idle();
//                }
//            }
//            return false;
//        }
//        return false;
//    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("gravity", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
