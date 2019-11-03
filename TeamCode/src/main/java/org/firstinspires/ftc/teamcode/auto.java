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
   * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
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
//自动程序许多环节未经过测试，只是作为一个事例
  package org.firstinspires.ftc.teamcode;

  import com.qualcomm.ftccommon.configuration.EditLegacyServoControllerActivity;
  import com.qualcomm.hardware.ams.AMSColorSensor;
  import com.qualcomm.hardware.bosch.BNO055IMU;
  import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
  import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
  import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
  import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
  import com.qualcomm.robotcore.hardware.DcMotor;
  import com.qualcomm.robotcore.hardware.DcMotorSimple;
  import com.qualcomm.robotcore.hardware.Servo;
  import com.qualcomm.robotcore.util.ElapsedTime;
  import com.qualcomm.robotcore.util.Range;

  import org.firstinspires.ftc.robotcore.external.ClassFactory;
  import org.firstinspires.ftc.robotcore.external.Func;
  import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
  import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
  import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
  import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
  import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
  import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
  import org.firstinspires.ftc.robotcore.external.navigation.Position;
  import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
  import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
  import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
  import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

  import java.util.List;
  import java.util.Locale;

  @Autonomous(name="auto", group="Linear Opmode")
  public class auto extends LinearOpMode {

      //region 定义变量
      private ElapsedTime runtime = new ElapsedTime();
      //定义时间
      private DcMotor motor_zuoqian;
      private DcMotor motor_youqian;
      private DcMotor motor_zuohou;
      private DcMotor motor_youhou;
      private DcMotor motor_xuangua;
      private DcMotor motor_xuanzhuan;
      //定义电机
      private double yp, rp;
      //定义电机的前后平移左右平移所需的矢量
      private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
      //定义在场地四面上的信标
      private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
      private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
      //定义金矿、银矿
      double time, time1;
      //定义两个可随意赋值的时间
      BNO055IMU imu;
      Orientation angles;
      Acceleration gravity;
      //定义惯性传感器
      private static final String VUFORIA_KEY = "-- YOUR NEW VUFORIA KEY GOES HERE  ---";
      //图像识别所需的密码
//这里的图像识别是借助VUFORIA公司所给的SDK，需要去其公司的网址上注册信息得到一个密码才能使用他们的SDK
      private VuforiaLocalizer vuforia;
      //定义摄像头
      private TFObjectDetector tfod;
      //定义图像识别 tfod——TensorFlow Object Detection的简称，具体使用方式请看示例中ConceptTensorFlowObjectDetection
      double angle = 0;
      //定义机器人摄像头所在是直线与金矿的夹角
      enum MotorMode{
          Forward,Back,Left,Right,Stop,Clock,AntiClock
      }
      //为了写程序方便与美观，枚举了机器人运行时的各种状态，分为向前、向后、向左、向右、停止、顺时针自转、逆时针自转，实现所用的函数在程序底部
      //endregion

      @Override
      public void runOpMode() {
          //region 初始化
          //region 配对电机
          motor_zuoqian = hardwareMap.get(DcMotor.class, "motor_zuoqian");
          motor_youqian = hardwareMap.get(DcMotor.class, "motor_youqian");
          motor_zuohou = hardwareMap.get(DcMotor.class, "motor_zuohou");
          motor_youhou = hardwareMap.get(DcMotor.class, "motor_youhou");
          motor_xuangua = hardwareMap.get(DcMotor.class, "motor_xuangua");
          motor_xuanzhuan = hardwareMap.get(DcMotor.class, "motor_xuanzhuan");
          //endregion
          //region 改变电机自转方向
          motor_zuoqian.setDirection(DcMotor.Direction.FORWARD);
          motor_zuohou.setDirection(DcMotor.Direction.FORWARD);
          motor_youqian.setDirection(DcMotor.Direction.FORWARD);
          motor_youhou.setDirection(DcMotor.Direction.FORWARD);
          //endregion
          //region 设置零功率刹车
          motor_xuangua.getController().setMotorZeroPowerBehavior(2, DcMotor.ZeroPowerBehavior.BRAKE);
          //endregion
          initVuforia();
          //初始化Vuforia
          if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
              initTfod();
          } else {
              telemetry.addData("Sorry!", "This device is not compatible with TFOD");
          }
          //初始化Tfod
          telemetry.addData("Status", "初始化完毕");
          telemetry.update();
          //遥测
//endregion
          waitForStart();//中止程序直到按下开始
          runtime.reset();//重置时间
//region 着陆
          if (opModeIsActive()) {
          //region 空降
              motor_xuangua.getController().setMotorPower(2, 0.5);
              //机器人通过抬升电机以0.5的功率下降
              sleep(4000);
              //中断程序4秒
              motor_xuangua.getController().setMotorPower(2, 0);
              //机器人的功率设置为0
              time = time1 = runtime.milliseconds();
              //记录当前的时间
              telemetry.addData(">", "1");
              telemetry.update();//遥测说明第一环节的程序已完成
          //endregion
          //region 脱离
              runMotor(MotorMode.Forward);
              //机器人通过底盘电机的运转使挂钩脱离
              time=runtime.milliseconds();
              //记录当前时间
              while(runtime.milliseconds()-time<=150)
              {
                  idle();
              }
              //运用实际时间与之前记录时间的差值，中断程序0.15秒
//中断程序有很多种方法，最好都试试选取其中最有效的
              runMotor(MotorMode.Stop);
              //停下机器人
              time = runtime.milliseconds();
              //记录时间
              while (motor_zuohou.getPower() != 0) {
                  idle();
              }
              //等待机器人停止
              telemetry.addData(">", "2");
              telemetry.update();
              //遥测——第二环节的程序已完成
              //endregion
          //region 回收
              motor_xuangua.getController().setMotorPower(2, -0.5);
              sleep(4000);
              //四秒时间收回抬升结构
              motor_xuangua.getController().setMotorPower(2, 0);
              //停下抬升结构的电机
              telemetry.addData(">", "3");
              telemetry.update();
              //遥测——第三环节程序已完成/
              //endregion
          }
          telemetry.addData(">>", "着陆成功");
          telemetry.update();
          //遥测——机器着陆成功

//endregion
//region 取样
          telemetry.addData(">>>", "进入取样阶段");
          telemetry.update();
          //遥测——进入第二阶段
          //region 瞄准
          if (tfod != null) {
              tfod.activate();
              telemetry.addData(">", "超级瞄准已部署");
          }
          //运行tfod
          telemetry.update();
          //遥测——tfod已开启
          while (opModeIsActive()) {
              List<Recognition> updatedRecognition = tfod.getUpdatedRecognitions();//赋值识别的集合
              if (updatedRecognition != null) {//如果识别的集合已被赋值
                  for (Recognition recognition : updatedRecognition) {//顺序列举识别的集合
                      runMotor(MotorMode.AntiClock);
                      //逆时针自转来寻找金矿石
                      if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {//如果找到了金矿石
                          runMotor(MotorMode.Stop);//停止自转
                          angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);//计算机器人正方向与金矿石的夹角
                          telemetry.addData("<", "发现敌人");//遥测——找到了金矿石
                          telemetry.addData("角度:", angle);//遥测——与金矿石的夹角
                      }
                  }
                  while (angle <= -4 || angle >= 4) {
                      if (angle <= -4)
                          runMotor(MotorMode.Clock);
                      else
                          runMotor(MotorMode.AntiClock);
                  }
                  runMotor(MotorMode.Stop);
                  /*通过机器人正方向与金矿石的夹角来找到金矿石，摄像头从最左侧到最右侧一共从-15到15
                  通过自转来调整机器人，直到夹角小于等于4时停下
                   */
              }
              telemetry.update();
          }
          telemetry.update();
          //endregion
          //region 攻击
          telemetry.addData(">","发起进攻");
          runMotor(MotorMode.Forward);
          sleep(3000);
          runMotor(MotorMode.Stop);
          //机器人向前平移三秒后停下
          time = runtime.milliseconds();
          //记录时间

          //endregion

//endregion
//region 宣示主权
          motor_xuanzhuan.setPower(-1);
          sleep(1000);
          motor_xuanzhuan.setPower(0);
          telemetry.addData("我方主权", "已宣示");
          telemetry.update();
//endregion
//region 停止
          runMotor(0, 0, 0, 0);
          motor_xuanzhuan.setPower(0);
          telemetry.addData("自动阶段", "完成");
          //endregion
      }
//region 以下为一些为了程序美观或方便书写所使用的函数
      public void runMotor(double LF, double LB, double RF, double RB) {
          motor_zuoqian.setPower(LF);
          motor_zuohou.setPower(LB);
          motor_youhou.setPower(RB);
          motor_youqian.setPower(RF);
      }
      public void runMotor(MotorMode mode) {
          switch (mode) {
              case Forward:
                  motor_zuoqian.setPower(1);
                  motor_zuohou.setPower(1);
                  motor_youqian.setPower(1);
                  motor_youhou.setPower(1);
              case Back:
                  motor_zuoqian.setPower(-1);
                  motor_zuohou.setPower(-1);
                  motor_youqian.setPower(-1);
                  motor_youhou.setPower(-1);
              case Left:
                  motor_zuoqian.setPower(-1);
                  motor_zuohou.setPower(1);
                  motor_youqian.setPower(1);
                  motor_youhou.setPower(-1);
              case Right:
                  motor_zuoqian.setPower(1);
                  motor_zuohou.setPower(-1);
                  motor_youqian.setPower(-1);
                  motor_youhou.setPower(1);
              case Stop:
                  motor_zuoqian.setPower(0);
                  motor_zuohou.setPower(0);
                  motor_youqian.setPower(0);
                  motor_youhou.setPower(0);
                  motor_xuangua.setPower(0);
              case Clock:
                  motor_zuoqian.setPower(0.2);
                  motor_zuohou.setPower(0.2);
                  motor_youqian.setPower(-0.2);
                  motor_youhou.setPower(-0.2);
              case AntiClock:
                  motor_zuoqian.setPower(-0.2);
                  motor_zuohou.setPower(-0.2);
                  motor_youqian.setPower(0.2);
                  motor_youhou.setPower(0.2);
          }
/* 电机的两种给功率的方式：
    一种是“runmotor（左前功率，右前功率，左后功率，右后功率）”
    一种是“runmotor（MotorMode.枚举出来的运动方式）
   通过这个函数可以大幅简化程序
 */
      }
      private  boolean panduan(double input,double standard,double wucha) {
          if(Math.abs(input-standard)<wucha)return true;
          else return false;
      }
      //计算误差，当输入的值与我们给定的值的范围小于一定量时，给出true的判断
      /**
       * 初始化Vuforia
       */
      private void initVuforia() {
          /*
           * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
           */
          VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

          parameters.vuforiaLicenseKey = VUFORIA_KEY;
          parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

          //  Instantiate the Vuforia engine
          vuforia = ClassFactory.getInstance().createVuforia(parameters);

          // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
      }

      /**
       * 初始化Tfod
       */
      private void initTfod() {
          int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                  "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
          TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
          tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
          tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
      }
      //endregion
  }

