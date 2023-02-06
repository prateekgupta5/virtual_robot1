package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import virtual_robot.controller.VirtualRobotController;

@TeleOp
public class IMUheadingCorrection extends OpMode {

    BNO055IMU imu;
    DcMotor lf, lb, rf, rb;


    public void init() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        telemetry.addData("imu", "initialised");
        telemetry.addData("heading", imu.getAngularOrientation() );

        lf = hardwareMap.dcMotor.get("front_left_motor");
        lb = hardwareMap.dcMotor.get("back_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        rb = hardwareMap.dcMotor.get("back_right_motor");
        telemetry.addData("motors", "initialised");

        telemetry.addData("hardware", "initialised");

        telemetry.addData("axies order", imu.getAngularOrientation().axesOrder);
    }


    public void start () {
        telemetry.addData("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", "");
    }

    public void loop() {

         lf.setPower(gamepad1.left_stick_x);
         rf.setPower(gamepad1.left_stick_x);
         lb.setPower(gamepad1.left_stick_x);
         rb.setPower(gamepad1.left_stick_x);



         if (gamepad1.left_stick_x == 0){

              if (imu.getAngularOrientation().firstAngle < - ( Math.PI / 50 ) ) {

                  lf.setPower(.5);
                  rf.setPower(.5);

                  lb.setPower(.5);
                  rb.setPower(.5);

              }

              else if (imu.getAngularOrientation().firstAngle > ( Math.PI / 50 ) ) {

                  lf.setPower(-.5);
                  rf.setPower(-.5);

                  lb.setPower(-.5);
                  rb.setPower(-.5);


              }
         }

         telemetry.addData("heading", imu.getAngularOrientation());
         telemetry.update();
    }
}

