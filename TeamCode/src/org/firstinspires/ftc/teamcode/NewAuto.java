package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class NewAuto extends OpMode {
    public DcMotor lf, lb, rf, rb;

    @Override
    public void init () {
        lf = hardwareMap.dcMotor.get("front_left_motor");
        lb = hardwareMap.dcMotor.get("back_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        rb = hardwareMap.dcMotor.get("back_right_motor");


        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void loop() {

        lf.setPower(1);
        lb.setPower(1);
        rf.setPower(1);
        rb.setPower(1);

    }

}
