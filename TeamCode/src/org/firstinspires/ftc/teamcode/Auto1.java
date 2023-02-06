package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class Auto1 extends OpMode {

    public DcMotor lf, lb, rf, rb;
    public DistanceSensor f, b, l, r;

    int bounceCoolDown;

    double px;
    double py;

    public void stolenDriveCode () {

        double p1 = -px + py;
        double p2 = px + py;
        double p3 = -px + py;
        double p4 = px + py;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        lf.setPower(-p2);
        lb.setPower(-p1);
        rf.setPower(p3);
        rb.setPower(p4);

    }

    //------------------------------------------------------------------------------------------------------------------

    public void init() {
        lf = hardwareMap.dcMotor.get("front_left_motor");
        lb = hardwareMap.dcMotor.get("back_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        rb = hardwareMap.dcMotor.get("back_right_motor");

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        f = hardwareMap.get(DistanceSensor.class, "front_distance");
        b = hardwareMap.get(DistanceSensor.class, "back_distance");
        l = hardwareMap.get(DistanceSensor.class, "left_distance");
        r = hardwareMap.get(DistanceSensor.class, "right_distance");

        telemetry.addData("hardware", "initialized");
    }

    public void start() {
        px = py = -.5;
    }

    public void loop() {
        stolenDriveCode();

        telemetry.addData("", "%.1f, %.1f, %.1f, %.1f", f.getDistance(DistanceUnit.MM), b.getDistance(DistanceUnit.MM), l.getDistance(DistanceUnit.MM), r.getDistance(DistanceUnit.MM));

        if (f.getDistance(DistanceUnit.MM) <= 49 && bounceCoolDown == 0) {
            py = -py;

            bounceCoolDown = 200;
        }

        if (b.getDistance(DistanceUnit.MM) <= 49 && bounceCoolDown == 0) {
            py = -py;

            bounceCoolDown = 200;
        }

        if (l.getDistance(DistanceUnit.MM) <= 49 && bounceCoolDown == 0) {
            px = -px;

            bounceCoolDown = 200;
        }

        if (r.getDistance(DistanceUnit.MM) <= 49 && bounceCoolDown == 0) {
            px = -px;

            bounceCoolDown = 200;
        }

        if (bounceCoolDown != 0) {
            if (bounceCoolDown < 0) {
                telemetry.addData("cool down error", "negative cool down");

                bounceCoolDown = 0;
            }

            else {
                bounceCoolDown --;
            }
        }

    }
}