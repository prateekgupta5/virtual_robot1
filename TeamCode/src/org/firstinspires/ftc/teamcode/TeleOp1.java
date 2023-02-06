package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@TeleOp
public class TeleOp1 extends OpMode{
    public DcMotor lf, lb, rf, rb;
    public DistanceSensor f, b, l, r;
    public ColorSensor colorSensor;

    public int lfEncoderTotal, lbEncoderTotal, rfEncoderTotal, rbEncoderTotal;

    public int lfPrevPos, lbPrevPos, rfPrevPos, rbPrevPos;

    public int iTPS, avgTPS;


    public class TPS extends Thread {

        public void run (String motor) {
            int start;
            int end;
            DcMotor encoder = hardwareMap.dcMotor.get(motor);

            while (true) {

                try {
                    start = encoder.getCurrentPosition();
                    wait(1000);
                    end = encoder.getCurrentPosition();

                    telemetry.addData("TPS:", Math.abs(end - start));
                }

                catch ( Exception e ) {
                    telemetry.addData("TPS Measure", "Failed");
                }

            }

        }

    }

    public class Thread2 extends Thread {

        public void run() {
            try {
                telemetry.addData("yee yee", "yee");
            }

            catch (Exception e) {
                telemetry.addData("no", "u");
            }
        }
    }

    public class Thread3 implements Runnable {
        public void run () {
            telemetry.addData("yee yee yee yee", "yee");
        }
    }

    //Thread2 z;

    //Thread3 thread3;

    public TPS tpsInit;


    public void stolenDriveCode () {

        double px = gamepad1.left_stick_x;
        double py = -gamepad1.left_stick_y;
        double pa = -gamepad1.right_stick_x;

        double p1 = -px + py - pa;
        double p2 = px + py -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;

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

    public void loopTPS () {
        iTPS++;

        lfEncoderTotal = Math.abs(lf.getCurrentPosition()-lfPrevPos);
        lbEncoderTotal = Math.abs(lb.getCurrentPosition()-lbPrevPos);
        rfEncoderTotal = Math.abs(rf.getCurrentPosition()-rfPrevPos);
        rbEncoderTotal = Math.abs(rb.getCurrentPosition()-rbPrevPos);

        if (iTPS == 667) {
            iTPS = 0;

            avgTPS = (lfEncoderTotal+lbEncoderTotal+rfEncoderTotal+rbEncoderTotal)/4;
        }

        telemetry.addData("avg TPS", avgTPS);

    }
//----------------------------------------------------------------------------------------------------------------------

    @Override
    public void init() {
        telemetry.addData("hello", "world");

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

        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        telemetry.addData("hardware", "initialized");
    }

    @Override
    public void start() {

        //telemetry.addData("yee","yee");

        //try {z.run();}

        //catch (Exception e){
            //telemetry.addData("", e.getClass().getName());
            //telemetry.addData("", "");
        //}

        //telemetry.addData("", lf);
        //try {
            //tpsInit.run("front_left_motor");
            //tpsInit.run("back_left_motor");
            //tpsInit.run("front_right_motor");
            //tpsInit.run("back_right_motor");

            //telemetry.addData("Successfully initialised", "TPS counters");
        //}

        //catch ( Exception e ) {
            //telemetry.addData("Could not initialise", "TPS counters");
            //telemetry.addData("error:", e.getClass().getName());
            //telemetry.addData("", e.getStackTrace());

        //}

        //try {
            //thread3.run();
        //}

        //catch (Exception e) {
            //telemetry.addData("", e.getClass().getName());
            //telemetry.addData("", "");
        //}

    }

    @Override
    public void loop() {

        stolenDriveCode();


        telemetry.update();
        telemetry.addData("pos (ticks)", "%d, %d, %d, %d", lf.getCurrentPosition(), rf.getCurrentPosition(), lb.getCurrentPosition(), rb.getCurrentPosition());
        telemetry.addData("color:", "%d, %d, %d", colorSensor.red(), colorSensor.green(), colorSensor.blue() );
        loopTPS();
        //telemetry.addData("distance:", "%.1f CM, %.1f CM, %.1f CM, %.1f CM", f.getDistance(DistanceUnit.CM), b.getDistance(DistanceUnit.CM), l.getDistance(DistanceUnit.CM), r.getDistance(DistanceUnit.CM));
        //
    }
}