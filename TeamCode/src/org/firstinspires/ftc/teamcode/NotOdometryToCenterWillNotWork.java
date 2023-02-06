package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Vector;


@TeleOp
public class NotOdometryToCenterWillNotWork extends OpMode {

    public int lfPrevPos, lbPrevPos, rfPrevPos, rbPrevPos;

    // for readability
    public final int LF = 0;
    public final int LB = 1;
    public final int RF = 2;
    public final int RB = 3;

    boolean isTurning;
    boolean wasTurning;

      //int[][] distances = new int[][];
    Vector < float [] > angles = new Vector <> ();

    BNO055IMU imu;
    DcMotor lf, lb, rf, rb;

    public void stolenDriveCode () {

        double px = gamepad1.left_stick_x;
        double py = -gamepad1.left_stick_y;

        double p1 = px - py;
        double p2 = px + py;
        double p3 = px - py;
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

    public int[][] addDistance (int[][] distances) { // need odometry

        distances [distances.length-1] [LF] = Math.abs(lf.getCurrentPosition() - lfPrevPos);
        distances [distances.length-1] [LB] = Math.abs(lb.getCurrentPosition() - lbPrevPos);
        distances [distances.length-1] [RF] = Math.abs(rf.getCurrentPosition() - rfPrevPos);
        distances [distances.length-1] [RB] = Math.abs(rb.getCurrentPosition() - rbPrevPos);

        lfPrevPos = lf.getCurrentPosition();
        lbPrevPos = lb.getCurrentPosition();
        rfPrevPos = rf.getCurrentPosition();
        rbPrevPos = rb.getCurrentPosition();

        //int[][]

        return distances;
    }

    //------------------------------------------------------------------------------------------------------------------
    public void init() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        telemetry.addData("imu", "initialised");

        telemetry.addData("heading", imu.getAngularOrientation() );
        telemetry.addData("axies order", imu.getAngularOrientation().axesOrder);


        lf = hardwareMap.dcMotor.get("front_left_motor");
        lb = hardwareMap.dcMotor.get("back_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        rb = hardwareMap.dcMotor.get("back_right_motor");

        telemetry.addData("motors", "initialised");


        telemetry.addData("hardware", "initialised");

    }

    public void start () {
    }

    public void loop () {

        stolenDriveCode();



        // have an array of lenght 2 arrays with a double (angle) to an int (ticks traveled) used this to map the movement of the robot so you can essentially copy it to move back

    }

}
