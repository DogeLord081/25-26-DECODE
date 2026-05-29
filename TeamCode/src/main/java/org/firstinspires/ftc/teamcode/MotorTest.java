package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "MotorTest")
public class MotorTest extends OpMode {
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftBack;
    protected DcMotor rightBack;

    @Override
    public void init() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // Original straight-line values: LF=0.3525, RF=0.32, LB=0.41, RB=0.3125
        // Scaled proportionally so max (0.41) becomes 1.0
        // These exact ratios should maintain straight driving
        double scaleFactor = 1 / 0.41;

        /*
        leftFront.setPower(0.3525 * scaleFactor);   // = 0.8597560976
        rightFront.setPower(0.32 * scaleFactor);    // = 0.7804878049
        leftBack.setPower(0.41 * scaleFactor);      // = 1.0
        rightBack.setPower(0.3125 * scaleFactor);   // = 0.7621951220
         */
        leftFront.setPower(0.5);   // = 0.8597560976
        rightFront.setPower(0);    // = 0.7804878049
        leftBack.setPower(0);      // = 1.0
        rightBack.setPower(0);   // = 0.7621951220
    }
}