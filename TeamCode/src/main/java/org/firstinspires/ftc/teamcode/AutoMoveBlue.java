package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DECODE 25-26 AutoMoveBlue")
public class AutoMoveBlue extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions (adjust based on your robot configuration)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move RIGHT at half power for 2 seconds
            telemetry.addData("Status", "Moving RIGHT");
            telemetry.update();

            // Strafe right: LF forward, RF backward, LB backward, RB forward
            leftFront.setPower(0.5);
            rightFront.setPower(-0.5);
            leftBack.setPower(-0.5);
            rightBack.setPower(0.5);

            timer.reset();
            while (opModeIsActive() && timer.seconds() < 2.0) {
                telemetry.addData("Status", "Moving RIGHT");
                telemetry.addData("Time", "%.1f / 2.0 seconds", timer.seconds());
                telemetry.update();
            }

            // Stop motors
            stopMotors();

            telemetry.addData("Status", "Complete");
            telemetry.update();
        }
    }

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}

