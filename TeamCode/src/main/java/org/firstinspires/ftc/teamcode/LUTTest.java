package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "LUTTest")
public class LUTTest extends OpMode {
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftBack;
    protected DcMotor rightBack;
    protected DcMotor shooter;
    protected Servo leftHoodAdjustment;
    protected Servo rightHoodAdjustment;
    protected IMU imu;
    protected Servo leftTransfer;
    protected Servo rightTransfer;

    // Motor correction multipliers (to make robot drive straight)
    private static final double LF_MULTIPLIER = 0.3525 / 0.41;  // ≈ 0.8598
    private static final double RF_MULTIPLIER = 0.35 / 0.41;    // ≈ 0.8537
    private static final double LB_MULTIPLIER = 1.0;            // = 1.0
    private static final double RB_MULTIPLIER = 0.3425 / 0.41;  // ≈ 0.8354

    // Shooter PID Constants
    private static final double kP = 0.0015;
    private static final double kI = 0.00001;
    private static final double kD = 0.00001;
    private static final double kF = 1.0 / 4900.0;  // Feedforward based on max RPM
    private static final double TICKS_PER_REV = 28.0;

    // Shooter state
    private double targetRPM = 0.0;
    private double shooterRPM = 0.0;
    private double shooterPower = 0.0;
    private int lastEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double lastError = 0.0;
    private double integralSum = 0.0;

    // Hood adjustment positions
    private double leftHoodPosition = 0.15;
    private double rightHoodPosition = 0.15;
    private boolean lastDpadRightState = false;
    private boolean lastDpadLeftState = false;
    private boolean transfersOpen = false;
    private boolean lastGamepad2AState = false;
    private boolean lastGamepad1RightBumperState = false;
    private boolean lastGamepad1LeftBumperState = false;


    @Override
    public void init() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");

        // Initialize shooter motor
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // Initialize hood adjustment servos
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        // Initialize IMU for field-centric driving
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize shooter encoder
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize velocity timer
        velocityTimer.reset();
        lastEncoderPosition = 0;

        // Set hood adjustments to initial position
        leftHoodAdjustment.setPosition(leftHoodPosition);
        rightHoodAdjustment.setPosition(rightHoodPosition);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // ========== DRIVING ==========

        // Get the robot's heading from the IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Get joystick inputs
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Rotate the joystick inputs by the robot's heading (field-centric)
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        float lateral = (float) rotX;
        float axial = (float) rotY;

        // Apply a strafe correction factor
        lateral = lateral * 1.1f;

        // Combine the joystick requests for each wheel's power
        float leftFrontPower = axial + lateral + (float) yaw;
        float rightFrontPower = axial - lateral - (float) yaw;
        float leftBackPower = axial - lateral + (float) yaw;
        float rightBackPower = axial + lateral - (float) yaw;

        // Clip values
        rightFrontPower = (float) Range.clip(rightFrontPower, -1.0, 1.0);
        leftFrontPower = (float) Range.clip(leftFrontPower, -1.0, 1.0);
        leftBackPower = (float) Range.clip(leftBackPower, -1.0, 1.0);
        rightBackPower = (float) Range.clip(rightBackPower, -1.0, 1.0);

        // Apply motor correction multipliers
        rightFrontPower *= RF_MULTIPLIER;
        leftFrontPower *= LF_MULTIPLIER;
        leftBackPower *= LB_MULTIPLIER;
        rightBackPower *= RB_MULTIPLIER;

        // Set motor powers
        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        if (gamepad1.a && !lastGamepad2AState) {
            transfersOpen = !transfersOpen;
            if (transfersOpen) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
            } else {
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
            }
        }
        lastGamepad2AState = gamepad1.a;

        // ========== SHOOTER SPEED CONTROL (Controller 1) ==========

        // Right Bumper: Increase shooter speed by 0.5
        if (gamepad1.right_bumper && !lastGamepad1RightBumperState) {
            targetRPM = Range.clip(targetRPM + 50.0, 0.0, 5700.0);
        }
        lastGamepad1RightBumperState = gamepad1.right_bumper;

        // Left Bumper: Decrease shooter speed by 0.5
        if (gamepad1.left_bumper && !lastGamepad1LeftBumperState) {
            targetRPM = Range.clip(targetRPM - 50.0, 0.0, 5700.0);
        }
        lastGamepad1LeftBumperState = gamepad1.left_bumper;

        // ========== SHOOTER PID CONTROL ==========

        // Calculate shooter RPM
        int currentPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update every 20ms
            int deltaTicks = currentPosition - lastEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            shooterRPM = (ticksPerSecond / TICKS_PER_REV) * 60.0;

            // Calculate PID output
            double error = targetRPM - shooterRPM;
            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            // Feedforward + PID
            double feedforward = targetRPM * kF;
            double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
            shooterPower = Range.clip(feedforward + pidOutput, 0.0, 1.0);

            // Set shooter power
            shooter.setPower(shooterPower);

            // Update variables for next loop
            lastError = error;
            lastEncoderPosition = currentPosition;
            velocityTimer.reset();
        }

        // ========== HOOD ADJUSTMENT CONTROL (Controller 1 D-Pad) ==========

        // Right D-Pad: Increase leftHoodAdjustment by 0.05, decrease rightHoodAdjustment by 0.05
        if (gamepad1.dpad_right && !lastDpadRightState) {
            leftHoodPosition = Range.clip(leftHoodPosition + 0.05, 0.0, 1.0);
            rightHoodPosition = Range.clip(rightHoodPosition - 0.05, 0.0, 1.0);
            leftHoodAdjustment.setPosition(leftHoodPosition);
            rightHoodAdjustment.setPosition(rightHoodPosition);
        }
        lastDpadRightState = gamepad1.dpad_right;

        // Left D-Pad: Decrease leftHoodAdjustment by 0.05, increase rightHoodAdjustment by 0.05
        if (gamepad1.dpad_left && !lastDpadLeftState) {
            leftHoodPosition = Range.clip(leftHoodPosition - 0.05, 0.0, 1.0);
            rightHoodPosition = Range.clip(rightHoodPosition + 0.05, 0.0, 1.0);
            leftHoodAdjustment.setPosition(leftHoodPosition);
            rightHoodAdjustment.setPosition(rightHoodPosition);
        }
        lastDpadLeftState = gamepad1.dpad_left;

        // ========== TELEMETRY ==========
        telemetry.addData("Shooter Target RPM", "%.2f", targetRPM);
        telemetry.addData("Shooter Actual RPM", "%.2f", shooterRPM);
        telemetry.addData("Shooter Power", "%.2f", shooterPower);
        telemetry.addData("Left Hood Position", "%.2f", leftHoodPosition);
        telemetry.addData("Right Hood Position", "%.2f", rightHoodPosition);
        telemetry.update();
    }
}
