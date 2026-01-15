package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.dfrobot.HuskyLens;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "HybridPIDPowerTest")
public class HybridPIDPowerTest extends OpMode {
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
    protected HuskyLens huskyLens;

    // HuskyLens calibration constants
    private static final double TAG_WIDTH_INCHES = 6.5;
    private static final double FOCAL_LENGTH = 300.461;
    private static final double HEIGHT_DIFF_INCHES = 18.0;
    private static final int HUSKYLENS_WIDTH = 320;  // HuskyLens resolution width

    // Auto-aim target position (75% from right = 25% from left = 80 pixels on 320 width screen)
    private static final double TARGET_X_PERCENT = 0.25;  // 25% from left edge
    private static final int TARGET_X_PIXELS = (int)(HUSKYLENS_WIDTH * TARGET_X_PERCENT);  // 80 pixels
    private static final double AIM_TOLERANCE_PIXELS = 15.0;  // Tolerance for "centered"
    private static final double AIM_KP = 0.003;  // Proportional gain for auto-aim

    // Lookup table: {distance (inches), leftHood position, power (0.0-1.0)}
    private static final double[][] SHOOTER_LOOKUP_TABLE = {
        {18, 0.05, 0.40},
        {24, 0.05, 0.40},
        {30, 0.05, 0.40},
        {36, 0.30, 0.425},
        {42, 0.30, 0.425},
        {48, 0.20, 0.45},
        {54, 0.30, 0.45},
        {60, 0.30, 0.45},
        {66, 0.30, 0.47},
        {72, 0.30, 0.49},
        {118, 0.30, 0.55}
    };

    // Motor correction multipliers (to make robot drive straight)
    private static final double LF_MULTIPLIER = 0.3525 / 0.41;  // ≈ 0.8598
    private static final double RF_MULTIPLIER = 0.35 / 0.41;    // ≈ 0.8537
    private static final double LB_MULTIPLIER = 1.0;            // = 1.0
    private static final double RB_MULTIPLIER = 0.3425 / 0.41;  // ≈ 0.8354

    // Shooter speed
    private double shooterSpeed = 0.0;

    // Hood adjustment positions
    private double leftHoodPosition = 0.15;
    private double rightHoodPosition = 0.15;
    private boolean lastDpadRightState = false;
    private boolean lastDpadLeftState = false;
    private boolean transfersOpen = false;
    private boolean lastGamepad2AState = false;

    // Shooter encoder tracking for automatic transfer
    private int lastShooterEncoderPosition = 0;
    private double shooterVelocity = 0.0;  // Ticks per second
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean autoTransferTriggered = false;

    // Target shooter velocity threshold (as a percentage of current speed, 0.0-1.0)
    private double targetShooterSpeedThreshold = 0.95;  // 95% of current speed reached

    // AprilTag tracking variables
    private double detectedDistance = 0.0;
    private int detectedTagX = -1;
    private boolean tagDetected = false;
    private boolean autoAimEnabled = false;
    private boolean lastYButtonState = false;
    private double autoAimRotation = 0.0;
    private boolean isAimed = false;

    // Shooter constants - adjust these based on your motor
    // This maps power to expected velocity: at power X, expect X * this value in ticks/sec
    private static final double SHOOTER_TICKS_PER_SECOND_AT_FULL_POWER = 2000.0;  // Ticks per second at power = 1.0


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
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set hood adjustments to initial position
        leftHoodAdjustment.setPosition(leftHoodPosition);
        rightHoodAdjustment.setPosition(rightHoodPosition);

        // Set transfers to closed position
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);

        // Initialize velocity tracking
        velocityTimer.reset();
        lastShooterEncoderPosition = 0;

        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Problem communicating with HuskyLens");
        } else {
            telemetry.addData("HuskyLens", "Connected");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

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

        // ========== HUSKYLENS APRILTAG DETECTION ==========
        HuskyLens.Block[] blocks = huskyLens.blocks();
        tagDetected = false;
        detectedDistance = 0.0;
        detectedTagX = -1;
        isAimed = false;

        if (blocks.length > 0) {
            // Use the first detected tag
            HuskyLens.Block block = blocks[0];
            tagDetected = true;
            detectedTagX = block.x;

            // Calculate distance from tag width
            double directDistance = (TAG_WIDTH_INCHES * FOCAL_LENGTH) / block.width;

            // Calculate horizontal distance using Pythagorean theorem
            if (directDistance > HEIGHT_DIFF_INCHES) {
                detectedDistance = Math.sqrt(Math.pow(directDistance, 2) - Math.pow(HEIGHT_DIFF_INCHES, 2));
            } else {
                detectedDistance = directDistance;  // Fallback if too close
            }

            // Auto-aim calculation: calculate rotation needed to center tag at TARGET_X_PIXELS
            double aimError = detectedTagX - TARGET_X_PIXELS;  // Positive = tag is to the right of target
            isAimed = Math.abs(aimError) <= AIM_TOLERANCE_PIXELS;

            if (autoAimEnabled && !isAimed) {
                // Calculate rotation power based on error
                autoAimRotation = aimError * AIM_KP;
                autoAimRotation = Range.clip(autoAimRotation, -0.3, 0.3);  // Limit rotation speed
            } else {
                autoAimRotation = 0.0;
            }

            // Apply lookup table values based on distance
            double[] lookupValues = interpolateLookupTable(detectedDistance);
            leftHoodPosition = lookupValues[0];
            shooterSpeed = lookupValues[1];

            // Apply hood positions
            leftHoodAdjustment.setPosition(leftHoodPosition);
            // Map left hood to right hood: left 0.05 -> right 0.25, left 0.3 -> right 0.0
            double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
            rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
            rightHoodAdjustment.setPosition(rightHoodPosition);
        }

        // Toggle auto-aim with Y button
        if (gamepad1.y && !lastYButtonState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastYButtonState = gamepad1.y;

        // Apply auto-aim rotation to drive motors if enabled
        if (autoAimEnabled && tagDetected && !isAimed) {
            // Override yaw with auto-aim rotation
            leftFrontPower += (float) autoAimRotation;
            rightFrontPower -= (float) autoAimRotation;
            leftBackPower += (float) autoAimRotation;
            rightBackPower -= (float) autoAimRotation;

            // Re-clip values after adding auto-aim
            rightFrontPower = (float) Range.clip(rightFrontPower, -1.0, 1.0);
            leftFrontPower = (float) Range.clip(leftFrontPower, -1.0, 1.0);
            leftBackPower = (float) Range.clip(leftBackPower, -1.0, 1.0);
            rightBackPower = (float) Range.clip(rightBackPower, -1.0, 1.0);
        }

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

        // ========== SHOOTER SPEED CONTROL ==========
        // Shooter speed is automatically set from lookup table based on AprilTag distance
        // Manual bumper control removed - speed is now automatic

        // Set shooter power
        shooter.setPower(shooterSpeed);

        // ========== SHOOTER VELOCITY TRACKING AND AUTO TRANSFER ==========

        // Calculate shooter velocity from encoder
        int currentShooterPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update velocity every 20ms
            int deltaTicks = currentShooterPosition - lastShooterEncoderPosition;
            shooterVelocity = Math.abs(deltaTicks / deltaTime);  // Ticks per second
            lastShooterEncoderPosition = currentShooterPosition;
            velocityTimer.reset();
        }

        // Calculate current speed as percentage of motor's max capacity (0.0 to 1.0)
        // This is directly comparable to setPower values
        double currentSpeedPercent = shooterVelocity / SHOOTER_TICKS_PER_SECOND_AT_FULL_POWER;

        // Target range: 95% to 105% of shooter power setting
        // e.g., for 0.4 power: lower = 0.38, upper = 0.42
        double targetLowerBound = shooterSpeed * targetShooterSpeedThreshold;  // 95% of target
        double targetUpperBound = shooterSpeed * 1.05;  // 105% of target

        // Check if speed is in the valid range
        boolean speedInRange = shooterSpeed > 0 && currentSpeedPercent >= targetLowerBound && currentSpeedPercent <= targetUpperBound;

        // Auto transfer logic: open when B is pressed AND speed is within target range, close after 2.5 seconds
        if (gamepad1.b && speedInRange && !autoTransferTriggered) {
            // Shooter has reached target speed - open transfer
            autoTransferTriggered = true;
            transfersOpen = true;
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            transferTimer.reset();
        }

        // Close transfer after 2.5 seconds
        if (autoTransferTriggered && transferTimer.seconds() >= 2.5) {
            autoTransferTriggered = false;
            transfersOpen = false;
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
        }

        // Reset auto transfer trigger when shooter is turned off
        if (shooterSpeed == 0) {
            autoTransferTriggered = false;
        }

        // ========== HOOD ADJUSTMENT CONTROL (Controller 1 D-Pad) ==========

        // Right D-Pad: Increase leftHoodAdjustment by 0.05
        if (gamepad1.dpad_right && !lastDpadRightState) {
            leftHoodPosition = Range.clip(leftHoodPosition + 0.05, 0.05, 0.3);
            leftHoodAdjustment.setPosition(leftHoodPosition);
            // Map left hood to right hood: left 0.05 -> right 0.25, left 0.3 -> right 0.0
            double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * 0.25;
            rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
            rightHoodAdjustment.setPosition(rightHoodPosition);
        }
        lastDpadRightState = gamepad1.dpad_right;

        // Left D-Pad: Decrease leftHoodAdjustment by 0.05
        if (gamepad1.dpad_left && !lastDpadLeftState) {
            leftHoodPosition = Range.clip(leftHoodPosition - 0.05, 0.05, 0.3);
            leftHoodAdjustment.setPosition(leftHoodPosition);
            // Map left hood to right hood: left 0.05 -> right 0.25, left 0.3 -> right 0.0
            double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * 0.25;
            rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
            rightHoodAdjustment.setPosition(rightHoodPosition);
        }
        lastDpadLeftState = gamepad1.dpad_left;

        // ========== TELEMETRY ==========
        telemetry.addData("--- AprilTag ---", "");
        telemetry.addData("Tag Detected", tagDetected);
        if (tagDetected) {
            telemetry.addData("Distance (in)", "%.1f", detectedDistance);
            telemetry.addData("Tag X Position", "%d (target: %d)", detectedTagX, TARGET_X_PIXELS);
            telemetry.addData("Aimed", isAimed ? "YES" : "NO");
        }
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED (Y to toggle)" : "DISABLED (Y to toggle)");

        telemetry.addData("--- Shooter ---", "");
        telemetry.addData("Shooter Power Set", "%.0f%%", shooterSpeed * 100);
        telemetry.addData("Shooter Velocity (ticks/s)", "%.0f", shooterVelocity);
        telemetry.addData("Actual Speed %", "%.1f%%", currentSpeedPercent * 100);
        telemetry.addData("Target Range", "%.1f%% - %.1f%%", targetLowerBound * 100, targetUpperBound * 100);
        telemetry.addData("Speed In Range", speedInRange ? "YES - Press B to shoot!" : "NO");
        telemetry.addData("Auto Transfer", autoTransferTriggered ? "ACTIVE" : "Ready");

        telemetry.addData("--- Hood ---", "");
        telemetry.addData("Left Hood Position", "%.2f", leftHoodPosition);
        telemetry.addData("Right Hood Position", "%.2f", rightHoodPosition);
        telemetry.addData("Transfer Open", transfersOpen);
        telemetry.update();
    }

    /**
     * Interpolates between lookup table entries to get hood position and shooter power
     * based on detected distance. Returns {hoodPosition, shooterPower}.
     */
    private double[] interpolateLookupTable(double distance) {
        // If distance is less than minimum, use minimum values
        if (distance <= SHOOTER_LOOKUP_TABLE[0][0]) {
            return new double[] {SHOOTER_LOOKUP_TABLE[0][1], SHOOTER_LOOKUP_TABLE[0][2]};
        }

        // If distance is greater than maximum, use maximum values
        int lastIndex = SHOOTER_LOOKUP_TABLE.length - 1;
        if (distance >= SHOOTER_LOOKUP_TABLE[lastIndex][0]) {
            return new double[] {SHOOTER_LOOKUP_TABLE[lastIndex][1], SHOOTER_LOOKUP_TABLE[lastIndex][2]};
        }

        // Find the two entries to interpolate between
        for (int i = 0; i < SHOOTER_LOOKUP_TABLE.length - 1; i++) {
            double dist1 = SHOOTER_LOOKUP_TABLE[i][0];
            double dist2 = SHOOTER_LOOKUP_TABLE[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                // Calculate interpolation factor (0.0 to 1.0)
                double factor = (distance - dist1) / (dist2 - dist1);

                // Interpolate hood position
                double hood1 = SHOOTER_LOOKUP_TABLE[i][1];
                double hood2 = SHOOTER_LOOKUP_TABLE[i + 1][1];
                double interpolatedHood = hood1 + factor * (hood2 - hood1);

                // Interpolate shooter power
                double power1 = SHOOTER_LOOKUP_TABLE[i][2];
                double power2 = SHOOTER_LOOKUP_TABLE[i + 1][2];
                double interpolatedPower = power1 + factor * (power2 - power1);

                return new double[] {interpolatedHood, interpolatedPower};
            }
        }

        // Fallback (should never reach here)
        return new double[] {0.15, 0.0};
    }
}
