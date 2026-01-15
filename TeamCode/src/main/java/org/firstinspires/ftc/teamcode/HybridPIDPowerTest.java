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

    // Auto-aim target position - dynamically calculated based on approach angle
    // Angle lookup: {angle (degrees), target X percent from left}
    // 45 degrees left → 90% from left, 90 degrees (head on) → 75%, 45 degrees right → 60%
    private static final double[][] ANGLE_TO_TARGET_LOOKUP = {
        {45, 0.90},   // 45 degrees to the left → tag at 90% from left
        {90, 0.75},   // Head on (90 degrees) → tag at 75% from left
        {135, 0.60}   // 45 degrees to the right → tag at 60% from left
    };
    private static final double AIM_TOLERANCE_PIXELS = 15.0;  // Tolerance for "centered"
    private static final double AIM_KP = 0.003;  // Proportional gain for auto-aim

    // HuskyLens horizontal field of view (degrees) - used to calculate approach angle from tag position
    private static final double HUSKYLENS_HFOV_DEGREES = 60.0;

    // Lookup table: {distance (inches), leftHood position, target RPM}
    // RPM values calculated from power percentages assuming max RPM at full power
    // Adjust MAX_SHOOTER_RPM based on your motor specs (e.g., 6000 RPM for a typical shooter motor)
    private static final double MAX_SHOOTER_RPM = 4900.0;
    private static final double[][] SHOOTER_LOOKUP_TABLE = {
        {18, 0.05, 1800},   // 40% -> 2400 RPM
        {24, 0.05, 1800},   // 40% -> 2400 RPM
        {30, 0.05, 1800},   // 40% -> 2400 RPM
        {36, 0.30, 1800},   // 42.5% -> 2550 RPM
        {42, 0.30, 1800},   // 42.5% -> 2550 RPM
        {48, 0.20, 1850},   // 45% -> 2700 RPM
        {54, 0.30, 1850},   // 45% -> 2700 RPM
        {60, 0.30, 1900},   // 45% -> 2700 RPM
        {66, 0.30, 2000},   // 47% -> 2820 RPM
        {72, 0.30, 2050},   // 49% -> 2940 RPM
        {118, 0.30, 2250}   // 55% -> 3300 RPM
    };

    // Shooter PID constants for RPM control
    private static final double SHOOTER_KP = 0.0002;   // Proportional gain
    private static final double SHOOTER_KI = 0.00001;  // Integral gain
    private static final double SHOOTER_KD = 0.00001;  // Derivative gain
    private static final double SHOOTER_KF = 1.0 / MAX_SHOOTER_RPM;  // Feedforward gain (1/maxRPM gives base power)

    // Motor correction multipliers (to make robot drive straight)
    private static final double LF_MULTIPLIER = 0.3525 / 0.41;  // ≈ 0.8598
    private static final double RF_MULTIPLIER = 0.35 / 0.41;    // ≈ 0.8537
    private static final double LB_MULTIPLIER = 1.0;            // = 1.0
    private static final double RB_MULTIPLIER = 0.3425 / 0.41;  // ≈ 0.8354

    // Shooter speed
    private double targetShooterRPM = 0.0;  // Target RPM from lookup table
    private double shooterPower = 0.0;      // Actual power sent to motor (controlled by PID)

    // Hood adjustment positions
    private double leftHoodPosition = 0.15;
    private double rightHoodPosition = 0.15;
    private boolean lastDpadRightState = false;
    private boolean lastDpadLeftState = false;
    private boolean transfersOpen = false;
    private boolean lastGamepad2AState = false;

    // Shooter encoder tracking for automatic transfer
    private int lastShooterEncoderPosition = 0;
    private double shooterRPM = 0.0;  // Actual measured RPM
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean autoTransferTriggered = false;

    // Shooter PID state variables
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;

    // Encoder ticks per revolution - adjust based on your motor
    // Common values: 28 (bare motor), 288 (20:1 gearbox), 537.7 (19.2:1 gearbox)
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // AprilTag tracking variables
    private double detectedDistance = 0.0;
    private int detectedTagX = -1;
    private boolean tagDetected = false;
    private boolean autoAimEnabled = false;
    private boolean lastYButtonState = false;
    private double autoAimRotation = 0.0;
    private boolean isAimed = false;
    private double approachAngle = 90.0;  // Approach angle in degrees (90 = head on), calculated from tag position + distance
    private int targetXPixels = HUSKYLENS_WIDTH / 2;  // Dynamic target X position based on angle

    // Shooter constants - RPM tolerance for auto transfer
    private static final double RPM_TOLERANCE_PERCENT = 0.05;  // 5% tolerance


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

            // Calculate horizontal approach angle from tag position and distance
            // Using the tag's X position on screen and the distance, we can calculate
            // the actual horizontal angle to the goal using trigonometry.
            //
            // The key insight: the tag's pixel offset from center corresponds to an angle,
            // and combined with distance, we can find the lateral offset, then the approach angle.
            //
            // 90 degrees = facing goal head-on
            // <90 degrees = goal is to our right (approaching from left side of field)
            // >90 degrees = goal is to our left (approaching from right side of field)

            // Calculate angle offset from center of camera view
            double pixelOffsetFromCenter = detectedTagX - (HUSKYLENS_WIDTH / 2.0);  // -160 to +160
            double angleOffsetRadians = Math.toRadians((pixelOffsetFromCenter / (HUSKYLENS_WIDTH / 2.0)) * (HUSKYLENS_HFOV_DEGREES / 2.0));

            // Calculate lateral distance to goal (positive = goal is to the right of camera center)
            double lateralDistance = detectedDistance * Math.tan(angleOffsetRadians);

            // The approach angle is based on where the goal is relative to straight ahead
            // If goal is to our right (positive lateral), we're approaching from the left, angle > 90
            // If goal is to our left (negative lateral), we're approaching from the right, angle < 90
            // Use atan2 to get the angle: 90 + degrees offset
            double lateralAngleDegrees = Math.toDegrees(Math.atan2(lateralDistance, detectedDistance));
            approachAngle = 90.0 + lateralAngleDegrees;

            // Clamp angle to lookup table range
            double clampedAngle = Range.clip(approachAngle, 45.0, 135.0);

            // Calculate target X percent based on approach angle using interpolation
            double targetXPercent = interpolateAngleToTarget(clampedAngle);
            targetXPixels = (int)(HUSKYLENS_WIDTH * targetXPercent);

            // Auto-aim calculation: calculate rotation needed to center tag at dynamic targetXPixels
            double aimError = detectedTagX - targetXPixels;  // Positive = tag is to the right of target
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
            targetShooterRPM = lookupValues[1];  // Now in RPM instead of power percentage

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
        // Uses PID control to maintain target RPM

        // ========== SHOOTER RPM TRACKING AND PID CONTROL ==========

        // Calculate shooter RPM from encoder
        int currentShooterPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update velocity every 20ms
            int deltaTicks = currentShooterPosition - lastShooterEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            // Convert ticks/second to RPM: (ticks/sec) / (ticks/rev) * 60 = RPM
            shooterRPM = (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
            lastShooterEncoderPosition = currentShooterPosition;
            velocityTimer.reset();
        }

        // PID control for shooter RPM
        if (targetShooterRPM > 0) {
            double error = targetShooterRPM - shooterRPM;

            // Integrate error (with anti-windup)
            shooterIntegral += error * deltaTime;
            shooterIntegral = Range.clip(shooterIntegral, -5000, 5000);  // Limit integral accumulation

            // Calculate derivative
            double derivative = (error - shooterLastError) / deltaTime;
            shooterLastError = error;

            // Calculate feedforward (base power to reach target RPM)
            double feedforward = targetShooterRPM * SHOOTER_KF;

            // Calculate PID output
            double pidOutput = (SHOOTER_KP * error) + (SHOOTER_KI * shooterIntegral) + (SHOOTER_KD * derivative);

            // Combine feedforward and PID
            shooterPower = feedforward + pidOutput;
            shooterPower = Range.clip(shooterPower, 0.0, 1.0);
        } else {
            // Reset PID state when shooter is off
            shooterPower = 0.0;
            shooterIntegral = 0.0;
            shooterLastError = 0.0;
        }

        // Set shooter power
        shooter.setPower(shooterPower);

        // Check if RPM is within tolerance for auto transfer
        double rpmLowerBound = targetShooterRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = targetShooterRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmInRange = targetShooterRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        // Auto transfer logic: open when B is pressed AND RPM is within target range, close after 2.5 seconds
        if (gamepad1.b && rpmInRange && !autoTransferTriggered) {
            // Shooter has reached target RPM - open transfer
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
        if (targetShooterRPM == 0) {
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
            telemetry.addData("Approach Angle", "%.1f°", approachAngle);
            telemetry.addData("Tag X Position", "%d (target: %d)", detectedTagX, targetXPixels);
            telemetry.addData("Aimed", isAimed ? "YES" : "NO");
        }
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED (Y to toggle)" : "DISABLED (Y to toggle)");

        telemetry.addData("--- Shooter ---", "");
        telemetry.addData("Target RPM", "%.0f", targetShooterRPM);
        telemetry.addData("Actual RPM", "%.0f", shooterRPM);
        telemetry.addData("Shooter Power", "%.1f%%", shooterPower * 100);
        telemetry.addData("RPM Range", "%.0f - %.0f", rpmLowerBound, rpmUpperBound);
        telemetry.addData("RPM In Range", rpmInRange ? "YES - Press B to shoot!" : "NO");
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

    /**
     * Interpolates between angle lookup table entries to get target X percent
     * based on detected approach angle. Returns target X position as percent from left (0.0 to 1.0).
     */
    private double interpolateAngleToTarget(double angle) {
        // If angle is less than minimum, use minimum value
        if (angle <= ANGLE_TO_TARGET_LOOKUP[0][0]) {
            return ANGLE_TO_TARGET_LOOKUP[0][1];
        }

        // If angle is greater than maximum, use maximum value
        int lastIndex = ANGLE_TO_TARGET_LOOKUP.length - 1;
        if (angle >= ANGLE_TO_TARGET_LOOKUP[lastIndex][0]) {
            return ANGLE_TO_TARGET_LOOKUP[lastIndex][1];
        }

        // Find the two entries to interpolate between
        for (int i = 0; i < ANGLE_TO_TARGET_LOOKUP.length - 1; i++) {
            double angle1 = ANGLE_TO_TARGET_LOOKUP[i][0];
            double angle2 = ANGLE_TO_TARGET_LOOKUP[i + 1][0];

            if (angle >= angle1 && angle <= angle2) {
                // Calculate interpolation factor (0.0 to 1.0)
                double factor = (angle - angle1) / (angle2 - angle1);

                // Interpolate target X percent
                double target1 = ANGLE_TO_TARGET_LOOKUP[i][1];
                double target2 = ANGLE_TO_TARGET_LOOKUP[i + 1][1];
                return target1 + factor * (target2 - target1);
            }
        }

        // Fallback (should never reach here)
        return 0.75;  // Default to center-ish
    }
}
