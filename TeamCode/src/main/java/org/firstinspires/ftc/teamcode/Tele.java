package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.dfrobot.HuskyLens;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Tele")
public class Tele extends OpMode {
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftBack;
    protected DcMotor rightBack;
    protected DcMotor shooter;
    protected DcMotor intake;
    protected DcMotor leftLift;
    protected DcMotor rightLift;
    protected Servo leftTrapdoor;
    protected Servo rightTrapdoor;
    protected Servo leftTransfer;
    protected Servo rightTransfer;
    protected Servo leftKickerArm;
    protected Servo rightKickerArm;
    protected Servo leftHoodAdjustment;
    protected Servo rightHoodAdjustment;
    protected ColorSensor colorSensorRight;
    protected ColorSensor colorSensorLeft;
    protected IMU imu;
    protected DistanceSensor distanceSensor;
    protected HuskyLens huskyLens;

    // Motor correction multipliers (to make robot drive straight)
    // Original values: LF=0.3525, RF=0.35, LB=0.41, RB=0.3425
    // Scaled so max (0.41) = 1.0, preserving ratios
    private static final double LF_MULTIPLIER = 0.3525 / 0.41;  // ≈ 0.8598
    private static final double RF_MULTIPLIER = 0.35 / 0.41;    // ≈ 0.8537
    private static final double LB_MULTIPLIER = 1.0;            // = 1.0
    private static final double RB_MULTIPLIER = 0.3425 / 0.41;  // ≈ 0.8354

    // ========== HUSKYLENS & AUTO-AIM CONSTANTS ==========
    private static final double TAG_WIDTH_INCHES = 6.5;
    private static final double FOCAL_LENGTH = 300.461;
    private static final double HEIGHT_DIFF_INCHES = 18.0;
    private static final int HUSKYLENS_WIDTH = 320;  // HuskyLens resolution width
    private static final double HUSKYLENS_HFOV_DEGREES = 60.0;  // Horizontal field of view

    // Auto-aim target position - dynamically calculated based on approach angle
    // Angle lookup: {angle (degrees), target X percent from left}
    private static final double[][] ANGLE_TO_TARGET_LOOKUP = {
            {45, 0.90},   // 45 degrees to the left → tag at 90% from left
            {90, 0.75},   // Head on (90 degrees) → tag at 75% from left
            {135, 0.60}   // 45 degrees to the right → tag at 60% from left
    };
    private static final double AIM_TOLERANCE_PIXELS = 15.0;
    private static final double AIM_KP = 0.003;

    // ========== SHOOTER LOOKUP TABLE & PID CONSTANTS ==========
    private static final double MAX_SHOOTER_RPM = 4900.0;
    private static final double[][] SHOOTER_LOOKUP_TABLE = {
            {18, 0.05, 1800},
            {24, 0.05, 1800},
            {30, 0.05, 1800},
            {36, 0.30, 1800},
            {42, 0.30, 1800},
            {48, 0.20, 1850},
            {54, 0.30, 1850},
            {60, 0.30, 1900},
            {66, 0.30, 2000},
            {72, 0.30, 2050},
            {118, 0.30, 2250}
    };

    // Shooter PID constants
    private static final double SHOOTER_KP = 0.0002;
    private static final double SHOOTER_KI = 0.00001;
    private static final double SHOOTER_KD = 0.00001;
    private static final double SHOOTER_KF = 1.0 / MAX_SHOOTER_RPM;
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE_PERCENT = 0.05;  // 5% tolerance

    // ========== CONTROLLER 1 (DRIVER) STATE ==========
    // Intake toggle state (Right Bumper)
    private boolean intakeToggleOn = false;
    private boolean lastGamepad1RightBumperState = false;

    // Auto-Aim toggle state (Right Trigger)
    private boolean autoAimEnabled = false;
    private boolean lastGamepad1RightTriggerState = false;

    // ========== AUTO-AIM STATE ==========
    private double detectedDistance = 0.0;
    private int detectedTagX = -1;
    private boolean tagDetected = false;
    private double autoAimRotation = 0.0;
    private boolean isAimed = false;
    private double approachAngle = 90.0;
    private int targetXPixels = HUSKYLENS_WIDTH / 2;

    // ========== SHOOTER STATE ==========
    private double targetShooterRPM = 0.0;
    private double shooterPower = 0.0;
    private double shooterRPM = 0.0;
    private int lastShooterEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;

    // Hood adjustment positions
    private double leftHoodPosition = 0.15;
    private double rightHoodPosition = 0.15;

    // ========== CONTROLLER 2 (OPERATOR) STATE ==========
    // Trapdoor toggles (Face Buttons)
    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean bothTrapdoorsOpen = false;
    private boolean lastGamepad2XState = false; // Left Trapdoor
    private boolean lastGamepad2BState = false; // Right Trapdoor
    private boolean lastGamepad2YState = false; // Both Trapdoors

    // Transfer toggle (A Button)
    private boolean transfersOpen = false;
    private boolean lastGamepad2AState = false;

    // Kicker Arm states (D-Pad toggles)
    private boolean leftKickerArmOpen = false;
    private boolean rightKickerArmOpen = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    // Color selection state (Bumpers)
    private boolean colorPurpleSelected = false;
    private boolean colorGreenSelected = false;
    private boolean lastGamepad2LeftBumperState = false;
    private boolean lastGamepad2RightBumperState = false;

    // Shooter speed toggle (Left Trigger) - keeps shooter at minimum RPM for reduced windup
    private boolean shooterSpeedOn = false;
    private boolean lastGamepad2LeftTriggerState = false;

    // Minimum shooter RPM when idling (for faster windup)
    private static final double MIN_IDLE_SHOOTER_RPM = 1800;

    // Auto shoot (Right Trigger or Button?) - Image says "Auto shoot". Assuming RT based on position.
    // No toggle needed if it's a sequence trigger, but we need debouncing.
    private boolean lastGamepad2RightTriggerState = false;

    // Timer for shooting sequence
    private ElapsedTime shootSequenceTimer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime shotFiredTimer = new ElapsedTime();  // Timer for after shot is fired
    private boolean shootSequenceActive = false;
    private boolean autoTransferTriggered = false;  // For RPM-based auto transfer
    private boolean distanceCheckPassed = false; // Tracks if ball was detected at 3000ms
    private boolean shotFired = false;  // Tracks if the transfer was opened (shot fired)
    private boolean kickLeft = false; // Track if left side should be kicked
    private boolean kickRight = false; // Track if right side should be kicked
    private boolean singleBallMode = false; // True if proximity > 6.5 (only one ball)
    private boolean trapdoorsOpenedForSingleBall = false; // Track if trapdoors were opened in single ball mode
    private boolean manualBothTrapdoorsOverride = false; // Track if Y button (both trapdoors) was pressed during auto shoot

    // Debug timing
    private ElapsedTime debugTimer = new ElapsedTime();


    @Override
    public void init() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize other motors
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");

        // Initialize servos
        leftTrapdoor = hardwareMap.get(Servo.class, "leftTrapdoor");
        rightTrapdoor = hardwareMap.get(Servo.class, "rightTrapdoor");
        leftKickerArm = hardwareMap.get(Servo.class, "leftKickerArm");
        rightKickerArm = hardwareMap.get(Servo.class, "rightKickerArm");
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        // Initialize color sensors
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        // Get the distance sensor and motor from hardwareMap
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        // Configure HuskyLens for AprilTag recognition
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Problem communicating with HuskyLens");
        } else {
            telemetry.addData("HuskyLens", "Connected");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Initialize IMU for field-centric driving
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw(); // Reset heading to 0 at start

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize velocity tracking for shooter
        velocityTimer.reset();
        lastShooterEncoderPosition = 0;

        // Initialize hood adjustments
        leftHoodAdjustment.setPosition(leftHoodPosition);
        rightHoodAdjustment.setPosition(rightHoodPosition);

        // Initialize trapdoors to closed position
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    @Override
    public void loop() {

        // ========== CONTROLLER 1: THE DRIVER (Mobility & Acquisition) ==========

        // Get joystick inputs (robot-centric driving)
        double y = gamepad1.left_stick_y;  // Forward/backward (reversed - pushing stick forward goes backward)
        double x = -gamepad1.left_stick_x;   // Left/right strafe
        double yaw = gamepad1.right_stick_x; // Rotation

        // Use joystick values directly for robot-centric movement
        float axial = (float) y;
        float lateral = (float) x;

        // Apply a strafe correction factor (strafing is typically less efficient)
        lateral = lateral * 1.1f;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        float leftFrontPower = axial + lateral + (float) yaw;
        float rightFrontPower = axial - lateral - (float) yaw;
        float leftBackPower = axial - lateral + (float) yaw;
        float rightBackPower = axial + lateral - (float) yaw;

        // clip the right/left values so that the values never exceed +/- 1
        rightFrontPower = (float) Range.clip(rightFrontPower, -1.0, 1.0);
        leftFrontPower = (float) Range.clip(leftFrontPower, -1.0, 1.0);
        leftBackPower = (float) Range.clip(leftBackPower, -1.0, 1.0);
        rightBackPower = (float) Range.clip(rightBackPower, -1.0, 1.0);

        // Apply motor correction multipliers to compensate for motor imbalances
        rightFrontPower *= RF_MULTIPLIER;
        leftFrontPower *= LF_MULTIPLIER;
        leftBackPower *= LB_MULTIPLIER;
        rightBackPower *= RB_MULTIPLIER;

        // ========== HUSKYLENS APRILTAG DETECTION & AUTO-AIM ==========
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
                detectedDistance = directDistance;
            }

            // Calculate horizontal approach angle from tag position and distance
            double pixelOffsetFromCenter = detectedTagX - (HUSKYLENS_WIDTH / 2.0);
            double angleOffsetRadians = Math.toRadians((pixelOffsetFromCenter / (HUSKYLENS_WIDTH / 2.0)) * (HUSKYLENS_HFOV_DEGREES / 2.0));
            double lateralDistance = detectedDistance * Math.tan(angleOffsetRadians);
            double lateralAngleDegrees = Math.toDegrees(Math.atan2(lateralDistance, detectedDistance));
            approachAngle = 90.0 + lateralAngleDegrees;

            // Clamp angle to lookup table range
            double clampedAngle = Range.clip(approachAngle, 45.0, 135.0);

            // Calculate target X percent based on approach angle using interpolation
            double targetXPercent = interpolateAngleToTarget(clampedAngle);
            targetXPixels = (int)(HUSKYLENS_WIDTH * targetXPercent);

            // Auto-aim calculation
            double aimError = detectedTagX - targetXPixels;
            isAimed = Math.abs(aimError) <= AIM_TOLERANCE_PIXELS;

            if (autoAimEnabled && !isAimed) {
                autoAimRotation = aimError * AIM_KP;
                autoAimRotation = Range.clip(autoAimRotation, -0.3, 0.3);
            } else {
                autoAimRotation = 0.0;
            }

            // Apply lookup table values based on distance - calculate target RPM when tag detected
            double[] lookupValues = interpolateLookupTable(detectedDistance);
            leftHoodPosition = lookupValues[0];
            double lookupRPM = lookupValues[1];

            // Apply hood positions and set target RPM when auto-aim is enabled and looking at tag
            if (autoAimEnabled) {
                leftHoodAdjustment.setPosition(leftHoodPosition);
                double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                rightHoodAdjustment.setPosition(rightHoodPosition);
                // When auto-aim enabled and tag detected, use full RPM from lookup table
                targetShooterRPM = lookupRPM;
            } else {
                // Not auto-aiming, so RPM will be set to idle (if shooter is on) later
                targetShooterRPM = 0.0;
            }
        } else {
            // No tag detected - if auto-aim is enabled, reset target RPM
            if (autoAimEnabled) {
                targetShooterRPM = 0.0;
            }
        }

        // Right Trigger (Toggle): Auto-Aim / Position
        if (gamepad1.right_trigger > 0.5 && !lastGamepad1RightTriggerState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastGamepad1RightTriggerState = gamepad1.right_trigger > 0.5;

        // Apply auto-aim rotation to drive motors if enabled
        if (autoAimEnabled && tagDetected && !isAimed) {
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

        // --- Intake Controls (Controller 1) ---
        // Right Bumper: Intake Toggle (Press once to turn ON, press again to stop)
        /*
        if (gamepad1.right_bumper && !lastGamepad1RightBumperState) {
            intakeToggleOn = !intakeToggleOn;
        }
        lastGamepad1RightBumperState = gamepad1.right_bumper;

         */

        // Left Trigger (Hold): Intake IN - overrides toggle
        // Left Bumper: Intake OUT (Reverse/Unjam) - overrides toggle
        if (gamepad1.left_trigger > 0) {
            intake.setPower(-1.0); // Intake in
        } else if (gamepad1.left_bumper) {
            intake.setPower(1.0); // Intake out (reverse/unjam)
        } else if (intakeToggleOn) {
            intake.setPower(-1.0); // Toggle is on, keep intake running
        } else {
            intake.setPower(0.0);
        }

        // ========== CONTROLLER 2: THE OPERATOR (Scoring Logic) ==========

        // --- Face Buttons ---
        // X Button: Left trapdoor toggle
        if (gamepad2.x && !lastGamepad2XState) {
            leftTrapdoorOpen = !leftTrapdoorOpen;
            if (leftTrapdoorOpen) {
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.0);
            } else {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2XState = gamepad2.x;

        // B Button: Right trapdoor toggle
        if (gamepad2.b && !lastGamepad2BState) {
            rightTrapdoorOpen = !rightTrapdoorOpen;
            if (rightTrapdoorOpen) {
                rightTrapdoor.setPosition(0.2);
                leftTrapdoor.setPosition(0.2);
            } else {
                rightTrapdoor.setPosition(0.1);
                leftTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2BState = gamepad2.b;

        // Y Button: Open both trapdoors toggle
        if (gamepad2.y && !lastGamepad2YState) {
            bothTrapdoorsOpen = !bothTrapdoorsOpen;
            if (bothTrapdoorsOpen) {
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.2);
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
                // If during auto shoot sequence, set the manual override flag
                if (shootSequenceActive) {
                    manualBothTrapdoorsOverride = true;
                }
            } else {
                rightTrapdoor.setPosition(0.1);
                leftTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
            }
        }
        lastGamepad2YState = gamepad2.y;

        // A Button: Transfer toggle
        if (gamepad2.a && !lastGamepad2AState) {
            transfersOpen = !transfersOpen;
            if (transfersOpen) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
            } else {
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
            }
        }
        lastGamepad2AState = gamepad2.a;

        // --- D-Pad Controls (Kicker Arms) ---
        // D-Pad Left: Left kicker arm toggle
        if (gamepad2.dpad_left && !lastDpadLeftState) {
            leftKickerArmOpen = !leftKickerArmOpen;
            leftKickerArm.setPosition(leftKickerArmOpen ? 0.5 : 0.0);
        }
        lastDpadLeftState = gamepad2.dpad_left; // Note: using lastDpadLeftState from Tele class fields

        // D-Pad Right: Right kicker arm toggle
        if (gamepad2.dpad_right && !lastDpadRightState) {
            rightKickerArmOpen = !rightKickerArmOpen;
            rightKickerArm.setPosition(rightKickerArmOpen ? 0.075 : 0.5);
        }
        lastDpadRightState = gamepad2.dpad_right;

        // --- Bumpers: Ball Side Selection + Auto Shoot ---
        // Left Bumper: Select ball on RIGHT side
        if (gamepad2.left_bumper && !lastGamepad2LeftBumperState) {
            kickRight = true;
            kickLeft = false;
            colorPurpleSelected = true;  // Set a color so shoot sequence can start
            colorGreenSelected = false;
        }
        lastGamepad2LeftBumperState = gamepad2.left_bumper;

        // Right Bumper: Select ball on LEFT side
        if (gamepad2.right_bumper && !lastGamepad2RightBumperState) {
            kickLeft = true;
            kickRight = false;
            colorGreenSelected = true;  // Set a color so shoot sequence can start
            colorPurpleSelected = false;
        }
        lastGamepad2RightBumperState = gamepad2.right_bumper;

        // --- Triggers ---
        // Left Trigger: Shooter speed toggle (keeps shooter at minimum RPM for reduced windup)
        boolean leftTriggerPressed = gamepad2.left_trigger > 0.5;
        if (leftTriggerPressed && !lastGamepad2LeftTriggerState) {
            shooterSpeedOn = !shooterSpeedOn;
            if (!shooterSpeedOn) {
                // Reset PID state when shooter is turned off
                shooterIntegral = 0.0;
                shooterLastError = 0.0;
            }
        }
        lastGamepad2LeftTriggerState = leftTriggerPressed;

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
        // Determine effective target RPM:
        // - If shooter is off: 0 RPM
        // - If shooter is on but not auto-aiming at a tag: idle RPM
        // - If shooter is on AND auto-aiming at a tag: full RPM from lookup table
        double effectiveTargetRPM = 0.0;
        if (shooterSpeedOn) {
            if (autoAimEnabled && tagDetected && targetShooterRPM > 0) {
                effectiveTargetRPM = targetShooterRPM;
            } else {
                effectiveTargetRPM = MIN_IDLE_SHOOTER_RPM;
            }
        }

        if (effectiveTargetRPM > 0) {
            double error = effectiveTargetRPM - shooterRPM;

            // Integrate error (with anti-windup)
            shooterIntegral += error * deltaTime;
            shooterIntegral = Range.clip(shooterIntegral, -5000, 5000);

            // Calculate derivative
            double derivative = (error - shooterLastError) / deltaTime;
            shooterLastError = error;

            // Calculate feedforward (base power to reach target RPM)
            double feedforward = effectiveTargetRPM * SHOOTER_KF;

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

        // Right Trigger: Auto-shoot sequence
        // Starts if a color is selected. Uses tag data if available, otherwise uses fallback defaults.
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (rightTriggerPressed && !lastGamepad2RightTriggerState) {
            if (shootSequenceActive) {
                // If sequence is active, stop it
                stopShootSequence();
            } else if (colorPurpleSelected || colorGreenSelected) {
                // Color is selected - start shoot sequence
                if (!tagDetected || !autoAimEnabled) {
                    // Tag not detected or auto-aim disabled - use default values
                    // Set default hood position (0.3 left) and RPM (2000)
                    leftHoodPosition = 0.3;
                    leftHoodAdjustment.setPosition(leftHoodPosition);
                    double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                    rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                    rightHoodAdjustment.setPosition(rightHoodPosition);
                    targetShooterRPM = 2000.0;
                }
                // If tag detected and auto-aim enabled, use lookup table values (already set by AprilTag detection code)
                startShootSequence();
            }
        }
        lastGamepad2RightTriggerState = rightTriggerPressed;

        // No auto-start - shoot sequence only starts when right trigger is pressed with conditions met

        if (shootSequenceActive) {
            executeShootSequence();
        }

        // Close transfer after 2.5 seconds if auto transfer was triggered
        if (autoTransferTriggered && transferTimer.seconds() >= 2.5) {
            autoTransferTriggered = false;
            transfersOpen = false;
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
        }

        // ========== TELEMETRY ==========
        telemetry.addData("--- DRIVER (Gamepad 1) ---", "");
        telemetry.addData("Intake Toggle", intakeToggleOn ? "ON" : "OFF");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");

        telemetry.addData("--- APRILTAG ---", "");
        telemetry.addData("Tag Detected", tagDetected);
        if (tagDetected) {
            telemetry.addData("Distance (in)", "%.1f", detectedDistance);
            telemetry.addData("Approach Angle", "%.1f°", approachAngle);
            telemetry.addData("Tag X Position", "%d (target: %d)", detectedTagX, targetXPixels);
            telemetry.addData("Aimed", isAimed ? "YES" : "NO");
        }

        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Shooter Enabled", shooterSpeedOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", targetShooterRPM);
        telemetry.addData("Actual RPM", "%.0f", shooterRPM);
        telemetry.addData("Shooter Power", "%.1f%%", shooterPower * 100);
        telemetry.addData("RPM Range (5%%)", "%.0f - %.0f", rpmLowerBound, rpmUpperBound);
        telemetry.addData("RPM In Range", rpmInRange ? "YES - READY TO FIRE!" : "NO - WAITING...");
        telemetry.addData("Hood Position", "L:%.2f R:%.2f", leftHoodPosition, rightHoodPosition);

        // Shoot sequence status
        if (shootSequenceActive) {
            telemetry.addData("--- SHOOT SEQUENCE ---", "ACTIVE");
            telemetry.addData("Sequence Time", "%.1f sec", shootSequenceTimer.seconds());
            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING...");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
        }

        // Show what's needed to shoot
        boolean readyToShoot = (colorPurpleSelected || colorGreenSelected);
        String shootMode = (tagDetected && autoAimEnabled) ? "TAG MODE" : "FALLBACK MODE";
        telemetry.addData("Ready to Shoot", readyToShoot ? ("YES - " + shootMode + " - Press RT!") : "NO - Select Side");
        if (!readyToShoot) {
            String missing = "";
            if (!(colorPurpleSelected || colorGreenSelected)) missing += "Side(LB/RB) ";
            telemetry.addData("Missing", missing);
        }

        telemetry.addData("--- OPERATOR (Gamepad 2) ---", "");
        telemetry.addData("Left Trapdoor", leftTrapdoorOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Right Trapdoor", rightTrapdoorOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Both Trapdoors", bothTrapdoorsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Transfers", transfersOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Left Kicker Arm", leftKickerArmOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Right Kicker Arm", rightKickerArmOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Ball Side Selected", kickLeft ? "LEFT" : (kickRight ? "RIGHT" : "NONE"));

        // Distance Sensor Telemetry
        telemetry.addData("Distance (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));

        // Color sensor telemetry (HSV)
        float[] leftHSV = new float[3];
        float[] rightHSV = new float[3];
        Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
        Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);
        telemetry.addData("Left Color Sensor (H,S,V,D)", "(%.1f, %.2f, %.2f, %.3f)",
                leftHSV[0], leftHSV[1], leftHSV[2], ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM));
        telemetry.addData("Right Color Sensor (H,S,V,D)", "(%.1f, %.2f, %.2f, %.3f)",
                rightHSV[0], rightHSV[1], rightHSV[2], ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM));
        telemetry.addData("Left Detected Color", leftHSV[0] > 175 ? "PURPLE" : "GREEN");
        telemetry.addData("Right Detected Color", rightHSV[0] > 175 ? "PURPLE" : "GREEN");

        telemetry.update();
    }

    // ========== PLACEHOLDER FUNCTIONS (TODO: Implement) ==========

    // private void autoAimToTarget() {
    //     // TODO: Use camera/AprilTag to center on target and strafe to correct distance
    // }

    private void startShootSequence() {
        shootSequenceActive = true;
        shootSequenceTimer.reset();
        shotFired = false;

        // If both trapdoors were manually opened before starting, set the override flag
        manualBothTrapdoorsOverride = bothTrapdoorsOpen;

        // Enable shooter so PID control will run the motor to target RPM
        shooterSpeedOn = true;

        /* COMMENTED OUT - Color sorting code
        // Color detection logic
        float[] leftHSV = new float[3];
        float[] rightHSV = new float[3];
        Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
        Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);

        // Check proximity sensors to determine if single ball mode
        double leftProximity = ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM);
        double rightProximity = ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM);
        singleBallMode = (leftProximity > 6.5 || rightProximity > 6.5);
        trapdoorsOpenedForSingleBall = false;

        // Determine colors (Purple if hue > 175, otherwise Green)
        boolean leftIsPurple = leftHSV[0] > 175;
        boolean leftIsGreen = !leftIsPurple;
        boolean rightIsPurple = rightHSV[0] > 175;
        boolean rightIsGreen = !rightIsPurple;

        boolean openLeft = false;
        boolean openRight = false;

        // Check against selected color logic
        if (colorPurpleSelected) {
            if (leftIsPurple) openLeft = true;
            if (rightIsPurple) openRight = true;
        } else if (colorGreenSelected) {
            if (leftIsGreen) openLeft = true;
            if (rightIsGreen) openRight = true;
        }

        // Track which kicker arms should activate (only used if not single ball mode)
        // If both balls are the same color, use proximity to determine which side:
        // - If left proximity < 4 and right > 4, use left
        // - If right proximity < 4 and left > 4, use right
        // - Otherwise (both < 4 or both > 4), prioritize left
        if (openLeft && openRight) {
            boolean leftClose = leftProximity < 5;
            boolean rightClose = rightProximity < 5;
            if (leftClose && !rightClose) {
                kickLeft = true;
                kickRight = false;
            } else if (rightClose && !leftClose) {
                kickLeft = false;
                kickRight = true;
            } else {
                // Both close or both far - prioritize left
                kickLeft = true;
                kickRight = false;
            }
        } else {
            kickLeft = openLeft;
            kickRight = openRight;
        }
        END COMMENTED OUT */

        // Ball side is now selected directly via bumpers (kickLeft/kickRight already set)
        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;

        if (singleBallMode) {
            // Single ball mode: stop intake, open only left trapdoor
            intake.setPower(0.0);
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);  // Keep right closed (matches X button left trapdoor logic)
        } else if (manualBothTrapdoorsOverride) {
            // Both trapdoors were manually opened - keep them both open
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.2);
        } else {
            // Normal mode: Only open ONE trapdoor based on ball side selection (never both automatically)
            if (kickLeft) {
                // Left Trapdoor Open logic (matches X button)
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.0);
            } else if (kickRight) {
                // Right Trapdoor Open logic (matches B button)
                rightTrapdoor.setPosition(0.2);
                leftTrapdoor.setPosition(0.2);
            } else {
                // No selection, ensure closed
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.0);
            }
        }

        // Immediate servo actions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);
    }

    private void executeShootSequence() {
        // Use PID-controlled shooter power (calculated in main loop)
        // The main loop's PID already handles targetShooterRPM from the LUT
        // We just need to make sure shooterSpeedOn is enabled during the sequence

        // Check if RPM is within 5% tolerance of target
        double rpmLowerBound = targetShooterRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = targetShooterRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = targetShooterRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        if (singleBallMode) {
            // Single ball mode: intake stopped at start, resume after 300ms
            if (shootSequenceTimer.milliseconds() < 300) {
                intake.setPower(0.0);
            } else {
                // After 300ms, resume intake
                if (shootSequenceTimer.milliseconds() < 900 + 300) {
                    intake.setPower(-1.0);
                } else if (shootSequenceTimer.milliseconds() < 1400 + 300) {
                    intake.setPower(0.0);
                } else {
                    intake.setPower(-1.0);
                }
            }
            // Do NOT activate kicker arms in single ball mode
        } else {
            // Normal mode: Intake runs for first 900ms, pauses 900-1400ms, then resumes until end
            if (shootSequenceTimer.milliseconds() < 900) {
                intake.setPower(-1.0);
            } else if (shootSequenceTimer.milliseconds() < 1400) {
                intake.setPower(0.0);
            } else {
                intake.setPower(-1.0);
            }

            // After 500ms delay, set the kicker arm position (only for the side with the ball)
            if (shootSequenceTimer.milliseconds() >= 500) {
                if (kickLeft) {
                    leftKickerArm.setPosition(0.5);
                }
                if (kickRight) {
                    rightKickerArm.setPosition(0.075);
                }
            }
        }

// CONTINUOUS DISTANCE CHECK (Starts after 200ms to allow trapdoor movement)
        if (shootSequenceTimer.milliseconds() >= 200 && !distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance < 20) { // Ball Detected
                distanceCheckPassed = true;
                if (!manualBothTrapdoorsOverride) {
                    leftTrapdoor.setPosition(0.1);
                    rightTrapdoor.setPosition(0.1);
                }
            }
        }

        // TIMEOUT SAFETY (3 seconds)
        if (shootSequenceTimer.milliseconds() >= 3000 && !distanceCheckPassed) {
            restartShootSequence();
            return;
        }

        // IMMEDIATE FIRE TRIGGER
        // If distance passed AND rpm is ready -> FIRE! (No 2000ms wait)
        if (distanceCheckPassed && rpmReady && !shotFired) {
            rightTransfer.setPosition(0.0);
            leftTransfer.setPosition(0.5);
            shotFired = true;
            shotFiredTimer.reset();
        }

        // End sequence 1 second after firing
        if (shotFired && shotFiredTimer.milliseconds() >= 1000) {
            rightTrapdoor.setPosition(0.1);
            leftTrapdoor.setPosition(0.1);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            rightTransfer.setPosition(0.5);
            leftTransfer.setPosition(0.0);

            // shooter.setPower(0.0);
            // shooterSpeedOn = false;

            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            leftKickerArmOpen = false;
            rightKickerArmOpen = false;
            transfersOpen = false;

            shootSequenceActive = false;
            distanceCheckPassed = false;
            shotFired = false;
            kickLeft = false;
            kickRight = false;
            singleBallMode = false;
            trapdoorsOpenedForSingleBall = false;
            manualBothTrapdoorsOverride = false;
        }
    }

    private void stopShootSequence() {
        // Stop the sequence and reset all mechanisms
        shootSequenceActive = false;
        distanceCheckPassed = false;
        shotFired = false;
        kickLeft = false;
        kickRight = false;
        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;
        manualBothTrapdoorsOverride = false;

        // Reset servos to closed positions
        leftTrapdoor.setPosition(0.2);
        rightTrapdoor.setPosition(0.0);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
        rightTransfer.setPosition(0.5);
        leftTransfer.setPosition(0.0);

        // Stop motors
        // shooter.setPower(0.0);
        // shooterSpeedOn = false;  // Turn off shooter speed toggle
        intake.setPower(0.0);

        // Reset state variables to match physical state
        leftTrapdoorOpen = false;
        rightTrapdoorOpen = false;
        bothTrapdoorsOpen = false;
        leftKickerArmOpen = false;
        rightKickerArmOpen = false;
        transfersOpen = false;
    }

    private void restartShootSequence() {
        // Reset the timer to restart the cycle from the beginning
        shootSequenceTimer.reset();
        distanceCheckPassed = false;
        shotFired = false;

        // Do NOT re-read color sensors - keep using the same kickLeft/kickRight values
        // that were originally determined until the ball passes the distance sensor check

        // Check if manual override was used - if so, open both trapdoors
        if (manualBothTrapdoorsOverride) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.2);
        } else if (singleBallMode) {
            // Single ball mode: open only left trapdoor
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else if (kickLeft) {
            // Left side only (prioritized when both match)
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else if (kickRight) {
            // Right side
            rightTrapdoor.setPosition(0.2);
            leftTrapdoor.setPosition(0.2);
        } else {
            // No match (backup - use left side)
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        }

        // Reset transfer positions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /**
     * Interpolates between lookup table entries to get hood position and shooter RPM
     * based on detected distance. Returns {hoodPosition, targetRPM}.
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

                // Interpolate shooter RPM
                double rpm1 = SHOOTER_LOOKUP_TABLE[i][2];
                double rpm2 = SHOOTER_LOOKUP_TABLE[i + 1][2];
                double interpolatedRPM = rpm1 + factor * (rpm2 - rpm1);

                return new double[] {interpolatedHood, interpolatedRPM};
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


