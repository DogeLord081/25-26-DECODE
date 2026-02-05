package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.util.Size;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.ColorDetectionTest.ColorRegionProcessor;


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
    protected IMU imu;
    protected DistanceSensor distanceSensor;
    protected HuskyLens huskyLens;

    // Webcam color detection
    protected ColorRegionProcessor colorProcessor;
    protected VisionPortal visionPortal;

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
            {45, 0.60},   // 45 degrees to the left → tag at 90% from left
            {90, 0.60},   // Head on (90 degrees) → tag at 60% from left
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
    private static final double SHOOTER_KP = 0.0015;
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

    // Transfer toggle (A Button) - UP = closed position, DOWN = open position
    private boolean transfersUp = false;  // Start with transfers down (open)
    private boolean lastGamepad2AState = false;

    // Kicker Arm states (D-Pad toggles)
    private boolean leftKickerArmOpen = false;
    private boolean rightKickerArmOpen = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    // Color selection state (Bumpers) - Left = Purple, Right = Green
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
    private boolean intakeReversed = false; // Tracks if intake was reversed after distance check
    private boolean restartIntakePulseActive = false; // Tracks if restart intake pulse is in progress
    private ElapsedTime restartIntakePulseTimer = new ElapsedTime(); // Timer for restart intake pulse
    private ElapsedTime intakePulseTimer = new ElapsedTime(); // Timer for intake pulsing during shoot sequence
    private boolean kickLeft = false; // Track if left side should be kicked
    private boolean kickRight = false; // Track if right side should be kicked
    private boolean singleBallMode = false; // True if proximity > 6.5 (only one ball)
    private boolean trapdoorsOpenedForSingleBall = false; // Track if trapdoors were opened in single ball mode

    // AprilTag pattern scanning state (for 3-ball auto shoot)
    private int detectedAprilTagId = -1;
    private char[] ballOrder = new char[3];  // Ball order based on AprilTag (P = Purple/Left, G = Green/Right)
    private boolean lastGamepad1AState = false;

    // 3-ball auto shoot sequence state
    private boolean threeBallSequenceActive = false;
    private int currentBallIndex = 0;  // 0, 1, 2 for the three balls
    private boolean shootSideDecided = false;
    private ElapsedTime shootTimer = new ElapsedTime();

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
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize velocity tracking for shooter
        velocityTimer.reset();
        lastShooterEncoderPosition = 0;

        // Initialize hood adjustments
        leftHoodAdjustment.setPosition(leftHoodPosition);
        rightHoodAdjustment.setPosition(rightHoodPosition);

        // Initialize trapdoors to closed position (0.2 = closed, 0.0/0.2 = open)
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);

        // Initialize transfers to down position (DOWN = open = shooting position)
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);

        // Initialize webcam color detection
        colorProcessor = new ColorRegionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colorProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
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
                // Only set if not in active shoot sequence (let shoot sequence maintain its own target)
                if (!shootSequenceActive) {
                    targetShooterRPM = lookupRPM;
                }
            } else {
                // Not auto-aiming, so RPM will be set to idle (if shooter is on) later
                // Only set if not in active shoot sequence
                if (!shootSequenceActive) {
                    targetShooterRPM = 0.0;
                }
            }
        } else {
            // No tag detected - if auto-aim is enabled and not in shoot sequence, reset target RPM
            if (autoAimEnabled && !shootSequenceActive) {
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
        } else if (!shootSequenceActive && !threeBallSequenceActive) {
            // Only stop intake if no shoot sequence is active
            intake.setPower(0.0);
        }

        // A Button: Scan for AprilTag pattern (like in AutoShoot.java)
        if (gamepad1.a && !lastGamepad1AState) {
            HuskyLens.Block[] aprilTagBlocks = huskyLens.blocks();
            if (aprilTagBlocks.length > 0) {
                detectedAprilTagId = aprilTagBlocks[0].id;

                // Set ball order based on AprilTag ID
                // ID 1: PPG, ID 2: PGP, ID 3: GPP
                switch (detectedAprilTagId) {
                    case 1:
                        ballOrder = new char[]{'P', 'P', 'G'};
                        break;
                    case 2:
                        ballOrder = new char[]{'P', 'G', 'P'};
                        break;
                    case 3:
                        ballOrder = new char[]{'G', 'P', 'P'};
                        break;
                    default:
                        ballOrder = new char[]{'P', 'P', 'G'};  // Default fallback
                        break;
                }
            }
        }
        lastGamepad1AState = gamepad1.a;

        // ========== CONTROLLER 2: THE OPERATOR (Scoring Logic) ==========

        // --- Face Buttons ---
        // X Button: Left trapdoor toggle
        if (gamepad2.x && !lastGamepad2XState) {
            leftTrapdoorOpen = !leftTrapdoorOpen;
            if (leftTrapdoorOpen) {
                leftTrapdoor.setPosition(0.2);  // Open
                rightTrapdoor.setPosition(0.1);  // Keep right closed
            } else {
                leftTrapdoor.setPosition(0.1);  // Closed
                rightTrapdoor.setPosition(0.1);  // Keep right closed
            }
        }
        lastGamepad2XState = gamepad2.x;

        // B Button: Right trapdoor toggle
        if (gamepad2.b && !lastGamepad2BState) {
            rightTrapdoorOpen = !rightTrapdoorOpen;
            if (rightTrapdoorOpen) {
                rightTrapdoor.setPosition(0.0);  // Open
                leftTrapdoor.setPosition(0.1);  // Keep left closed
            } else {
                rightTrapdoor.setPosition(0.1);  // Closed
                leftTrapdoor.setPosition(0.1);  // Keep left closed
            }
        }
        lastGamepad2BState = gamepad2.b;

        // Y Button: Open both trapdoors toggle
        if (gamepad2.y && !lastGamepad2YState) {
            bothTrapdoorsOpen = !bothTrapdoorsOpen;
            if (bothTrapdoorsOpen) {
                leftTrapdoor.setPosition(0.2);  // Open
                rightTrapdoor.setPosition(0.0);  // Open
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
            } else {
                leftTrapdoor.setPosition(0.1);  // Closed
                rightTrapdoor.setPosition(0.1);  // Closed
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
            }
        }
        lastGamepad2YState = gamepad2.y;

        // A Button: Transfer toggle (UP = closed, DOWN = open)
        if (gamepad2.a && !lastGamepad2AState) {
            transfersUp = !transfersUp;
            if (transfersUp) {
                // UP position (closed)
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
            } else {
                // DOWN position (open)
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
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

        // --- Bumpers: Color Selection ---
        // Left Bumper: Select PURPLE color and start auto shoot sequence
        if (gamepad2.left_bumper && !lastGamepad2LeftBumperState) {
            colorPurpleSelected = true;
            colorGreenSelected = false;
            // Start shoot sequence if not already active
            if (!shootSequenceActive) {
                if (!tagDetected || !autoAimEnabled) {
                    // Tag not detected or auto-aim disabled - use default values
                    leftHoodPosition = 0.3;
                    leftHoodAdjustment.setPosition(leftHoodPosition);
                    double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                    rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                    rightHoodAdjustment.setPosition(rightHoodPosition);
                    targetShooterRPM = 2000.0;
                }
                startShootSequence();
            }
        }
        lastGamepad2LeftBumperState = gamepad2.left_bumper;

        // Right Bumper: Select GREEN color and start auto shoot sequence
        if (gamepad2.right_bumper && !lastGamepad2RightBumperState) {
            colorGreenSelected = true;
            colorPurpleSelected = false;
            // Start shoot sequence if not already active
            if (!shootSequenceActive) {
                if (!tagDetected || !autoAimEnabled) {
                    // Tag not detected or auto-aim disabled - use default values
                    leftHoodPosition = 0.3;
                    leftHoodAdjustment.setPosition(leftHoodPosition);
                    double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                    rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                    rightHoodAdjustment.setPosition(rightHoodPosition);
                    targetShooterRPM = 2000.0;
                }
                startShootSequence();
            }
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
        // - If shoot sequence is active: use targetShooterRPM (set when sequence started)
        // - If shooter is on but no active sequence and auto-aiming at a tag: use lookup RPM
        // - If shooter is on but no active sequence and not auto-aiming: idle RPM
        double effectiveTargetRPM = 0.0;
        if (shooterSpeedOn) {
            if (shootSequenceActive && targetShooterRPM > 0) {
                // During shoot sequence, use the target RPM set when sequence started
                effectiveTargetRPM = targetShooterRPM;
            } else if (autoAimEnabled && tagDetected && targetShooterRPM > 0) {
                // Auto-aiming at tag, use lookup RPM
                effectiveTargetRPM = targetShooterRPM;
            } else {
                // Idle - use minimum RPM
                effectiveTargetRPM = MIN_IDLE_SHOOTER_RPM;
                // Update targetShooterRPM to reflect what we're actually using (for telemetry)
                if (!shootSequenceActive) {
                    targetShooterRPM = MIN_IDLE_SHOOTER_RPM;
                }
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

        // Right Trigger: Start 3-ball auto shoot sequence (like in AutoShoot.java)
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (rightTriggerPressed && !lastGamepad2RightTriggerState) {
            if (!threeBallSequenceActive && !shootSequenceActive && detectedAprilTagId != -1) {
                // Start 3-ball auto shoot sequence if AprilTag was scanned
                startThreeBallSequence();
            } else if (threeBallSequenceActive || shootSequenceActive) {
                // If a sequence is active, stop it
                stopThreeBallSequence();
                stopShootSequence();
            }
        }
        lastGamepad2RightTriggerState = rightTriggerPressed;

        // Shoot sequence starts automatically when bumpers are pressed to select color

        if (shootSequenceActive && !threeBallSequenceActive) {
            executeShootSequence();
        }

        // Execute 3-ball auto shoot sequence
        if (threeBallSequenceActive) {
            executeThreeBallSequence();
        }

        // Close transfer after 2.5 seconds if auto transfer was triggered
        if (autoTransferTriggered && transferTimer.seconds() >= 2.5) {
            autoTransferTriggered = false;
            transfersUp = false;  // Transfer goes down (open position)
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
        }

        // Get webcam color detection for telemetry
        ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();

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

        // AprilTag Pattern Scan Status (for 3-ball auto shoot)
        telemetry.addData("--- PATTERN SCAN ---", "");
        if (detectedAprilTagId != -1) {
            telemetry.addData("Scanned Tag ID", detectedAprilTagId);
            telemetry.addData("Ball Order", "" + ballOrder[0] + ballOrder[1] + ballOrder[2]);
            telemetry.addData("3-Ball Ready", "Press RT to shoot!");
        } else {
            telemetry.addData("Scanned Tag ID", "None - Press A to scan");
        }

        telemetry.addData("--- WEBCAM COLOR DETECTION ---", "");
        telemetry.addData("Left Camera", colorResult.leftColor);
        telemetry.addData("Right Camera", colorResult.rightColor);
        telemetry.addData("Selected Color", colorGreenSelected ? "GREEN" : (colorPurpleSelected ? "PURPLE" : "NONE"));

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

        // 3-ball sequence status
        if (threeBallSequenceActive) {
            telemetry.addData("--- 3-BALL SEQUENCE ---", "ACTIVE");
            telemetry.addData("Current Ball", (currentBallIndex + 1) + " of 3");
            if (currentBallIndex < 3) {
                telemetry.addData("Target Color", ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN");
            }
            telemetry.addData("Shoot Timer", "%.2f sec", shootTimer.seconds());
            telemetry.addData("Side Decided", shootSideDecided ? "YES" : "SCANNING...");
            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING...");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
            telemetry.addData("Kick Left", kickLeft);
            telemetry.addData("Kick Right", kickRight);
        }

        // Show what's needed to shoot
        boolean readyToShoot = (colorPurpleSelected || colorGreenSelected);
        String shootMode = (tagDetected && autoAimEnabled) ? "TAG MODE" : "FALLBACK MODE";
        telemetry.addData("Ready to Shoot", readyToShoot ? ("YES - " + shootMode + " - Press RT!") : "NO - Select Color");
        if (!readyToShoot) {
            String missing = "";
            if (!(colorPurpleSelected || colorGreenSelected)) missing += "Color(LB=Purple/RB=Green) ";
            telemetry.addData("Missing", missing);
        }

        telemetry.addData("--- OPERATOR (Gamepad 2) ---", "");
        telemetry.addData("Left Trapdoor", leftTrapdoorOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Right Trapdoor", rightTrapdoorOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Both Trapdoors", bothTrapdoorsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Transfers", transfersUp ? "UP" : "DOWN");
        telemetry.addData("Left Kicker Arm", leftKickerArmOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Right Kicker Arm", rightKickerArmOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Ball Side Selected", kickLeft ? "LEFT" : (kickRight ? "RIGHT" : "NONE"));

        // Distance Sensor Telemetry
        telemetry.addData("Distance (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));


        telemetry.update();
    }

    // ========== PLACEHOLDER FUNCTIONS (TODO: Implement) ==========

    // private void autoAimToTarget() {
    //     // TODO: Use camera/AprilTag to center on target and strafe to correct distance
    // }

    private void startShootSequence() {
        shootSequenceActive = true;
        shootSequenceTimer.reset();
        intakePulseTimer.reset();
        shotFired = false;
        intakeReversed = false;

        // Enable shooter so PID control will run the motor to target RPM
        shooterSpeedOn = true;

        // Use webcam color detection to determine which side to kick
        ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
        String leftColor = colorResult.leftColor;
        String rightColor = colorResult.rightColor;

        // Determine which side has the selected color
        kickLeft = false;
        kickRight = false;
        boolean leftMatchesSelected = false;
        boolean rightMatchesSelected = false;

        if (colorPurpleSelected) {
            leftMatchesSelected = leftColor.equals("PURPLE");
            rightMatchesSelected = rightColor.equals("PURPLE");
        } else if (colorGreenSelected) {
            leftMatchesSelected = leftColor.equals("GREEN");
            rightMatchesSelected = rightColor.equals("GREEN");
        }

        // Logic: If selected color is on left side, open left trapdoor and kick from left
        // If selected color is on right side, open right trapdoor and kick from right
        // If no color detected on either side (both NEITHER), open both trapdoors
        if (leftMatchesSelected && !rightMatchesSelected) {
            kickLeft = true;
            kickRight = false;
        } else if (rightMatchesSelected && !leftMatchesSelected) {
            kickLeft = false;
            kickRight = true;
        } else if (leftMatchesSelected && rightMatchesSelected) {
            // Both sides have the selected color - prioritize left
            kickLeft = true;
            kickRight = false;
        } else {
            // No color detected on either side (both NEITHER) - open both trapdoors
            kickLeft = true;
            kickRight = true;
        }

        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;

        if (kickLeft && kickRight) {
            // Both trapdoors open
            leftTrapdoor.setPosition(0.2);   // Open
            rightTrapdoor.setPosition(0.0);  // Open
        } else if (kickRight) {
            // Left Trapdoor Open
            leftTrapdoor.setPosition(0.2);   // Open
            rightTrapdoor.setPosition(0.2);  // Closed
        } else if (kickLeft) {
            // Right Trapdoor Open
            leftTrapdoor.setPosition(0.0);   // Closed
            rightTrapdoor.setPosition(0.0);  // Open
        } else {
            // No selection, ensure closed
            leftTrapdoor.setPosition(0.1);   // Closed
            rightTrapdoor.setPosition(0.1);  // Closed
        }

        // Set transfer to down (open) position for shooting
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);
        transfersUp = false;
    }

    private void executeShootSequence() {
        // Use PID-controlled shooter power (calculated in main loop)
        // The main loop's PID already handles targetShooterRPM from the LUT
        // We just need to make sure shooterSpeedOn is enabled during the sequence

        // Check if RPM is within 5% tolerance of target
        double rpmLowerBound = targetShooterRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = targetShooterRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = targetShooterRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        // Handle restart intake pulse completion (after 100ms, set intake back to -1.0 and transfers down)
        if (restartIntakePulseActive && restartIntakePulseTimer.milliseconds() >= 150) {
            intake.setPower(-1.0);
            restartIntakePulseActive = false;
            // Now set transfer positions (DOWN)
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            transfersUp = false;
        }

        // Pulse intake while waiting for distance check: -1 for 250ms, 0 for 250ms
        if (!restartIntakePulseActive && !distanceCheckPassed) {
            long pulsePhase = (long) intakePulseTimer.milliseconds() % 500;
            if (pulsePhase < 250) {
                intake.setPower(-1.0);  // Intake on for first 250ms
            } else {
                intake.setPower(0.0);   // Intake off for next 250ms
            }
        }

// CONTINUOUS DISTANCE CHECK (Starts after 200ms to allow trapdoor movement)
        if (shootSequenceTimer.milliseconds() >= 200 && !distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance < 20 && rpmReady) { // Ball Detected
                distanceCheckPassed = true;
                // Immediately move transfer UP (to bring ball to flywheel)
                leftTransfer.setPosition(0.0);   // UP position
                rightTransfer.setPosition(0.5);  // UP position
                transfersUp = true;

                // Spin intake forward to push ball up
                intake.setPower(-1.0);
                intakeReversed = false;
                transferTimer.reset();  // Start timer for intake reversal

                // Close both trapdoors immediately
                leftTrapdoor.setPosition(0.1);   // Closed
                rightTrapdoor.setPosition(0.1);  // Closed
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;
            }
        }

        // After 50ms, set intake back to -1.0
        if (distanceCheckPassed && !intakeReversed && transferTimer.milliseconds() >= 50) {
            intake.setPower(-1.0);
            intakeReversed = true;
        }

        // TIMEOUT SAFETY (3 seconds)
        if (shootSequenceTimer.milliseconds() >= 1500 && !distanceCheckPassed) {
            restartShootSequence();
            return;
        }

        // IMMEDIATE FIRE TRIGGER
        // Transfer is already UP when distance check passes. Shot fires automatically when RPM is ready.
        if (distanceCheckPassed && rpmReady && !shotFired) {
            // Ball is already at flywheel (transfer UP), RPM is ready - shot is being fired!
            shotFired = true;
            shotFiredTimer.reset();
        }

        // End sequence 1 second after firing
        if (shotFired && shotFiredTimer.milliseconds() >= 750) {
            leftTrapdoor.setPosition(0.1);   // Closed
            rightTrapdoor.setPosition(0.1);  // Closed
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            leftTransfer.setPosition(0.5);   // DOWN position
            rightTransfer.setPosition(0.0);  // DOWN position

            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            leftKickerArmOpen = false;
            rightKickerArmOpen = false;
            transfersUp = false;  // Transfer ends in down position

            shootSequenceActive = false;
            distanceCheckPassed = false;
            shotFired = false;
            intakeReversed = false;
            restartIntakePulseActive = false;
            kickLeft = false;
            kickRight = false;
            singleBallMode = false;
            trapdoorsOpenedForSingleBall = false;
        }
    }

    private void stopShootSequence() {
        // Stop the sequence and reset all mechanisms
        shootSequenceActive = false;
        distanceCheckPassed = false;
        shotFired = false;
        intakeReversed = false;
        restartIntakePulseActive = false;
        kickLeft = false;
        kickRight = false;
        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;

        // Reset servos to closed positions
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        rightTransfer.setPosition(0.0);  // DOWN position
        leftTransfer.setPosition(0.5);   // DOWN position

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
        transfersUp = false;  // Transfer ends in down position
    }

    private void restartShootSequence() {
        // Reset the timer to restart the cycle from the beginning
        shootSequenceTimer.reset();
        distanceCheckPassed = false;
        shotFired = false;
        intakeReversed = false;

        // Do NOT re-read color sensors - keep using the same kickLeft/kickRight values
        // that were originally determined until the ball passes the distance sensor check

        // Check if both sides should be kicked (both trapdoors open)
        if (kickLeft && kickRight) {
            leftTrapdoor.setPosition(0.2);  // Open
            rightTrapdoor.setPosition(0.0);  // Open
        } else if (singleBallMode) {
            // Single ball mode: open only left trapdoor
            leftTrapdoor.setPosition(0.2);  // Open
            rightTrapdoor.setPosition(0.1);  // Closed
        } else if (kickLeft) {
            // Left side only (prioritized when both match)
            leftTrapdoor.setPosition(0.2);  // Open
            rightTrapdoor.setPosition(0.1);  // Closed
        } else if (kickRight) {
            // Right side
            rightTrapdoor.setPosition(0.1);  // Closed
            leftTrapdoor.setPosition(0.0);  // Open
        } else {
            // No match (backup - use left side)
            leftTrapdoor.setPosition(0.2);  // Open
            rightTrapdoor.setPosition(0.1);  // Closed
        }

        // Start intake pulse - spin forward to help ball drop
        intake.setPower(-1.0);
        restartIntakePulseActive = true;
        restartIntakePulseTimer.reset();

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /**
     * Starts the 3-ball auto shoot sequence based on the scanned AprilTag pattern.
     */
    private void startThreeBallSequence() {
        threeBallSequenceActive = true;
        currentBallIndex = 0;
        shootTimer.reset();
        shotFired = false;
        distanceCheckPassed = false;
        shootSideDecided = false;
        intakeReversed = false;
        restartIntakePulseActive = false;
        kickLeft = false;
        kickRight = false;

        // Enable shooter so PID control will run the motor to target RPM
        shooterSpeedOn = true;

        // Set default hood position and RPM if no tag is currently detected
        if (!tagDetected || !autoAimEnabled) {
            leftHoodPosition = 0.3;
            leftHoodAdjustment.setPosition(leftHoodPosition);
            double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
            rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
            rightHoodAdjustment.setPosition(rightHoodPosition);
            targetShooterRPM = 2000.0;
        }

        // Set transfer to down (open) position for shooting
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);
        transfersUp = false;

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /**
     * Executes the 3-ball auto shoot sequence using the ball order from AprilTag scan.
     * Similar logic to AutoShoot.java's executeShootSequence.
     */
    private void executeThreeBallSequence() {
        if (currentBallIndex >= 3) {
            // All 3 balls shot, end the sequence
            stopThreeBallSequence();
            return;
        }

        double rpmLowerBound = targetShooterRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = targetShooterRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = targetShooterRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        // Determine which side to shoot from using webcam color detection
        // Wait 300ms for ball to settle/intake to move it before scanning
        if (!shootSideDecided && shootTimer.milliseconds() > 300) {
            char targetColor = ballOrder[currentBallIndex];
            boolean isThirdBall = (currentBallIndex == 2);

            if (isThirdBall) {
                // Third ball - open both trapdoors no matter what
                kickLeft = true;
                kickRight = true;
            } else {
                // Use webcam color detection to find the ball
                ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
                String leftColor = colorResult.leftColor;
                String rightColor = colorResult.rightColor;

                boolean leftMatchesTarget = false;
                boolean rightMatchesTarget = false;

                if (targetColor == 'P') {
                    leftMatchesTarget = leftColor.equals("PURPLE");
                    rightMatchesTarget = rightColor.equals("PURPLE");
                } else { // targetColor == 'G'
                    leftMatchesTarget = leftColor.equals("GREEN");
                    rightMatchesTarget = rightColor.equals("GREEN");
                }

                // Determine which side to kick based on color detection
                if (leftMatchesTarget && !rightMatchesTarget) {
                    kickLeft = true;
                    kickRight = false;
                } else if (rightMatchesTarget && !leftMatchesTarget) {
                    kickLeft = false;
                    kickRight = true;
                } else if (leftMatchesTarget && rightMatchesTarget) {
                    // Both sides have target color - prioritize left
                    kickLeft = true;
                    kickRight = false;
                } else {
                    // No color detected - open both trapdoors
                    kickLeft = true;
                    kickRight = true;
                }
            }

            shootSideDecided = true;
            intakePulseTimer.reset();

            // Open appropriate trapdoor
            if (kickLeft && kickRight) {
                leftTrapdoor.setPosition(0.2);   // Open
                rightTrapdoor.setPosition(0.0);  // Open
                bothTrapdoorsOpen = true;
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
            } else if (kickLeft) {
                // Ball is on LEFT side, open RIGHT trapdoor
                leftTrapdoor.setPosition(0.0);   // Closed
                rightTrapdoor.setPosition(0.0);  // Open
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = true;
            } else {
                // Ball is on RIGHT side, open LEFT trapdoor
                leftTrapdoor.setPosition(0.2);   // Open
                rightTrapdoor.setPosition(0.2);  // Closed
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = false;
            }
        }

        // Handle restart intake pulse completion
        if (restartIntakePulseActive && restartIntakePulseTimer.milliseconds() >= 150) {
            intake.setPower(-1.0);
            restartIntakePulseActive = false;
            // Set transfer DOWN
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            transfersUp = false;
        }

        // Pulse intake while waiting for distance check: -1 for 250ms, 0 for 250ms
        if (!restartIntakePulseActive && !distanceCheckPassed) {
            long pulsePhase = (long) intakePulseTimer.milliseconds() % 500;
            if (pulsePhase < 250) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
        }

        // Continuous distance check (starts after 200ms for trapdoor movement)
        if (shootTimer.milliseconds() >= 200 && !distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance < 20 && rpmReady) {
                distanceCheckPassed = true;

                leftTransfer.setPosition(0.0);   // UP
                rightTransfer.setPosition(0.5);  // UP
                transfersUp = true;
                intake.setPower(-1.0);
                intakeReversed = false;
                transferTimer.reset();

                // Close both trapdoors
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;

                // Shot is being fired
                shotFired = true;
                shotFiredTimer.reset();
            }
        }

        // After 50ms, set intake to -1.0
        if (distanceCheckPassed && !intakeReversed && transferTimer.milliseconds() >= 50) {
            intake.setPower(-1.0);
            intakeReversed = true;
        }

        // Timeout safety (1.5 seconds)
        if (shootTimer.milliseconds() >= 1500 && !distanceCheckPassed) {
            restartBallInSequence();
            return;
        }

        // End sequence 750ms after firing - transfers go back down and next ball
        if (shotFired && shotFiredTimer.milliseconds() >= 750) {
            // Close trapdoors and reset kicker arms
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);

            // Transfer goes back down
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);

            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            transfersUp = false;

            // Move to next ball
            currentBallIndex++;
            shotFired = false;
            distanceCheckPassed = false;
            intakeReversed = false;
            shootSideDecided = false;
            restartIntakePulseActive = false;
            kickLeft = false;
            kickRight = false;
            shootTimer.reset();
        }
    }

    /**
     * Restarts the current ball in the 3-ball sequence when distance sensor doesn't detect ball.
     */
    private void restartBallInSequence() {
        shootTimer.reset();
        distanceCheckPassed = false;
        shotFired = false;
        intakeReversed = false;

        // Reopen trapdoors based on current kick settings
        if (kickLeft && kickRight) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.0);
        } else if (kickLeft) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else if (kickRight) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        } else {
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
        }

        // Start intake pulse
        intake.setPower(-1.0);
        restartIntakePulseActive = true;
        restartIntakePulseTimer.reset();

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /**
     * Stops the 3-ball auto shoot sequence.
     */
    private void stopThreeBallSequence() {
        threeBallSequenceActive = false;
        currentBallIndex = 0;
        shootSideDecided = false;
        distanceCheckPassed = false;
        shotFired = false;
        intakeReversed = false;
        restartIntakePulseActive = false;
        kickLeft = false;
        kickRight = false;

        // Reset servos to closed positions
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        rightTransfer.setPosition(0.0);  // DOWN position
        leftTransfer.setPosition(0.5);   // DOWN position

        // Stop intake
        intake.setPower(0.0);

        // Reset state variables to match physical state
        leftTrapdoorOpen = false;
        rightTrapdoorOpen = false;
        bothTrapdoorsOpen = false;
        leftKickerArmOpen = false;
        rightKickerArmOpen = false;
        transfersUp = false;
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


