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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "TelePPG")
public class TelePPG extends OpMode {
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

    // Motor correction multipliers
    private static final double LF_MULTIPLIER = 0.3525 / 0.41;
    private static final double RF_MULTIPLIER = 0.35 / 0.41;
    private static final double LB_MULTIPLIER = 1.0;
    private static final double RB_MULTIPLIER = 0.3425 / 0.41;

    // ========== HUSKYLENS & AUTO-AIM CONSTANTS ==========
    private static final double TAG_WIDTH_INCHES = 6.5;
    private static final double FOCAL_LENGTH = 300.461;
    private static final double HEIGHT_DIFF_INCHES = 18.0;
    private static final int HUSKYLENS_WIDTH = 320;
    private static final double HUSKYLENS_HFOV_DEGREES = 60.0;

    private static final double[][] ANGLE_TO_TARGET_LOOKUP = {
            {45, 0.60},
            {90, 0.60},
            {135, 0.60}
    };
    private static final double AIM_TOLERANCE_PIXELS = 15.0;
    private static final double AIM_KP = 0.003;

    // ========== SHOOTER CONSTANTS ==========
    private static final double MAX_SHOOTER_RPM = 4900.0;
    private static final double DUMP_RPM = 1000.0;
    private static final double[][] SHOOTER_LOOKUP_TABLE = {
            {18, 0.05, 1800}, {24, 0.05, 1800}, {30, 0.05, 1800},
            {36, 0.30, 1800}, {42, 0.30, 1800}, {48, 0.20, 1850},
            {54, 0.30, 1850}, {60, 0.30, 1900}, {66, 0.30, 2000},
            {72, 0.30, 2050}, {118, 0.30, 2250}
    };

    private static final double SHOOTER_KP = 0.0015;
    private static final double SHOOTER_KI = 0.00001;
    private static final double SHOOTER_KD = 0.00001;
    private static final double SHOOTER_KF = 1.0 / MAX_SHOOTER_RPM;
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE_PERCENT = 0.05;

    // ========== STATE VARIABLES ==========
    private boolean intakeToggleOn = false;
    private boolean lastGamepad1RightBumperState = false;
    private boolean autoAimEnabled = false;
    private boolean lastGamepad1RightTriggerState = false;

    private double detectedDistance = 0.0;
    private int detectedTagX = -1;
    private boolean tagDetected = false;
    private double autoAimRotation = 0.0;
    private boolean isAimed = false;
    private double approachAngle = 90.0;
    private int targetXPixels = HUSKYLENS_WIDTH / 2;

    private double targetShooterRPM = 0.0;
    private double shooterPower = 0.0;
    private double shooterRPM = 0.0;
    private int lastShooterEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;
    private boolean encoderWorking = true;

    private double leftHoodPosition = 0.15;
    private double rightHoodPosition = 0.15;

    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean bothTrapdoorsOpen = false;
    private boolean lastGamepad2XState = false;
    private boolean lastGamepad2BState = false;
    private boolean lastGamepad2YState = false;

    private boolean transfersUp = false;
    private boolean lastGamepad2AState = false;

    private boolean leftKickerArmOpen = false;
    private boolean rightKickerArmOpen = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    private boolean colorPurpleSelected = false;
    private boolean colorGreenSelected = false;
    private boolean lastGamepad2LeftBumperState = false;
    private boolean lastGamepad2RightBumperState = false;

    private boolean shooterSpeedOn = false;
    private boolean lastGamepad2LeftTriggerState = false;

    private static final double MIN_IDLE_SHOOTER_RPM = 1800;

    private boolean lastGamepad2RightTriggerState = false;

    private ElapsedTime shootSequenceTimer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime shotFiredTimer = new ElapsedTime();
    private boolean shootSequenceActive = false;
    private boolean autoTransferTriggered = false;
    private boolean distanceCheckPassed = false;
    private boolean ballDetectedWaitingForRpm = false;
    private boolean shotFired = false;
    private boolean intakeReversed = false;
    private boolean restartIntakePulseActive = false;
    private ElapsedTime restartIntakePulseTimer = new ElapsedTime();
    private ElapsedTime intakePulseTimer = new ElapsedTime();
    private boolean kickLeft = false;
    private boolean kickRight = false;
    private boolean singleBallMode = false;
    private boolean trapdoorsOpenedForSingleBall = false;

    // --- FIXED PATTERN FOR THIS FILE: PPG ---
    private char[] ballOrder = {'P', 'P', 'G'};

    private boolean lastGamepad1YState = false;

    private boolean threeBallSequenceActive = false;
    private int currentBallIndex = 0;
    private boolean shootSideDecided = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean dumpMode = false;
    private double currentTargetRPM = 0.0;
    private boolean waitingForTransferCycle = false;
    private ElapsedTime transferCycleTimer = new ElapsedTime();

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize servos
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftTrapdoor = hardwareMap.get(Servo.class, "leftTrapdoor");
        rightTrapdoor = hardwareMap.get(Servo.class, "rightTrapdoor");
        leftKickerArm = hardwareMap.get(Servo.class, "leftKickerArm");
        rightKickerArm = hardwareMap.get(Servo.class, "rightKickerArm");
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Problem communicating with HuskyLens");
        } else {
            telemetry.addData("HuskyLens", "Connected");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

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

        velocityTimer.reset();
        lastShooterEncoderPosition = 0;

        leftHoodAdjustment.setPosition(leftHoodPosition);
        rightHoodAdjustment.setPosition(rightHoodPosition);
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);

        colorProcessor = new ColorRegionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colorProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.addData("Status", "Initialized - MODE: PPG");
    }

    @Override
    public void loop() {
        // ========== DRIVER CONTROLS ==========
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        float axial = (float) y;
        float lateral = (float) x * 1.1f;

        float leftFrontPower = axial + lateral + (float) yaw;
        float rightFrontPower = axial - lateral - (float) yaw;
        float leftBackPower = axial - lateral + (float) yaw;
        float rightBackPower = axial + lateral - (float) yaw;

        rightFrontPower = (float) Range.clip(rightFrontPower, -1.0, 1.0);
        leftFrontPower = (float) Range.clip(leftFrontPower, -1.0, 1.0);
        leftBackPower = (float) Range.clip(leftBackPower, -1.0, 1.0);
        rightBackPower = (float) Range.clip(rightBackPower, -1.0, 1.0);

        rightFrontPower *= RF_MULTIPLIER;
        leftFrontPower *= LF_MULTIPLIER;
        leftBackPower *= LB_MULTIPLIER;
        rightBackPower *= RB_MULTIPLIER;

        // HuskyLens Auto-Aim Logic
        HuskyLens.Block[] blocks = huskyLens.blocks();
        tagDetected = false;
        detectedDistance = 0.0;
        detectedTagX = -1;
        isAimed = false;

        if (blocks.length > 0) {
            HuskyLens.Block block = blocks[0];
            tagDetected = true;
            detectedTagX = block.x;
            double directDistance = (TAG_WIDTH_INCHES * FOCAL_LENGTH) / block.width;
            if (directDistance > HEIGHT_DIFF_INCHES) {
                detectedDistance = Math.sqrt(Math.pow(directDistance, 2) - Math.pow(HEIGHT_DIFF_INCHES, 2));
            } else {
                detectedDistance = directDistance;
            }

            double pixelOffsetFromCenter = detectedTagX - (HUSKYLENS_WIDTH / 2.0);
            double angleOffsetRadians = Math.toRadians((pixelOffsetFromCenter / (HUSKYLENS_WIDTH / 2.0)) * (HUSKYLENS_HFOV_DEGREES / 2.0));
            double lateralDistance = detectedDistance * Math.tan(angleOffsetRadians);
            double lateralAngleDegrees = Math.toDegrees(Math.atan2(lateralDistance, detectedDistance));
            approachAngle = 90.0 + lateralAngleDegrees;

            double clampedAngle = Range.clip(approachAngle, 45.0, 135.0);
            double targetXPercent = interpolateAngleToTarget(clampedAngle);
            targetXPixels = (int)(HUSKYLENS_WIDTH * targetXPercent);

            double aimError = detectedTagX - targetXPixels;
            isAimed = Math.abs(aimError) <= AIM_TOLERANCE_PIXELS;

            if (autoAimEnabled && !isAimed) {
                autoAimRotation = aimError * AIM_KP;
                autoAimRotation = Range.clip(autoAimRotation, -0.3, 0.3);
            } else {
                autoAimRotation = 0.0;
            }

            double[] lookupValues = interpolateLookupTable(detectedDistance);
            leftHoodPosition = lookupValues[0];
            double lookupRPM = lookupValues[1];

            if (autoAimEnabled) {
                leftHoodAdjustment.setPosition(leftHoodPosition);
                double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                rightHoodAdjustment.setPosition(rightHoodPosition);
                if (!shootSequenceActive) {
                    targetShooterRPM = lookupRPM;
                }
            } else {
                if (!shootSequenceActive && !threeBallSequenceActive) {
                    targetShooterRPM = 0.0;
                }
            }
        } else {
            if (autoAimEnabled && !shootSequenceActive && !threeBallSequenceActive) {
                targetShooterRPM = 0.0;
            }
        }

        if (gamepad1.right_trigger > 0.5 && !lastGamepad1RightTriggerState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastGamepad1RightTriggerState = gamepad1.right_trigger > 0.5;

        if (autoAimEnabled && tagDetected && !isAimed) {
            leftFrontPower += (float) autoAimRotation;
            rightFrontPower -= (float) autoAimRotation;
            leftBackPower += (float) autoAimRotation;
            rightBackPower -= (float) autoAimRotation;

            rightFrontPower = (float) Range.clip(rightFrontPower, -1.0, 1.0);
            leftFrontPower = (float) Range.clip(leftFrontPower, -1.0, 1.0);
            leftBackPower = (float) Range.clip(leftBackPower, -1.0, 1.0);
            rightBackPower = (float) Range.clip(rightBackPower, -1.0, 1.0);
        }

        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Intake Controls
        if (gamepad1.right_bumper && !lastGamepad1RightBumperState) {
            intakeToggleOn = !intakeToggleOn;
        }
        lastGamepad1RightBumperState = gamepad1.right_bumper;

        if (gamepad1.left_trigger > 0) {
            intake.setPower(-1.0);
        } else if (gamepad1.left_bumper) {
            intake.setPower(1.0);
        } else if (intakeToggleOn) {
            intake.setPower(-1.0);
        } else if (!shootSequenceActive && !threeBallSequenceActive) {
            intake.setPower(0.0);
        }

        // --- REMOVED GAMEPAD1.A LOGIC (Pattern Scan) ---
        // PPG Pattern is hardcoded.

        // Y Button: AUTO-AIM + SHOOT (Immediate start, no scan check needed)
        if (gamepad1.y && !lastGamepad1YState) {
            if (!autoAimEnabled) {
                autoAimEnabled = true;
            }
            if (!threeBallSequenceActive && !shootSequenceActive) {
                startThreeBallSequence();
            }
        }
        lastGamepad1YState = gamepad1.y;

        // ========== OPERATOR CONTROLS ==========
        if (gamepad2.x && !lastGamepad2XState) {
            leftTrapdoorOpen = !leftTrapdoorOpen;
            if (leftTrapdoorOpen) {
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.1);
            } else {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2XState = gamepad2.x;

        if (gamepad2.b && !lastGamepad2BState) {
            rightTrapdoorOpen = !rightTrapdoorOpen;
            if (rightTrapdoorOpen) {
                rightTrapdoor.setPosition(0.0);
                leftTrapdoor.setPosition(0.1);
            } else {
                rightTrapdoor.setPosition(0.1);
                leftTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2BState = gamepad2.b;

        if (gamepad2.y && !lastGamepad2YState) {
            bothTrapdoorsOpen = !bothTrapdoorsOpen;
            if (bothTrapdoorsOpen) {
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.0);
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
            } else {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
            }
        }
        lastGamepad2YState = gamepad2.y;

        if (gamepad2.a && !lastGamepad2AState) {
            transfersUp = !transfersUp;
            if (transfersUp) {
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
            } else {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
            }
        }
        lastGamepad2AState = gamepad2.a;

        if (gamepad2.dpad_left && !lastDpadLeftState) {
            leftKickerArmOpen = !leftKickerArmOpen;
            leftKickerArm.setPosition(leftKickerArmOpen ? 0.5 : 0.0);
        }
        lastDpadLeftState = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastDpadRightState) {
            rightKickerArmOpen = !rightKickerArmOpen;
            rightKickerArm.setPosition(rightKickerArmOpen ? 0.075 : 0.5);
        }
        lastDpadRightState = gamepad2.dpad_right;

        if (gamepad2.left_bumper && !lastGamepad2LeftBumperState) {
            colorPurpleSelected = true;
            colorGreenSelected = false;
            if (!shootSequenceActive) {
                if (!tagDetected || !autoAimEnabled) {
                    leftHoodPosition = 0.3;
                    leftHoodAdjustment.setPosition(leftHoodPosition);
                    double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                    rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                    rightHoodAdjustment.setPosition(rightHoodPosition);
                    targetShooterRPM = 1800.0;
                }
                startShootSequence();
            }
        }
        lastGamepad2LeftBumperState = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !lastGamepad2RightBumperState) {
            colorGreenSelected = true;
            colorPurpleSelected = false;
            if (!shootSequenceActive) {
                if (!tagDetected || !autoAimEnabled) {
                    leftHoodPosition = 0.3;
                    leftHoodAdjustment.setPosition(leftHoodPosition);
                    double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
                    rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
                    rightHoodAdjustment.setPosition(rightHoodPosition);
                    targetShooterRPM = 1800.0;
                }
                startShootSequence();
            }
        }
        lastGamepad2RightBumperState = gamepad2.right_bumper;

        boolean leftTriggerPressed = gamepad2.left_trigger > 0.5;
        if (leftTriggerPressed && !lastGamepad2LeftTriggerState) {
            shooterSpeedOn = !shooterSpeedOn;
            if (!shooterSpeedOn) {
                shooterIntegral = 0.0;
                shooterLastError = 0.0;
            }
        }
        lastGamepad2LeftTriggerState = leftTriggerPressed;

        // Shooter PID
        int currentShooterPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();
        if (deltaTime > 0.02) {
            int deltaTicks = currentShooterPosition - lastShooterEncoderPosition;
            double ticksPerSecond = (double) deltaTicks / deltaTime;
            shooterRPM = Math.abs((ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0);
            lastShooterEncoderPosition = currentShooterPosition;
            velocityTimer.reset();
        }

        double effectiveTargetRPM = 0.0;
        if (shooterSpeedOn) {
            if (threeBallSequenceActive && currentTargetRPM > 0) {
                effectiveTargetRPM = currentTargetRPM;
            } else if (shootSequenceActive && targetShooterRPM > 0) {
                effectiveTargetRPM = targetShooterRPM;
            } else if (autoAimEnabled && tagDetected && targetShooterRPM > 0) {
                effectiveTargetRPM = targetShooterRPM;
            } else {
                effectiveTargetRPM = MIN_IDLE_SHOOTER_RPM;
                if (!shootSequenceActive && !threeBallSequenceActive) {
                    targetShooterRPM = MIN_IDLE_SHOOTER_RPM;
                }
            }
        }

        encoderWorking = !(shooterPower > 0.3 && shooterRPM < 100 && velocityTimer.seconds() > 0.5);

        if (effectiveTargetRPM > 0) {
            if (encoderWorking) {
                double error = effectiveTargetRPM - shooterRPM;
                shooterIntegral += error * deltaTime;
                shooterIntegral = Range.clip(shooterIntegral, -5000, 5000);
                double derivative = (error - shooterLastError) / deltaTime;
                shooterLastError = error;
                double feedforward = effectiveTargetRPM * SHOOTER_KF;
                double pidOutput = (SHOOTER_KP * error) + (SHOOTER_KI * shooterIntegral) + (SHOOTER_KD * derivative);
                shooterPower = feedforward + pidOutput;
                shooterPower = Range.clip(shooterPower, 0.0, 1.0);
            } else {
                shooterPower = effectiveTargetRPM * SHOOTER_KF;
                shooterPower = Range.clip(shooterPower, 0.0, 0.8);
                shooterIntegral = 0.0;
                shooterLastError = 0.0;
                shooterRPM = effectiveTargetRPM * shooterPower / (effectiveTargetRPM * SHOOTER_KF);
            }
        } else {
            shooterPower = 0.0;
            shooterIntegral = 0.0;
            shooterLastError = 0.0;
        }
        shooter.setPower(shooterPower);

        double rpmLowerBound = targetShooterRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = targetShooterRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmInRange = targetShooterRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (rightTriggerPressed && !lastGamepad2RightTriggerState) {
            if (!threeBallSequenceActive && !shootSequenceActive) {
                // Scan check removed - start immediately
                startThreeBallSequence();
            } else if (threeBallSequenceActive || shootSequenceActive) {
                stopThreeBallSequence();
                stopShootSequence();
            }
        }
        lastGamepad2RightTriggerState = rightTriggerPressed;

        if (shootSequenceActive && !threeBallSequenceActive) {
            executeShootSequence();
        }

        if (threeBallSequenceActive) {
            executeThreeBallSequence();
        }

        if (autoTransferTriggered && transferTimer.seconds() >= 2.5) {
            autoTransferTriggered = false;
            transfersUp = false;
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
        }

        ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();

        // ========== TELEMETRY ==========
        telemetry.addData("--- MODE: PPG ---", "Purple-Purple-Green");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Tag Detected", tagDetected);

        // Removed dynamic scan telemetry, added static
        telemetry.addData("--- PATTERN ---", "PPG (Hardcoded)");
        telemetry.addData("Ball Order", "PPG");
        telemetry.addData("3-Ball Ready", "Press RT or Driver Y to shoot!");

        if (threeBallSequenceActive) {
            telemetry.addData("--- 3-BALL SEQUENCE ---", "ACTIVE");
            telemetry.addData("Current Ball", (currentBallIndex + 1) + " of 3");
            if (currentBallIndex < 3) {
                telemetry.addData("Target Color", ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN");
            }
        }

        telemetry.addData("Left Camera", colorResult.leftColor);
        telemetry.addData("Right Camera", colorResult.rightColor);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooterRPM, targetShooterRPM);
        telemetry.addData("Distance", "%.2f cm", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    private void startShootSequence() {
        shootSequenceActive = true;
        shootSequenceTimer.reset();
        intakePulseTimer.reset();
        shotFired = false;
        intakeReversed = false;
        shooterSpeedOn = true;

        ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
        String leftColor = colorResult.leftColor;
        String rightColor = colorResult.rightColor;

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

        if (leftMatchesSelected && !rightMatchesSelected) {
            kickLeft = true;
            kickRight = false;
        } else if (rightMatchesSelected && !leftMatchesSelected) {
            kickLeft = false;
            kickRight = true;
        } else if (leftMatchesSelected && rightMatchesSelected) {
            kickLeft = true;
            kickRight = false;
        } else {
            kickLeft = true;
            kickRight = true;
        }

        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;

        if (kickLeft && kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.0);
        } else if (kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        } else if (kickLeft && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
        }

        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);
        transfersUp = false;
    }

    private void executeShootSequence() {
        double rpmLowerBound = targetShooterRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = targetShooterRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = targetShooterRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        if (restartIntakePulseActive && restartIntakePulseTimer.milliseconds() >= 150) {
            intake.setPower(-1.0);
            restartIntakePulseActive = false;
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            transfersUp = false;
        }

        if (!restartIntakePulseActive && !distanceCheckPassed) {
            long pulsePhase = (long) intakePulseTimer.milliseconds() % 500;
            if (pulsePhase < 250) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
        }

        if (!distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance < 20 && !ballDetectedWaitingForRpm) {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;
                ballDetectedWaitingForRpm = true;
            }
            if (ballDetectedWaitingForRpm && rpmReady) {
                distanceCheckPassed = true;
                ballDetectedWaitingForRpm = false;
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
                transfersUp = true;
                intake.setPower(-1.0);
                intakeReversed = false;
                transferTimer.reset();
            }
        }

        if (distanceCheckPassed && !intakeReversed && transferTimer.milliseconds() >= 50) {
            intake.setPower(-1.0);
            intakeReversed = true;
        }

        if (shootSequenceTimer.milliseconds() >= 1500 && !distanceCheckPassed) {
            restartShootSequence();
            return;
        }

        if (distanceCheckPassed && rpmReady && !shotFired) {
            shotFired = true;
            shotFiredTimer.reset();
        }

        if (shotFired && shotFiredTimer.milliseconds() >= 750) {
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            leftKickerArmOpen = false;
            rightKickerArmOpen = false;
            transfersUp = false;
            shootSequenceActive = false;
            distanceCheckPassed = false;
            ballDetectedWaitingForRpm = false;
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
        shootSequenceActive = false;
        distanceCheckPassed = false;
        ballDetectedWaitingForRpm = false;
        shotFired = false;
        intakeReversed = false;
        restartIntakePulseActive = false;
        kickLeft = false;
        kickRight = false;
        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        rightTransfer.setPosition(0.0);
        leftTransfer.setPosition(0.5);
        intake.setPower(0.0);
        leftTrapdoorOpen = false;
        rightTrapdoorOpen = false;
        bothTrapdoorsOpen = false;
        leftKickerArmOpen = false;
        rightKickerArmOpen = false;
        transfersUp = false;
    }

    private void restartShootSequence() {
        shootSequenceTimer.reset();
        distanceCheckPassed = false;
        ballDetectedWaitingForRpm = false;
        shotFired = false;
        intakeReversed = false;

        if (kickLeft && kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.0);
        } else if (singleBallMode && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.1);
        } else if (kickLeft && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.1);
        } else if (kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
            rightTrapdoor.setPosition(0.1);
            leftTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
        }

        intake.setPower(-1.0);
        restartIntakePulseActive = true;
        restartIntakePulseTimer.reset();
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    private void startThreeBallSequence() {
        threeBallSequenceActive = true;
        currentBallIndex = 0;
        shootTimer.reset();
        shotFired = false;
        distanceCheckPassed = false;
        ballDetectedWaitingForRpm = false;
        shootSideDecided = false;
        intakeReversed = false;
        restartIntakePulseActive = false;
        kickLeft = false;
        kickRight = false;
        dumpMode = false;
        waitingForTransferCycle = false;
        shooterSpeedOn = true;

        if (!tagDetected || !autoAimEnabled) {
            leftHoodPosition = 0.3;
            leftHoodAdjustment.setPosition(leftHoodPosition);
            double rightHoodCalc = 0.25 - ((leftHoodPosition - 0.05) / (0.3 - 0.05)) * (0.25 - 0.0);
            rightHoodPosition = Range.clip(rightHoodCalc, 0.0, 0.25);
            rightHoodAdjustment.setPosition(rightHoodPosition);
            targetShooterRPM = 1800.0;
        }

        currentTargetRPM = targetShooterRPM;
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);
        transfersUp = false;
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
    }

    private void executeThreeBallSequence() {
        if (currentBallIndex >= 3) {
            stopThreeBallSequence();
            return;
        }

        if (waitingForTransferCycle) {
            if (transferCycleTimer.milliseconds() >= 500 && transfersUp) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                transfersUp = false;
            }
            double transferCycleWaitTime = (currentBallIndex == 0) ? 600 : 300;
            if (transferCycleTimer.milliseconds() >= transferCycleWaitTime) {
                waitingForTransferCycle = false;
                currentBallIndex++;
                shotFired = false;
                distanceCheckPassed = false;
                ballDetectedWaitingForRpm = false;
                intakeReversed = false;
                shootSideDecided = false;
                restartIntakePulseActive = false;
                kickLeft = false;
                kickRight = false;
                dumpMode = false;
                currentTargetRPM = targetShooterRPM;
                shootTimer.reset();
                if (currentBallIndex >= 3) {
                    stopThreeBallSequence();
                    return;
                }
            }
            return;
        }

        double scanWaitTime = (currentBallIndex == 0) ? 300 : 500;
        if (!shootSideDecided && shootTimer.milliseconds() > scanWaitTime) {
            char targetColor = ballOrder[currentBallIndex];
            boolean isThirdBall = (currentBallIndex == 2);

            ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
            String leftColor = colorResult.leftColor;
            String rightColor = colorResult.rightColor;

            boolean leftMatchesTarget = false;
            boolean rightMatchesTarget = false;
            boolean leftHasWrongColor = false;
            boolean rightHasWrongColor = false;
            boolean anyColorDetected = !leftColor.equals("NEITHER") || !rightColor.equals("NEITHER");

            if (targetColor == 'P') {
                leftMatchesTarget = leftColor.equals("PURPLE");
                rightMatchesTarget = rightColor.equals("PURPLE");
                leftHasWrongColor = leftColor.equals("GREEN");
                rightHasWrongColor = rightColor.equals("GREEN");
            } else {
                leftMatchesTarget = leftColor.equals("GREEN");
                rightMatchesTarget = rightColor.equals("GREEN");
                leftHasWrongColor = leftColor.equals("PURPLE");
                rightHasWrongColor = rightColor.equals("PURPLE");
            }

            if (!anyColorDetected && shootTimer.milliseconds() < 1200) {
                return;
            }

            if (isThirdBall) {
                kickLeft = true;
                kickRight = true;
                dumpMode = false;
                currentTargetRPM = targetShooterRPM;
            } else {
                if (leftMatchesTarget || rightMatchesTarget) {
                    dumpMode = false;
                    currentTargetRPM = targetShooterRPM;
                    if (leftMatchesTarget && !rightMatchesTarget) {
                        kickLeft = true;
                        kickRight = false;
                    } else if (rightMatchesTarget && !leftMatchesTarget) {
                        kickLeft = false;
                        kickRight = true;
                    } else {
                        kickLeft = true;
                        kickRight = false;
                    }
                } else if (leftHasWrongColor || rightHasWrongColor) {
                    dumpMode = true;
                    currentTargetRPM = DUMP_RPM;
                    if (leftHasWrongColor && !rightHasWrongColor) {
                        kickLeft = true;
                        kickRight = false;
                    } else if (rightHasWrongColor && !leftHasWrongColor) {
                        kickLeft = false;
                        kickRight = true;
                    } else {
                        kickLeft = true;
                        kickRight = false;
                    }
                } else {
                    kickLeft = true;
                    kickRight = true;
                    dumpMode = false;
                    currentTargetRPM = targetShooterRPM;
                }
            }

            shootSideDecided = true;
            intakePulseTimer.reset();

            if (kickLeft && kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.0);
                bothTrapdoorsOpen = true;
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
            } else if (kickLeft && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.0);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = true;
            } else if (kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.2);
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = false;
            } else {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;
            }
        }

        if (restartIntakePulseActive && restartIntakePulseTimer.milliseconds() >= 150) {
            intake.setPower(-1.0);
            restartIntakePulseActive = false;
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            transfersUp = false;
        }

        if (!restartIntakePulseActive && !distanceCheckPassed) {
            long pulsePhase = (long) intakePulseTimer.milliseconds() % 500;
            if (pulsePhase < 250) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
        }

        double rpmLowerBound = currentTargetRPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = currentTargetRPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = currentTargetRPM > 0 && shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        if (shootTimer.milliseconds() >= 200 && !distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance < 25 && !ballDetectedWaitingForRpm) {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;
                ballDetectedWaitingForRpm = true;
                kickLeft = false;
                kickRight = false;
            }
            if (ballDetectedWaitingForRpm && rpmReady) {
                distanceCheckPassed = true;
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
                transfersUp = true;
                intake.setPower(-1.0);
                intakeReversed = false;
                transferTimer.reset();
                shotFired = true;
                shotFiredTimer.reset();
                ballDetectedWaitingForRpm = false;
            }
        }

        if (distanceCheckPassed && !intakeReversed && transferTimer.milliseconds() >= 50) {
            intake.setPower(-1.0);
            intakeReversed = true;
        }

        if (shootTimer.milliseconds() >= 1500 && !distanceCheckPassed) {
            restartBallInSequence();
            return;
        }

        if (shotFired && shotFiredTimer.milliseconds() >= 750) {
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            waitingForTransferCycle = true;
            transferCycleTimer.reset();
            shotFired = false;
            if (currentBallIndex >= 2) {
                stopThreeBallSequence();
                autoAimEnabled = false;
            }
        }
    }

    private void restartBallInSequence() {
        shootTimer.reset();
        distanceCheckPassed = false;
        ballDetectedWaitingForRpm = false;
        shotFired = false;
        intakeReversed = false;

        if (!ballDetectedWaitingForRpm && !distanceCheckPassed) {
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
        }

        intake.setPower(-1.0);
        restartIntakePulseActive = true;
        restartIntakePulseTimer.reset();
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    private void stopThreeBallSequence() {
        threeBallSequenceActive = false;
        currentBallIndex = 0;
        shootSideDecided = false;
        distanceCheckPassed = false;
        ballDetectedWaitingForRpm = false;
        shotFired = false;
        intakeReversed = false;
        restartIntakePulseActive = false;
        kickLeft = false;
        kickRight = false;
        dumpMode = false;
        currentTargetRPM = 0.0;
        waitingForTransferCycle = false;
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        rightTransfer.setPosition(0.0);
        leftTransfer.setPosition(0.5);
        intake.setPower(0.0);
        leftTrapdoorOpen = false;
        rightTrapdoorOpen = false;
        bothTrapdoorsOpen = false;
        leftKickerArmOpen = false;
        rightKickerArmOpen = false;
        transfersUp = false;
    }

    private double[] interpolateLookupTable(double distance) {
        if (distance <= SHOOTER_LOOKUP_TABLE[0][0]) {
            return new double[] {SHOOTER_LOOKUP_TABLE[0][1], SHOOTER_LOOKUP_TABLE[0][2]};
        }
        int lastIndex = SHOOTER_LOOKUP_TABLE.length - 1;
        if (distance >= SHOOTER_LOOKUP_TABLE[lastIndex][0]) {
            return new double[] {SHOOTER_LOOKUP_TABLE[lastIndex][1], SHOOTER_LOOKUP_TABLE[lastIndex][2]};
        }
        for (int i = 0; i < SHOOTER_LOOKUP_TABLE.length - 1; i++) {
            double dist1 = SHOOTER_LOOKUP_TABLE[i][0];
            double dist2 = SHOOTER_LOOKUP_TABLE[i + 1][0];
            if (distance >= dist1 && distance <= dist2) {
                double factor = (distance - dist1) / (dist2 - dist1);
                double hood1 = SHOOTER_LOOKUP_TABLE[i][1];
                double hood2 = SHOOTER_LOOKUP_TABLE[i + 1][1];
                double interpolatedHood = hood1 + factor * (hood2 - hood1);
                double rpm1 = SHOOTER_LOOKUP_TABLE[i][2];
                double rpm2 = SHOOTER_LOOKUP_TABLE[i + 1][2];
                double interpolatedRPM = rpm1 + factor * (rpm2 - rpm1);
                return new double[] {interpolatedHood, interpolatedRPM};
            }
        }
        return new double[] {0.15, 0.0};
    }

    private double interpolateAngleToTarget(double angle) {
        if (angle <= ANGLE_TO_TARGET_LOOKUP[0][0]) {
            return ANGLE_TO_TARGET_LOOKUP[0][1];
        }
        int lastIndex = ANGLE_TO_TARGET_LOOKUP.length - 1;
        if (angle >= ANGLE_TO_TARGET_LOOKUP[lastIndex][0]) {
            return ANGLE_TO_TARGET_LOOKUP[lastIndex][1];
        }
        for (int i = 0; i < ANGLE_TO_TARGET_LOOKUP.length - 1; i++) {
            double angle1 = ANGLE_TO_TARGET_LOOKUP[i][0];
            double angle2 = ANGLE_TO_TARGET_LOOKUP[i + 1][0];
            if (angle >= angle1 && angle <= angle2) {
                double factor = (angle - angle1) / (angle2 - angle1);
                double target1 = ANGLE_TO_TARGET_LOOKUP[i][1];
                double target2 = ANGLE_TO_TARGET_LOOKUP[i + 1][1];
                return target1 + factor * (target2 - target1);
            }
        }
        return 0.75;
    }
}