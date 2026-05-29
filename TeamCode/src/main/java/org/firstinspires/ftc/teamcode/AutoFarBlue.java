package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.ColorDetectionTest.ColorRegionProcessor;

@Autonomous(name = "DECODE 25-26 AutoFarBlue")
public class AutoFarBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private HuskyLens huskyLens;

    // Hardware for shooting
    private DcMotor shooter;
    private DcMotor intake;
    private Servo leftTrapdoor;
    private Servo rightTrapdoor;
    private Servo leftTransfer;
    private Servo rightTransfer;
    private Servo leftKickerArm;
    private Servo rightKickerArm;
    private Servo leftHoodAdjustment;
    private Servo rightHoodAdjustment;
    private DistanceSensor distanceSensor;

    // Webcam color detection
    private ColorRegionProcessor colorProcessor;
    private VisionPortal visionPortal;

    // Shooter constants
    private static final double TARGET_RPM = 2335.0;
    private static final double RPM_TOLERANCE_PERCENT = 0.05;  // 5% tolerance
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double MAX_SHOOTER_RPM = 4900.0;

    // Shooter PIDF constants
    private static final double SHOOTER_KP = 0.0015;
    private static final double SHOOTER_KI = 0.00001;
    private static final double SHOOTER_KD = 0.00001;
    private static final double SHOOTER_KF = 1.0 / MAX_SHOOTER_RPM;

    // Shooter state tracking
    private double shooterRPM = 0.0;
    private int lastShooterEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // PIDF state variables
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;
    private ElapsedTime shooterPIDTimer = new ElapsedTime();

    // Intake pulse timer for -1/0 pattern (250ms each)
    private ElapsedTime intakePulseTimer = new ElapsedTime();

    // Transfer and shooting state
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transfersUp = false;
    private boolean intakeReversed = false;

    // Restart intake pulse state
    private boolean restartIntakePulseActive = false;
    private ElapsedTime restartIntakePulseTimer = new ElapsedTime();

    // Shooting sequence state
    private int currentBallIndex = 0;  // 0, 1, 2 for the three balls
    private boolean shotFired = false;
    private boolean distanceCheckPassed = false;
    private boolean ballDetectedWaitingForRpm = false;
    private ElapsedTime shotFiredTimer = new ElapsedTime();
    private boolean shootSideDecided = false;
    private boolean kickLeft = false;
    private boolean kickRight = false;
    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean bothTrapdoorsOpen = false;

    // Ball order based on AprilTag (P = Purple/Left, G = Green/Right)
    private char[] ballOrder = new char[3];

    /* Define poses for the autonomous routine - PLACEHOLDER POSITIONS */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(270)).mirror();  // PLACEHOLDER: Starting position
    private final Pose scanPose = new Pose(0, 30, Math.toRadians(270)).mirror();  // PLACEHOLDER: Position to scan AprilTag
    private final Pose shootPose = new Pose(0, 15, Math.toRadians(250)).mirror();  // PLACEHOLDER: Position to shoot balls
    private final Pose endPose = new Pose(30, 0, Math.toRadians(0)).mirror();  // PLACEHOLDER: Final parking position

    /* Path declarations */
    private Path toScanPath;
    private Path toShootPath;
    private Path toEndPath;

    /* AprilTag scanning state */
    private int detectedAprilTagId = -1;

    /** Build all paths for the autonomous routine **/
    public void buildPaths() {
        toScanPath = new Path(new BezierLine(startPose, scanPose));
        toScanPath.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());

        toShootPath = new Path(new BezierLine(scanPose, shootPose));
        toShootPath.setLinearHeadingInterpolation(scanPose.getHeading(), shootPose.getHeading());

        toEndPath = new Path(new BezierLine(shootPose, endPose));
        toEndPath.setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading());
    }

    /** Main state machine for autonomous path progression **/
    public void autonomousPathUpdate() {
        // Update shooter RPM tracking
        updateShooterRPM();

        switch (pathState) {
            case 0:
                // Move to scan position
                follower.followPath(toScanPath);
                setPathState(1);
                break;

            case 1:
                // Wait until robot reaches scan position
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // Scan for AprilTag at scanPose
                HuskyLens.Block[] blocks = huskyLens.blocks();
                if (blocks.length > 0 && detectedAprilTagId == -1) {
                    detectedAprilTagId = blocks[0].id;

                    // Set ball order based on AprilTag ID
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
                            ballOrder = new char[]{'P', 'P', 'G'};
                            break;
                    }

                    // Start shooter and move to shoot position
                    shooter.setPower(0.5);
                    follower.followPath(toShootPath);

                    // Initialize shooting sequence
                    currentBallIndex = 0;
                    shootTimer.reset();
                    shotFired = false;
                    distanceCheckPassed = false;
                    ballDetectedWaitingForRpm = false;
                    shootSideDecided = false;

                    setPathState(3);
                }
                break;

            case 3:
                // Moving to shoot position while spinning up shooter
                controlShooterPID();
                if (!follower.isBusy()) {
                    follower.holdPoint(shootPose);
                    setPathState(4);
                }
                break;

            case 4:
                // Shooting state - execute three ball sequence
                controlShooterPID();
                executeShootSequence();

                // Check if all balls are shot
                if (currentBallIndex >= 3) {
                    setPathState(5);
                }
                break;

            case 5:
                // All balls shot, move to end position
                shooter.setPower(0);
                intake.setPower(0);
                follower.followPath(toEndPath);
                setPathState(6);
                break;

            case 6:
                // Wait for robot to reach end position
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7:
                // Final state - done
                break;
        }
    }

    /** Update shooter RPM from encoder **/
    private void updateShooterRPM() {
        int currentShooterPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {
            int deltaTicks = currentShooterPosition - lastShooterEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            shooterRPM = (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
            lastShooterEncoderPosition = currentShooterPosition;
            velocityTimer.reset();
        }
    }

    /** Full PIDF control for shooter RPM **/
    private void controlShooterPID() {
        double deltaTime = shooterPIDTimer.seconds();
        shooterPIDTimer.reset();

        if (deltaTime <= 0) deltaTime = 0.02;

        double error = TARGET_RPM - shooterRPM;

        // Integral term with windup prevention
        shooterIntegral += error * deltaTime;
        shooterIntegral = Math.max(-1000, Math.min(1000, shooterIntegral));

        // Derivative term
        double derivative = (error - shooterLastError) / deltaTime;
        shooterLastError = error;

        // PIDF calculation
        double feedforward = TARGET_RPM * SHOOTER_KF;
        double pTerm = error * SHOOTER_KP;
        double iTerm = shooterIntegral * SHOOTER_KI;
        double dTerm = derivative * SHOOTER_KD;

        double power = feedforward + pTerm + iTerm + dTerm;
        power = Math.max(0.0, Math.min(1.0, power));
        shooter.setPower(power);
    }

    /** Execute the shooting sequence using webcam color detection **/
    private void executeShootSequence() {
        if (currentBallIndex >= 3) {
            return;
        }

        double rpmLowerBound = TARGET_RPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = TARGET_RPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        // Determine which side to shoot from using webcam color detection
        if (!shootSideDecided && shootTimer.milliseconds() > 750) {
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
                } else {
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
                    kickLeft = true;
                    kickRight = false;
                } else {
                    kickLeft = true;
                    kickRight = true;
                }
            }

            shootSideDecided = true;
            intakePulseTimer.reset();

            // Open appropriate trapdoor (only if ball not yet detected)
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

        // Handle restart intake pulse completion
        if (restartIntakePulseActive && restartIntakePulseTimer.milliseconds() >= 150) {
            intake.setPower(-1.0);
            restartIntakePulseActive = false;
            leftTransfer.setPosition(0.5);
            rightTransfer.setPosition(0.0);
            transfersUp = false;
        }

        // Pulse intake while waiting for distance check
        if (!restartIntakePulseActive && !distanceCheckPassed) {
            long pulsePhase = (long) intakePulseTimer.milliseconds() % 500;
            if (pulsePhase < 250) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
        }

        // Continuous distance check
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

                shotFired = true;
                shotFiredTimer.reset();
            }
        }

        // If distance check passed but RPM wasn't ready, keep checking
        if (distanceCheckPassed && !shotFired && rpmReady) {
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
            transfersUp = true;
            intake.setPower(-1.0);

            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;

            shotFired = true;
            shotFiredTimer.reset();
        }

        // After 50ms, set intake to -1.0
        if (distanceCheckPassed && !intakeReversed && transferTimer.milliseconds() >= 50) {
            intake.setPower(-1.0);
            intakeReversed = true;
        }

        // Timeout safety (1.5 seconds)
        if (shootTimer.milliseconds() >= 1500 && !distanceCheckPassed) {
            restartBallSequence();
            return;
        }

        // End sequence 750ms after firing
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
            transfersUp = false;

            currentBallIndex++;
            shotFired = false;
            distanceCheckPassed = false;
            ballDetectedWaitingForRpm = false;
            intakeReversed = false;
            shootSideDecided = false;
            restartIntakePulseActive = false;
            kickLeft = false;
            kickRight = false;
            shootTimer.reset();
        }
    }

    /** Restart the sequence for the current ball when distance sensor doesn't detect ball **/
    private void restartBallSequence() {
        shootTimer.reset();
        distanceCheckPassed = false;
        ballDetectedWaitingForRpm = false;
        shotFired = false;
        intakeReversed = false;

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

        intake.setPower(-1.0);
        restartIntakePulseActive = true;
        restartIntakePulseTimer.reset();

        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /** Change the path state and reset the timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** Main loop of the OpMode **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        if (detectedAprilTagId != -1) {
            telemetry.addData("AprilTag ID", detectedAprilTagId);
            telemetry.addData("Ball Order", "" + ballOrder[0] + ballOrder[1] + ballOrder[2]);
        } else {
            telemetry.addData("AprilTag", "Scanning...");
        }

        telemetry.addData("Shooter RPM", "%.0f", shooterRPM);

        if (pathState == 4) {
            telemetry.addData("Current Ball", currentBallIndex + 1);
            telemetry.addData("Target Color", currentBallIndex < 3 ? (ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN") : "Done");
            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
        }

        telemetry.update();
    }

    /** Initialize the OpMode **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Problem communicating with HuskyLens");
        } else {
            telemetry.addData("HuskyLens", "Connected");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Initialize shooter hardware
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        leftTrapdoor = hardwareMap.get(Servo.class, "leftTrapdoor");
        rightTrapdoor = hardwareMap.get(Servo.class, "rightTrapdoor");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        leftKickerArm = hardwareMap.get(Servo.class, "leftKickerArm");
        rightKickerArm = hardwareMap.get(Servo.class, "rightKickerArm");
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        // Initialize distance sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Initialize webcam color detection
        colorProcessor = new ColorRegionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colorProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Set initial servo positions
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        leftTransfer.setPosition(0.5);
        rightTransfer.setPosition(0.0);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);

        // Set hood position for shooting
        leftHoodAdjustment.setPosition(0);
        rightHoodAdjustment.setPosition(0.3);

        velocityTimer.reset();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}

