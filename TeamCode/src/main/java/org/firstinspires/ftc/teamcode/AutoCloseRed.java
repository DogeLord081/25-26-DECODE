package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "DECODE 25-26 AutoCloseRed")
public class AutoCloseRed extends OpMode {

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
    private static final double TARGET_RPM = 1800.0;
    private static final double RPM_TOLERANCE_PERCENT = 0.05;  // 5% tolerance
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double MAX_SHOOTER_RPM = 4900.0;

    // Shooter PIDF constants (from Tele.java)
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
    private ElapsedTime intakeReversalTimer = new ElapsedTime();

    // Restart intake pulse state
    private boolean restartIntakePulseActive = false;
    private ElapsedTime restartIntakePulseTimer = new ElapsedTime();

    // Shooting sequence state
    private int currentBallIndex = 0;  // 0, 1, 2 for the three balls
    private boolean shotFired = false;
    private boolean distanceCheckPassed = false;
    private boolean ballDetectedWaitingForRpm = false; // Ball detected, trapdoors closed, waiting for RPM
    private ElapsedTime shotFiredTimer = new ElapsedTime();
    private boolean shootSideDecided = false;
    private boolean kickLeft = false;
    private boolean kickRight = false;
    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean bothTrapdoorsOpen = false;

    // Ball order based on AprilTag (P = Purple/Left, G = Green/Right)
    // ID 1: PPG, ID 2: PGP, ID 3: GPP
    private char[] ballOrder = new char[3];

    private final Pose startPose = new Pose(26.63157142857142, 127.60802107728338, Math.toRadians(325)).mirror();
    private final Pose scorePose = new Pose(56.328, 115.18032786885244, Math.toRadians(250)).mirror();
    private final Pose afterScanPose = new Pose(52.328, 100.18032786885244, Math.toRadians(320)).mirror();
    private final Pose afterShootPose = new Pose(56.55750819672132, 60, Math.toRadians(160)).mirror();
    private final Pose intakeBallsPose = new Pose(25.55750819672132, 60, Math.toRadians(200)).mirror();
    private final Pose intakeBallsPosePart2 = new Pose(20, 60, Math.toRadians(180)).mirror();
    private final Pose afterShootPose2 = new Pose(54.55750819672132, 40, Math.toRadians(160)).mirror();
    private final Pose intakeBallsPose2 = new Pose(25.55750819672132, 40, Math.toRadians(200)).mirror();
    private final Pose intakeBallsPose2Part2 = new Pose(22.55750819672132, 40, Math.toRadians(180)).mirror();

    /* Path declarations */
    private Path scorePreload;
    private Path afterScanPath;
    private Path afterShootPath;
    private Path afterShootPath2;
    private Path intakeBallsPath;
    private Path intakeBallsPathPart2;
    private Path intakeBallsPath2;
    private Path intakeBallsPath2Part2;
    private Path returnToAfterShootPath;
    private Path returnToAfterShootPath2;
    private Path returnToScanPath;
    private Path returnToScanPath2;

    /* AprilTag scanning state */
    private int detectedAprilTagId = -1;

    /** Build all paths for the autonomous routine **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        afterScanPath = new Path(new BezierLine(scorePose, afterScanPose));
        afterScanPath.setLinearHeadingInterpolation(scorePose.getHeading(), afterScanPose.getHeading());

        afterShootPath = new Path(new BezierLine(afterScanPose, afterShootPose));
        afterShootPath.setLinearHeadingInterpolation(afterScanPose.getHeading(), afterShootPose.getHeading());

        intakeBallsPath = new Path(new BezierLine(afterShootPose, intakeBallsPose));
        intakeBallsPath.setLinearHeadingInterpolation(afterShootPose.getHeading(), intakeBallsPose.getHeading());

        intakeBallsPathPart2 = new Path(new BezierLine(intakeBallsPose, intakeBallsPosePart2));
        intakeBallsPathPart2.setLinearHeadingInterpolation(intakeBallsPose.getHeading(), intakeBallsPosePart2.getHeading());

        returnToAfterShootPath = new Path(new BezierLine(intakeBallsPosePart2, afterShootPose));
        returnToAfterShootPath.setLinearHeadingInterpolation(intakeBallsPosePart2.getHeading(), afterShootPose.getHeading());

        afterShootPath2 = new Path(new BezierLine(afterScanPose, afterShootPose2));
        afterShootPath2.setLinearHeadingInterpolation(afterScanPose.getHeading(), afterShootPose2.getHeading());

        intakeBallsPath2 = new Path(new BezierLine(afterShootPose2, intakeBallsPose2));
        intakeBallsPath2.setLinearHeadingInterpolation(afterShootPose2.getHeading(), intakeBallsPose2.getHeading());

        intakeBallsPath2Part2 = new Path(new BezierLine(intakeBallsPose2, intakeBallsPose2Part2));
        intakeBallsPath2Part2.setLinearHeadingInterpolation(intakeBallsPose2.getHeading(), intakeBallsPose2Part2.getHeading());

        returnToScanPath = new Path(new BezierCurve(
                intakeBallsPosePart2,
                new Pose(55.0, 85.0, 0),
                afterScanPose
        ));
        returnToScanPath.setLinearHeadingInterpolation(intakeBallsPosePart2.getHeading(), afterScanPose.getHeading());

        returnToScanPath2 = new Path(new BezierCurve(
                intakeBallsPose2Part2,
                new Pose(55.0, 85.0, 0),
                afterScanPose
        ));
        returnToScanPath2.setLinearHeadingInterpolation(intakeBallsPose2Part2.getHeading(), afterScanPose.getHeading());
    }

    /** Main state machine for autonomous path progression **/
    public void autonomousPathUpdate() {
        // Update shooter RPM tracking
        updateShooterRPM();

        // Emergency park logic
        if (opmodeTimer.getElapsedTimeSeconds() > 28.5 && pathState != 99 && pathState != 100) {
            shooter.setPower(0);
            intake.setPower(0);
            Pose currentPose = follower.getPose();
            Path emergencyParkPath = new Path(new BezierLine(currentPose, afterShootPose));
            emergencyParkPath.setLinearHeadingInterpolation(currentPose.getHeading(), afterShootPose.getHeading());
            follower.followPath(emergencyParkPath);
            setPathState(99);
            return;
        }

        switch (pathState) {
            case 0:
                // Start shooter spinning to target RPM (1850)
                shooter.setPower(0.5);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // Wait until robot reaches scorePose, maintain shooter RPM
                controlShooterPID();
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // Scan for AprilTag at scorePose
                controlShooterPID();
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

                    // Start moving to afterScanPose
                    follower.followPath(afterScanPath);

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
                // Moving to afterScanPose while starting the shooting sequence
                controlShooterPID();
                executeShootSequence();

                if (!follower.isBusy()) {
                    // Arrived at afterScanPose, continue shooting
                    follower.holdPoint(afterScanPose);
                    setPathState(4);
                }
                break;

            case 4:
                // Shooting state at afterScanPose
                controlShooterPID();
                executeShootSequence();

                // Check if all balls are shot
                if (currentBallIndex >= 3) {
                    setPathState(5);
                }
                break;

            case 5:
                // All balls shot, go to afterShootPose
                shooter.setPower(0);
                intake.setPower(0);
                follower.followPath(afterShootPath);
                setPathState(6);
                break;

            case 6:
                // Wait for robot to reach afterShootPose
                if (!follower.isBusy()) {
                    // Turn on intake to pick up balls
                    intake.setPower(-1.0);
                    follower.followPath(intakeBallsPath);
                    setPathState(7);
                }
                break;

            case 7:
                // Wait for robot to reach intakeBallsPose, then go to Part2
                if (!follower.isBusy()) {
                    // Continue intake and go to Part2
                    follower.followPath(intakeBallsPathPart2);
                    setPathState(8);
                }
                break;

            case 8:
                // Wait for robot to reach intakeBallsPosePart2
                if (!follower.isBusy()) {
                    // Start return trip
                    follower.followPath(returnToAfterShootPath);

                    // Reset for next round of shooting
                    currentBallIndex = 0;
                    shootTimer.reset();
                    shotFired = false;
                    distanceCheckPassed = false;
                    ballDetectedWaitingForRpm = false;
                    shootSideDecided = false;

                    setPathState(9);
                }
                break;

            case 9:
                // Traveling back to afterShootPose
                controlShooterPID();

                if (!follower.isBusy()) {
                    shooter.setPower(0.5);  // Spin up shooter again
                    follower.followPath(returnToScanPath);
                    setPathState(10);
                }
                break;

            case 10:
                // Traveling back to afterScanPose
                controlShooterPID();
                // executeShootSequence();

                if (!follower.isBusy()) {
                    follower.holdPoint(afterScanPose);
                    setPathState(11);
                }
                break;

            case 11:
                // Round 2 shooting at afterScanPose
                controlShooterPID();
                executeShootSequence();

                if (currentBallIndex >= 3) {
                    setPathState(12);
                }
                break;

            case 12:
                // Round 2 done, go to afterShootPose2
                shooter.setPower(0);
                intake.setPower(0);
                follower.followPath(afterShootPath2);
                setPathState(13);
                break;

            case 13:
                // Wait for robot to reach afterShootPose2
                if (!follower.isBusy()) {
                    // Turn on intake to pick up balls
                    intake.setPower(-1.0);
                    follower.followPath(intakeBallsPath2);
                    setPathState(14);
                }
                break;

            case 14:
                // Wait for robot to reach intakeBallsPose2, then go to Part2
                if (!follower.isBusy()) {
                    // Continue intake and go to Part2
                    follower.followPath(intakeBallsPath2Part2);
                    setPathState(15);
                }
                break;

            case 15:
                // Wait for robot to reach intakeBallsPose2Part2
                if (!follower.isBusy()) {
                    // Start return trip to afterScanPose
                    follower.followPath(returnToScanPath2);

                    // Reset for third round of shooting
                    currentBallIndex = 0;
                    shootTimer.reset();
                    shotFired = false;
                    distanceCheckPassed = false;
                    ballDetectedWaitingForRpm = false;
                    shootSideDecided = false;

                    setPathState(16);
                }
                break;

            case 16:
                // Traveling back to afterScanPose
                controlShooterPID();

                if (!follower.isBusy()) {
                    shooter.setPower(0.5);  // Spin up shooter again
                    follower.holdPoint(afterScanPose);
                    setPathState(17);
                }
                break;

            case 17:
                // Round 3 shooting at afterScanPose
                controlShooterPID();
                executeShootSequence();

                if (currentBallIndex >= 3) {
                    setPathState(18);
                }
                break;

            case 18:
                // Final state - stop
                shooter.setPower(0);
                intake.setPower(0);
                setPathState(-1);
                break;

            case 99:
                // Emergency park - waiting to reach parking spot
                if (!follower.isBusy()) {
                    setPathState(100);
                }
                break;

            case 100:
                // Robot parked
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

    /** Execute the shooting sequence using webcam color detection like Tele.java **/
    private void executeShootSequence() {
        if (currentBallIndex >= 3) {
            return;
        }

        double rpmLowerBound = TARGET_RPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = TARGET_RPM * (1.0 + RPM_TOLERANCE_PERCENT);
        boolean rpmReady = shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;

        // Determine which side to shoot from using webcam color detection
        // Wait 300ms for ball to settle/intake to move it before scanning to prevent errors
        if (!shootSideDecided && shootTimer.milliseconds() > 400) {
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

            // Open appropriate trapdoor and kicker arm (only if ball not yet detected)
            if (kickLeft && kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
                leftTrapdoor.setPosition(0.2);   // Open
                rightTrapdoor.setPosition(0.0);  // Open
                bothTrapdoorsOpen = true;
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
            } else if (kickLeft && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
                // Ball is on LEFT side, open RIGHT trapdoor
                leftTrapdoor.setPosition(0.0);   // Closed
                rightTrapdoor.setPosition(0.0);  // Open
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = true;
            } else if (kickRight && !ballDetectedWaitingForRpm && !distanceCheckPassed) {
                // Ball is on RIGHT side, open LEFT trapdoor
                leftTrapdoor.setPosition(0.2);   // Open
                rightTrapdoor.setPosition(0.2);  // Closed
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = false;
            } else {
                // Ball already detected - keep closed
                leftTrapdoor.setPosition(0.1);   // Closed
                rightTrapdoor.setPosition(0.1);  // Closed
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;
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
        if (!distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            // Ball detected - close trapdoors immediately (regardless of RPM)
            if (distance < 20 && !ballDetectedWaitingForRpm) {
                // Close both trapdoors immediately
                leftTrapdoor.setPosition(0.1);   // Closed
                rightTrapdoor.setPosition(0.1);  // Closed
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
                bothTrapdoorsOpen = false;
                ballDetectedWaitingForRpm = true;
            }

            // Once ball detected and RPM is ready, proceed with transfers and intake
            if (ballDetectedWaitingForRpm && rpmReady) {
                distanceCheckPassed = true;
                ballDetectedWaitingForRpm = false;

                leftTransfer.setPosition(0.0);   // UP
                rightTransfer.setPosition(0.5);  // UP
                transfersUp = true;
                intake.setPower(-1.0);
                intakeReversed = false;
                transferTimer.reset();


                // Shot is being fired
                shotFired = true;
                shotFiredTimer.reset();
            }
        }

        // If distance check passed but RPM wasn't ready, keep checking
        if (distanceCheckPassed && !shotFired && rpmReady) {
            leftTransfer.setPosition(0.0);   // UP
            rightTransfer.setPosition(0.5);  // UP
            transfersUp = true;
            intake.setPower(-1.0);

            // Close both trapdoors
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

        // End sequence 750ms after firing - transfers go back down and next trapdoor opens
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
        telemetry.addData("Time Remaining", String.format("%.1f s", 30.0 - opmodeTimer.getElapsedTimeSeconds()));

        // AprilTag detection
        if (detectedAprilTagId != -1) {
            telemetry.addData("AprilTag ID (LOCKED)", detectedAprilTagId);
            telemetry.addData("Ball Order", "" + ballOrder[0] + ballOrder[1] + ballOrder[2]);
        } else {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                telemetry.addData("AprilTag ID", blocks[0].id);
            } else {
                telemetry.addData("AprilTag", "None detected");
            }
        }

        // Shooter status
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", shooterRPM);
        // telemetry.addData("RPM Ready", rpmReady ? "YES" : "NO");

        // Shooting sequence status
        if (pathState == 3 || pathState == 4 || pathState == 11 || pathState == 17) {
            telemetry.addData("--- SHOOTING ---", "");
            telemetry.addData("Current Ball", currentBallIndex + 1);
            telemetry.addData("Target Color", currentBallIndex < 3 ? (ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN") : "Done");

            ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
            telemetry.addData("Left Color (Webcam)", colorResult.leftColor);
            telemetry.addData("Right Color (Webcam)", colorResult.rightColor);

            double transferDistance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Transfer Distance", String.format("%.1f", transferDistance) + "cm" +
                    (transferDistance < 20 ? " BALL DETECTED" : " NO BALL"));

            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
            telemetry.addData("Shoot Timer", String.format("%.2f", shootTimer.seconds()) + "s");
            telemetry.addData("Kick Left", kickLeft);
            telemetry.addData("Kick Right", kickRight);
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
        leftHoodAdjustment.setPosition(0.05);
        rightHoodAdjustment.setPosition(0.25);

        // Initialize velocity timer
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

