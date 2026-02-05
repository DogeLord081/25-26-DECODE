package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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

@Autonomous(name = "DECODE 25-26 Auto")
public class Auto extends OpMode {

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

    // Shooter constants (matching Tele.java PIDF)
    private static final double TARGET_RPM = 1900.0;
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

    // Intake reversal state (for new shooting sequence)
    private boolean intakeReversed = false;
    private ElapsedTime intakeReversalTimer = new ElapsedTime();
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transfersUp = false;

    // Intake pulse timer for -1/0 pattern (250ms each)
    private ElapsedTime intakePulseTimer = new ElapsedTime();

    // Restart intake pulse state
    private boolean restartIntakePulseActive = false;
    private ElapsedTime restartIntakePulseTimer = new ElapsedTime();

    // Shooting sequence state
    private int currentBallIndex = 0;  // 0, 1, 2 for the three balls
    private boolean shotFired = false;
    private boolean distanceCheckPassed = false;
    private boolean firstBallPreloaded = false;  // Track if first ball was loaded during turning
    private ElapsedTime shotFiredTimer = new ElapsedTime();  // Track time since shot was fired
    private boolean currentShootLeft = false;  // Cached value for which side to shoot from
    private boolean shootSideDecided = false;  // Track if we've decided which side to shoot from
    private boolean kickLeft = false;  // Track if we're kicking from left side
    private boolean kickRight = false;  // Track if we're kicking from right side
    private boolean singleBallMode = false;  // Track if in single ball mode
    private boolean trapdoorsOpenedForSingleBall = false;  // Track trapdoor state
    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean bothTrapdoorsOpen = false;
    private boolean leftKickerArmOpen = false;
    private boolean rightKickerArmOpen = false;



    // Ball order based on AprilTag (P = Purple/Left, G = Green/Right)
    // ID 1: PPG, ID 2: PGP, ID 3: GPP
    private char[] ballOrder = new char[3];

    /* Define poses for the autonomous routine */
    private final Pose startPose = new Pose(16.63157142857142, 141.60802107728338, Math.toRadians(325));
    private final Pose scorePose = new Pose(52.328, 115.18032786885244, Math.toRadians(250));
    private final Pose afterScanPose = new Pose(52.328, 100.18032786885244, Math.toRadians(328));
    private final Pose afterShootPose = new Pose(41.55750819672132, 73.73770491803278, Math.toRadians(180));
    private final Pose intakeBallsPose = new Pose(8.55750819672132, 73.73770491803278, Math.toRadians(180));

    /* Path and PathChain declarations */
    private Path scorePreload;
    private Path afterScanPath;
    private Path afterShootPath;
    private Path intakeBallsPath;

    // New Return Paths
    private Path returnToAfterShootPath;
    private Path returnToScanPath;

    /* AprilTag scanning state */
    private int detectedAprilTagId = -1;  // -1 means not yet detected

    /** Build all paths for the autonomous routine **/
    public void buildPaths() {
        /* This is our scorePreload path using a BezierLine (straight line) */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Path after scanning AprilTag */
        afterScanPath = new Path(new BezierLine(scorePose, afterScanPose));
        afterScanPath.setLinearHeadingInterpolation(scorePose.getHeading(), afterScanPose.getHeading());


        /* Path after shooting all balls */
        afterShootPath = new Path(new BezierLine(afterScanPose, afterShootPose));
        afterShootPath.setLinearHeadingInterpolation(afterScanPose.getHeading(), afterShootPose.getHeading());

        intakeBallsPath = new Path(new BezierLine(afterShootPose, intakeBallsPose));
        intakeBallsPath.setLinearHeadingInterpolation(afterShootPose.getHeading(), intakeBallsPose.getHeading());

        // Return trip paths
        returnToAfterShootPath = new Path(new BezierLine(intakeBallsPose, afterShootPose));
        returnToAfterShootPath.setLinearHeadingInterpolation(intakeBallsPose.getHeading(), afterShootPose.getHeading());

        returnToScanPath = new Path(new BezierCurve(
                afterShootPose, // Start Point
                new Pose(55.0, 85.0, 0), // Control Point (Heading is ignored)
                afterScanPose // End Point
        ));
        returnToScanPath.setLinearHeadingInterpolation(afterShootPose.getHeading(), afterScanPose.getHeading());
    }

    /** Main state machine for autonomous path progression **/
    public void autonomousPathUpdate() {
        // Update shooter RPM tracking
        updateShooterRPM();

        // *** EMERGENCY PARK LOGIC ***
        // If we have crossed 27 seconds and aren't already parking, abort and move to park
        // We check pathState != 99 and != 100 to ensure we don't re-trigger this once started
        if (opmodeTimer.getElapsedTimeSeconds() > 28.5 && pathState != 99 && pathState != 100) {
            // Shut down mechanisms
            shooter.setPower(0);
            intake.setPower(0);

            // Create a dynamic path from WHEREVER we are right now to the parking spot
            // This ensures we don't try to jump to the start of a pre-recorded path
            Pose currentPose = follower.getPose();
            Path emergencyParkPath = new Path(new BezierLine(currentPose, afterShootPose));
            emergencyParkPath.setLinearHeadingInterpolation(currentPose.getHeading(), afterShootPose.getHeading());

            // Follow it immediately
            follower.followPath(emergencyParkPath);
            setPathState(99);
            return; // Exit the function so we don't execute other cases below
        }

        switch (pathState) {
            case 0:
                // Start shooter spinning to target RPM
                shooter.setPower(0.5);  // Initial power to start spinning up
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* Wait until the robot has finished following the path */
                controlShooterPID();  // Keep controlling shooter
                if (!follower.isBusy()) {                    /* Move to state 2 to scan for AprilTag */
                    setPathState(2);
                }
                break;
            case 2:
                /* Wait for AprilTag to be detected */
                controlShooterPID();  // Keep controlling shooter
                HuskyLens.Block[] blocks = huskyLens.blocks();
                if (blocks.length > 0 && detectedAprilTagId == -1) {
                    /* Freeze the detected AprilTag ID */
                    detectedAprilTagId = blocks[0].id;

                    /* Set ball order based on AprilTag ID */
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
                            ballOrder = new char[]{'P', 'P', 'G'};  // Default to ID 1 pattern
                            break;
                    }

                    /* Turn in place to afterScanPose heading using holdPoint */
                    follower.holdPoint(afterScanPose);

                    /* Start loading first ball immediately to save time */
                    currentBallIndex = 0;
                    shootTimer.reset();
                    shotFired = false;
                    distanceCheckPassed = false;

                    setPathState(3);
                }
                break;
            case 3:
                /* Wait for the robot to finish turning while loading and shooting first ball */
                controlShooterPID();  // Keep controlling shooter

                // Run the shooting sequence - this will load the ball and fire when RPM ready
                executeShootingSequence(false);

                // Move to next state when turn is complete (4 seconds) or all balls are shot
                if (pathTimer.getElapsedTimeSeconds() > 4.0 || currentBallIndex >= 3) {
                    if (currentBallIndex >= 3) {
                        setPathState(5);  // All balls shot, move on
                    } else {
                        setPathState(4);  // Continue shooting remaining balls
                    }
                }
                break;
            case 4:
                /* ROUND 1: Shooting state - shoot balls in order (Assumption based) */
                controlShooterPID();
                executeShootingSequence(false); // False = Don't use sensors, use fixed assumption
                break;
            case 5:
                /* All balls shot, go to afterShootPose */
                shooter.setPower(0);
                intake.setPower(0);
                follower.followPath(afterShootPath);
                setPathState(6);
                break;
            case 6:
                /* Wait for robot to reach afterShootPose */
                if (!follower.isBusy()) {
                    intake.setPower(-1.0);
                    follower.followPath(intakeBallsPath);
                    setPathState(7);
                }
                break;
            case 7:
                /* Wait for robot to reach intakeBallsPose */
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    // Start return trip
                    follower.followPath(returnToAfterShootPath);

                    // Prepare to preload first ball during the return trip
                    // Reset timers/flags so executeFirstBallLoading() can run while moving
                    shootTimer.reset();
                    shotFired = false;
                    distanceCheckPassed = false;
                    shootSideDecided = false;
                    currentBallIndex = 0; // We'll be preloading ball index 0

                    setPathState(8);
                }
                break;

            case 8:
                /* Traveling back to afterShootPose, start preloading */
                controlShooterPID();

                // While we're still traveling back, run shooting sequence to preload
                if (follower.isBusy()) {
                    executeShootingSequence(true);  // Use sensors for Round 2
                }

                if(!follower.isBusy()){
                    shooter.setPower(0.5); // Spin up shooter again
                    follower.followPath(returnToScanPath);
                    setPathState(9);
                }
                break;

            case 9:
                /* Traveling back to Shooting Position */
                controlShooterPID();

                // While traveling the final leg to the shooting point, continue shooting sequence
                if (follower.isBusy()) {
                    executeShootingSequence(true);  // Use sensors for Round 2
                }

                if(!follower.isBusy()){
                    follower.holdPoint(afterScanPose);
                    setPathState(10);
                }
                break;

            case 10:
                /* ROUND 2: Shooting with Color Sensors */
                controlShooterPID();
                executeShootingSequence(true); // True = Use sensors to find the correct ball

                // Check if all balls done for Round 2
                if(currentBallIndex >= 3) {
                    setPathState(11); // Done
                }
                break;

            case 11:
                // Final state - Park or Stop
                shooter.setPower(0);
                intake.setPower(0);
                setPathState(-1);
                break;

            // *** EMERGENCY PARK STATE ***
            case 99:
                // Waiting for the robot to reach parking spot
                if(!follower.isBusy()) {
                    setPathState(100);
                }
                break;
            case 100:
                // Robot parked, do nothing
                break;
        }
    }

    /** Update shooter RPM from encoder **/
    private void updateShooterRPM() {
        int currentShooterPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update velocity every 20ms
            int deltaTicks = currentShooterPosition - lastShooterEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            shooterRPM = (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
            lastShooterEncoderPosition = currentShooterPosition;
            velocityTimer.reset();
        }
    }

    /** Full PIDF control for shooter RPM (matching Tele.java) **/
    private void controlShooterPID() {
        double deltaTime = shooterPIDTimer.seconds();
        shooterPIDTimer.reset();

        // Prevent division by zero on first call
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

    /** Check if RPM is within tolerance **/
    private boolean isRPMReady() {
        double rpmLowerBound = TARGET_RPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = TARGET_RPM * (1.0 + RPM_TOLERANCE_PERCENT);
        return shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;
    }

    /** Execute the shooting sequence for current ball
     * @param useSensors - If true, uses color sensors to determine which side to shoot.
     * If false, assumes balls are in the pre-loaded spots.
     **/
    private void executeShootingSequence(boolean useSensors) {
        if (currentBallIndex >= 3) {
            // All balls shot
            // Case switching is handled in the main loop for Round 2,
            // but for Round 1 we might need to trigger state change here.
            if (!useSensors) setPathState(5);
            return;
        }

        // Determine which side to shoot from
        if (!shootSideDecided) {
            char targetColor = ballOrder[currentBallIndex];
            boolean isThirdBall = (currentBallIndex == 2);

            if (isThirdBall) {
                // Third ball - will use both trapdoors regardless
                currentShootLeft = true;
                kickLeft = true;
                kickRight = true;
            } else if (useSensors) {
                // *** WEBCAM COLOR DETECTION LOGIC ***
                ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
                String leftColor = colorResult.leftColor;
                String rightColor = colorResult.rightColor;

                if (targetColor == 'P') {
                    // We need Purple. If Left is Purple, shoot Left. Else shoot Right.
                    currentShootLeft = leftColor.equals("PURPLE");
                } else {
                    // We need Green. If Left is Green, shoot Left. Else shoot Right.
                    currentShootLeft = leftColor.equals("GREEN");
                }
                kickLeft = currentShootLeft;
                kickRight = !currentShootLeft;
            } else {
                // Standard Round 1 Logic (Assumption based on AprilTag pattern)
                currentShootLeft = (targetColor == 'P');
                kickLeft = currentShootLeft;
                kickRight = !currentShootLeft;
            }
            shootSideDecided = true;
            intakePulseTimer.reset();

            // Open appropriate trapdoor and kicker arm at sequence start
            boolean isBothKick = kickLeft && kickRight;
            if (isBothKick) {
                leftTrapdoor.setPosition(0.2);   // Open (right physical)
                rightTrapdoor.setPosition(0.0);  // Open (left physical)
                leftKickerArm.setPosition(0.5);
                rightKickerArm.setPosition(0.075);
                bothTrapdoorsOpen = true;
            } else if (kickLeft) {
                // Ball is on LEFT side, open RIGHT trapdoor (left physical)
                leftTrapdoor.setPosition(0.1);   // Closed
                rightTrapdoor.setPosition(0.0);  // Open
                rightKickerArm.setPosition(0.075);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = true;
            } else {
                // Ball is on RIGHT side, open LEFT trapdoor (right physical)
                leftTrapdoor.setPosition(0.2);   // Open
                rightTrapdoor.setPosition(0.1);  // Closed
                leftKickerArm.setPosition(0.5);
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = false;
            }
        }

        // Handle restart intake pulse completion (after 150ms, set intake back to -1.0 and transfers down)
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
        if (shootTimer.milliseconds() >= 200 && !distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance < 20) { // Ball Detected
                distanceCheckPassed = true;
                // Immediately move transfer UP (to bring ball to flywheel)
                leftTransfer.setPosition(0.0);   // UP position
                rightTransfer.setPosition(0.5);  // UP position
                transfersUp = true;

                // Keep intake at -1 to push ball up
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

        // TIMEOUT SAFETY (1.5 seconds)
        if (shootTimer.milliseconds() >= 1500 && !distanceCheckPassed) {
            restartBallSequence(currentShootLeft);
            return;
        }

        // IMMEDIATE FIRE TRIGGER
        // When distance check passes and RPM is ready, move transfer up and fire immediately
        if (distanceCheckPassed && isRPMReady() && !shotFired) {
            // Move transfer UP if not already up
            if (!transfersUp) {
                leftTransfer.setPosition(0.0);   // UP position
                rightTransfer.setPosition(0.5);  // UP position
                transfersUp = true;
                intake.setPower(-1.0);
            }
            // Ball is at flywheel (transfer UP), RPM is ready - shot is being fired!
            shotFired = true;
            shotFiredTimer.reset();
        }

        // End sequence 750ms after firing
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
            transfersUp = false;

            currentBallIndex++;
            shotFired = false;
            distanceCheckPassed = false;
            intakeReversed = false;
            firstBallPreloaded = false;
            shootSideDecided = false;
            restartIntakePulseActive = false;
            kickLeft = false;
            kickRight = false;
            singleBallMode = false;
            trapdoorsOpenedForSingleBall = false;
            shootTimer.reset();
        }
    }

    /** Restart the sequence for the current ball when distance sensor doesn't detect ball **/
    private void restartBallSequence(boolean shootLeft) {
        // Reset the timer to restart the cycle from the beginning
        shootTimer.reset();
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
            leftTrapdoor.setPosition(0.1);  // Closed
            rightTrapdoor.setPosition(0.0);  // Open
        } else if (kickRight) {
            // Right side
            leftTrapdoor.setPosition(0.2);  // Open
            rightTrapdoor.setPosition(0.1);  // Closed
        } else {
            // No match (backup - use left side)
            leftTrapdoor.setPosition(0.1);  // Closed
            rightTrapdoor.setPosition(0.0);  // Open
        }

        // Start intake pulse - spin forward to help ball drop
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

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Time Remaining", String.format("%.1f s", 30.0 - opmodeTimer.getElapsedTimeSeconds()));

        // HuskyLens AprilTag detection
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (detectedAprilTagId != -1) {
            // Show the frozen/locked AprilTag ID
            telemetry.addData("AprilTag ID (LOCKED)", detectedAprilTagId);
            telemetry.addData("Ball Order", "" + ballOrder[0] + ballOrder[1] + ballOrder[2]);
        } else if (blocks.length > 0) {
            telemetry.addData("AprilTag ID", blocks[0].id);
            telemetry.addData("AprilTag X", blocks[0].x);
            telemetry.addData("AprilTag Y", blocks[0].y);
        } else {
            telemetry.addData("AprilTag", "None detected");
        }

        // Shooter status
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", shooterRPM);
        telemetry.addData("RPM Ready", isRPMReady() ? "YES" : "NO");

        // Shooting sequence status
        if (pathState == 4 || pathState == 10) {
            telemetry.addData("--- SHOOTING ---", "");
            telemetry.addData("Round", (pathState >= 10) ? "2 (SENSORS)" : "1 (ASSUMED)");
            telemetry.addData("Current Ball", currentBallIndex + 1);
            telemetry.addData("Target Color", currentBallIndex < 3 ? (ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN") : "Done");

            // Show webcam color detection readings
            ColorRegionProcessor.AnalysisResult colorResult = colorProcessor.getAnalysis();
            telemetry.addData("Left Color (Webcam)", colorResult.leftColor);
            telemetry.addData("Right Color (Webcam)", colorResult.rightColor);

            // Show transfer distance sensor
            double transferDistance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Transfer Distance", String.format("%.1f", transferDistance) + "cm" +
                    (transferDistance < 20 ? " BALL DETECTED" : " NO BALL - WILL RETRY"));

            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
            telemetry.addData("Shoot Timer", String.format("%.2f", shootTimer.seconds()) + "s");
        } else if (pathState == 99 || pathState == 100) {
            telemetry.addData("--- EMERGENCY PARK ---", "MOVING TO PARK");
        }

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
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
        rightTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        leftTransfer = hardwareMap.get(Servo.class, "rightTransfer");
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

        // Set initial servo positions (closed/neutral)
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);

        // Set hood position for shooting
        leftHoodAdjustment.setPosition(0.3);
        rightHoodAdjustment.setPosition(0.0);  // Inverse relationship

        // Initialize velocity timer
        velocityTimer.reset();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
