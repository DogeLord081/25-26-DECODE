package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;

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
    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorRight;

    // Shooter constants
    private static final double TARGET_RPM = 1800.0;
    private static final double RPM_TOLERANCE_PERCENT = 0.05;  // 5% tolerance
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Shooter state tracking
    private double shooterRPM = 0.0;
    private int lastShooterEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // Shooting sequence state
    private int currentBallIndex = 0;  // 0, 1, 2 for the three balls
    private boolean shotFired = false;
    private boolean distanceCheckPassed = false;
    private boolean firstBallPreloaded = false;  // Track if first ball was loaded during turning
    private ElapsedTime shotFiredTimer = new ElapsedTime();  // Track time since shot was fired
    private boolean currentShootLeft = false;  // Cached value for which side to shoot from
    private boolean shootSideDecided = false;  // Track if we've decided which side to shoot from

    // Ball order based on AprilTag (P = Purple/Left, G = Green/Right)
    // ID 1: PPG, ID 2: PGP, ID 3: GPP
    private char[] ballOrder = new char[3];

    /* Define poses for the autonomous routine */
    private final Pose startPose = new Pose(57.328, 134.590, Math.toRadians(270));
    private final Pose scorePose = new Pose(52.328, 115.18032786885244, Math.toRadians(250));
    private final Pose afterScanPose = new Pose(52.328, 100.18032786885244, Math.toRadians(325));

    /* Path and PathChain declarations */
    private Path scorePreload;
    private Path afterScanPath;

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
    }

    /** Main state machine for autonomous path progression **/
    public void autonomousPathUpdate() {
        // Update shooter RPM tracking
        updateShooterRPM();

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
                if (!follower.isBusy()) {
                    /* Move to state 2 to scan for AprilTag */
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
                    // ID 1: PPG, ID 2: PGP, ID 3: GPP
                    // P = Purple (left side), G = Green (right side)
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
                /* Wait for the robot to finish turning while also loading first ball */
                controlShooterPID();  // Keep controlling shooter

                // Run the first ball loading sequence in parallel with turning
                executeFirstBallLoading();

                if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                    /* Turn complete, continue to shooting state */
                    // If first ball was pre-loaded, mark it so we skip loading phase
                    if (distanceCheckPassed) {
                        firstBallPreloaded = true;
                    }
                    // Reset shoot timer for the shooting phase
                    shootTimer.reset();
                    setPathState(4);
                }
                break;
            case 4:
                /* Shooting state - shoot balls in order */
                controlShooterPID();  // Keep controlling shooter
                executeShootingSequence();
                break;
            case 5:
                /* All balls shot, done */
                shooter.setPower(0);
                intake.setPower(0);
                setPathState(-1);
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

    /** Simple proportional control for shooter RPM **/
    private void controlShooterPID() {
        double error = TARGET_RPM - shooterRPM;
        double kP = 0.0003;
        double kF = 1.0 / 4900.0;  // Feedforward based on max RPM

        double power = (TARGET_RPM * kF) + (error * kP);
        power = Math.max(0.0, Math.min(1.0, power));
        shooter.setPower(power);
    }

    /** Check if RPM is within tolerance **/
    private boolean isRPMReady() {
        double rpmLowerBound = TARGET_RPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = TARGET_RPM * (1.0 + RPM_TOLERANCE_PERCENT);
        return shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;
    }

    /** Execute the shooting sequence for current ball **/
    private void executeShootingSequence() {
        if (currentBallIndex >= 3) {
            // All balls shot
            setPathState(5);
            return;
        }

        // Only decide which side to shoot from ONCE per ball (at the start)
        if (!shootSideDecided) {
            // Get current target ball color (P = Purple, G = Green)
            char targetColor = ballOrder[currentBallIndex];

            // Use color sensors to detect which side has the target color
            // Purple if hue > 175, otherwise Green
            float[] leftHSV = new float[3];
            float[] rightHSV = new float[3];
            Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
            Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);

            // Check proximity sensors to see if ball is present on each side
            double leftProximity = ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM);
            double rightProximity = ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM);
            boolean leftHasBall = leftProximity < 6.5;
            boolean rightHasBall = rightProximity < 6.5;

            boolean leftIsPurple = leftHSV[0] > 175;
            boolean rightIsPurple = rightHSV[0] > 175;

            // Determine which side to shoot from based on target color AND ball presence
            currentShootLeft = false;
            if (targetColor == 'P') {
                // Looking for purple
                if (leftHasBall && leftIsPurple) {
                    currentShootLeft = true;
                } else if (rightHasBall && rightIsPurple) {
                    currentShootLeft = false;
                } else if (leftHasBall) {
                    // Fallback: if only left has ball, use left
                    currentShootLeft = true;
                }
            } else {
                // Looking for green (not purple)
                if (leftHasBall && !leftIsPurple) {
                    currentShootLeft = true;
                } else if (rightHasBall && !rightIsPurple) {
                    currentShootLeft = false;
                } else if (leftHasBall) {
                    // Fallback: if only left has ball, use left
                    currentShootLeft = true;
                }
            }
            shootSideDecided = true;
        }

        double elapsed = shootTimer.seconds();

        // If first ball was pre-loaded, skip directly to shooting phase
        if (currentBallIndex == 0 && firstBallPreloaded && distanceCheckPassed && !shotFired) {
            // First ball is already in transfer, just wait for RPM and shoot
            if (isRPMReady()) {
                // Open transfer to shoot
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                shotFired = true;
                shotFiredTimer.reset();
            }
        } else if (!shotFired && !(currentBallIndex == 0 && firstBallPreloaded)) {
            // Normal loading sequence for balls 2 and 3 (or if first ball wasn't pre-loaded)

            if (elapsed < 0.5) {
                // Open appropriate trapdoor and run intake
                intake.setPower(-1.0);
                if (currentShootLeft) {
                    leftTrapdoor.setPosition(0.0);
                    rightTrapdoor.setPosition(0.0);
                } else {
                    leftTrapdoor.setPosition(0.2);
                    rightTrapdoor.setPosition(0.2);
                }
            } else if (elapsed < 1.5) {
                // Keep intake running and activate kicker arm
                intake.setPower(-1.0);
                if (currentShootLeft) {
                    leftKickerArm.setPosition(0.5);
                } else {
                    rightKickerArm.setPosition(0.075);
                }
            } else if (!distanceCheckPassed) {
                // Check distance sensor - if ball not detected, restart this ball's sequence
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                if (distance > 20) {
                    // Ball not detected, restart the cycle for this ball
                    restartBallSequence(currentShootLeft);
                    return;
                } else {
                    // Ball detected, proceed with transfers
                    distanceCheckPassed = true;
                    // Close trapdoors
                    leftTrapdoor.setPosition(0.1);
                    rightTrapdoor.setPosition(0.1);
                    intake.setPower(0);  // Stop intake

                    // Immediately try to shoot if RPM is ready (don't wait for next loop)
                    if (isRPMReady()) {
                        leftTransfer.setPosition(0.5);
                        rightTransfer.setPosition(0.0);
                        shotFired = true;
                        shotFiredTimer.reset();
                    }
                }
            }
        }

        // If distance check passed but shot not yet fired, keep checking RPM
        if (distanceCheckPassed && !shotFired) {
            if (isRPMReady()) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                shotFired = true;
                shotFiredTimer.reset();
            }
        }

        // After shot is fired, wait 1 second then move to next ball
        if (shotFired && shotFiredTimer.seconds() >= 1.0) {
            // Reset for next ball
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);

            // Move to next ball
            currentBallIndex++;
            shotFired = false;
            distanceCheckPassed = false;
            firstBallPreloaded = false;  // Clear the preload flag
            shootSideDecided = false;  // Reset so we decide side for next ball
            shootTimer.reset();
        }
    }

    /** Restart the sequence for the current ball when distance sensor doesn't detect ball **/
    private void restartBallSequence(boolean shootLeft) {
        // Reset the timer to restart the cycle from the beginning
        shootTimer.reset();
        distanceCheckPassed = false;
        shotFired = false;

        // Re-open the appropriate trapdoor
        if (shootLeft) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        }

        // Reset transfer positions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /** Load the first ball into the transfer while turning (don't wait for RPM, just get ball ready) **/
    private void executeFirstBallLoading() {
        // Only run this for the first ball and if we haven't already passed the distance check
        if (currentBallIndex != 0 || distanceCheckPassed) {
            return;
        }

        // Only decide which side to shoot from ONCE (at the start of loading)
        if (!shootSideDecided) {
            // Get current target ball color (P = Purple, G = Green)
            char targetColor = ballOrder[0];

            // Use color sensors to detect which side has the target color
            float[] leftHSV = new float[3];
            float[] rightHSV = new float[3];
            Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
            Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);

            // Check proximity sensors to see if ball is present on each side
            double leftProximity = ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM);
            double rightProximity = ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM);
            boolean leftHasBall = leftProximity < 6.5;
            boolean rightHasBall = rightProximity < 6.5;

            boolean leftIsPurple = leftHSV[0] > 175;

            // Determine which side to shoot from
            currentShootLeft = false;
            if (targetColor == 'P') {
                if (leftHasBall && leftIsPurple) {
                    currentShootLeft = true;
                } else if (rightHasBall && !leftIsPurple) {
                    currentShootLeft = false;
                } else if (leftHasBall) {
                    currentShootLeft = true;
                }
            } else {
                if (leftHasBall && !leftIsPurple) {
                    currentShootLeft = true;
                } else if (rightHasBall && leftIsPurple) {
                    currentShootLeft = false;
                } else if (leftHasBall) {
                    currentShootLeft = true;
                }
            }
            shootSideDecided = true;
        }

        double elapsed = shootTimer.seconds();

        // Loading sequence (same as shooting but stop before firing)
        if (elapsed < 0.5) {
            // Open appropriate trapdoor and run intake
            intake.setPower(-1.0);
            if (currentShootLeft) {
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.0);
            } else {
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.2);
            }
        } else if (elapsed < 1.5) {
            // Keep intake running and activate kicker arm
            intake.setPower(-1.0);
            if (currentShootLeft) {
                leftKickerArm.setPosition(0.5);
            } else {
                rightKickerArm.setPosition(0.075);
            }
        } else {
            // Check distance sensor - if ball not detected, restart
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance > 20) {
                // Ball not detected, restart the loading
                restartBallSequence(currentShootLeft);
            } else {
                // Ball detected in transfer - ready to shoot when RPM is up
                distanceCheckPassed = true;
                // Close trapdoors
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                intake.setPower(0);  // Stop intake
            }
        }
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
        if (pathState == 4) {
            telemetry.addData("--- SHOOTING ---", "");
            telemetry.addData("Current Ball", currentBallIndex + 1);
            telemetry.addData("Target Color", currentBallIndex < 3 ? (ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN") : "Done");

            // Show color sensor readings
            float[] leftHSV = new float[3];
            float[] rightHSV = new float[3];
            Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
            Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);

            // Show proximity readings
            double leftProximity = ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM);
            double rightProximity = ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM);

            telemetry.addData("Left Sensor", (leftHSV[0] > 175 ? "PURPLE" : "GREEN") +
                " (prox: " + String.format("%.1f", leftProximity) + "cm" +
                (leftProximity < 6.5 ? " BALL" : "") + ")");
            telemetry.addData("Right Sensor", (rightHSV[0] > 175 ? "PURPLE" : "GREEN") +
                " (prox: " + String.format("%.1f", rightProximity) + "cm" +
                (rightProximity < 6.5 ? " BALL" : "") + ")");

            // Show transfer distance sensor
            double transferDistance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Transfer Distance", String.format("%.1f", transferDistance) + "cm" +
                (transferDistance < 20 ? " BALL DETECTED" : " NO BALL - WILL RETRY"));

            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
            telemetry.addData("Shoot Timer", String.format("%.2f", shootTimer.seconds()) + "s");
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
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftKickerArm = hardwareMap.get(Servo.class, "leftKickerArm");
        rightKickerArm = hardwareMap.get(Servo.class, "rightKickerArm");
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        // Initialize distance sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Initialize color sensors
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");

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
