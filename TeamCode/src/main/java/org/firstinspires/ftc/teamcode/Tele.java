package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

    // Toggle states for trapdoors
    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean lastSquareState = false;
    private boolean lastCircleState = false;

    // Toggle states for kicker arms (gamepad1 dpad)
    private boolean leftKickerArmOpen = false;
    private boolean rightKickerArmOpen = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    // Toggle state for transfers (gamepad1)
    private boolean transfersOpen = false;
    private boolean lastCrossState = false;

    // Toggle state for hood adjustment (gamepad1)
    private boolean hoodAdjustmentOpen = false;
    private boolean lastTriangleState = false;

    // Timer for right bumper sequence
    private ElapsedTime rightBumperTimer = new ElapsedTime();
    private boolean rightBumperSequenceActive = false;
    private boolean lastRightBumperState = false;

    // Debug timing for left trigger release vs left dpad press
    private ElapsedTime debugTimer = new ElapsedTime();
    private boolean lastLeftTriggerPressed = false;
    private boolean lastDpadLeftForDebug = false;
    private double leftTriggerReleaseTime = -1;
    private double leftDpadPressTime = -1;
    private double timeDifference = 0;


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
        telemetry.addData("Status", "Initialized");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // Get the robot's heading from the IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Get joystick inputs
        double y = -gamepad1.left_stick_y;  // Forward/backward (pushing stick forward gives negative value)
        double x = gamepad1.left_stick_x;   // Left/right strafe
        double yaw = gamepad1.right_stick_x; // Rotation (stays robot-centric)

        // Rotate the joystick inputs by the negative of the robot's heading
        // This makes forward on joystick always move the robot away from driver
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Use rotated values for axial and lateral movement
        float axial = (float) rotY;
        float lateral = (float) rotX;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        float leftFrontPower = axial + lateral + (float) yaw;
        float rightFrontPower = axial - lateral - (float) yaw;
        float leftBackPower = axial - lateral + (float) yaw;
        float rightBackPower = axial + lateral - (float) yaw;

        // clip the right/left values so that the values never exceed +/- 1
        rightFrontPower = (float) Range.clip(rightFrontPower, -0.8, 0.8);
        leftFrontPower = (float) Range.clip(leftFrontPower, -0.8, 0.8);
        leftBackPower = (float) Range.clip(leftBackPower, -0.8, 0.8);
        rightBackPower = (float) Range.clip(rightBackPower, -0.8, 0.8);

        // write the values to the motors
        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Only use normal shooter/intake controls if sequence is not active
        if (!rightBumperSequenceActive) {
            if (gamepad1.right_trigger != 0) {
                shooter.setPower(0.5);
            } else {
                shooter.setPower(0.0);
            }

            if (gamepad1.left_trigger != 0) {
                intake.setPower(-1.0); // Intake in
            } else if (gamepad1.left_bumper) {
                intake.setPower(1.0); // Intake out
            } else {
                intake.setPower(0.0);
            }
        }

        // ORIGINAL TRAPDOOR CONTROLS - COMMENTED OUT FOR TESTING
        // if (gamepad1.square && !lastSquareState) {
        //     // Toggle left trapdoor on button press
        //     leftTrapdoorOpen = !leftTrapdoorOpen;
        //     if (leftTrapdoorOpen) {
        //         // Left trapdoor opened (0.0), put right trapdoor to special position 0.0
        //         leftTrapdoor.setPosition(0.0);
        //         rightTrapdoor.setPosition(0.0);
        //     } else {
        //         // Left trapdoor closed, close right trapdoor as well (0.1)
        //         leftTrapdoor.setPosition(0.1);
        //         rightTrapdoor.setPosition(0.1);
        //     }
        // }
        // lastSquareState = gamepad1.square;

        // if (gamepad1.circle && !lastCircleState) {
        //     // Toggle right trapdoor on button press
        //     rightTrapdoorOpen = !rightTrapdoorOpen;
        //     if (rightTrapdoorOpen) {
        //         // Right trapdoor opened (0.2), put left trapdoor to 0.2
        //         rightTrapdoor.setPosition(0.2);
        //         leftTrapdoor.setPosition(0.2);
        //     } else {
        //         // Right trapdoor closed, put left trapdoor to 0.1
        //         rightTrapdoor.setPosition(0.1);
        //         leftTrapdoor.setPosition(0.1);
        //     }
        // }
        // lastCircleState = gamepad1.circle;

        // TEST TRAPDOOR CONTROLS - Square opens both, Circle closes both
        if (gamepad1.square) {
            // Open both trapdoors
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.2);
        }
        if (gamepad1.circle) {
            // Close both trapdoors
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
        }

        if (gamepad1.cross && !lastCrossState) {
            // Toggle both transfers on button press
            transfersOpen = !transfersOpen;
            if (transfersOpen) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
            } else {
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
            }
        }
        lastCrossState = gamepad1.cross;

        if (gamepad1.dpad_left && !lastDpadLeftState) {
            // Toggle left kicker arm on button press
            leftKickerArmOpen = !leftKickerArmOpen;
            leftKickerArm.setPosition(leftKickerArmOpen ? 0.5 : 0.0);
        }
        lastDpadLeftState = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRightState) {
            // Toggle right kicker arm on button press
            rightKickerArmOpen = !rightKickerArmOpen;
            rightKickerArm.setPosition(rightKickerArmOpen ? 0.075 : 0.5);
        }
        lastDpadRightState = gamepad1.dpad_right;

        if (gamepad1.triangle && !lastTriangleState) {
            // Toggle both hood adjustments on button press
            hoodAdjustmentOpen = !hoodAdjustmentOpen;
            if (hoodAdjustmentOpen) {
                leftHoodAdjustment.setPosition(0.3);
                rightHoodAdjustment.setPosition(0.0);
            } else {
                leftHoodAdjustment.setPosition(0.0);
                rightHoodAdjustment.setPosition(0.3);
            }
        }
        lastTriangleState = gamepad1.triangle;

        if (gamepad1.dpad_up) {
            leftLift.setPower(-1.0);
            rightLift.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            leftLift.setPower(1.0);
            rightLift.setPower(-1.0);
        } else {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }

        // Right bumper sequence - press once to start the sequence
        if (gamepad1.right_bumper && !lastRightBumperState && !rightBumperSequenceActive) {
            // Start the sequence
            rightBumperSequenceActive = true;
            rightBumperTimer.reset();

            // Immediate servo actions
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        }
        lastRightBumperState = gamepad1.right_bumper;

        // Execute the sequence steps based on timer
        if (rightBumperSequenceActive) {
            // Keep shooter running throughout the sequence
            shooter.setPower(0.5);

            // Intake runs for first 900ms, pauses 900-1000ms, then resumes until end
            if (rightBumperTimer.milliseconds() < 900) {
                intake.setPower(-1.0);
            } else if (rightBumperTimer.milliseconds() < 1400) {
                intake.setPower(0.0);
            } else {
                intake.setPower(-1.0);
            }

            // After 500ms delay, set the kicker arm position
            if (rightBumperTimer.milliseconds() >= 500) {
                leftKickerArm.setPosition(0.5);
            }

            // After 3500ms, set the transfers
            if (rightBumperTimer.milliseconds() >= 3500) {
                rightTransfer.setPosition(0.0);
                leftTransfer.setPosition(0.5);
            }

            // End the sequence and reset positions (e.g., after 4000ms)
            if (rightBumperTimer.milliseconds() >= 4500) {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                leftKickerArm.setPosition(0.0);
                rightTransfer.setPosition(0.5);
                leftTransfer.setPosition(0.0);

                // Turn off shooter and intake at end of sequence
                shooter.setPower(0.0);

                leftTrapdoorOpen = false;
                leftKickerArmOpen = false;
                rightBumperSequenceActive = false;
            }
        }

        // Debug telemetry for timing analysis
        telemetry.addData("Left Trigger Pressed", gamepad1.left_trigger != 0);
        telemetry.addData("Last Left Trigger Pressed", lastLeftTriggerPressed);
        telemetry.addData("Left Trigger Release Time", leftTriggerReleaseTime);
        telemetry.addData("Left Dpad Press Time", leftDpadPressTime);
        telemetry.addData("Time Difference", timeDifference);

        // Color sensor telemetry (HSV)
        float[] leftHSV = new float[3];
        float[] rightHSV = new float[3];
        Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
        Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);
        telemetry.addData("Left Color Sensor (H,S,V,D)", "(%.1f, %.2f, %.2f, %.3f)",
                leftHSV[0], leftHSV[1], leftHSV[2], ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM));
        telemetry.addData("Right Color Sensor (H,S,V,D)", "(%.1f, %.2f, %.2f, %.3f)",
                rightHSV[0], rightHSV[1], rightHSV[2], ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM));
        telemetry.update();

        // Update debug timer and calculate time difference between left trigger release and left dpad press
        if (gamepad1.left_trigger == 0 && lastLeftTriggerPressed) {
            // Left trigger was just released
            leftTriggerReleaseTime = debugTimer.milliseconds();
        }

        if (gamepad1.dpad_left && !lastDpadLeftForDebug) {
            // Left dpad was just pressed
            leftDpadPressTime = debugTimer.milliseconds();
        }

        // Calculate time difference if both events have been registered
        // Negative = trigger released first, Positive = dpad pressed first
        if (leftTriggerReleaseTime != -1 && leftDpadPressTime != -1) {
            timeDifference = leftTriggerReleaseTime - leftDpadPressTime;
        }

        // Update last pressed states
        lastLeftTriggerPressed = gamepad1.left_trigger != 0;
        lastDpadLeftForDebug = gamepad1.dpad_left;
    }
}