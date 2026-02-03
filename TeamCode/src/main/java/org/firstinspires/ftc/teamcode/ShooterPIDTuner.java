package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter PID Tuner", group = "Tuning")
public class ShooterPIDTuner extends OpMode {

    private DcMotor shooter;

    // PID Constants (starting values)
    private double kP = 0.0002;
    private double kD = 0.00001;
    private double kF = 1.0 / 4900.0;  // Feedforward based on max RPM

    // Increment values (can be changed by factor of 10)
    private double pIncrement = 0.0001;
    private double dIncrement = 0.00001;

    // Target RPM options
    private static final double RPM_LOW = 1800.0;
    private static final double RPM_HIGH = 2000.0;
    private double targetRPM = 0.0;
    private boolean usingHighRPM = false;

    // Shooter state
    private boolean shooterEnabled = false;
    private double shooterRPM = 0.0;
    private double shooterPower = 0.0;
    private int lastEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();

    // PID state
    private double lastError = 0.0;
    private static final double TICKS_PER_REV = 28.0;

    // Button debouncing
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    private boolean lastLeftTriggerState = false;
    private boolean lastRightTriggerState = false;

    // Which parameter is selected for tuning (0 = P, 1 = D)
    private int selectedParam = 0;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityTimer.reset();
        lastEncoderPosition = 0;

        telemetry.addData("Status", "Initialized - Press A to start shooter");
        telemetry.addData("Controls", "See telemetry for button mappings");
    }

    @Override
    public void loop() {
        // ========== CALCULATE SHOOTER RPM ==========
        int currentPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update every 20ms
            int deltaTicks = currentPosition - lastEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            shooterRPM = (ticksPerSecond / TICKS_PER_REV) * 60.0;
            lastEncoderPosition = currentPosition;
            velocityTimer.reset();
        }

        // ========== BUTTON CONTROLS ==========

        // A Button: Toggle shooter ON/OFF
        if (gamepad1.a && !lastAState) {
            shooterEnabled = !shooterEnabled;
            if (!shooterEnabled) {
                shooter.setPower(0);
                lastError = 0;
            }
        }
        lastAState = gamepad1.a;

        // B Button: Toggle between 1800 and 2000 RPM
        if (gamepad1.b && !lastBState) {
            usingHighRPM = !usingHighRPM;
        }
        lastBState = gamepad1.b;

        // X Button: Select P for tuning
        if (gamepad1.x && !lastXState) {
            selectedParam = 0;
        }
        lastXState = gamepad1.x;

        // Y Button: Select D for tuning
        if (gamepad1.y && !lastYState) {
            selectedParam = 1;
        }
        lastYState = gamepad1.y;

        // D-Pad Up: Increase selected parameter
        if (gamepad1.dpad_up && !lastDpadUpState) {
            if (selectedParam == 0) {
                kP += pIncrement;
            } else {
                kD += dIncrement;
            }
        }
        lastDpadUpState = gamepad1.dpad_up;

        // D-Pad Down: Decrease selected parameter
        if (gamepad1.dpad_down && !lastDpadDownState) {
            if (selectedParam == 0) {
                kP = Math.max(0, kP - pIncrement);
            } else {
                kD = Math.max(0, kD - dIncrement);
            }
        }
        lastDpadDownState = gamepad1.dpad_down;

        // D-Pad Right: Increase increment by 10x
        if (gamepad1.dpad_right && !lastDpadRightState) {
            if (selectedParam == 0) {
                pIncrement *= 10;
            } else {
                dIncrement *= 10;
            }
        }
        lastDpadRightState = gamepad1.dpad_right;

        // D-Pad Left: Decrease increment by 10x
        if (gamepad1.dpad_left && !lastDpadLeftState) {
            if (selectedParam == 0) {
                pIncrement /= 10;
            } else {
                dIncrement /= 10;
            }
        }
        lastDpadLeftState = gamepad1.dpad_left;

        // Left Bumper: Quick set P to 0
        if (gamepad1.left_bumper && !lastLeftBumperState) {
            if (selectedParam == 0) {
                kP = 0;
            } else {
                kD = 0;
            }
        }
        lastLeftBumperState = gamepad1.left_bumper;

        // Right Bumper: Reset selected param to default
        if (gamepad1.right_bumper && !lastRightBumperState) {
            if (selectedParam == 0) {
                kP = 0.0002;
                pIncrement = 0.0001;
            } else {
                kD = 0.00001;
                dIncrement = 0.00001;
            }
        }
        lastRightBumperState = gamepad1.right_bumper;

        // ========== PID CONTROL ==========
        if (shooterEnabled) {
            targetRPM = usingHighRPM ? RPM_HIGH : RPM_LOW;

            double error = targetRPM - shooterRPM;

            // Calculate derivative
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            // Feedforward
            double feedforward = targetRPM * kF;

            // PID output (P + D + F)
            double pidOutput = (kP * error) + (kD * derivative);
            shooterPower = feedforward + pidOutput;
            shooterPower = Range.clip(shooterPower, 0.0, 1.0);

            shooter.setPower(shooterPower);
        } else {
            targetRPM = 0;
            shooterPower = 0;
        }

        // ========== TELEMETRY ==========
        telemetry.addData("=== SHOOTER PID TUNER ===", "");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF (Press A)");
        telemetry.addData("Target RPM", "%.0f (%s) - Press B to switch", targetRPM, usingHighRPM ? "HIGH" : "LOW");
        telemetry.addData("Actual RPM", "%.0f", shooterRPM);
        telemetry.addData("RPM Error", "%.0f", targetRPM - shooterRPM);
        telemetry.addData("Power", "%.3f", shooterPower);

        telemetry.addData("", "");
        telemetry.addData("=== PID VALUES ===", "");
        telemetry.addData(selectedParam == 0 ? ">> kP" : "   kP", "%.7f (increment: %.7f)", kP, pIncrement);
        telemetry.addData(selectedParam == 1 ? ">> kD" : "   kD", "%.7f (increment: %.7f)", kD, dIncrement);
        telemetry.addData("   kF", "%.7f (fixed)", kF);

        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Toggle Shooter ON/OFF");
        telemetry.addData("B", "Switch RPM (1800/2000)");
        telemetry.addData("X", "Select P for tuning");
        telemetry.addData("Y", "Select D for tuning");
        telemetry.addData("D-Pad Up/Down", "Increase/Decrease value");
        telemetry.addData("D-Pad Left/Right", "Decrease/Increase increment (10x)");
        telemetry.addData("Left Bumper", "Set selected to 0");
        telemetry.addData("Right Bumper", "Reset selected to default");

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}

