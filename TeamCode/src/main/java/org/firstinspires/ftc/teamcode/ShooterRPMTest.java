package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Test OpMode to find:
 * 1. Maximum RPM of your shooter motor (run at 100% power)
 * 2. RPM at different power levels for lookup table calibration
 *
 * Controls:
 * - Right Bumper: Increase power by 5%
 * - Left Bumper: Decrease power by 5%
 * - Y Button: Set to 100% power (find max RPM)
 * - X Button: Set to 0% power (stop)
 * - A Button: Cycle through lookup table test powers (40%, 42.5%, 45%, 47%, 49%, 55%)
 * - B Button: Record current RPM to telemetry log
 *
 * INSTRUCTIONS:
 * 1. First press Y to run at 100% and find your MAX_SHOOTER_RPM
 * 2. Then use A to cycle through the power percentages from your lookup table
 * 3. Record the RPM values shown for each power level
 * 4. Update the lookup table in HybridPIDPowerTest.java with actual RPM values
 */
@TeleOp(name = "Shooter RPM Test", group = "Test")
public class ShooterRPMTest extends OpMode {

    private DcMotor shooter;

    // Encoder ticks per revolution - ADJUST THIS FOR YOUR MOTOR
    // Common values: 28 (bare REV HD Hex), 288 (20:1), 537.7 (19.2:1)
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Test power levels from the lookup table
    private static final double[] TEST_POWERS = {0.40, 0.425, 0.45, 0.47, 0.49, 0.55};
    private int currentTestIndex = 0;

    // Current shooter state
    private double shooterPower = 0.0;
    private double shooterRPM = 0.0;
    private double maxRecordedRPM = 0.0;

    // RPM tracking
    private int lastEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime stabilityTimer = new ElapsedTime();

    // RPM averaging for more stable readings
    private double[] rpmHistory = new double[10];
    private int rpmHistoryIndex = 0;
    private double averageRPM = 0.0;

    // Button state tracking
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastYButton = false;
    private boolean lastXButton = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;

    // Recorded values
    private StringBuilder recordedValues = new StringBuilder();
    private int recordCount = 0;

    @Override
    public void init() {
        // Initialize shooter motor
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityTimer.reset();
        stabilityTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Use bumpers to adjust power, Y=100%, X=Stop, A=Cycle test powers");
        telemetry.addData("IMPORTANT", "Adjust SHOOTER_TICKS_PER_REV for your motor!");
        telemetry.addData("Current Ticks/Rev", SHOOTER_TICKS_PER_REV);
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========== BUTTON CONTROLS ==========

        // Right Bumper: Increase power by 5%
        if (gamepad1.right_bumper && !lastRightBumper) {
            shooterPower = Range.clip(shooterPower + 0.05, 0.0, 1.0);
            stabilityTimer.reset();
        }
        lastRightBumper = gamepad1.right_bumper;

        // Left Bumper: Decrease power by 5%
        if (gamepad1.left_bumper && !lastLeftBumper) {
            shooterPower = Range.clip(shooterPower - 0.05, 0.0, 1.0);
            stabilityTimer.reset();
        }
        lastLeftBumper = gamepad1.left_bumper;

        // Y Button: Set to 100% (find max RPM)
        if (gamepad1.y && !lastYButton) {
            shooterPower = 1.0;
            stabilityTimer.reset();
        }
        lastYButton = gamepad1.y;

        // X Button: Stop
        if (gamepad1.x && !lastXButton) {
            shooterPower = 0.0;
            stabilityTimer.reset();
        }
        lastXButton = gamepad1.x;

        // A Button: Cycle through test powers
        if (gamepad1.a && !lastAButton) {
            shooterPower = TEST_POWERS[currentTestIndex];
            currentTestIndex = (currentTestIndex + 1) % TEST_POWERS.length;
            stabilityTimer.reset();
        }
        lastAButton = gamepad1.a;

        // B Button: Record current reading
        if (gamepad1.b && !lastBButton) {
            recordCount++;
            String record = String.format("#%d: Power=%.1f%%, RPM=%.0f (Avg=%.0f)",
                recordCount, shooterPower * 100, shooterRPM, averageRPM);
            recordedValues.append(record).append("\n");
            telemetry.speak("Recorded");
        }
        lastBButton = gamepad1.b;

        // ========== SET MOTOR POWER ==========
        shooter.setPower(shooterPower);

        // ========== CALCULATE RPM ==========
        int currentPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update every 20ms
            int deltaTicks = currentPosition - lastEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            shooterRPM = (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;

            // Update RPM history for averaging
            rpmHistory[rpmHistoryIndex] = shooterRPM;
            rpmHistoryIndex = (rpmHistoryIndex + 1) % rpmHistory.length;

            // Calculate average RPM
            double sum = 0;
            for (double rpm : rpmHistory) {
                sum += rpm;
            }
            averageRPM = sum / rpmHistory.length;

            // Track max RPM
            if (averageRPM > maxRecordedRPM && stabilityTimer.seconds() > 2.0) {
                maxRecordedRPM = averageRPM;
            }

            lastEncoderPosition = currentPosition;
            velocityTimer.reset();
        }

        // ========== TELEMETRY ==========
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("RB/LB", "Adjust power ±5%");
        telemetry.addData("Y", "100% power (find max)");
        telemetry.addData("X", "Stop motor");
        telemetry.addData("A", "Cycle test powers");
        telemetry.addData("B", "Record current reading");

        telemetry.addData("", "");
        telemetry.addData("=== CURRENT STATE ===", "");
        telemetry.addData("Power", "%.1f%%", shooterPower * 100);
        telemetry.addData("Instant RPM", "%.0f", shooterRPM);
        telemetry.addData("Average RPM", "%.0f", averageRPM);
        telemetry.addData("Stability Time", "%.1f sec", stabilityTimer.seconds());

        telemetry.addData("", "");
        telemetry.addData("=== MAX RPM ===", "");
        telemetry.addData("Max Recorded RPM", "%.0f", maxRecordedRPM);
        telemetry.addData("(stable for 2+ sec)", "");

        telemetry.addData("", "");
        telemetry.addData("=== TEST POWERS ===", "");
        telemetry.addData("Next Test Power (A)", "%.1f%%", TEST_POWERS[currentTestIndex] * 100);

        telemetry.addData("", "");
        telemetry.addData("=== LOOKUP TABLE HELPER ===", "");
        if (shooterPower > 0 && stabilityTimer.seconds() > 2.0) {
            telemetry.addData("STABLE READING", "Power %.1f%% = %.0f RPM", shooterPower * 100, averageRPM);
        } else if (shooterPower > 0) {
            telemetry.addData("Waiting for stable...", "%.1f sec remaining", 2.0 - stabilityTimer.seconds());
        }

        telemetry.addData("", "");
        telemetry.addData("=== RECORDED VALUES ===", "");
        telemetry.addData("Count", recordCount);
        if (recordedValues.length() > 0) {
            telemetry.addData("Values", "\n" + recordedValues.toString());
        }

        telemetry.addData("", "");
        telemetry.addData("=== ENCODER INFO ===", "");
        telemetry.addData("Ticks/Rev Setting", SHOOTER_TICKS_PER_REV);
        telemetry.addData("Current Encoder", currentPosition);

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}

