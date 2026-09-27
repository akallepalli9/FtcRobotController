package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * SingleMotorTeleOp
 *
 * This OpMode demonstrates how to control a single 312 RPM motor (e.g., goBILDA 5202/5203 Series)
 * using the gamepad joysticks or triggers.
 *
 * Hardware Configuration:
 * - Motor Name: "testMotor" (or adjust name to match your FTC Driver Station Configuration)
 */
@TeleOp(name = "Single 312 RPM Motor Control", group = "TeleOp")
public class SingleMotorTeleOp extends LinearOpMode {

    // Declare the motor variable
    private DcMotor testMotor = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware map. The string "testMotor" must match the configuration on the Driver Station.
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        // Optional: Set motor direction (FORWARD or REVERSE depending on mounting)
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set brake mode so the motor stops immediately when power is set to 0
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Initialized. Ready to start!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * Control Scheme:
             * - Left Stick Y: Smooth variable speed forward and backward.
             *   Note: Push stick UP gives negative Y, so we invert it (-gamepad1.left_stick_y).
             * - Right Bumper / Left Bumper: Quick full power or set presets if desired.
             */
            double motorPower = -gamepad1.left_stick_y;

            // Apply power to the motor
            testMotor.setPower(motorPower);

            // Display telemetry data
            telemetry.addData("Motor Power", "%5.2f", motorPower);
            telemetry.addData("Target Motor", "312 RPM Motor ('testMotor')");
            telemetry.update();
        }
    }
}
