package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name="MecanumFieldOrientatedOpMode", group="TeleOp")
public class MecanumFieldOrientatedOpMode extends LinearOpMode {

    // Target standing distance set to 36 inches
    final double DESIRED_DISTANCE = 36.0;

    // Proportional Gains (Tuning constants) - Doubled for better response
    final double SPEED_GAIN  = 0.04;   // Forward/Backward sensitivity
    final double STRAFE_GAIN = 0.03;   // Sideways sensitivity
    final double TURN_GAIN   = 0.02;   // Rotation sensitivity

    // Target AprilTag ID
    final int DESIRED_TAG_ID = 5;

    // Hardware Declare
    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightBackDrive  = null;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        // Initialize Mecanum Motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Reverse left motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Vision with Inches
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized. Hold Gamepad1 'A' to track at 36 inches.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive  = 0;
            double strafe = 0;
            double turn   = 0;

            if (gamepad1.a) {
                boolean targetFound = false;
                AprilTagDetection desiredTag = null;

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }

                if (targetFound) {
                    double x = desiredTag.ftcPose.x;
                    double y = desiredTag.ftcPose.y;

                    // Deadzone for x: between -0.1 and 0.1 becomes 0
                    if (Math.abs(x) < 0.1) x = 0;

                    // Formula: m = (y - 36) / x
                    double m = (x != 0) ? (y - 36.0) / x : 0;

                    // Compute powers using x and y
                    drive  = (y - DESIRED_DISTANCE) * SPEED_GAIN;
                    strafe = x * STRAFE_GAIN;
                    turn   = desiredTag.ftcPose.yaw * TURN_GAIN;

                    telemetry.addData("Auto-Aligning", "Tag ID: %d", desiredTag.id);
                    telemetry.addData("Calculated M", "%.2f", m);
                    telemetry.addData("X (inches)", "%.2f", x);
                    telemetry.addData("Y (inches)", "%.2f", y);
                } else {
                    telemetry.addData("Vision Error", "Tag ID %d not found!", DESIRED_TAG_ID);
                }
            } else {
                drive  = -gamepad1.left_stick_y;
                strafe =  gamepad1.left_stick_x;
                turn   =  gamepad1.right_stick_x;
                telemetry.addData("Driving Mode", "Manual Driver Control");
            }

            // Mix vectors into Mecanum kinematics equations
            // Note: If rotation is reversed, change -turn to +turn in the equations below
            double leftFrontPower  = drive + strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double leftBackPower   = drive - strafe + turn;
            double rightBackPower  = drive + strafe - turn;

            // Normalize motor powers
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.update();
            sleep(10);
        }
    }
}
