package org.firstinspires.ftc;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum: Field Centric (NEW)", group = "Drive")
public class MecanumDrive extends LinearOpMode {
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public IMU imu;

    @Override
    public void runOpMode() {
        initializeHardware(hardwareMap);
        
        telemetry.addLine("ROBOT READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options && imu != null) {
                imu.resetYaw();
            }

            driveFieldRelative(y, x, rx);
            
            // Debug Telemetry
            telemetry.addData("Stick Y", "%.2f", y);
            telemetry.addData("Stick X", "%.2f", x);
            telemetry.addData("Heading", imu != null ? Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) : "N/A");
            telemetry.update();
        }
    }

    public void initializeHardware(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "frontleftmotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backleftmotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontrightmotor");
        backRightMotor = hwMap.get(DcMotor.class, "backrightmotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        try {
            imu = hwMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);
        } catch (Exception e) {
            imu = null;
        }
    }

    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double bl = forward - strafe + rotate;
        double fr = forward - strafe - rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
        
        frontLeftMotor.setPower(fl / max);
        backLeftMotor.setPower(bl / max);
        frontRightMotor.setPower(fr / max);
        backRightMotor.setPower(br / max);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        if (imu == null) {
            drive(forward, strafe, rotate);
            return;
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        
        drive(rotY, rotX, -+rotate);
    }
}
