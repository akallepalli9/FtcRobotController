package org.firstinspires.ftc;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Mecanum Field Orientated OpMode (Original)")
public class MecanumFieldOrientatedOpMode_Original extends OpMode {
    private final MecanumDrive drive = new MecanumDrive();
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private String cameraError;

    private double forward, strafe, rotate;

    @Override
    public void init() {
        drive.initializeHardware(hardwareMap);


        try {
            initAprilTag();
            telemetry.addLine("Webcam 1 initialized");
        } catch (RuntimeException e) {
            cameraError = e.getMessage();
            aprilTag = null;
            visionPortal = null;
            telemetry.addData("Camera error", cameraError);
        }

        telemetry.update();
    }






    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y; // Invert Y as up is negative
        strafe = gamepad1.left_stick_x * 1.1;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);
        showAprilTagTelemetry();

        if (visionPortal != null) {
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
        }

        telemetry.update();
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        for (int id = 0; id < 20; id++) {
            tagLibraryBuilder.addTag(
                    id,
                    "Tag " + id,
                    6.75,
                    DistanceUnit.INCH);
        }
        AprilTagLibrary tagLibrary = tagLibraryBuilder.build();

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(tagLibrary)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTag.setDecimation(3);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new  Size(640, 480))
                .addProcessor(aprilTag)
                .build();
    }

    private void showAprilTagTelemetry() {
        if (aprilTag == null) {
            telemetry.addData("Camera error", cameraError);
            return;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTags detected", detections.size());
        telemetry.addData("Camera state", visionPortal.getCameraState());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s",
                        detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)",
                        detection.center.x,
                        detection.center.y));
            }
        }

        telemetry.addLine("\nXYZ = Right, Forward, Up");
        telemetry.addLine("PRY = Pitch, Roll, Yaw");
        telemetry.addLine("RBE = Range, Bearing, Elevation");
        telemetry.addLine("D-pad down/up = stop/resume camera");
    }

    @Override
    public void stop() {
        if (drive.frontLeftMotor != null) {
            drive.drive(0, 0, 0);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
