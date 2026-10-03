package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name="AprilTagTracking Left Trigger Sync Backup", group = "Backup")
public class AprilTagTracking_LeftTriggerSync_Backup extends LinearOpMode {

    double DISTANCE = 36;
    final double DESIRED_DISTANCE = (DISTANCE);

    final double SPEED_GAIN  =  0.02;
    final double STRAFE_GAIN =  0.015;
    final double TURN_GAIN   =  0.01;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shooterMotor = null;
    private CRServo frontrightservo = null;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override public void runOpMode() {
        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        initAprilTag();

        backRightDrive = hardwareMap.get(DcMotor.class, "backrightmotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (USE_WEBCAM) setManualExposure(6, 250);

        telemetry.addData("Status", "Backup initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }
            }

            if (gamepad1.left_bumper && targetFound) {
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            } else if (gamepad1.right_bumper && targetFound) {
                double range = desiredTag.ftcPose.range;
                double servoPower = (0.68 * range + 5.19) / 100.0;
                servoPower = Range.clip(servoPower, -1.0, 1.0);

                if (frontrightservo != null) frontrightservo.setPower(servoPower);
                drive = 0; turn = 0; strafe = 0;
            } else {
                drive = gamepad1.left_trigger;
                strafe = -gamepad1.left_stick_x / 2.0;
                turn = -gamepad1.right_stick_x / 3.0;

                if (targetFound) {
                    double range = desiredTag.ftcPose.range;
                    double servoPower = (0.68 * range + 5.19) / 100.0;
                    servoPower = Range.clip(servoPower, -1.0, 1.0);
                    if (frontrightservo != null) frontrightservo.setPower(gamepad1.left_trigger > 0.05 ? servoPower : 0);
                } else {
                    if (frontrightservo != null) frontrightservo.setPower(gamepad1.left_trigger);
                }
            }

            double shooterPower = gamepad1.right_trigger;
            shooterMotor.setPower(shooterPower);

            telemetry.update();
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        double backRightPower = x - y + yaw;
        if (backRightDrive != null) backRightDrive.setPower(backRightPower);
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        tagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        for (int id = 0; id < 20; id++) {
            tagLibraryBuilder.addTag(id, "Tag " + id, 6.75, DistanceUnit.INCH);
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

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new android.util.Size(640, 480))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
