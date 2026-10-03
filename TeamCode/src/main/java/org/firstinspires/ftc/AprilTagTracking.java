/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc;

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

@TeleOp(name="Mecanum Drive To AprilTag", group = "Concept")
// @Disabled
public class AprilTagTracking extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    double DISTANCE = 36;
    final double DESIRED_DISTANCE = (DISTANCE); //  this is how close the camera should get to the target (inches)

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel
    private DcMotor shooterMotor = null;    //  Used to control 312 RPM shooter motor
    private DcMotor transferMotor = null;   //  Used to control transfer motor
    private CRServo frontrightservo = null;       //  Intake servo

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Safe Hardware Mapping: try to fetch each motor/servo so missing items don't crash OpMode
        try {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "frontleftmotor");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'frontleftmotor'");
        }

        try {
            frontRightDrive = hardwareMap.get(DcMotor.class, "frontrightmotor");
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'frontrightmotor'");
        }

        try {
            backLeftDrive = hardwareMap.get(DcMotor.class, "backleftmotor");
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'backleftmotor'");
        }

        try {
            backRightDrive = hardwareMap.get(DcMotor.class, "backrightmotor");
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'backrightmotor'");
        }

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'shooter_motor'");
        }

        try {
            transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
            transferMotor.setDirection(DcMotor.Direction.FORWARD);
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'transfer_motor'");
        }

        try {
            frontrightservo = hardwareMap.get(CRServo.class, "frontrightservo");
        } catch (Exception e) {
            telemetry.addData("Config Warning", "Missing 'frontrightservo'");
        }

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream" + DESIRED_DISTANCE);
        telemetry.addData(">", "Touch START to start OpMode");

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addLine(String.format("Found ID %d (%s)", desiredTag.id, desiredTag.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        desiredTag.ftcPose.x,
                        desiredTag.ftcPose.y,
                        desiredTag.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        desiredTag.ftcPose.pitch,
                        desiredTag.ftcPose.roll,
                        desiredTag.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        desiredTag.ftcPose.range,
                        desiredTag.ftcPose.bearing,
                        desiredTag.ftcPose.elevation));
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else if (gamepad1.right_bumper && targetFound) {

                // Custom Servo Power Equation: y = 0.68x + 5.19
                double range = desiredTag.ftcPose.range;
                double servoPower = (0.68 * range + 5.19) / 100.0;
                servoPower = Range.clip(servoPower, -1.0, 1.0);

                if (frontrightservo != null) frontrightservo.setPower(servoPower);

                drive  = 0;
                turn   = 0;
                strafe = 0;

                telemetry.addData("Servo Mode (RB)", "Range: %5.2f, Servo Power: %5.2f", range, servoPower);
            } else {

                // Drive using Left Trigger for forward drive instead of left stick Y.
                drive  = gamepad1.left_trigger;          // Drive forward based on Left Trigger analog squeeze (0.0 to 1.0)
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.

                if (frontrightservo != null) {
                    frontrightservo.setPower(0);
                }

                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            // Control 312 RPM shooter motor & transfer motor together using Right Trigger or A Button (Gamepad 1 or Gamepad 2)
            double shooterPower = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);

            // Also check A button on either controller for 100% full power testing
            if (gamepad1.a || gamepad2.a) {
                shooterPower = 1.0;
            }

            if (shooterMotor != null) {
                shooterMotor.setPower(shooterPower);
            }
            if (transferMotor != null) {
                transferMotor.setPower(shooterPower);
            }

            telemetry.addData("Shooter/Transfer Power", "%5.2f (G1 Trigger: %5.2f, G2 Trigger: %5.2f)",
                    shooterPower, gamepad1.right_trigger, gamepad2.right_trigger);

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels safely
        if (frontLeftDrive != null) frontLeftDrive.setPower(frontLeftPower);
        if (frontRightDrive != null) frontRightDrive.setPower(frontRightPower);
        if (backLeftDrive != null) backLeftDrive.setPower(backLeftPower);
        if (backRightDrive != null) backRightDrive.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        tagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

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

    /*
     Manually set the camera gain and exposure.
    */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
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
