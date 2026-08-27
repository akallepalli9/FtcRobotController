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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Original Mecanum Drive To AprilTag", group = "Concept")
public class AprilTagTracking_Original extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    double DISTANCE = 36;
    final double DESIRED_DISTANCE = (DISTANCE); //  this is how close the camera should get to the target (inches)

    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override public void runOpMode()
    {
        boolean targetFound     = false;
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;

        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backleftmotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backrightmotor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        if (USE_WEBCAM)
            setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream" + DESIRED_DISTANCE);
        telemetry.addData(">", "Touch START to start OpMode");

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
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
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            } else {
                drive  = -gamepad1.left_stick_y  / 2.0;
                strafe = -gamepad1.left_stick_x  / 2.0;
                turn   = -gamepad1.right_stick_x / 3.0;
            }
            telemetry.update();

            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void initAprilTag() {
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();
        for (int id = 0; id < 20; id++) {
            tagLibraryBuilder.addTag(
                    id,
                    "Tag " + id,
                    6.75,
                    DistanceUnit.INCH);
            // This is the buggy part from the original code:
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    .build();
        }

        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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
        if (visionPortal == null) {
            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
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
