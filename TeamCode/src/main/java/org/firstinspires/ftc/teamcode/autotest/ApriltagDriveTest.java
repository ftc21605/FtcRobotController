
/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.autotest;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.BlueFinder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Test: April tag drive Test", group = "ZTest")
//@Disabled
public class ApriltagDriveTest extends LinearOpMode {

    boolean skip_opencv = false;
    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private AHRS navx_device;
    private navXPIDController yawPIDController;
       private navXPIDController.PIDResult yawPIDResult;

     private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 20.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
        static final int DEVICE_TIMEOUT_MS = 500;
    static final double navx_drive_speed = 0.5;

    private boolean calibration_complete = false;


    private VisionPortal visionPortal;
    private WebcamName LogitechWebCam, MicrosoftWebCam;

    private BlueFinder ColorFinder = new BlueFinder();
    //ColorFinder.drawthr();
    BlueFinder.Selected myselect = BlueFinder.Selected.NONE;
    private ElapsedTime runtime = new ElapsedTime();

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag

    boolean targetFound = false;

    @Override
    public void runOpMode() {
        initAprilTag();
        startVisionPortal();
        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        yawPIDController = new navXPIDController( navx_device,
                                    navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
	//	navXPIDController.PIDResult yawPIDResult null;
	
        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();
        //BlueFinder.Selected selected;
        // here is what happens after we hit start
        doCameraSwitching(MicrosoftWebCam);
        visionPortal.setProcessorEnabled(ColorFinder, false);
        int DESIRED_TAG_ID = 4;    // Choose the tag you want to approach or set to -1 for ANY tag.
        boolean targetFound = false;
        double angle = 0;
        double distance = 0;
        while ((!isStarted() && !isStopRequested()) || !targetFound) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData(">", "number of detections %d", currentDetections.size());
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                telemetry.addData(">", "in detection loop");
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        telemetry.addData(">", "Found Tag ID %d", detection.id);
                        break;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                angle = desiredTag.ftcPose.bearing;
                distance = desiredTag.ftcPose.range;
            }
            telemetry.update();
        }

        sleep(10000);
    }


    private void doCameraSwitching(WebcamName webcam) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (!visionPortal.getActiveCamera().equals(webcam)) {
                visionPortal.setActiveCamera(webcam);
            }
        }
    }   // end method doCameraSwitching()

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);
    }

    private void startVisionPortal() {
        LogitechWebCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        MicrosoftWebCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(LogitechWebCam, MicrosoftWebCam);

        // Create the vision portal by using a builder.
        ColorFinder.setTelemetry(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(switchableCamera, ColorFinder, aprilTag);
    }
    private void navx_drive_to(double angle, double distance) throws InterruptedException
    {
	int current_left_position = leftFrontDrive.getCurrentPosition();
	int current_right_position = rightFrontDrive.getCurrentPosition();
       navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
try
    {
       while (distance >  ((leftFrontDrive.getCurrentPosition()-current_left_position + rightFrontDrive.getCurrentPosition() - current_right_position)/2./ COUNTS_PER_INCH) &&
	       !Thread.currentThread().isInterrupted()) {
	    
	    if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
		if (yawPIDResult.isOnTarget()) {
		    leftBackDrive.setPower(navx_drive_speed);
		    leftFrontDrive.setPower(navx_drive_speed);
		    rightFrontDrive.setPower(navx_drive_speed);
		    rightBackDrive.setPower(navx_drive_speed);
		    telemetry.addData("PIDOutput", "speed %.1f, %.1f", navx_drive_speed, navx_drive_speed);
		} else {
		    double output = yawPIDResult.getOutput();
		    leftFrontDrive.setPower(navx_drive_speed + output);
		    leftBackDrive.setPower(navx_drive_speed + output);
		    rightBackDrive.setPower(navx_drive_speed - output);
		    rightFrontDrive.setPower(navx_drive_speed - output);
		    telemetry.addData("PIDOutput", "speed %.1f, %.1f", (navx_drive_speed + output), (navx_drive_speed - output));
		}
			telemetry.addData("Yaw", "%.1f", navx_device.getYaw());
	    } else{
		/* A timeout occurred */
		    //		Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
	    }
	}
    }
       catch(InterruptedException ex) {
	     Thread.currentThread().interrupt();
	 }
	
    }

}


