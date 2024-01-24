
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: RobotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backward for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This method assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "autonomous blue back", group = "Wallace")
//@Disabled
public class autonomousblue extends LinearOpMode {

    boolean override_tfod = false;
    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    private DcMotor Intake = null;

    IMU imu;

    private VisionPortal visionPortal;
    private WebcamName webcam1, webcam2;

    private TfodProcessor tfod;
    private static final String TFOD_MODEL_ASSET = "cylinder.tflite";
    private static final String[] LABELS = {
            "b", "r"
    };
    private ElapsedTime runtime = new ElapsedTime();

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag
    private int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.

    boolean targetFound = false;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */

    @Override
    public void runOpMode() {
        initAprilTag();
        initTfod();
        startVisionPortal();

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// set up the imu, our controller which contains the imu faces forward and up
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); // just reset to zero to have a defined starting value
        // Send telemetry message to indicate successful Encoder reset
        String object_id = "0";
        double col = 0;
        double row = 0;
        double x = col;
        double y = row;
        double phi = Math.atan2(x, y);
        double deg = phi * 180. / Math.PI;
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.addData(">", "angle %.1f", (imu.getRobotYawPitchRollAngles()).getYaw(AngleUnit.DEGREES));
        telemetry.addData("angle", "deg %.1f", (deg));
        telemetry.update();


        // here is what happens after we hit start
        while (!isStarted() && !isStopRequested()) {

            int icnt = 0;
            if (!override_tfod) {
                while (Objects.equals(object_id, "0") && !isStopRequested()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        if (currentRecognitions != null) {
                            telemetry.addData("# Objects Detected", currentRecognitions.size());

                            // step through the list of recognitions and display image position/size information for each one
                            // Note: "Image number" refers to the randomized image orientation/number
                            for (Recognition recognition : currentRecognitions) {
                                col = (recognition.getLeft() + recognition.getRight()) / 2;
                                row = (recognition.getTop() + recognition.getBottom()) / 2;
                                double width = Math.abs(recognition.getRight() - recognition.getLeft());
                                double height = Math.abs(recognition.getTop() - recognition.getBottom());

                                telemetry.addData("", " ");
                                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                                object_id = recognition.getLabel();
                                telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                                telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                            }
                            telemetry.update();
                        } else {
                            sleep(10);
                            icnt++;
                            telemetry.addData("tried ", "%d", icnt);
                            telemetry.update();

                        }
                    }
                }
                if (isStopRequested())
                    return; // if we do not find anything and get a stop after 30secs, quit here
                telemetry.addData("found ", "%s", object_id);
                telemetry.addData("deg", "x %.2f y %.2f phi %.1f deg %.1f", x, y, phi, deg);
                telemetry.update();
                sleep(1000);
            } else {
                object_id = "b";
                col = 10.;
                row = 10.;
            }
        }
        imu.resetYaw();


        sleep(1000);
        if (Objects.equals(object_id, "r")) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            //telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

            //sleep(10000);
            //double x = col;
            //double y = row;
            //double phi = Math.atan2(x, y);
            //double deg = phi * 180. / Math.PI;
            telemetry.addData("wrong one. this is for BLUE team guys. deg", "x %.2f y %.2f phi %.1f deg %.1f", x, y, phi, deg);
            telemetry.update();
            sleep(2000);

        }
        //encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        else if (Objects.equals(object_id, "b")) {
//            encoderDrive(DRIVE_SPEED, 2, 2, 5.0);  // S1: Forward 47
            //telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));


            telemetry.addData("deg", "x %.2f y %.2f phi %.1f deg %.1f", x, y, phi, deg);
            telemetry.update();
            sleep(2000);
            if (60 < deg && deg < 65) {
                encoderDrive(DRIVE_SPEED, 26, 26, 5.0);
                pixel_release();
                //pixel_lock();// S1: Forward 47
                encoderDrive(-DRIVE_SPEED, -2, -2, 5.0);
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                left_turn(85);
                encoderDrive(DRIVE_SPEED, 75, 75, 25.0);  // S1: Forward 47
                right_turn(85);
                sleep(1000);
                encoderDrive(DRIVE_SPEED, 20, 20, 25.0);  // S1: Forward 47
                imu.resetYaw();
                orientation = imu.getRobotYawPitchRollAngles();
                left_turn(85);
                encoderDrive(DRIVE_SPEED, 23, 23, 5.0);  // S1: Forward 47
                sleep(10000);
                return;

            } else if (65 < deg) {
                encoderDrive(DRIVE_SPEED, 5, 5, 5.0);  // S1: Forward 47
                right_turn(25);
                sleep(1000);
                encoderDrive(DRIVE_SPEED, 13, 13, 5.0);
                pixel_release();// S1: Forward 47
                // pixel_lock();
                encoderDrive(-DRIVE_SPEED, -13, -13, 5.0);
                left_turn(25);
                sleep(1000);
                encoderDrive(DRIVE_SPEED, 40, 40, 25.0);  // S1: Forward 47
                left_turn(85);
                encoderDrive(DRIVE_SPEED, 65, 65, 25.0);  // S1: Forward 47
                left_turn(85);
                encoderDrive(DRIVE_SPEED, 20, 20, 25.0);  // S1: Forward 47
                right_turn(85);


            } else if (deg < 60) {
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47
                sleep(1000);
                left_turn(25);
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47
                pixel_release();
                encoderDrive(-DRIVE_SPEED, -2, -2, 5.0);
                //pixel_lock();
                encoderDrive(-DRIVE_SPEED, -18, -18, 5.0);

                right_turn(25);
                sleep(1000);
                encoderDrive(DRIVE_SPEED, 40, 40, 25.0);  // S1: Forward 47
                left_turn(85);
                encoderDrive(DRIVE_SPEED, 65, 65, 25.0);  // S1: Forward 47
                left_turn(85);
                encoderDrive(DRIVE_SPEED, 20, 20, 25.0);  // S1: Forward 47
                right_turn(85);

            }
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading and range error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
                telemetry.update();
                sleep(10000);
                return;
            }
        }
    }


    // now for the april tags


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        telemetry.addData("lfin", "%.0f %.0f / %.0f", speed, leftInches, rightInches);
        telemetry.update();
        //   sleep(10000);  // pause to display final telemetry message.

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower((speed));
            rightFrontDrive.setPower((speed));
            leftBackDrive.setPower((speed));
            rightBackDrive.setPower((speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250);   // optional pause after each move.
        }
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        float conf = (float) 0.6;
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can increase the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        //tfod.setZoom(1.0, 16.0 / 9.0);

        tfod.setMinResultConfidence(conf);
    }

    private void doCameraSwitching() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (visionPortal.getActiveCamera().equals(webcam1)) {
                visionPortal.setActiveCamera(webcam2);
                // If the left bumper is pressed, use Webcam 1.
            } else {
                visionPortal.setActiveCamera(webcam1);
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
        aprilTag.setDecimation(2);
    }

    private void startVisionPortal() {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(tfod)
                .addProcessor(aprilTag)
                .build();

    }

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower = x - yaw;
        double rightPower = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }

    public void pixel_release() {
        double Power = 0.2;
        int tics = 8;
        //   tics = Intake.getCurrentPosition() + tics;
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setPower(Power);
        while (Intake.getCurrentPosition() < tics) {
            sleep(1);
        }
        Intake.setPower(0);


    }

    public void pixel_lock() {
        double Power = -0.2;
        int tics = 8;
        //  tics = Intake.getCurrentPosition() - tics;
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setPower(Power);
        while (Intake.getCurrentPosition() < tics) {
            sleep(1);
        }
        Intake.setPower(0);
    }
    void left_turn(double ANGLE){
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        leftFrontDrive.setPower(-TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        leftBackDrive.setPower(-TURN_SPEED);
        rightBackDrive.setPower(TURN_SPEED);
        while (orientation.getYaw(AngleUnit.DEGREES) < ANGLE) {
            orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("angle ", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            sleep(5);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void right_turn ( double ANGLE){
        imu.resetYaw();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        leftFrontDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(-TURN_SPEED);
        leftBackDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(-TURN_SPEED);
        while (orientation.getYaw(AngleUnit.DEGREES) > -ANGLE) {
            orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("angle ", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
            sleep(5);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}


