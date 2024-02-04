
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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.processors.BlueFinder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "autonomous opencv blue front", group = "Wallace")
//@Disabled
public class autonomousopencvbluefront extends LinearOpMode {

    boolean skip_opencv = false;
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

    private BlueFinder visionProcessor = new BlueFinder();
    //visionProcessor.drawthr();
    BlueFinder.Selected myselect = BlueFinder.Selected.NONE;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor PixelLift = null;
    Servo CRservo;
    private DistanceSensor sensorDistance;

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag
    private int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    static final double MAX_POS = 0.15;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position

    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.

    boolean targetFound = false;

    @Override
    public void runOpMode() {
        //initAprilTag();
        initBlueFinding();
        //initTfod();
        startVisionPortal();
        //visionProcessor.drawthr();
        // Initialize the drive system variables.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        PixelLift = hardwareMap.get(DcMotor.class, "pixellift");
        CRservo = hardwareMap.get(Servo.class, "pixelbucket");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        PixelLift.setDirection(DcMotor.Direction.REVERSE);
        PixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PixelLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        //BlueFinder.Selected selected;
        // here is what happens after we hit start
        while (!isStarted() && !isStopRequested()) {
            if (!skip_opencv) {
                visionProcessor.print_selection();
                //BlueFinder.Selected myselect = BlueFinder.Selected.NONE;
                while ((myselect = visionProcessor.getSelection()) == BlueFinder.Selected.NONE) {
                    if (isStopRequested()) {
                        return;
                    }
                    sleep(10);
                }
                visionProcessor.print_selection();
                telemetry.update();
            } else {
                myselect = BlueFinder.Selected.MIDDLE;
            }
        }
        imu.resetYaw();

        if (myselect == BlueFinder.Selected.MIDDLE) {
            DESIRED_TAG_ID = 2;
            if (!skip_opencv) {
                encoderDrive(DRIVE_SPEED, 26, 26, 5.0);
                pixel_release();
                //pixel_lock();// S1: Forward 47
                encoderDrive(-DRIVE_SPEED, -4, -4, 5.0);
                right_turn(85);
                encoderDrive(-DRIVE_SPEED, -24, -26, 25.0);  // S1: Forward 47
                right_turn(2);
                double to_go = -(sensorDistance.getDistance(DistanceUnit.INCH) - 3); // seems to result in 1.5 inch
                if (to_go < 0) {
                    encoderDrive(-0.2, to_go, to_go, 5);
                }
                //while (!gamepad1.a){
                //   sleep(1);
                //}
                PixelLift.setTargetPosition(1100);
                PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                PixelLift.setPower(0.5);
                while (PixelLift.isBusy()) {
                    sleep(10);
                }
                sleep(100);
                CRservo.setPosition(MAX_POS);
                sleep(3000);
                CRservo.setPosition(MIN_POS);
                sleep(100);
                PixelLift.setTargetPosition(100);
                PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                PixelLift.setPower(0.2);
                while (PixelLift.isBusy()) {
                    sleep(10);
                }
                PixelLift.setPower(0);
            } else {
                sleep(1000);
            }
            //doCameraSwitching();
            telemetry.addData(">", "switched camera, waiting for 1sec");
            telemetry.update();
            sleep(1000);

            return;

        } else if (myselect == BlueFinder.Selected.LEFT) {
            encoderDrive(DRIVE_SPEED, 12, 12, 5.0);
         //   while (!gamepad1.a) {
         //       sleep(1);
         //   }

           // sleep(100);
            left_turn(35);
            telemetry.addData("> l25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
      //      while (!gamepad1.a) {
       //         sleep(1);
       //     }
            encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S1: Forward 47
            pixel_release();
        //    while (!gamepad1.a) {
        //        sleep(1);
         //   }
            encoderDrive(-DRIVE_SPEED, -10, -10, 5.0);
       //     while (!gamepad1.a) {
        //        sleep(1);
         //   }
            right_turn(35);
            telemetry.addData("> r25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
     //       while (!gamepad1.a) {
       //         sleep(1);
       //     }
            sleep(100);
            encoderDrive((DRIVE_SPEED+0.2), 30, 30, 25.0);
      //      while (!gamepad1.a) {
      //          sleep(1);
       //     }

            left_turn(90);
            telemetry.addData("> l85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
    //        while (!gamepad1.a) {
      //          sleep(1);
       //     }
            encoderDrive((DRIVE_SPEED), 25, 25, 25.0);  // S1: Forward 47
         //   while (!gamepad1.a) {
           //     sleep(1);
          //  }

            right_turn(82);
            telemetry.addData("> l85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
    //        while (!gamepad1.a) {
    //            sleep(1);
     //       }
            encoderDrive(-(DRIVE_SPEED+0.2), -26, -26, 25.0);  // S1: Forward 47
    //        while (!gamepad1.a) {
     //           sleep(1);
     //       }

            right_turn(85);
            telemetry.addData("> r25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
     //       while (!gamepad1.a) {
     //           sleep(1);
     //       }

            double to_go = -(sensorDistance.getDistance(DistanceUnit.INCH) - 3); // seems to result in 1.5 inch
            if (to_go < -200) {
                to_go = -15;
            }
            if (to_go < 0) {
                encoderDrive(-0.2, to_go, to_go, 5);
            }
    //        while (!gamepad1.a) {
     //           sleep(1);
      //      }

            PixelLift.setTargetPosition(1100);
            PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            PixelLift.setPower(0.5);
            while (PixelLift.isBusy()) {
                sleep(10);
            }
            sleep(100);
            CRservo.setPosition(MAX_POS);
            sleep(3000);
            CRservo.setPosition(MIN_POS);
            sleep(100);
            PixelLift.setTargetPosition(100);
            PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            PixelLift.setPower(0.2);
            while (PixelLift.isBusy()) {
                sleep(10);
            }
            PixelLift.setPower(0);
        } else if (myselect == BlueFinder.Selected.RIGHT) {
            encoderDrive(DRIVE_SPEED, 5, 5, 5.0);  // S1: Forward 47
            right_turn(18);
            telemetry.addData("> r20", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            //    while (!gamepad1.a){
            //      sleep(1);
            //    }
//            sleep(1000);
            encoderDrive(DRIVE_SPEED, 15, 15, 5.0);
            pixel_release();// S1: Forward 47
            //     while (!gamepad1.a){
            //        sleep(1);
            //    }
            // pixel_lock();
            encoderDrive(-DRIVE_SPEED, -12, -12, 5.0);
            //           while (!gamepad1.a){
            //              sleep(1);
            //         }
            left_turn(20);
            telemetry.addData("> l25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            //         while (!gamepad1.a){
            //            sleep(1);
            //        }

            sleep(100);
            encoderDrive((DRIVE_SPEED+0.2), 39, 39, 25.0);  // S1: Forward 47
            //           while (!gamepad1.a){
//                sleep(1);
            //           }
            left_turn(92);
            telemetry.addData("> l90", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            //          while (!gamepad1.a){
            //              sleep(1);
            //         }
            encoderDrive((DRIVE_SPEED+0.2), 36, 36, 25.0);  // S1: Forward 47
            //          while (!gamepad1.a){
            //              sleep(1);
            //         }
            right_turn(85);
            telemetry.addData("> l85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            //         while (!gamepad1.a){
            //          sleep(1);
            //        }
            encoderDrive(-DRIVE_SPEED, -18, -18, 25.0);  // S1: Forward 47
            //      while (!gamepad1.a){
            //           sleep(1);
            //      }
            right_turn(85);
            telemetry.addData("> r85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("> r85", "distance: %.1f", sensorDistance.getDistance(DistanceUnit.INCH) - 5);
            telemetry.update();
            telemetry.addData("> r85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("> d", "distance: %.1f", sensorDistance.getDistance(DistanceUnit.INCH));


            //      while (!gamepad1.a){
            //         sleep(1);
            //      }
            double to_go = -(sensorDistance.getDistance(DistanceUnit.INCH) - 7); // seems to result in 1.5 inch
            telemetry.addData("> ", "to_go: %.1f", to_go);
            telemetry.addData("> r85", "distance now: %.1f", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (to_go < -200) {
                to_go = -16;
            }
            if (to_go < 0) {
                encoderDrive(-0.2, to_go, to_go, 5);
            }
            telemetry.addData("> r85", "to_go: %.1f", to_go);
            telemetry.addData("> r85", "distance now: %.1f", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();

            //     while (!gamepad1.a){
            //        sleep(1);
            //     }

            PixelLift.setTargetPosition(1100);
            PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            PixelLift.setPower(0.5);
            while (PixelLift.isBusy()) {
                sleep(10);
            }
            sleep(100);
            CRservo.setPosition(MAX_POS);
            sleep(2000);
            CRservo.setPosition(MIN_POS);
            sleep(100);
            PixelLift.setTargetPosition(100);
            PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            PixelLift.setPower(0.2);
            while (PixelLift.isBusy()) {
                sleep(10);
            }
            PixelLift.setPower(0);

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

    void initBlueFinding() {
        //visionProcessor = new BlueFinder();
        // visionProcessor.setTelemetry(telemetry);
        //  visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), visionProcessor, aprilTag);

    }

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
        visionProcessor.setTelemetry(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(switchableCamera, visionProcessor);
    }

    public void moveRobot(double x, double yaw, double distance) {
        // Calculate left and right wheel powers.
        double rightPower = -(x - yaw);
        double leftPower = -(x + yaw);

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        int currpos = leftFrontDrive.getCurrentPosition();
        Double newData = new Double(distance * COUNTS_PER_INCH);
        int gohere = currpos - (newData.intValue());

        telemetry.addData("move robot", "%d, %.1f, %.1f, %d", gohere, distance, COUNTS_PER_INCH, currpos);
        telemetry.update();
        while (!gamepad1.a) {
            sleep(1);
        }
        // Send powers to the wheels.
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);

        while (leftFrontDrive.getCurrentPosition() > gohere) {
            sleep(1);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    public void pixel_release() {
        double Power = 0.3;
        int tics = -16;
        //   tics = Intake.getCurrentPosition() + tics;
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setTargetPosition(tics);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setPower(Power);
        while (Intake.isBusy()) {
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

    double left_turn(double ANGLE) {
        double current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double driveto_angle = current_angle + ANGLE;
        telemetry.addData(">", "input: %.1f, current angle %.1f, driveto: %.1f", ANGLE, current_angle, driveto_angle);
        telemetry.update();

        leftFrontDrive.setPower(-TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        leftBackDrive.setPower(-TURN_SPEED);
        rightBackDrive.setPower(TURN_SPEED);
        if (driveto_angle > 180) {
            while ((current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 0 || current_angle > (driveto_angle - 360)) {
                telemetry.addData(">", "angle %.1f", current_angle);
                telemetry.update();
                sleep(1);
            }

        } else {
            while ((current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) < driveto_angle) {
//            orientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData(">", "angle %.1f", current_angle);
                telemetry.update();
                sleep(1);
            }
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        telemetry.addData(">", "final angle %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        return driveto_angle;
    }

    double right_turn(double ANGLE) {
        double current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double driveto_angle = current_angle - ANGLE;
        telemetry.addData("right turn: ", "input: %.1f, current angle %.1f, driveto: %.1f", ANGLE, current_angle, driveto_angle);
        telemetry.update();
        leftFrontDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(-TURN_SPEED);
        leftBackDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(-TURN_SPEED);
        if (driveto_angle < -180) {
            //current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            while ((current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) < 0 || current_angle > (driveto_angle + 360)) {
                telemetry.addData(">", "angle %.1f", current_angle);
                telemetry.update();
                sleep(1);
            }
        } else {
            while ((current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > driveto_angle) {
                telemetry.addData(">", "angle %.1f", current_angle);
                telemetry.update();
                sleep(1);
            }
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        return driveto_angle;
    }

}


