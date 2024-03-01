
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.OurLinearOpBase;
import org.firstinspires.ftc.teamcode.processors.RedFinder;


@Autonomous(name = "autonomous red front chp", group = "Wallace")
//@Disabled
public class autonomousredfrontchp extends OurLinearOpBase {

    boolean skip_opencv = false;
    /* Declare OpMode members. */

    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    // private RedFinder visionProcessor = new RedFinder();
    //visionProcessor.drawthr();
    //RedFinder.Selected myselect = RedFinder.Selected.NONE;
   
    private int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    static final double MAX_POS = 0.15;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position

    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.

    boolean targetFound = false;

    @Override
    public void runOpMode() {
	    setup_imu();
        setup_drive_motors();
        //initAprilTag();
        setup_redfinder();
        //initTfod();
        startVisionPortal();
	setup_pixel_lift();
	setup_intake();
	setup_pixel_bucket();
	setup_distance_sensor();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        //telemetry.addData(">", "angle %.1f", (imu.getRobotYawPitchRollAngles()).getYaw(AngleUnit.DEGREES));
        telemetry.update();
        //BlueFinder.Selected selected;
        // here is what happens after we hit start
        while (!isStarted() && !isStopRequested()) {
            if (!skip_opencv) {
                RedColorFinder.print_selection();
                //BlueFinder.Selected redselect = BlueFinder.Selected.NONE;
                while ((redselect = RedColorFinder.getSelection()) == RedFinder.Selected.NONE) {
                    if (isStopRequested()) {
                        return;
                    }
                    sleep(10);
                }
                RedColorFinder.print_selection();
                telemetry.update();
            } else {
                redselect = RedFinder.Selected.MIDDLE;
            }
        }


        if (redselect == RedFinder.Selected.MIDDLE) {
            DESIRED_TAG_ID = 2;
            if (!skip_opencv) {
                navx_drive_forward_straight(DRIVE_SPEED, 27);  // S1: Forward 47
                pixel_release();
          //      while (!gamepad1.a){
          //         sleep(1);
          //      }
                navx_drive_backward_straight(DRIVE_SPEED, 3);
             //   while (!gamepad1.a){
             //       sleep(1);
             //   }
                telemetry.addData("> r85", "angle: %.1f", navx_device.getYaw());
                telemetry.update();
                while (!gamepad1.a){
                    sleep(1);
                }

                navx_turn_left(95);
                telemetry.addData("> r85", "angle: %.1f", navx_device.getYaw());
                telemetry.update();
                while (!gamepad1.a){
                   sleep(1);
                }
                //encoderDrive(-(DRIVE_SPEED+0.2), -40, -40, 25.0);  // S1: Forward 47
                //right_turn(3);
                navx_drive_backward_straight((DRIVE_SPEED+0.2), 35);  // S1: Forward 47

                sleep(100);
                //navx_turn_right(1);
                double to_go = -(sensorDistance.getDistance(DistanceUnit.INCH) - 4); // seems to result in 1.5 inch
                if (to_go < 0) {
                    navx_drive_backward_straight(0.2, to_go);
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
                BucketTurnServo.setPosition(BUCKETTURNSERVO_MAXPOS);
                sleep(3000);
                BucketTurnServo.setPosition(BUCKETTURNSERVO_MINPOS);
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
    } else if(redselect ==RedFinder.Selected.LEFT)

    {
        navx_drive_forward_straight(DRIVE_SPEED, 12);  // S1: Forward 47
      //  while (!gamepad1.a) {
      //      sleep(1);
     //   }
        sleep(100);
        navx_turn_left(35);
        //telemetry.addData("> l25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
      //  while (!gamepad1.a) {
      //      sleep(1);
      //  }
        navx_drive_forward_straight(DRIVE_SPEED, 11);  // S1: Forward 47
        pixel_release();
    //    while (!gamepad1.a) {
    //        sleep(1);
    //    }
        navx_drive_backward_straight(DRIVE_SPEED, 10);
   //     while (!gamepad1.a) {
    //        sleep(1);
     //   }
        navx_turn_right(35);
        //telemetry.addData("> r25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    //    while (!gamepad1.a) {
     //       sleep(1);
     //   }

        navx_drive_forward_straight((DRIVE_SPEED+0.2), 32);  // S1: Forward 47
    //    while (!gamepad1.a) {
     //       sleep(1);
     //   }
       navx_turn_right(80);
        //telemetry.addData("> r85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    //    while (!gamepad1.a) {
     //       sleep(1);
     //   }
        navx_drive_forward_straight((DRIVE_SPEED+0.2), 25);  // S1: Forward 47
     //   while (!gamepad1.a) {
     //       sleep(1);
     //   }
        navx_turn_left(90);
       // telemetry.addData("> l85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        navx_drive_backward_straight(DRIVE_SPEED, 10);  // S1: Forward 47
    //    while (!gamepad1.a) {
     //       sleep(1);
    //    }
        navx_turn_left(90);
       // telemetry.addData("> r85", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    //    while (!gamepad1.a) {
    //        sleep(1);
     //   }
        double to_go = -(sensorDistance.getDistance(DistanceUnit.INCH) - 3); // seems to result in 1.5 inch
        if (to_go < 0) {
            encoderDrive(-0.2, to_go, to_go, 5);
        }
        PixelLift.setTargetPosition(1100);
        PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PixelLift.setPower(0.5);
        while (PixelLift.isBusy()) {
            sleep(10);
        }
        sleep(100);
        BucketTurnServo.setPosition(BUCKETTURNSERVO_MAXPOS);
        sleep(2000);
        BucketTurnServo.setPosition(BUCKETTURNSERVO_MINPOS);
        sleep(100);
        PixelLift.setTargetPosition(100);
        PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PixelLift.setPower(0.2);
        while (PixelLift.isBusy()) {
            sleep(10);
        }
        PixelLift.setPower(0);

    } else if(redselect ==RedFinder.Selected.RIGHT)

    {
        navx_drive_forward_straight(DRIVE_SPEED, 14);  // S1: Forward 47
     //   while (!gamepad1.a) {
     //       sleep(1);
      //  }
        navx_turn_right(35);
    //    while (!gamepad1.a) {
    //        sleep(1);
     //   }
        sleep(100);
        navx_drive_forward_straight(DRIVE_SPEED, 8);
        pixel_release();// S1: Forward 47
        // pixel_lock();
    //    while (!gamepad1.a) {
    //        sleep(1);
    //    }
        navx_drive_backward_straight(DRIVE_SPEED, 5);
    //    while (!gamepad1.a) {
    //        sleep(1);
     //   }
        navx_turn_left(35);
      //  while (!gamepad1.a) {
     //       sleep(1);
      //  }

        sleep(100);
        navx_drive_forward_straight((DRIVE_SPEED+0.2), 28);  // S1: Forward 47
   //     while (!gamepad1.a) {
    //        sleep(1);
   //     }
        navx_turn_right(80);
      //  while (!gamepad1.a) {
      //      sleep(1);
      //  }
        navx_drive_forward_straight((DRIVE_SPEED+0.2), 25);  // S1: Forward 47
     //   while (!gamepad1.a) {
      //      sleep(1);
      //  }
        navx_turn_left(90);
        navx_drive_backward_straight(DRIVE_SPEED, 20);  // S1: Forward 47
     //   while (!gamepad1.a) {
      //      sleep(1);
      //  }
        navx_turn_left(90);
     //   while (!gamepad1.a) {
     //       sleep(1);
     //   }
        double to_go = -(sensorDistance.getDistance(DistanceUnit.INCH) - 3); // seems to result in 1.5 inch
        if (to_go < -200) {
            to_go = -10;
        }
        if (to_go < 0) {
            encoderDrive(-0.2, to_go, to_go, 5);
        }

        PixelLift.setTargetPosition(1100);
        PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PixelLift.setPower(0.5);
        while (PixelLift.isBusy()) {
            sleep(10);
        }
        sleep(100);
        BucketTurnServo.setPosition(BUCKETTURNSERVO_MAXPOS);
        sleep(3000);
        BucketTurnServo.setPosition(BUCKETTURNSERVO_MINPOS);
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

}




