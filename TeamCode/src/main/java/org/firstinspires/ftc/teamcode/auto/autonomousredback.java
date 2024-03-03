
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OurLinearOpBase;
import org.firstinspires.ftc.teamcode.processors.RedFinder;


@Autonomous(name = "autonomous red back", group = "Wallace")
//@Disabled
public class autonomousredback extends OurLinearOpBase {

    boolean skip_opencv = false;
    /* Declare OpMode members. */

    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    // private RedFinder visionProcessor = new RedFinder();
    //visionProcessor.drawthr();
    //RedFinder.Selected myselect = RedFinder.Selected.NONE;
   
    private int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    //  static final double MAX_POS = 0.15;     // Maximum rotational position
    //static final double MIN_POS = 0.5;     // Minimum rotational position

    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.

    boolean targetFound = false;

    @Override
    public void runOpMode() {
	debuglevel = 1;
	    setup_imu();
        setup_drive_motors();
        //initAprilTag();
        setup_redfinder();
        //initTfod();
        startVisionPortal();
	setup_pixel_lift();
	setup_intake();
	setup_pixel_bucket();
	setup_bucketfront();
	setup_bucketback();

	setup_distance_sensor();
	bucketfront_lock(); // start with locked bucket
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
                navx_drive_forward_straight(DRIVE_SPEED, 35); 
                pixel_release();
		telemetry.addData(">", "final angle %.1f", navx_device.getYaw());

		//		wait_for_button_pushed(0);
          //      while (!gamepad1.a){
          //         sleep(1);
          //      }
                navx_drive_backward_straight(DRIVE_SPEED, 6);
		//	wait_for_button_pushed(1);
             //   while (!gamepad1.a){
             //       sleep(1);
             //   }
		//  telemetry.addData("> r85", "angle: %.1f", navx_device.getYaw());
		// telemetry.update();

                navx_turn_left(96);
                //telemetry.addData("> r85", "angle: %.1f", navx_device.getYaw());
                //telemetry.update();
		navx_drive_backward_straight(0.3,20);
		prep_pixel_drop();
		navx_drive_backward_distsensor_right(0.3,0);
		drop_pixel();
		//navx_drive_sideways(10);
		//navx_drive_backward_straight(0.3,5);


            }
            //doCameraSwitching();
            // telemetry.addData(">", "switched camera, waiting for 1sec");
            // telemetry.update();
            // sleep(1000);

            return;
    } else if(redselect ==RedFinder.Selected.RIGHT)

    {
        navx_drive_forward_straight(DRIVE_SPEED, 18);
        navx_turn_right(45);
        //telemetry.addData("> l25", "angle: %.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
      //  while (!gamepad1.a) {
      //      sleep(1);
      //  }
        navx_drive_forward_straight(DRIVE_SPEED, 11); 
        pixel_release();
        navx_drive_backward_straight(DRIVE_SPEED, 10);
        navx_turn_left(48);
        navx_drive_forward_straight(0.5, 33);
	navx_turn_right(93);
       navx_drive_forward_straight(0.5, 80);
	navx_turn_left(90);
		prep_pixel_drop();
       navx_drive_backward_straight(0.3, 30);
		
	navx_turn_left(95);

	navx_drive_backward_distsensor_right(0.25,0);
		drop_pixel();
		//		navx_drive_sideways(-10); // when moving to left of board
		//	navx_drive_sideways(8); // default is moving to the right

    } else if(redselect ==RedFinder.Selected.LEFT)

    {
        navx_drive_forward_straight(DRIVE_SPEED, 17);
        navx_turn_left(40);
        navx_drive_forward_straight(DRIVE_SPEED, 14);
        pixel_release();// S1: Forward 47
        navx_drive_backward_straight(DRIVE_SPEED, 10);
	navx_turn_right(36);
        navx_drive_forward_straight(0.5, 33);
	navx_turn_right(94);
       navx_drive_forward_straight(0.5, 80);
	navx_turn_left(90);
		prep_pixel_drop();
       navx_drive_backward_straight(0.5, 15);
		
	navx_turn_left(93);

	navx_drive_backward_distsensor_right(0.25,0);
		drop_pixel();
		//		navx_drive_sideways(-10); // when moving to left of board
		//	navx_drive_sideways(8); // default is moving to the right
		//navx_drive_backward_straight(0.3,5);
    }

    }
}
