
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
import org.firstinspires.ftc.teamcode.processors.BlueFinder;


@Autonomous(name = "autonomous blue back", group = "Wallace")
//@Disabled
public class autonomousblueback extends OurLinearOpBase {

    boolean skip_opencv = false;
    /* Declare OpMode members. */

    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

   
    private int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.

    boolean targetFound = false;

    @Override
    public void runOpMode() {
	debuglevel = 1;
	    setup_imu();
        setup_drive_motors();
        //initAprilTag();
        setup_bluefinder();
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
                BlueColorFinder.print_selection();
                //BlueFinder.Selected blueselect = BlueFinder.Selected.NONE;
                while ((blueselect = BlueColorFinder.getSelection()) == BlueFinder.Selected.NONE) {
                    if (isStopRequested()) {
                        return;
                    }
                    sleep(10);
                }
                BlueColorFinder.print_selection();
                telemetry.update();
            } else {
                blueselect = BlueFinder.Selected.MIDDLE;
            }
        }


        if (blueselect == BlueFinder.Selected.MIDDLE) {
            DESIRED_TAG_ID = 2;
            if (!skip_opencv) {
                navx_drive_forward_straight(DRIVE_SPEED, 34); 
                pixel_release();
		telemetry.addData(">", "final angle %.1f", navx_device.getYaw());
                navx_drive_backward_straight(DRIVE_SPEED, 7);
                navx_turn_right(98);
                //telemetry.addData("> r85", "angle: %.1f", navx_device.getYaw());
                //telemetry.update();
		navx_drive_backward_straight(0.3,20);
		prep_pixel_drop();
		navx_drive_backward_distsensor_right(0.3,0);
		drop_pixel();


            }
    } else if(blueselect ==BlueFinder.Selected.RIGHT)

    {
        navx_drive_forward_straight(DRIVE_SPEED, 18);
        navx_turn_right(45);
        telemetry.update();
        navx_drive_forward_straight(DRIVE_SPEED, 11); 
        pixel_release();
        navx_drive_backward_straight(DRIVE_SPEED, 7);
        navx_turn_left(48);
        navx_drive_forward_straight(0.5, 31);
	navx_turn_left(100);
       navx_drive_forward_straight(0.5, 80);
	navx_turn_right(90);
		prep_pixel_drop();
       navx_drive_backward_straight(0.5, 17);
		
	navx_turn_right(101);

	navx_drive_backward_distsensor_right(0.25,0);
		drop_pixel();

    } else if(blueselect ==BlueFinder.Selected.LEFT)

    {
        navx_drive_forward_straight(DRIVE_SPEED, 18);  // S1: Forward 47
        navx_turn_left(45);
        navx_drive_forward_straight(DRIVE_SPEED, 14);
        pixel_release();
        navx_drive_backward_straight(DRIVE_SPEED, 8);
	navx_turn_right(45);
       navx_drive_forward_straight(0.5, 28);
	navx_turn_left(99);
       navx_drive_forward_straight(0.5, 80);
	navx_turn_right(95);
		prep_pixel_drop();
       navx_drive_backward_straight(0.5, 35);
		
	navx_turn_right(95);

	navx_drive_backward_distsensor_right(0.25,0);
		drop_pixel();
    }

    }
}
