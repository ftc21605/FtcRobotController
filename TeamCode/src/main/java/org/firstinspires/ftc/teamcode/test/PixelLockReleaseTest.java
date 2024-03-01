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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.OurLinearOpBase;


@TeleOp(name = "Test: Pixel Lock Release Test", group = "ZTest")
//@Disabled
public class PixelLockReleaseTest extends OurLinearOpBase {

    boolean pad2_leftbumper_pressed = false;
    boolean pad2_rightbumper_pressed = false;

    @Override
    public void runOpMode() {
	setup_bucketfront();
	setup_bucketback();
        // Wait for the start button
        telemetry.addData(">", "Press Start.");
        telemetry.update();
        waitForStart();


        // Scan PlaneServo till stop pressed.
        while (opModeIsActive()) {
            telemetry.addData(">", "Pad2: left bumper toggles bucket back lock servo");
            telemetry.addData(">", "Pad2: right bumper toggles bucket front lock servo");
            if (gamepad2.right_bumper)
		{
		    if (!pad2_rightbumper_pressed)
			{
			    pad2_rightbumper_pressed = true;
			    bucketfront_toggle();
			}
		}
	    else
		{
		    pad2_rightbumper_pressed = false;
		}

            if (gamepad2.left_bumper)
		{
		    if (!pad2_leftbumper_pressed)
			{
			    pad2_leftbumper_pressed = true;
			    bucketback_toggle();
			}
		}
	    else
		{
		    pad2_leftbumper_pressed = false;
		}

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the PlaneServo to the new position and pause;
        }

    }
}
