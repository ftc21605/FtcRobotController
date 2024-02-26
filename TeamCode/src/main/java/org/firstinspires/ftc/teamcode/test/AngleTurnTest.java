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

import org.firstinspires.ftc.teamcode.OurLinearOpBase;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "Test: Angle Turn Test", group = "ZTest")
//@Disabled
public class AngleTurnTest extends OurLinearOpBase {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    boolean x_pushed = false;
    boolean y_pushed = false;
    boolean a_pushed = false;
    boolean b_pushed = false;
    int n_bpushed = 0;
    double TURN_SPEED = 0.2;

    @Override
    public void runOpMode() {
        setup_drive_motors();
	setup_imu();
        // navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        // gyro = (IntegratingGyroscope) navxMicro;
        // navx_device = AHRS.getInstance(navxMicro,
        //         AHRS.DeviceDataType.kProcessedData,
        //         NAVX_DEVICE_UPDATE_RATE_HZ);
	// //        setup_imu();
        // double turn_speed_local = 0.2;
        double current_angle = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            if (gamepad1.b) {
                if (!b_pushed) {
		    if (n_bpushed == 4)
			{
		    navx_turn_right(90);
		    n_bpushed++;
			}
		    else if (n_bpushed == 3)
			{
		    navx_turn_left(90);
		    n_bpushed++;
			}
		    else if (n_bpushed == 2)
			{
		    navx_turn_right(90);
		    n_bpushed++;
			}
		    else if (n_bpushed == 1)
			{
		    navx_turn_right(25);
		    n_bpushed++;
			}
		    else if (n_bpushed == 0)
			{
		    navx_turn_left(25);
		    n_bpushed++;
			}
                    b_pushed = true;
                }
            } else {
                b_pushed = false;
            }
            if (gamepad1.a) {
                if (!a_pushed) {
		    for (int i=0; i<20; i++)
			{
                    navx_turn_right(25);
		    sleep(1000);
			}
		    for (int i=0; i<20; i++)
			{
                    navx_turn_left(25);
		    sleep(1000);
			}
                    a_pushed = true;
                }
            } else {
                a_pushed = false;
            }
            if (gamepad1.x) {
                if (!x_pushed) {
                    navx_turn_right(25);
                    x_pushed = true;
                }
            } else {
                x_pushed = false;
            }
            if (gamepad1.y) {
                if (!y_pushed) {
                    navx_turn_left(25);
                    y_pushed = true;

                }
            } else {
                y_pushed = false;
            }
	    /*
            telemetry.addData(">", "push x to turn +25deg");
            telemetry.addData(">", "push y to turn -25deg");
            telemetry.addData(">", "push a to turn +25deg");
            telemetry.addData(">", "push b to turn -25deg");
	    //            telemetry.addData(">", "yaw setpoint: %.1f, val: %.1ferror %.1f", yawPIDController.getSetpoint(), yawPIDController.get(), yawPIDController.getError());
	    //            telemetry.addData(">", "current yaw: %.1f", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData(">", "Should point to %.1f", current_angle);
	    telemetry.addData(">", "angle we should have %.1f, sum yaw: %.1f", angle_we_should_have, real_yaw);
	    */
        }
    }

    void left_turn(double ANGLE) {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current_angle = angles.firstAngle;
        double driveto_angle = current_angle + ANGLE;
        telemetry.addData(">", "input: %.1f, current angle %.1f, driveto: %.1f", ANGLE, current_angle, driveto_angle);
        telemetry.update();

        leftFrontDrive.setPower(-TURN_SPEED);
        rightFrontDrive.setPower(TURN_SPEED);
        leftBackDrive.setPower(-TURN_SPEED);
        rightBackDrive.setPower(TURN_SPEED);
        if (driveto_angle > 180) {
            while ((current_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 0 || current_angle > (driveto_angle - 360)) {
                telemetry.addData(">", "angle %.1f, drive to large angle %.1f", current_angle, driveto_angle);
                telemetry.update();
                sleep(1);
            }

        } else {
            while ((current_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < driveto_angle) {
//            orientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData(">", "angle %.1f, drive to %.1f", current_angle, driveto_angle);
                telemetry.update();
                sleep(1);
            }
        }
        // imu.resetYaw();
        // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // leftFrontDrive.setPower(-TURN_SPEED);
        // rightFrontDrive.setPower(TURN_SPEED);
        // leftBackDrive.setPower(-TURN_SPEED);
        // rightBackDrive.setPower(TURN_SPEED);
        // while (orientation.getYaw(AngleUnit.DEGREES) < ANGLE) {
        //     orientation = imu.getRobotYawPitchRollAngles();
        //     telemetry.addData("angle ", orientation.getYaw(AngleUnit.DEGREES));
        //     telemetry.update();
        //     sleep(5);
        // }
        drivemotors_off();
    }

    void right_turn(double ANGLE) {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current_angle = angles.firstAngle;
        double driveto_angle = current_angle - ANGLE;
        telemetry.addData("right turn: ", "input: %.1f, current angle %.1f, driveto: %.1f", ANGLE, current_angle, driveto_angle);
        telemetry.update();
        leftFrontDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(-TURN_SPEED);
        leftBackDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(-TURN_SPEED);
        if (driveto_angle < -180) {
            //current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            while ((current_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 0 || current_angle > (driveto_angle + 360)) {
                telemetry.addData(">", "angle %.1f, drive to small angle %.1f", current_angle, driveto_angle);
                telemetry.update();
                sleep(1);
            }
        } else {
            while ((current_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > driveto_angle) {
                telemetry.addData(">", "angle %.1f, drive to angle %.1f", current_angle, driveto_angle);
                telemetry.update();
                sleep(1);
            }
        }
        // imu.resetYaw();
        // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // leftFrontDrive.setPower(TURN_SPEED);
        // rightFrontDrive.setPower(-TURN_SPEED);
        // leftBackDrive.setPower(TURN_SPEED);
        // rightBackDrive.setPower(-TURN_SPEED);
        // while (orientation.getYaw(AngleUnit.DEGREES) > -ANGLE) {
        //     orientation = imu.getRobotYawPitchRollAngles();
        //     telemetry.addData("angle ", orientation.getYaw(AngleUnit.DEGREES));
        //     telemetry.update();
        //     sleep(5);
        // }
        drivemotors_off();
    }

}


