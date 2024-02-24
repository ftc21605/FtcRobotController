/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/*
 * An example linear op mode where the robot will rotate
 * to a specified angle an then stop.
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.
 */
@TeleOp(name = "Test: NavX Rotate Test", group = "ZTest")
//@Disabled // Comment this in to remove this from the Driver Station OpMode List
public class NavXRotateTest extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    // right turn positive angle, left turn negative angle
    private final double TARGET_ANGLE_DEGREES = -45.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* If possible, use encoders when driving, as it results in more */
        /* predictable drive system response.                           */
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        DcMotor.ZeroPowerBehavior save_left = leftFrontDrive.getZeroPowerBehavior();
        DcMotor.ZeroPowerBehavior save_right = rightFrontDrive.getZeroPowerBehavior();
        telemetry.addData(">", "left zero power behavior: %s", save_left);
        telemetry.addData(">", "right zero power behavior: %s", save_right);
        telemetry.update();
	DecimalFormat df = new DecimalFormat("#.##");

        waitForStart();

        while (!calibration_complete) {
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

        try {
            yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

            final double TOTAL_RUN_TIME_SECONDS = 100.0;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();


            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftFrontDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        telemetry.addData("PIDOutput", df.format(0.00));
                telemetry.update();
			break;
                    } else {
                        double output = yawPIDResult.getOutput();
                        output = Math.max(0.2, output)*Math.signum(yawPIDController.getSetpoint());
                         leftFrontDrive.setPower(output);
                         leftBackDrive.setPower(output);
                         rightFrontDrive.setPower(-output);
                         rightBackDrive.setPower(-output);
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                } else {
                    /* A timeout occurred */
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                telemetry.update();
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
	// if (!gamepad1.a)
	//     {
	// return;
	//     }
	sleep(1000);
        navx_device.zeroYaw();
	sleep(1000);
                telemetry.addData("Reset Yaw", df.format(navx_device.getYaw()));
                telemetry.update();
	sleep(5000);

	try {
            yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        yawPIDController.setSetpoint(-TARGET_ANGLE_DEGREES);
	                final double TOTAL_RUN_TIME_SECONDS = 100.0;
	                int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            while ((runtime.time() < TOTAL_RUN_TIME_SECONDS) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftFrontDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        telemetry.addData("PIDOutput", df.format(0.00));
                    } else {
                        double output = yawPIDResult.getOutput();
                        output = Math.max(0.2, output)*Math.signum(yawPIDController.getSetpoint());
                         leftFrontDrive.setPower(output);
                         leftBackDrive.setPower(output);
                         rightFrontDrive.setPower(-output);
                         rightBackDrive.setPower(-output);
                        telemetry.addData("PIDOutput", df.format(output) + ", " +
                                df.format(-output));
                    }
                } else {
                    /* A timeout occurred */
                    Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
                telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                telemetry.update();
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
