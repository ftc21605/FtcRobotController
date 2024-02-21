
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

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.processors.BlueFinder;
import org.firstinspires.ftc.teamcode.processors.RedFinder;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Autonomous(name = "Auto Base Class", group = "auto")
//@Disabled
public class OurLinearOpBase extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;

    public AHRS navx_device;
    public navXPIDController yawPIDController;
    public navXPIDController.PIDResult yawPIDResult;
    public IntegratingGyroscope gyro;
    public NavxMicroNavigationSensor navxMicro;

    public final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    public final double TARGET_ANGLE_DEGREES = 20.0;
    public final double TOLERANCE_DEGREES = 2.0;
    public final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    public final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    public final double YAW_PID_P = 0.005;
    public final double YAW_PID_I = 0.0;
    public final double YAW_PID_D = 0.0;
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final int DEVICE_TIMEOUT_MS = 500;
    static final double NAVX_DRIVE_SPEED = 0.5;
    static final double NAVX_MIN_TURN_POWER = 0.2;

    public boolean calibration_complete = false;


    public VisionPortal visionPortal;
    public WebcamName LogitechWebCam, MicrosoftWebCam;

    public BlueFinder BlueColorFinder = null;// = null;
    public RedFinder RedColorFinder = null;// = null;
    //    ColorFinder = new BlueFinder();
    //ColorFinder.drawthr();
    //   BlueFinder.Selected myselect = BlueFinder.Selected.NONE;

    public DcMotor PixelLift = null;
    public DcMotor Intake = null;
    public Servo CRservo;
    public DistanceSensor sensorDistance;

    public ElapsedTime runtime = new ElapsedTime();

    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag

    int debuglevel = 0;

    @Override
    public void runOpMode() {
    }

    public void doCameraSwitching(WebcamName webcam) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (!visionPortal.getActiveCamera().equals(webcam)) {
                visionPortal.setActiveCamera(webcam);
            }
        }
    }   // end method doCameraSwitching()

    public void initAprilTag() {
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

    public void startVisionPortal() {
        LogitechWebCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        MicrosoftWebCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(LogitechWebCam, MicrosoftWebCam);

        initAprilTag();
        // Create the vision portal by using a builder.
        if (BlueColorFinder != null) {
            BlueColorFinder.setTelemetry(telemetry);
            visionPortal = VisionPortal.easyCreateWithDefaults(switchableCamera, BlueColorFinder, aprilTag);
        } else if (RedColorFinder != null) {
            RedColorFinder.setTelemetry(telemetry);
            visionPortal = VisionPortal.easyCreateWithDefaults(switchableCamera, RedColorFinder, aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(switchableCamera, aprilTag);
        }
    }

    public void navx_drive_to(double angle, double distance) throws InterruptedException {
        int current_left_position = leftFrontDrive.getCurrentPosition();
        int current_right_position = rightFrontDrive.getCurrentPosition();
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        yawPIDController.setSetpoint(angle);
        try {
            while (distance > ((leftFrontDrive.getCurrentPosition() - current_left_position + rightFrontDrive.getCurrentPosition() - current_right_position) / 2. / COUNTS_PER_INCH) &&
                    !Thread.currentThread().isInterrupted()) {

                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftBackDrive.setPower(NAVX_DRIVE_SPEED);
                        leftFrontDrive.setPower(NAVX_DRIVE_SPEED);
                        rightFrontDrive.setPower(NAVX_DRIVE_SPEED);
                        rightBackDrive.setPower(NAVX_DRIVE_SPEED);
                        telemetry.addData("PIDOutput", "speed %.1f, %.1f", NAVX_DRIVE_SPEED, NAVX_DRIVE_SPEED);
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftFrontDrive.setPower(NAVX_DRIVE_SPEED + output);
                        leftBackDrive.setPower(NAVX_DRIVE_SPEED + output);
                        rightBackDrive.setPower(NAVX_DRIVE_SPEED - output);
                        rightFrontDrive.setPower(NAVX_DRIVE_SPEED - output);
                        telemetry.addData("PIDOutput", "speed %.1f, %.1f", (NAVX_DRIVE_SPEED + output), (NAVX_DRIVE_SPEED - output));
                    }
                    telemetry.addData("Yaw", "%.1f", navx_device.getYaw());
                } else {
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

    }

    public void navx_reverse_drive_to(double angle, double distance) throws InterruptedException {
        int current_left_position = leftFrontDrive.getCurrentPosition();
        int current_right_position = rightFrontDrive.getCurrentPosition();
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        yawPIDController.setSetpoint(angle);
        try {
            while (Math.abs(distance) > Math.abs(((leftFrontDrive.getCurrentPosition() - current_left_position + rightFrontDrive.getCurrentPosition() - current_right_position) / 2. / COUNTS_PER_INCH)) &&
                    !Thread.currentThread().isInterrupted()) {

                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        leftBackDrive.setPower(-NAVX_DRIVE_SPEED);
                        leftFrontDrive.setPower(-NAVX_DRIVE_SPEED);
                        rightFrontDrive.setPower(-NAVX_DRIVE_SPEED);
                        rightBackDrive.setPower(-NAVX_DRIVE_SPEED);
                        telemetry.addData("PIDOutput", "speed %.1f, %.1f", NAVX_DRIVE_SPEED, NAVX_DRIVE_SPEED);
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftFrontDrive.setPower(-NAVX_DRIVE_SPEED - output);
                        leftBackDrive.setPower(-NAVX_DRIVE_SPEED - output);
                        rightBackDrive.setPower(-NAVX_DRIVE_SPEED + output);
                        rightFrontDrive.setPower(-NAVX_DRIVE_SPEED + output);
                        telemetry.addData("PIDOutput", "speed %.1f, %.1f", (NAVX_DRIVE_SPEED + output), (NAVX_DRIVE_SPEED - output));
                    }
                    telemetry.addData("Yaw", "%.1f", navx_device.getYaw());
                } else {
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

    }

    public void navx_turn(double angle) {
        try {
            yawPIDController.setSetpoint(angle);
            while (!yawPIDResult.isOnTarget()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    double output = yawPIDResult.getOutput();
                    output = Math.max(NAVX_MIN_TURN_POWER, output);
                    leftFrontDrive.setPower(output);
                    leftBackDrive.setPower(output);
                    rightFrontDrive.setPower(-output);
                    rightBackDrive.setPower(-output);
                    telemetry.addData("PIDOutput", "power %.1f, %.1f", output, -output);
                    telemetry.update();
                }
            }
        } catch (InterruptedException ex) {
            telemetry.addData("reveived interrupt", "");
        }
        drivemotors_off();
    }

    public void drivemotors_off() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    public void wait_for_button_pushed(int dbglvl) {
        if (dbglvl <= debuglevel) {
            while (!gamepad1.a) {
                telemetry.addData(">", "Press A to proceed");
                telemetry.update();
                sleep(10);
            }
        }
        debuglevel++;
    }

    public void setup_drive_motors() {
        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setup_imu() {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;
        navx_device = AHRS.getInstance(navxMicro,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        yawPIDController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        //        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        //	navXPIDController.PIDResult yawPIDResult null;

        while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                sleep(10);
            }
        }
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();

    }

    public void setup_pixel_lift() {
        PixelLift = hardwareMap.get(DcMotor.class, "pixellift");
        PixelLift.setDirection(DcMotor.Direction.REVERSE);
        PixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PixelLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setup_intake() {
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setup_pixel_bucket() {
        CRservo = hardwareMap.get(Servo.class, "pixelbucket");
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

    public void setup_distance_sensor() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }

    public void setup_bluefinder() {
        BlueColorFinder = new BlueFinder();
        //ColorFinder.drawthr();
        BlueFinder.Selected myselect = BlueFinder.Selected.NONE;

    }

    public void setup_redfinder() {
        RedColorFinder = new RedFinder();
        //ColorFinder.drawthr();
        RedFinder.Selected myselect = RedFinder.Selected.NONE;

    }
}
