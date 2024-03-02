
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
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    /* Drive motors. */
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;

    // IMU 
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
    public static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    public static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    public static final double COUNTS_PER_INCH = 40;//experimental, instead of (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * Math.PI);
    //public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * Math.PI);
    public static final int DEVICE_TIMEOUT_MS = 500;
    public static final double NAVX_DRIVE_SPEED = 0.4;
    public static final double NAVX_MIN_TURN_POWER = 0.3;

    public boolean calibration_complete = false;
    public double total_angle = 0.;
    public double total_yaw = 0.;

    // Vision
    public VisionPortal visionPortal;
    public WebcamName LogitechWebCam, MicrosoftWebCam;

    public BlueFinder BlueColorFinder = null;// = null;
    public RedFinder RedColorFinder = null;// = null;
    public RedFinder.Selected redselect;
    // pixel lift
    public DcMotor PixelLift = null;

    // intake
    public DcMotor Intake = null;

    // pixel transport
    public DcMotor PixelTransport = null;

    public Servo BucketTurnServo;
    public double BUCKETTURNSERVO_MAXPOS = 0.23;
    public double BUCKETTURNSERVO_MINPOS = 0.85;

    public Servo BucketBackServo;
    public final double bucketback_release = 0.6;
    public final double bucketback_catch = 0.3;
    boolean bucket_back_locked = true;

    public Servo BucketFrontServo;
    public final double bucketfront_release = 0.6;
    public final double bucketfront_catch = 0.2;
    boolean bucket_front_locked = true;

    // Plane servo and its parameters
    public Servo PlaneServo;

    public static final double PLANE_LAUNCH = 1.;
    public static final double PLANE_LOAD = 0.;
    public boolean plane_launched = false;

    // Hook Elevator
    public Servo ElevatorServo;
    public TouchSensor ElevatorLimit; // magnet limit switch
    public static final double MOVE_UP = 1.0;     // move up (cont servo posistion = 1)
    public static final double MOVE_DOWN = 0.0;     // move down (cont servo position = 0)
    public static final double MOVE_STOP = 0.5;     // stop moving (cont servo position 0.5)
    public boolean moving_up = false;
    public boolean moving_down = false;
    public ElapsedTime ElevatorTimer = new ElapsedTime();

    // Hanger winch
    public DcMotor Hanger = null;

    public DistanceSensor sensorDistance;

    public ElapsedTime runtime = new ElapsedTime();

    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag

    public int debuglevel = 0;
    boolean navxturn_telemetry = true;

    @Override
    public void runOpMode() {
    }

    public void doCameraSwitching(WebcamName webcam) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            if (!visionPortal.getActiveCamera().equals(webcam)) {
		// telemetry.addData(">","camera switch to %s", webcam);
		// telemetry.update();
		// sleep(5000);
                visionPortal.setActiveCamera(webcam);
            }
	    // else
	    // 	{
	    // 	telemetry.addData(">","camera is already switched to %s", webcam);
	    // 	telemetry.update();
	    // 	sleep(5000);
	    // 	}		    
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
                .getCameraManager().nameForSwitchableCamera(MicrosoftWebCam, LogitechWebCam);

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

    // interesting coordinate system -> left turns are positive yaw, right turns are negative yaw
    // This only plays a role when calculating the total angle where left turn angles need to be added
    // and right turn angles are substracted
    public void navx_turn_left(double angle) {
        navxturn_telemetry = true;
        // double yaw_in = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // double adjusted_angle = total_angle - total_yaw;
        // adjusted_angle = map_angle(adjusted_angle);
        // double angle_turn = angle - adjusted_angle;
        navx_turn(-angle);
        // total_angle += angle;
        // total_angle = map_angle(total_angle);
        // double yaw_buff = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // total_yaw -= yaw_buff;
        // total_yaw = map_angle(total_yaw);
        // telemetry.addData(">", "angle: %.1f, adjustment angle: %.1f, angle turn: %.1f",angle, adjusted_angle, angle_turn);
        // telemetry.addData(">", "yaw in %.1f, end: %.1f, sum: %.1f",yaw_in, yaw_buff, total_yaw);
        // telemetry.update();
    }

    public void navx_turn_right(double angle) {
        navxturn_telemetry = false;
        // double yaw_in = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // double adjusted_angle = total_angle - total_yaw;
        // adjusted_angle = map_angle(adjusted_angle);
        // double angle_turn = angle - adjusted_angle;
        navx_turn(angle);
        // total_angle -= angle;
        // total_angle = map_angle(total_angle);
        // double yaw_buff = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // total_yaw += yaw_buff;
        // total_yaw = map_angle(total_yaw);
        // telemetry.addData(">", "angle: %.1f, total angle: %.1f, adjustment angle: %.1f, angle turn: %.1f",angle, total_angle, adjusted_angle, angle_turn);
        // telemetry.addData(">", "yaw in %.1f, end: %.1f, total yaw: %.1f",yaw_in, yaw_buff, total_yaw);
        // telemetry.update();
    }

    public void navx_turn(double angle) {
        try {
            navx_device.zeroYaw();
            yawPIDController.setSetpoint(angle);
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            while (!yawPIDResult.isOnTarget()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    double output = yawPIDResult.getOutput();
                    output = Math.max(NAVX_MIN_TURN_POWER, output) * Math.signum(yawPIDController.getSetpoint());
                    leftFrontDrive.setPower(output);
                    leftBackDrive.setPower(output);
                    rightFrontDrive.setPower(-output);
                    rightBackDrive.setPower(-output);
                    if (navxturn_telemetry) {
                        telemetry.addData(">", "yaw setpoint: %.1f, value: %.1f, error %.1f", yawPIDController.getSetpoint(), yawPIDController.get(), yawPIDController.getError());
                        telemetry.addData(">", "current yaw: %.1f", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                        telemetry.addData(">", "Should point to %.1f", angle);

                        telemetry.addData("PIDOutput", "power %.1f, %.1f", output, -output);
                        telemetry.update();
                    }

                }
            }
        } catch (InterruptedException ex) {
            telemetry.addData("reveived interrupt", "");
        }
        drivemotors_off();
    }

    // maps angles to range -180 to 180
    double map_angle(double angle) {
        if (angle > 180) {
            return angle - 360;
        }
        if (angle < -180) {
            return angle + 360;
        }
        return angle;
    }

    public void drivemotors_off() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

    public void wait_for_button_pushed(int dbglvl) {
        if (dbglvl <= debuglevel) {
            telemetry.addData(">", "Press A to proceed");
            telemetry.update();
            while (!gamepad1.a) {
                sleep(10);
            }
            while (gamepad1.a) {
                sleep(10);
            }
        }
        debuglevel++;
    }

    public void setup_drive_motors() {
        setup_drive_motors(true);
    }

    public void setup_drive_motors(boolean reset_decoders) {
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
        if (reset_decoders) {
            reset_motor_decoders();
        }
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
        //yawPIDController.setInputRange(-180.0,180.0);
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
        //        yawPIDResult = new navXPIDController.PIDResult();

    }

    public void setup_pixel_lift() {
        PixelLift = hardwareMap.get(DcMotor.class, "pixellift");
        PixelLift.setDirection(DcMotor.Direction.REVERSE);
        PixelLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PixelLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PixelLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setup_intake() {
        Intake = hardwareMap.get(DcMotor.class, "intake");
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setup_pixel_bucket() {
        BucketTurnServo = hardwareMap.get(Servo.class, "pixelbucket");
    }

    public void pixel_release() {
        double Power = 0.3;
        int tics = 20;
	Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tics = Intake.getCurrentPosition() + tics;
        Intake.setTargetPosition(tics);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setPower(Power);
        while (Intake.isBusy()) {
            sleep(1);
        }
        Intake.setPower(0);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void pixel_lock() {
        double Power = -0.3;
        int tics = 20;
	Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tics = Intake.getCurrentPosition() - tics;
        Intake.setTargetPosition(tics);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setPower(Power);
        while (Intake.isBusy()) {
            sleep(1);
        }
        Intake.setPower(0);
	Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        RedFinder.Selected redselect = RedFinder.Selected.NONE;

    }

    public void setup_pixel_transport() {
        PixelTransport = hardwareMap.get(DcMotor.class, "pixeltransport");
    }

    public void setup_planeservo() {
        PlaneServo = hardwareMap.get(Servo.class, "plane");
    }

    public void launch_plane() {
        if (!plane_launched) {
            PlaneServo.setPosition(PLANE_LAUNCH);
            plane_launched = true;
        } else {
            PlaneServo.setPosition(PLANE_LOAD);
            plane_launched = false;
        }
    }

    public void setup_elevator() {
        ElevatorServo = hardwareMap.get(Servo.class, "hook");
        ElevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorlimit");
        ElapsedTime ElevatorTimer = new ElapsedTime();

    }

    public void setup_hanger() {
        Hanger = hardwareMap.get(DcMotor.class, "hanger");
        Hanger.setDirection(DcMotor.Direction.FORWARD);
    }

    public void hang_the_bot() {

        double hangerpower = 0.8;
        Hanger.setPower(hangerpower);
    }

    public void navx_drive_forward_straight(double power, double inches) {
        navx_drive_forward_right(power, 0, inches);
    }

    public void navx_drive_forward_leftt(double power, double angle, double inches) {
        navx_drive_forward_right(power, -angle, inches);
    }


    public void navx_drive_forward_right(double power, double angle, double inches) {
        double drive_speed = power;

        try {
            navx_device.zeroYaw();
            int current_left_front_position = leftFrontDrive.getCurrentPosition();
            int current_right_front_position = rightFrontDrive.getCurrentPosition();
            int current_left_back_position = leftBackDrive.getCurrentPosition();
            int current_right_back_position = rightBackDrive.getCurrentPosition();
            yawPIDController.setSetpoint(angle);
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
            double counts_to_go = inches * COUNTS_PER_INCH;
            double counts_done = 0;
            while (counts_to_go > (counts_done = Math.abs(((leftFrontDrive.getCurrentPosition() - current_left_front_position + rightFrontDrive.getCurrentPosition() - current_right_front_position + leftBackDrive.getCurrentPosition() - current_left_back_position + rightBackDrive.getCurrentPosition() - current_right_back_position) / 4.))) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    double counts_left = counts_to_go - counts_done;
                    if (counts_to_go - counts_done < 200) {
                        drive_speed = 0.2;
                    }
                    if (yawPIDResult.isOnTarget()) {
                        leftBackDrive.setPower(drive_speed);
                        leftFrontDrive.setPower(drive_speed);
                        rightFrontDrive.setPower(drive_speed);
                        rightBackDrive.setPower(drive_speed);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", drive_speed, drive_speed);
                    } else {
                        double output = yawPIDResult.getOutput();
                        leftFrontDrive.setPower(drive_speed + output);
                        leftBackDrive.setPower(drive_speed + output);
                        rightBackDrive.setPower(drive_speed - output);
                        rightFrontDrive.setPower(drive_speed - output);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", (drive_speed + output), (drive_speed - output));
                    }
                    telemetry.addData(">", "Yaw %.1f", navx_device.getYaw());
                } else {
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        drivemotors_off();
    }

   
    public void navx_drive_backward_straight(double power, double inches) {
        navx_drive_backward_right(power, 0, inches);
    }

    public void navx_drive_backward_left(double power, double angle, double inches) {
        navx_drive_backward_right(power, -angle, inches);
    }

    public void navx_drive_backward_right(double power, double angle, double inches) {
        double drive_speed = -power;

        try {
            navx_device.zeroYaw();
            int current_left_front_position = leftFrontDrive.getCurrentPosition();
            int current_right_front_position = rightFrontDrive.getCurrentPosition();
            int current_left_back_position = leftBackDrive.getCurrentPosition();
            int current_right_back_position = rightBackDrive.getCurrentPosition();
            yawPIDController.setSetpoint(angle);
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
            double counts_to_go = inches * COUNTS_PER_INCH;
            double counts_done = 0;
            while (counts_to_go > (counts_done = Math.abs(((leftFrontDrive.getCurrentPosition() - current_left_front_position + rightFrontDrive.getCurrentPosition() - current_right_front_position + leftBackDrive.getCurrentPosition() - current_left_back_position + rightBackDrive.getCurrentPosition() - current_right_back_position) / 4.))) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    double counts_left = counts_to_go - counts_done;
                    if (counts_to_go - counts_done < 200) {
                        drive_speed = -0.2;
                    }
                    if (yawPIDResult.isOnTarget()) {
                        leftBackDrive.setPower(drive_speed);
                        leftFrontDrive.setPower(drive_speed);
                        rightFrontDrive.setPower(drive_speed);
                        rightBackDrive.setPower(drive_speed);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", drive_speed, drive_speed);
                    } else {
                        double output = yawPIDResult.getOutput();//*Math.signum(drive_speed);
                        leftFrontDrive.setPower(drive_speed + output);
                        leftBackDrive.setPower(drive_speed + output);
                        rightBackDrive.setPower(drive_speed - output);
                        rightFrontDrive.setPower(drive_speed - output);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", (drive_speed + output), (drive_speed - output));
                    }
                    telemetry.addData(">", "Yaw %.1f", navx_device.getYaw());
                } else {
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        drivemotors_off();
    }

        public void navx_drive_backward_distsensor_right(double power, double angle) {
        double drive_speed = -power;

        try {
            navx_device.zeroYaw();
            int current_left_front_position = leftFrontDrive.getCurrentPosition();
            int current_right_front_position = rightFrontDrive.getCurrentPosition();
            int current_left_back_position = leftBackDrive.getCurrentPosition();
            int current_right_back_position = rightBackDrive.getCurrentPosition();
            yawPIDController.setSetpoint(angle);
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
            while ((sensorDistance.getDistance(DistanceUnit.INCH) > 4) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
		    
                    // if (distleft < 10) {
                    //     drive_speed = -0.2;
                    // }
                    if (yawPIDResult.isOnTarget()) {
                        leftBackDrive.setPower(drive_speed);
                        leftFrontDrive.setPower(drive_speed);
                        rightFrontDrive.setPower(drive_speed);
                        rightBackDrive.setPower(drive_speed);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", drive_speed, drive_speed);
                    } else {
                        double output = yawPIDResult.getOutput();//*Math.signum(drive_speed);
                        leftFrontDrive.setPower(drive_speed + output);
                        leftBackDrive.setPower(drive_speed + output);
                        rightBackDrive.setPower(drive_speed - output);
                        rightFrontDrive.setPower(drive_speed - output);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", (drive_speed + output), (drive_speed - output));
                    }
                    telemetry.addData(">", "Yaw %.1f", navx_device.getYaw());
                } else {
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        drivemotors_off();
    }

    public void reset_motor_decoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setup_bucketback() {
        BucketBackServo = hardwareMap.get(Servo.class, "back");
    }

    public void bucketback_release() {
        BucketBackServo.setPosition(bucketback_release);
    }

    public void bucketback_lock() {
        BucketBackServo.setPosition(bucketback_catch);
    }

    public void bucketback_toggle() {
        if (bucket_back_locked) {
            bucketback_release();
            bucket_back_locked = false;
        } else {
            bucketback_lock();
            bucket_back_locked = true;
        }
    }

    public void bucketfront_toggle() {
        if (bucket_front_locked) {
            bucketfront_release();
            bucket_front_locked = false;
        } else {
            bucketfront_lock();
            bucket_front_locked = true;
        }
    }

    public void bucketfront_release() {
        BucketFrontServo.setPosition(bucketfront_release);
    }

    public void bucketfront_lock() {
        BucketFrontServo.setPosition(bucketfront_catch);
    }

    public void setup_bucketfront() {
        BucketFrontServo = hardwareMap.get(Servo.class, "front");
    }

    public void bucket_tilt_forward() {
        BucketTurnServo.setPosition(BUCKETTURNSERVO_MAXPOS);
    }

    public void bucket_tilt_backward() {
        BucketTurnServo.setPosition(BUCKETTURNSERVO_MINPOS);
    }
    public void prep_pixel_drop()
    {
        PixelLift.setTargetPosition(1100);
        PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PixelLift.setPower(0.5);
	bucket_tilt_forward();
    }
    
public void drop_pixel(){
	bucketfront_release();
        sleep(1000);
	navx_drive_forward_straight(0.3,5);
        PixelLift.setTargetPosition(600);
        PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         PixelLift.setPower(0.2);
         while (PixelLift.isBusy()) {
             sleep(10);
        }
	bucket_tilt_backward();
        sleep(1000);
	
        PixelLift.setTargetPosition(50);
        PixelLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         PixelLift.setPower(0.2);
         while (PixelLift.isBusy()) {
             sleep(10);
        }
        PixelLift.setPower(0);
    }
    
   public void  navx_drive_sideways(double inches)
    {
	double drive_speed = -0.4*Math.signum(inches);

        try {
            navx_device.zeroYaw();
            int current_left_front_position = leftFrontDrive.getCurrentPosition();
            int current_right_front_position = rightFrontDrive.getCurrentPosition();
            int current_left_back_position = leftBackDrive.getCurrentPosition();
            int current_right_back_position = rightBackDrive.getCurrentPosition();
            yawPIDController.setSetpoint(0.);
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
            double counts_to_go = Math.abs(inches)*100;
            double counts_done = 0;
            while (counts_to_go > (counts_done = Math.abs(leftFrontDrive.getCurrentPosition() - current_left_front_position)) &&
                    !Thread.currentThread().isInterrupted()) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    double counts_left = counts_to_go - counts_done;
                    if (counts_to_go - counts_done < 200) {
                        drive_speed = -0.2*Math.signum(inches);
                    }
                    if (yawPIDResult.isOnTarget()) {
                        leftBackDrive.setPower(-drive_speed);
                        leftFrontDrive.setPower(drive_speed);
                        rightFrontDrive.setPower(-drive_speed);
                        rightBackDrive.setPower(drive_speed);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", drive_speed, drive_speed);
                    } else {
                        double output = yawPIDResult.getOutput();//*Math.signum(drive_speed);
                        leftFrontDrive.setPower(drive_speed + output);
                        leftBackDrive.setPower(-drive_speed + output);
                        rightBackDrive.setPower(drive_speed - output);
                        rightFrontDrive.setPower(-drive_speed - output);
                        telemetry.addData(">", "PIDOutput %.1f, %.1f", (drive_speed + output), (drive_speed - output));
                    }
                    telemetry.addData(">", "Yaw %.1f", navx_device.getYaw());
                } else {
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        drivemotors_off();
    }


}
