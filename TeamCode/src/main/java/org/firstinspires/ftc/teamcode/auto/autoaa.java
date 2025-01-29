package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Distance;
import org.firstinspires.ftc.teamcode.hardware.DistanceBack;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Rotator;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "auto aa", group = "Wallace")
//@Disabled
public class autoaa extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // 4x and 5x gear boxes.
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) * .85;
    static final double DRIVE_SPEED = -0.3;
    static final double TURN_SPEED = -0.2;
    static final double MAX_POS = 0.15;     // Maximum rotational position
    static final double MIN_POS = 0.5;     // Minimum rotational position
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)
    private final ElapsedTime runtime = new ElapsedTime();
    private final Distance distance = new Distance(this);
    private final DistanceBack distance_back = new DistanceBack(this);
    boolean skip_opencv = false;
    /* Declare OpMode members. */
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    boolean run_with_distance_sensor = false;
    //int DESIRED_TAG_ID = 5;    // Choose the tag you want to approach or set to -1 for ANY tag.
    DriveTrain drive_train = new DriveTrain(this);
    Arm arm = new Arm(this);
    Rotator rotator = new Rotator(this);
    Grabber grabber = new Grabber(this);
    Slide slide = new Slide(this);
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = .5;        // Desired forward power/speed (-1 to +1) +ve is forward
        double turn = .5;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise

        drive_train.init();
        distance.init();
        distance_back.init();
        arm.init();
        slide.init();
        grabber.init();
        rotator.init();
        initAprilTag();
        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
        sleep(500);
        //	rotator.initpos();
        grabber.grab();

        // navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        // gyro = navxMicro;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //www.homeefficiencyexperts.com

        // Send telemetry message to indicate successful Encoder reset
        //BlueFinder.Selected selected;
        // here is what happens after we hit start
        arm.move(0.1);
        while (!isStarted() && !isStopRequested()) {
        }
        targetFound = false;
        desiredTag = null;
        int detection_id = 0;
        int isleep = 0;
        drive_train.coast();
        telemetry.addData(">", "Press A to proceed");
        telemetry.update();
        while (!run_with_distance_sensor) {
            targetFound = false;
            while (!targetFound) {
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
                            telemetry.addData("found tag ", "%d", detection.id);
                            telemetry.update();

                            break;  // don't look any further.
                        }
                    }
                }
		if (currentDetections == null || currentDetections.isEmpty())
		    {
			                telemetry.addData("distance, press A", "%5.2f",  distance_back.getDistanceMM());
                telemetry.update();

			drive_train.Stop();
			
			                if (!gamepad1.a) {
                    sleep(1);
                }

		    }
		sleep(1); // target not found. sleep and try again
            }
            // Determine heading and range error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double dist = distance_back.getDistanceMM();
            telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            telemetry.addData("Range/heading error", "Range %5.2f, Heading %5.2f", rangeError, headingError);
            telemetry.addData("desiredTag.ftcPose.range", "%5.2f", desiredTag.ftcPose.range);
            telemetry.addData("distance back", "%5.2f", dist);
            telemetry.update();
            if (dist < 450) { // 410 april tags out of vision
                run_with_distance_sensor = true;
                telemetry.addData("distance < 470, press A", "%5.2f", dist);
                telemetry.update();
		break;
            }
            // if (desiredTag.ftcPose.range < 0) {
            //     run_with_distance_sensor = true;
            // 	telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            // 	telemetry.addData("Range/heading error", "Range %5.2f, Heading %5.2f", rangeError, headingError);
            // 	telemetry.addData("Range/heading error", "Range %5.2f, Heading %5.2f", rangeError, headingError);
            // 	telemetry.addData("all done at", "Range %5.2f", desiredTag.ftcPose.range);
            // 	telemetry.update();
            // 	break;
            // }
            drive_train.moveRobot(drive, turn);
            sleep(10);
        }
        while (run_with_distance_sensor) {

		    telemetry.addData("distance: ", "%5.2f", distance_back.getDistanceMM());
		    telemetry.update();
            if (distance_back.getDistanceMM() > 30) {
                drive = MAX_AUTO_SPEED/2.;
            } else {
                drive = 0;
                run_with_distance_sensor = false;
		drive_train.Stop();
		break;
            }
	}
        drive_train.Stop();
                 telemetry.addData(">", "All done, press A");
                telemetry.update();
               if (!gamepad1.a) {
                    sleep(1);
                }

    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
      Manually set the camera gain and exposure.
      This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

}
