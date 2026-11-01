package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class navx {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    // Define a constructor that allows the OpMode to pass a reference to itself.
    private NavxMicroNavigationSensor navxMicro;
    private IntegratingGyroscope gyro;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    public navx(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        navxMicro = myOpMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navxMicro;
        myOpMode.telemetry.addData(">", "Navx Initialized");
    }

    void setLeftFrontDriveMotor(DcMotor motor) {
        leftFrontDrive = motor;
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions using the input values from the controler.
     * Then sends these power levels to the motors.
     */
    double right_turn(double ANGLE) {
        double TURN_SPEED = 0.2;
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current_angle = angles.firstAngle;
        double driveto_angle = current_angle - ANGLE;
        myOpMode.telemetry.addData("right turn: ", "input: %.1f, current angle %.1f, driveto: %.1f", ANGLE, current_angle, driveto_angle);
        myOpMode.telemetry.update();
        leftFrontDrive.setPower(TURN_SPEED);
        rightFrontDrive.setPower(-TURN_SPEED);
        leftBackDrive.setPower(TURN_SPEED);
        rightBackDrive.setPower(-TURN_SPEED);
        if (driveto_angle < -180) {
            //current_angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            while ((current_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 0 || current_angle > (driveto_angle + 360)) {
                myOpMode.telemetry.addData(">", "angle %.1f", current_angle);
                myOpMode.telemetry.update();
                //sleep(1);
            }
        } else {
            while ((current_angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > driveto_angle) {
                myOpMode.telemetry.addData(">", "angle %.1f", current_angle);
                myOpMode.telemetry.update();
                //sleep(1);
            }
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        return driveto_angle;
    }

}
