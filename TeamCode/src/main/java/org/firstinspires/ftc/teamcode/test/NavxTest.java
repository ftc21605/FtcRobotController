package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp(name = "Test: Navx Test", group = "Test")
//@Disabled
public class NavxTest extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    @Override
    public void runOpMode() {

         navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
telemetry.addData(">", "Press Start to run.");
        telemetry.update();
        waitForStart();
        // Scan servo till stop pressed.
        while (opModeIsActive()) {

        telemetry.addData(">", "ZXY first angle %.1f", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData(">", "ZXY second angle %.1f", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
        telemetry.addData(">", "ZXY third angle %.1f", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
           telemetry.update();
        }
        // Set the servo to the new position and pause;
        //            rotator_servo.setPosition(position);
        //            sleep(CYCLE_MS);
        //idle();


    }
}
