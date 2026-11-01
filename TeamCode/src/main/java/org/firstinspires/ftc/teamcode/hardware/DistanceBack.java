package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
public class DistanceBack {

    private DistanceSensor sensorDistance;
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public DistanceBack(LinearOpMode opmode) {
        myOpMode = opmode;
    }

     public void init() {
        sensorDistance = myOpMode.hardwareMap.get(DistanceSensor.class, "distance_back");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        myOpMode.telemetry.addData(">", "Backward Rev 2m distance sensor Initialized");
    }

    public double getDistanceMM()
    {
	return sensorDistance.getDistance(DistanceUnit.MM);
    }
}
