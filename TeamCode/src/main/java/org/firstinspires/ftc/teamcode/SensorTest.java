package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name="Sensor Test")

public class SensorTest extends LinearOpMode{

    ColorSensor colorsensor = null;
    CompassSensor compasssensor = null;
    ModernRoboticsI2cRangeSensor rangesensor = null;
    ModernRoboticsAnalogOpticalDistanceSensor distanceSensor = null;
    //CRServo crservo = null;



    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rangesensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangesensor");
        compasssensor = hardwareMap.get(CompassSensor.class, "compasssensor");
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        colorsensor.enableLed(true);

        waitForStart();

        while (opModeIsActive())
        {
            //crservo.setPower(0.2);
            telemetry.addData("Blue: ", colorsensor.blue());
            telemetry.addData("Compass Status: ", compasssensor.status());
            telemetry.addData("cm", "%.2f cm", rangesensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}