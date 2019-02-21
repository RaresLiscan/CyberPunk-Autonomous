package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "Autonomous V1")
public class AutonomousV1 extends LinearOpMode {

    public int getColor(ColorSensor colorSensor)
    {
        if (colorSensor.red() >= 230 && colorSensor.green() >= 230 && colorSensor.blue() <= 100)
            return 1; //gold
        else if (colorSensor.red() >= 160 && colorSensor.green() >= 160 && colorSensor.blue() >= 160)
            return 2; //silver
        else
            return 0;
    }

    public void moveX(double power)
    {
        rightDrive1.setPower(power);
        rightDrive2.setPower(power);
        leftDrive1.setPower(power);
        leftDrive2.setPower(power);
    }

    DcMotor rightDrive1 = null;
    DcMotor leftDrive1  = null;
    DcMotor rightDrive2 = null;
    DcMotor leftDrive2  = null;
    ColorSensor colorSensor;
    CompassSensor compassSensor;
    ModernRoboticsI2cRangeSensor rangeSensor = null;
    boolean found = false;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Press", "start");
        telemetry.update();

        rightDrive1 = hardwareMap.get(DcMotor.class, "rightDrive1");
        leftDrive1  = hardwareMap.get(DcMotor.class, "leftDrive1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rightDrive2");
        leftDrive2  = hardwareMap.get(DcMotor.class, "leftDrive2");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        compassSensor = hardwareMap.get(CompassSensor.class, "ccompassSensor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorSensor.enableLed(true);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Current ", "mineral: ");
            if (getColor(colorSensor) == 1)
                telemetry.addData("Gold", "mineral");
            else if (getColor(colorSensor) == 2)
                telemetry.addData("Silver", "mineral");
            else
                telemetry.addData("No", "mineral");

            while (!found && rangeSensor.getDistance(DistanceUnit.CM) > 10)
            {
                moveX(0.2);
            }
            found = false;

            telemetry.update();
        }
    }
}