package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoMineralCollector")

public class CRServoTest extends LinearOpMode {

    Servo mineralCollector=null;
    double power=0.7;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mineralCollector=hardwareMap.crservo.get("mineral_collector");

        waitForStart();

        while (opModeIsActive())
        {

            if (gamepad1.a)
                mineralCollector.setPosition(0);
            if (gamepad1.b)
                mineralCollector.setPosition(1);
        }

    }


}