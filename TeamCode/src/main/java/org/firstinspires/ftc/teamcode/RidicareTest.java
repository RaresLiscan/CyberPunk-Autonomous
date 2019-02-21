package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RidicareTest")

public class RidicareTest extends LinearOpMode {

    DcMotor ridicareDreapta = null;
    DcMotor ridicareStanga = null;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ridicareDreapta = hardwareMap.get(DcMotor.class, "ridicareDreapta");
        ridicareStanga = hardwareMap.get(DcMotor.class, "ridicareStanga");

        waitForStart();

        while (opModeIsActive())
        {
            while (gamepad1.a)
            {
                ridicareDreapta.setPower(1);
                ridicareStanga.setPower(-1);
            }

            while (gamepad1.b)
            {
                ridicareDreapta.setPower(-1);
                ridicareStanga.setPower(1);
            }

            ridicareStanga.setPower(0);
            ridicareDreapta.setPower(0);
        }

    }


}

