package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp (name="Motion")

public class Motion extends LinearOpMode{

    DcMotor leftDrive=null;
    DcMotor rightDrive=null;
    double power=0.2;

    @Override
    public void runOpMode(){
        telemetry.addData("Status","Initialized");
        telemetry.update();

        leftDrive=hardwareMap.dcMotor.get("left_Drive");
        rightDrive=hardwareMap.dcMotor.get("right_Drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            while(gamepad1.left_stick_y!=0)
            {
                double power=gamepad1.left_stick_y;
                leftDrive.setPower(power);
                rightDrive.setPower(power);
            }
            while(gamepad1.left_stick_y!=0)
            {
                double power=gamepad1.left_stick_y;
                leftDrive.setPower(power);
                rightDrive.setPower(-power);
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

    }

}