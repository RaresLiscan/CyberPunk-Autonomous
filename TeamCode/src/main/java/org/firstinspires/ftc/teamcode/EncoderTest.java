package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motor1 = null;
    DcMotor motor2 = null;

    @Override
    public void runOpMode()
    {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("motor1 pos: ", motor1.getCurrentPosition());
        telemetry.addData("motor2 pos: ", motor2.getCurrentPosition());

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {

                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motor1.setTargetPosition(1000);
                motor2.setTargetPosition(1000);

                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //runtime.reset();
                motor1.setPower(0.1);
                motor2.setPower(0.1);

                while (opModeIsActive()/* &&
                        (runtime.seconds() < 10)*/ &&
                        (motor1.isBusy() && motor2.isBusy())) {
                    //wait for motors to run;
                    telemetry.addData("left1 pos: ", motor1.getCurrentPosition());
                    telemetry.addData("right2 pos: ", motor2.getCurrentPosition());
                    telemetry.update();
                }

                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }
}
