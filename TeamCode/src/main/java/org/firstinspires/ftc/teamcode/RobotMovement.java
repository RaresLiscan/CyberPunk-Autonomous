package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotMovement {

    private RobotMap robot;
    private ElapsedTime runtime;
    public RobotMovement(RobotMap aRobot, ElapsedTime aRuntime) {  // Constructor
        robot = aRobot;
        runtime = aRuntime;
    }

    int cmToTicks(int cm) {
        double wheelDiam  = 10.16;
        double gearRatio  = 2;
        int    totalTicks = 1120; // Amount of ticks for a full rotation

        int ticksPerCm = (int) ((totalTicks * gearRatio) / (wheelDiam * Math.PI));

        return cm * ticksPerCm;
    }

    void stopDriving()
    {
        robot.stangaFata.setPower(0);
        robot.stangaSpate.setPower(0);
        robot.dreaptaFata.setPower(0);
        robot.dreaptaSpate.setPower(0);
    }

    void rotateArm (int distance, double power, double timeout) {
        robot.bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bratStanga.setTargetPosition(distance);
        robot.bratDreapta.setTargetPosition(-distance);

        robot.bratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.bratStanga.setPower(power);
        robot.bratDreapta.setPower(power);

        while (robot.bratDreapta.isBusy() && robot.bratStanga.isBusy() && runtime.seconds() < timeout);

        robot.bratStanga.setPower(0);
        robot.bratStanga.setPower(0);
    }

    void runEncoders (int distance, double power, double timeoutS) {
        robot.stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.stangaFata.setTargetPosition(-distance);
        robot.dreaptaFata.setTargetPosition(distance);
        robot.stangaSpate.setTargetPosition(-distance);
        robot.dreaptaSpate.setTargetPosition(distance);

        robot.stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.stangaFata.setPower(power);
        robot.dreaptaFata.setPower(power);
        robot.stangaSpate.setPower(power);
        robot.dreaptaSpate.setPower(power);


        while ( (robot.stangaFata.isBusy() && robot.dreaptaFata.isBusy() && robot.stangaSpate.isBusy() && robot.dreaptaSpate.isBusy()) && (runtime.seconds() < timeoutS))
        {
            //Wait for the motors to run
        }

        stopDriving();
    }

    void runEncodersLateral (int distance, double power, double timeoutS) { // -1 e stanga, 1 dreapta
        robot.stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.stangaFata.setTargetPosition(-distance);
        robot.stangaSpate.setTargetPosition(distance);
        robot.dreaptaFata.setTargetPosition(-distance);
        robot.dreaptaSpate.setTargetPosition(distance);

        robot.stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();
        robot.stangaFata.setPower(power);
        robot.dreaptaFata.setPower(power);
        robot.stangaSpate.setPower(power);
        robot.dreaptaSpate.setPower(power);


        while ( (robot.stangaFata.isBusy() && robot.dreaptaFata.isBusy() && robot.stangaSpate.isBusy() && robot.dreaptaSpate.isBusy()) &&  (runtime.seconds() < timeoutS))
        {
            //Wait for the motors to run
        }

        stopDriving();
    }

    void land (double power, int distance, int timeout) {

        robot.bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bratStanga.setTargetPosition(-distance);
        robot.bratDreapta.setTargetPosition(distance);

        robot.bratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();


        robot.bratStanga.setPower(power);
        robot.bratDreapta.setPower(power);


        while ((robot.bratStanga.isBusy() && robot.bratDreapta.isBusy()) && runtime.seconds() < timeout) {
            // Wait for the motors to run
            if (runtime.seconds() >= 0.2) robot.servoCarlig.setPosition(1);
        }

        robot.bratStanga.setPower(0);
        robot.bratDreapta.setPower(0);

        runtime.reset();
        while (runtime.seconds() < 1.5);
        runtime.reset();
        robot.servoLock.setPosition(0);
//        robot.servoCarlig.setPosition(0);

        while (runtime.seconds() < 0.5);

        // Coborare brat
        robot.bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int d = cmToTicks(32);
        robot.bratStanga.setTargetPosition(-1700);
        robot.bratDreapta.setTargetPosition(1700);
        robot.stangaFata.setTargetPosition(-d);
        robot.dreaptaFata.setTargetPosition(d);
        robot.stangaSpate.setTargetPosition(-d);
        robot.dreaptaSpate.setTargetPosition(d);

        robot.bratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        robot.bratStanga.setPower(power);
        robot.bratDreapta.setPower(power);
        robot.stangaFata.setPower(0.85);
        robot.dreaptaFata.setPower(0.85);
        robot.stangaSpate.setPower(0.85);
        robot.dreaptaSpate.setPower(0.85);


        while ( ((robot.bratStanga.isBusy() && robot.bratDreapta.isBusy()) || (robot.stangaFata.isBusy() && robot.dreaptaFata.isBusy()
                && robot.stangaSpate.isBusy() && robot.dreaptaSpate.isBusy())))
        {
            if (!(robot.bratStanga.isBusy() && robot.bratDreapta.isBusy()))
            {
                robot.bratStanga.setPower(0);
                robot.bratDreapta.setPower(0);
            }

            if (!(robot.stangaFata.isBusy() && robot.dreaptaFata.isBusy() && robot.stangaSpate.isBusy() && robot.dreaptaSpate.isBusy()))
            {
                robot.stangaFata.setPower(0);
                robot.dreaptaFata.setPower(0);
                robot.stangaSpate.setPower(0);
                robot.dreaptaSpate.setPower(0);
                return;
            }

            if (!(robot.bratStanga.isBusy() && robot.bratDreapta.isBusy()) && !(robot.stangaFata.isBusy() && robot.dreaptaFata.isBusy()
                    && robot.stangaSpate.isBusy() && robot.dreaptaSpate.isBusy())) {
                robot.bratStanga.setPower(0);
                robot.bratDreapta.setPower(0);
                robot.stangaFata.setPower(0);
                robot.dreaptaFata.setPower(0);
                robot.stangaSpate.setPower(0);
                robot.dreaptaSpate.setPower(0);
                return;
            }
        }

        robot.bratStanga.setPower(0);
        robot.bratDreapta.setPower(0);
        robot.stangaFata.setPower(0);
        robot.dreaptaFata.setPower(0);
        robot.stangaSpate.setPower(0);
        robot.dreaptaSpate.setPower(0);
        return;
    }
}
