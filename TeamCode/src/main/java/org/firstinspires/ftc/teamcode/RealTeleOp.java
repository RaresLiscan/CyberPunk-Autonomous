package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanoOpModeDenisa", group="Linear OpMode")
public class MecanoOpModeDenisa extends LinearOpMode{


    private DcMotor stangaSpate = null;
    private DcMotor stangaFata = null;
    private DcMotor dreaptaFata = null;
    private DcMotor dreaptaSpate = null;
    private DcMotor leftLandingMotor = null;
    private DcMotor rightLandingMotor = null;
    private DcMotor liftMotor = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo servoBob = null;
    private CRServo servoColectare = null;
    private DcMotor motorColectare = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftLandingMotor = hardwareMap.get(DcMotor.class, "leftLandingMotor");
        rightLandingMotor = hardwareMap.get(DcMotor.class, "rightLandingMotor");
        servoBob = hardwareMap.get(Servo.class, "Bob");
        servoColectare = hardwareMap.get(CRServo.class, "servoColectare");
        motorColectare = hardwareMap.get(DcMotor.class, "motorColectare");




        // motorColectare1= hardwareMap.get(DcMotor.class, "motorColectare1");
        // motorColectare2=hardwareMap.get(DcMotor.class, "motorColectare2");
//        motorColectare = hardwareMap.get(DcMotor.class,"motorColectare");
//        bratColectare = hardwareMap.get(DcMotor.class, "bratColectare");
//        bratDepozitare = hardwareMap.get(DcMotor.class, "bratDepozitare");
//        servoDepozitare = hardwareMap.get(CRServo.class, "servoDepozitare");
//        servoColectare = hardwareMap.get(CRServo.class, "servoColectare");


//       stangaFata.setDirection(DcMotor.Direction.FORWARD);
//      stangaSpate.setDirection(DcMotor.Direction.FORWARD);
//      dreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//      dreaptaSpate.setDirection(DcMotor.Direction.REVERSE);

        leftLandingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLandingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()) {

            double fataSpate = gamepad1.left_stick_y;
            double dreaptaStanga = gamepad1.left_stick_x;
            double rotire = gamepad1.right_stick_x;


            while (fataSpate != 0) {
                dreaptaFata.setPower(-fataSpate);
                dreaptaSpate.setPower(-fataSpate);
                stangaFata.setPower(fataSpate);
                stangaSpate.setPower(fataSpate);
            }

            dreaptaFata.setPower(0);
            dreaptaSpate.setPower(0);
            stangaFata.setPower(0);
            stangaSpate.setPower(0);

//            dreaptaFata.setPower(dreaptaStanga);
//            dreaptaSpate.setPower(-dreaptaStanga);
//            stangaFata.setPower(-dreaptaStanga);
//            stangaSpate.setPower(dreaptaStanga);
//
//
//            dreaptaFata.setPower(-rotire);
//            dreaptaSpate.setPower(-rotire);
//            stangaFata.setPower(-rotire);
//            stangaSpate.setPower(-rotire);
//
//            dreaptaFata.setPower(0);
//            dreaptaSpate.setPower(0);
//            stangaFata.setPower(0);
//            stangaSpate.setPower(0);

            while (gamepad1.y)
            {
                rightLandingMotor.setPower(1);
                leftLandingMotor.setPower(-1);
            }

            while (gamepad1.a)
            {
                rightLandingMotor.setPower(-0.6);
                leftLandingMotor.setPower(0.6);
            }

            rightLandingMotor.setPower(0);
            leftLandingMotor.setPower(0);

            if (gamepad1.left_bumper)
            {
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }

            if (gamepad1.right_bumper)
            {
                leftServo.setPosition(0);
                rightServo.setPosition(1);
            }

            if (gamepad1.x)
                servoBob.setPosition(0);

            if (gamepad1.b)
                servoBob.setPosition(1);
            ///
            /// Player 2 starts here
            ///

            while (gamepad2.dpad_up)
                liftMotor.setPower(-1);

            while (gamepad2.dpad_down)
                liftMotor.setPower(0.4);

            liftMotor.setPower(0);

            while (gamepad2.y)
            {
                rightLandingMotor.setPower(1);
                leftLandingMotor.setPower(-1);
            }

            while (gamepad2.a)
            {
                rightLandingMotor.setPower(-0.6);
                leftLandingMotor.setPower(0.6);
            }

            while (gamepad2.right_trigger != 0)
                motorColectare.setPower(gamepad2.right_trigger/2);
            while (gamepad2.left_trigger != 0)
                motorColectare.setPower(-gamepad2.left_trigger/2);

            while(gamepad2.right_bumper)
                servoColectare.setPower(1);
            while(gamepad2.left_bumper)
                servoColectare.setPower(-1);

            servoColectare.setPower(0);
        }
    }
}