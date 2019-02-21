package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Trigonometrie TeleOp", group="Linear Opmode")
public class Trigonometrie extends LinearOpMode {

    private DcMotor stangaSpate=null;
    private DcMotor stangaFata=null;
    private DcMotor dreaptaFata=null;
    private DcMotor dreaptaSpate=null;
//    private DcMotor motorColectare1=null;
//  private DcMotor motorColectare2=null;
//    private DcMotor motorColectare=null;
//    private DcMotor bratColectare=null;
//    private DcMotor bratDepozitare=null;
//    private CRServo servoDepozitare=null;
//    private CRServo servoColectare=null;


    private Servo servo_senzor_left = null;
    private Servo servo_senzor_right = null;
    private NormalizedColorSensor colorSensorLeft = null;
    private NormalizedColorSensor colorSensorCenter = null;
    private DcMotor leftLandingMotor = null;
    private DcMotor rightLandingMotor = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo servoBob = null;

    int direction = 1;
    private ElapsedTime runtime = new ElapsedTime();

    private void runLandingMotors (double power, int distance, int timeout) {
        leftLandingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLandingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLandingMotor.setTargetPosition(-distance);
        rightLandingMotor.setTargetPosition(distance);

        leftLandingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLandingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        leftLandingMotor.setPower(power);
        rightLandingMotor.setPower(power);

        while (leftLandingMotor.isBusy() && rightLandingMotor.isBusy() && opModeIsActive() && runtime.seconds() < timeout) {
            // Wait for the motors to run
        }

        leftLandingMotor.setPower(0);
        rightLandingMotor.setPower(0);
    }

    private void land (double power, int distance, int timeout) {
        leftLandingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLandingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLandingMotor.setTargetPosition(-distance);
        rightLandingMotor.setTargetPosition(distance);

        leftLandingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLandingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        leftLandingMotor.setPower(power);
        rightLandingMotor.setPower(power);

        while (leftLandingMotor.isBusy() && rightLandingMotor.isBusy() && opModeIsActive() && runtime.seconds() < timeout) {
            leftServo.setPosition(1);
            rightServo.setPosition(1);
        }

        leftLandingMotor.setPower(0);
        rightLandingMotor.setPower(0);
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        stangaSpate = hardwareMap.get(DcMotor.class,"stangaSpate");
        stangaFata = hardwareMap.get(DcMotor.class,"stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class,"dreaptaFata");
        dreaptaSpate = hardwareMap.get(DcMotor.class,"dreaptaSpate");


        servo_senzor_left = hardwareMap.get(Servo.class, "servoSensorLeft" );
        servo_senzor_right = hardwareMap.get(Servo.class, "servoSensorRight" );
        colorSensorCenter = hardwareMap.get(NormalizedColorSensor.class, "colorCenter" );
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorLeft" );
        leftLandingMotor = hardwareMap.get(DcMotor.class, "leftLandingMotor");
        rightLandingMotor = hardwareMap.get(DcMotor.class, "rightLandingMotor");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        servoBob = hardwareMap.get(Servo.class, "Bob");



        // motorColectare1= hardwareMap.get(DcMotor.class, "motorColectare1");
        // motorColectare2=hardwareMap.get(DcMotor.class, "motorColectare2");
//        motorColectare = hardwareMap.get(DcMotor.class,"motorColectare");
//        bratColectare = hardwareMap.get(DcMotor.class, "bratColectare");
//        bratDepozitare = hardwareMap.get(DcMotor.class, "bratDepozitare");
//        servoDepozitare = hardwareMap.get(CRServo.class, "servoDepozitare");
//        servoColectare = hardwareMap.get(CRServo.class, "servoColectare");
//
//        bratDepozitare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bratColectare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



      /*  stangaFata.setDirection(DcMotor.Direction.FORWARD);
        stangaSpate.setDirection(DcMotor.Direction.FORWARD);
        dreaptaFata.setDirection(DcMotor.Direction.REVERSE);
        dreaptaSpate.setDirection(DcMotor.Direction.REVERSE); */

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        waitForStart();
        double servoPos=0;
        double distanta1=-50;
        double distanta=-410;
        while (opModeIsActive()) {

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = java.lang.Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y,-gamepad1.left_stick_x)-Math.PI/4;
            double rightX=-gamepad1.right_stick_x;
            //double extindereBrat = gamepad2.left_stick_y * 0.5;

            double v1=r*Math.cos(robotAngle)+rightX;
            double v2=r*Math.sin(robotAngle)-rightX;
            double v3=r*Math.sin(robotAngle)+rightX;
            double v4=r*Math.cos(robotAngle)-rightX;


//            double power1 = gamepad2.left_bumper ? -1 : 0;
//            double power2 = gamepad2.right_bumper ? 1 : 0;
//            if (gamepad2.left_bumper) {
//                servoDepozitare.setPower(power1 * 0.5);
//            }
//            else if (gamepad2.right_bumper) servoDepozitare.setPower(power2 * 0.5);
//            else servoDepozitare.setPower(0);

//
//            double yPower1 = gamepad1.y ? 1 : 0;
//            double aPower1 = gamepad1.a ? 1 : 0;

//            motorColectare.setPower((gamepad2.right_stick_y)*0.4);
//
//
//            bratColectare.setPower(-extindereBrat);


//            double yPower2 = gamepad2.y ? 1 : 0;
//            double aPower2 = gamepad2.a ? 1 : 0;
            // motorColectare1.setPower(yPower2);
            // motorColectare2.setPower(aPower2);
//            double brat1=bratDepozitare.getCurrentPosition();

//            if(yPower2!=0){
//                servoColectare.setPower(-yPower2);}
//            else if(aPower2!=0){
//                servoColectare.setPower(aPower2);}
//            else servoColectare.setPower(0);
//            bratDepozitare.setPower(gamepad2.left_trigger * 0.5);
//            bratDepozitare.setPower(-gamepad2.right_trigger * 0.5);

            if (gamepad1.right_bumper)
                servoBob.setPosition(1);

            if (gamepad1.left_bumper)
                servoBob.setPosition(0);

            if (gamepad1.y) {
                runLandingMotors(0.4, 500, 10);
                leftServo.setPosition(1);
                leftServo.setPosition(1);
            }

            if (gamepad1.a) {
                land(0.3, 500, 3);
            }

            stangaFata.setPower(v1 * 0.5);
            dreaptaFata.setPower(v2 * 0.5);
            stangaSpate.setPower(v3 * 0.5);
            dreaptaSpate.setPower(v4 * 0.5);

//            telemetry.addData("Depozitare", bratDepozitare.getCurrentPosition());
            telemetry.update();
        }
    }
}