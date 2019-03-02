package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="CP", group="Linear Opmode")

public class CyberPunk_TeleOp extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor stangaFata = null;
    private DcMotor stangaSpate = null;
    private DcMotor dreaptaFata = null;
    private DcMotor dreaptaSpate = null;
    private DcMotor bratStanga = null;
    private DcMotor bratDreapta = null;
    private DcMotor extindereBrat = null;
    private DcMotor miscareCutie = null;

    private CRServo servoCutie = null;
    private Servo servoMarker = null;
    private Servo servoCarlig = null;
    private Servo servoLock = null;

    public static double btd(boolean b) {
        /* This function it used for converting boolean to double */
        if (b) {
            return 1;
        }
        return 0;
    }



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");
        bratStanga = hardwareMap.get(DcMotor.class, "bratStanga");
        bratDreapta = hardwareMap.get(DcMotor.class, "bratDreapta");
        extindereBrat = hardwareMap.get(DcMotor.class, "extindereBrat");
        miscareCutie = hardwareMap.get(DcMotor.class, "miscareCutie");

        servoCutie = hardwareMap.get(CRServo.class, "servoCutie");
        servoMarker = hardwareMap.get(Servo.class, "servoMarker");
        servoCarlig = hardwareMap.get(Servo.class, "servoCarlig");
        servoLock = hardwareMap.get(Servo.class, "servoLock");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        bratStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bratDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extindereBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //declaring variables
            double rotire = gamepad1.right_stick_x;
            double fata_spate = gamepad1.right_stick_y;
            double miscare_brat = gamepad2.left_stick_y;

           /*  valorile returnate de butoanele de tip dpad returneaza valori
            de tip boolean. Folosind functia btd (boolean to double), convertim
            valorile returante de butoanele de tip dpad si le stocam in variabilele
            cu nume aferente */
            double double_dpad_up = btd(gamepad2.dpad_up);
            double double_dpad_down = btd(gamepad2.dpad_down);
            double double_dpad_left= btd(gamepad1.dpad_left);
            double double_dpad_right = btd(gamepad1.dpad_right);

            double cutie_directie_fata=gamepad2.right_trigger;
            double cutie_directie_spate=gamepad2.left_trigger;
            double ridicareCutie=0.25*gamepad2.right_stick_y;
            double putere_carlig=0;
            boolean pc=false;
            double putere_lock=1;
            boolean lock=false;
            boolean latching=false;



            /* Pentru a calcula ce putere trebuie transmisa rotilor, folosim formula:
              fata_spate -/+ rotire -/+ double_dpad_left -/+ dobule_dpad_right

              semnele + sau minus sunt alese in functie de pozitia rotii si sensul de
              rotatie al motorului

              Functia functioneaza in felul urmator:
              functia calculeaza suma celor 4 variabile, suma care reprezinta
              putererea ce trebuie alocata motoarelor

              fata_spate reprezinta variabila unde stocam valorile returnate
                         de stick-ul pe care il actionam ca sa ne deplasam fata sau spate

              rotire reprezinta variabila unde stocam valorile returnate de stick-ul
              pe care il actionam ca sa ne rotim

              folosim double_dpad_left pentru a aloca motoroaleor putrea necesara pentru
                                       a efectua strafing in stanga
              folosim double_dpad_right pentru a aloca motoroaleor putrea necesara pentru
                                       a efectua strafing in dreapta

             cu ajutorul acestei functii, robotul poate face miscari combinate cum ar fi:
             strafing left si rotire, miscare fata si strafing, rotire si miscare fata
             etc.. */



            stangaFata.setPower(fata_spate - rotire + double_dpad_left - double_dpad_right);
            stangaSpate.setPower(fata_spate - rotire - double_dpad_left + double_dpad_right);
            dreaptaFata.setPower(-fata_spate - rotire + double_dpad_left - double_dpad_right);
            dreaptaSpate.setPower(-fata_spate - rotire - double_dpad_left + double_dpad_right);

            bratStanga.setPower(miscare_brat*0.8);
            bratDreapta.setPower(-miscare_brat*0.8);
            extindereBrat.setPower(double_dpad_up*0.7-double_dpad_down*0.7);

            miscareCutie.setPower(ridicareCutie);
            servoCutie.setPower(cutie_directie_fata - cutie_directie_spate);

            if(gamepad1.dpad_down) pc=!pc;
            if(pc)putere_carlig=1;
            if(gamepad1.dpad_up) lock=!lock;
            if(lock)putere_lock=0;
            servoLock.setPosition(putere_lock);
            servoCarlig.setPosition(putere_carlig);


            if(gamepad1.b && gamepad2.b){
                servoCarlig.setPosition(1);

                bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                bratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                bratStanga.setTargetPosition(-2500);
                bratDreapta.setTargetPosition(2500);

                bratStanga.setPower(0.4);
                bratDreapta.setPower(0.4);

                while(bratStanga.isBusy() && bratDreapta.isBusy() && opModeIsActive())
                {}

                servoCarlig.setPosition(0);

                bratStanga.setPower(0);
                bratDreapta.setPower(0);

                bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                latching=false;

            }
        }
    }
}