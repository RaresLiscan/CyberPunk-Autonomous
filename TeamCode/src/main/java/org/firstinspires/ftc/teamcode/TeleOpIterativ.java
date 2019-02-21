package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="TeleOp Iterativ")
public class TeleOpIterativ extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor stangaFata = null;
    private DcMotor stangaSpate = null;
    private DcMotor dreaptaFata = null;
    private DcMotor dreaptaSpate = null;
    private DcMotor stangaLiftMotor = null;
    private DcMotor dreaptaLiftMotor = null;

    RobotMap robot = new RobotMap();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        stangaFata = hardwareMap.get(DcMotor.class, "stangaFata");
        stangaSpate = hardwareMap.get(DcMotor.class, "stangaSpate");
        dreaptaFata = hardwareMap.get(DcMotor.class, "dreaptaFata");
        dreaptaSpate = hardwareMap.get(DcMotor.class, "dreaptaSpate");
        stangaLiftMotor = hardwareMap.get(DcMotor.class, "stangaLiftMotor");
        dreaptaLiftMotor = hardwareMap.get(DcMotor.class, "dreaptaLiftMotor");

        robot.init(hardwareMap);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double powerA = Boolean.compare(gamepad1.a, false);
        double powerY = Boolean.compare(gamepad1.y, false);

        // Miscare fata spate
        stangaFata.setPower(gamepad1.left_stick_y);
        stangaSpate.setPower(gamepad1.left_stick_y);
        dreaptaFata.setPower(-gamepad1.left_stick_y);
        dreaptaSpate.setPower(-gamepad1.left_stick_y);

        // Miscare stanga-dreapta (strafing)
//        stangaFata.setPower(-gamepad1.left_stick_x);
//        stangaSpate.setPower(gamepad1.left_stick_x);
//        dreaptaFata.setPower(-gamepad1.left_stick_x);
//        dreaptaSpate.setPower(gamepad1.left_stick_x);

        // Rotire
        stangaFata.setPower(-gamepad1.right_stick_x);
        stangaSpate.setPower(-gamepad1.right_stick_x);
        dreaptaFata.setPower(-gamepad1.right_stick_x);
        dreaptaSpate.setPower(-gamepad1.right_stick_x);

        // Ridicare-coborare brat
        stangaLiftMotor.setPower(powerA / 2);
        dreaptaLiftMotor.setPower(powerA / -2);
        stangaLiftMotor.setPower(powerY / -2);
        dreaptaLiftMotor.setPower(powerY / 2);

        telemetry.addData("Power A: ", powerA);
        telemetry.addData("Power Y: ", powerY);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
