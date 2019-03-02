package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


import java.util.List;



@Autonomous(name = "CraterAutonomous")
//@Disabled
public class CraterAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double fa,sa,ta; //first-angle, second-angle, third-angle
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AVkOf0j/////AAABmRjwq1ZdP0O/htcXkMim08CHBQt3z5YM6hHnfFqlJNDQbZf/M093kM6IX5wdvKvZox6Skid1Hw1FVuIr1PLvCtHY+q771YzcambEV+cAkbH/rJ3Z+0dbdiPAH6QycOPOWJqNT38H5uW8O2iXiT5IsUnlqph2E2Vl30s8ICcLl6+4TtLskwZlsUKr5QmqJROFmzMo/BCEBqmxb1njxKmjolTZcKiBGdAHKvI+Xh4rzXzJr4MO3mrDeHyLi/QIHTx5R5u6vpfQNavMEWKXA+pMCarpR/MLPwPI7anUwfRAKXaBz08it+5Fu2c1iXh/hmwLvujZVgRWQcHyAd4k7eT49cBsZvLvrhdgxcbux1YX92Oq";
//
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
//     * Detection engine.
//     */
//    private TFObjectDetector tfod;

    //End of Vuforia / Tensorflow methods
    //Start of motor declarations and custom methods

    ElapsedTime runtime = new ElapsedTime();
    RobotMap robot = new RobotMap();
    RobotMovement movement = new RobotMovement(robot, runtime);
    public int L,R;


    void rotate(double power, int direction, int degrees) {
        robot.stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = power * direction;
        gyroInit();
        if (direction == 1 && fa < degrees) {
            while (fa <= degrees) {
                gyroInit();
                robot.stangaFata.setPower(power);
                robot.stangaSpate.setPower(power);
                robot.dreaptaFata.setPower(power);
                robot.dreaptaSpate.setPower(power);
                telemetry.addData("heading", fa);
                telemetry.update();
            }

            movement.stopDriving();
        }

        else if (direction == -1 && fa > degrees){
            while (fa >= degrees) {
                gyroInit();
                robot.stangaFata.setPower(power);
                robot.stangaSpate.setPower(power);
                robot.dreaptaFata.setPower(power);
                robot.dreaptaSpate.setPower(power);
                telemetry.addData("heading", fa);
                telemetry.update();
            }

            movement.stopDriving();
        }
    }


    private void releaseMinerals (double power, int timeout, int degrees, int direction) {


        robot.bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bratStanga.setTargetPosition(900);
        robot.bratDreapta.setTargetPosition(-900);

        robot.bratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratStanga.setPower(power);
        robot.bratDreapta.setPower(power);

        while (robot.bratStanga.isBusy() && robot.bratDreapta.isBusy() && opModeIsActive() && runtime.seconds() < timeout);

        robot.bratStanga.setPower(0);
        robot.bratDreapta.setPower(0);
        robot.miscareCutie.setPower(0);

        robot.servoCutie.setPower(-1);
        sleep(2000);
        robot.servoCutie.setPower(0);

        rotate(power, direction, degrees);

        robot.bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bratDreapta.setTargetPosition(-400);
        robot.bratStanga.setTargetPosition(400);

        robot.bratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.bratStanga.setPower(power);
        robot.bratDreapta.setPower(power);

        runtime.reset();

        while (robot.bratStanga.isBusy() && robot.bratDreapta.isBusy() && runtime.seconds() < timeout);

        robot.bratDreapta.setPower(0);
        robot.bratStanga.setPower(0);

        robot.bratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.miscareCutie.setPower(0.3);
        sleep(2000);
        robot.miscareCutie.setPower(0);

    }

    private void testFindGold () {
        movement.runEncoders(50, 0.25, 15);
        rotate(0.25, -1, 135);
        releaseMinerals(0.25, 15, -45, -1);
        rotate(0.25, 1, 180);
    }


    /** Presupunem ca gold mineral e la 70cm distanta de robot */

    public void leftGold()
    {
        movement.runEncoders(movement.cmToTicks(40), 0.25, 3); // Deplasare in fata ca sa nu alunece pe banda izolatoare
        movement.runEncodersLateral(movement.cmToTicks(-42), 0.3, 4); // Deplasare in fata cubului
        movement.runEncoders(movement.cmToTicks(15), 0.3, 3); // Miscare cub
        movement.runEncoders(movement.cmToTicks(-15), 0.3, 3); // Inapoi la 10 cm de pozitia cubului
        movement.runEncodersLateral(movement.cmToTicks(-120), 0.3, 4); // Deplasare spre culoar
        rotate(0.25, -1, -45); // Orientare cu spatele la depou
        telemetry.addLine();
        telemetry.addData("Position", "left");
        telemetry.update();
    }


    public void centerGold()
    {
        movement.runEncoders(movement.cmToTicks(55),0.3,10); // Miscare cub de pe pozitie
        movement.runEncoders(movement.cmToTicks(-15), 0.3, 4); // 10 cm de pozitia cubului
        movement.runEncodersLateral(movement.cmToTicks(-160), 0.3, 4); // Deplasare spre culoar
        rotate(0.25, -1, -45); // Orientare cu spatele la depou
        telemetry.addLine();
        telemetry.addData("Position", "center");
        telemetry.update();
    }


    public void rightGold()
    {
        movement.runEncoders(movement.cmToTicks(40), 0.25, 3); // Deplasare in fata ca sa nu alunece pe banda izolatoare
        movement.runEncodersLateral(movement.cmToTicks(42), 0.3, 4); // Deplasare in fata cubului
        movement.runEncoders(movement.cmToTicks(15), 0.3, 3); // Miscare cub
        movement.runEncoders(movement.cmToTicks(-15), 0.3, 3); // Inapoi la 10 cm de pozitia cubului
        movement.runEncodersLateral(movement.cmToTicks(-200), 0.3, 4); // Deplasare spre culoar
        rotate(0.25, -1, -45); // Orientare cu spatele la depou
        telemetry.addLine();
        telemetry.addData("Position", "right");
        telemetry.update();
    }


    public void afterMineral()
    {
        movement.runEncoders(movement.cmToTicks(-80), 0.3, 4); // Deplasare cu spatele in depou
        robot.servoMarker.setPosition(1); // Coborare marker
        sleep(1000);
        robot.servoMarker.setPosition(0);
        movement.runEncoders(movement.cmToTicks(175), 0.3, 7); // Deplasare cu fata spre crater

        robot.extindereBrat.setPower(0.3); // "Parcare"
        sleep(1500);
        robot.extindereBrat.setPower(0);
    }

    int goldPos = -1;


    @Override
    public void runOpMode() {

        // Color senzor
        float[] hsvValuesL = new float[3];
        final float valuesL[] = hsvValuesL;

        float[] hsvValuesR = new float[3];
        final float valuesC[] = hsvValuesR;


        robot.init(hardwareMap);

//        initVuforia();
//
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //colorSensor.enableLed(true);

        /** Wait for the game to begin */
        // telemetry.addData("heading", fa);
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        robot.stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gyroInit();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (opModeIsActive()) {

//            movement.land(0.3, 520, 10); // Detensionare servoLock

            /** Activate Tensor Flow Object Detection. */
//            if (tfod != null) {
//                tfod.activate();
//            }
//
//            runtime.reset();
//
//            while (goldPos == -1 && tfod != null && runtime.seconds() < 5) {
//                telemetry.addData("in", "while");
//                telemetry.update();
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if (updatedRecognitions.size() <= 2) {
//                        int goldMineralX = -1;
//                        int silverMineral1X = -1;
//                        int silverMineral2X = -1;
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                goldMineralX = (int) recognition.getLeft();
//                                recognition.getTop();
//                            } else if (silverMineral1X == -1) {
//                                silverMineral1X = (int) recognition.getLeft();
//                            } else {
//                                silverMineral2X = (int) recognition.getLeft();
//                            }
//                        }
//                        if (goldMineralX != -1 && (silverMineral1X != -1 || silverMineral2X != -1)) {
//                            if ((goldMineralX < silverMineral1X) || (goldMineralX < silverMineral2X)) {
//                                goldPos = 2; // center
//                            } else if ((goldMineralX > silverMineral1X) || (goldMineralX > silverMineral2X)) {
//                                goldPos = 1; // right
//                            }
//                        }
//
//                        else {
//                            goldPos = 0; // left
//                        }
//                    }
//                }
//            }
//
//            if (tfod != null) {
//                tfod.shutdown();
//            }

//            if(goldPos != -1) {
//                if (goldPos == 0) {
//                    telemetry.addData("Gold Mineral Position", "Left");
//                    leftGold();
//                    afterMineral();
//
//                } else if (goldPos == 1) {
//                    telemetry.addData("Gold Mineral Position", "Right");
//                    rightGold();
//                    afterMineral();
//
//                } else if (goldPos == 2){
//                    telemetry.addData("Gold Mineral Position", "Center");
//                    centerGold();
//                    afterMineral();
//                }
//                telemetry.addData("position", goldPos);
//                telemetry.update();
//            }
//
//
//            else {
//                // Deplasare catre cubul din dreapta
//
//

            movement.land(0.3, 100, 10);
            telemetry.addData("Robot Stopped", "!");
            movement.runEncodersLateral(movement.cmToTicks(-135), 0.85, 10); // Senzorii de culoare indreptati spre cub sau bile
            rotate(0.7, 1, 134);
            movement.runEncoders(movement.cmToTicks(150), 1, 10);
            robot.servoMarker.setPosition(0);
            sleep(500);
            robot.servoMarker.setPosition(1);
            movement.runEncoders(movement.cmToTicks(-250), 1, 7);

            runtime.reset();

//                while(runtime.seconds() < 5 && L != 1 && R != 1) {
//                    NormalizedRGBA colorsC = robot.colorSensorRight.getNormalizedColors();
//                    NormalizedRGBA colorsL = robot.colorSensorLeft.getNormalizedColors();
//
//
//                    Color.colorToHSV(colorsC.toColor(), hsvValuesR);
//                    Color.colorToHSV(colorsL.toColor(), hsvValuesL);
//
//
//                    L=hsvValuesL[0]<70&&hsvValuesL[2]<0.02 ? (hsvValuesL[0]<10 ? -1 : 1) : 0; // 1 == cub, 0==bila, -1==teren
//                    R=hsvValuesR[0]<70&&hsvValuesR[2]<0.02 ? (hsvValuesR[0]<10 ? -1 : 1) : 0;
//
//
//                    telemetry.addData("L=", L);
//                    telemetry.addData("R=", R);
//
//                    telemetry.addData("Culoare1 ", colorsC);
//                    telemetry.addData("Culoare2 ", colorsL);
//
//
//                    if (L == 1) {
//                        telemetry.addData("Cube: ", "Cube-Center");
//                    }
//
//                    else if (R == 1) {
//                        telemetry.addData("Cube: ", "Cube-Right");
//                    }
//
//                    else {
//                        telemetry.addData("Cube: ", "Cube-Left");
//                    }
//
//
//                    telemetry.update();
//                }
//
//
//                if (L == 1) {
//                    movement.runEncodersLateral(movement.cmToTicks(-23), 0.2, 3); // Deplasare cu centrul robotului in fata cubului
//                    movement.runEncoders(movement.cmToTicks(10), 0.3, 2);
//                    movement.runEncoders(movement.cmToTicks(-16), 0.3, 3); // Miscare cub si revenire la pozitia din TensorFlow ca sa nu mai gandesc atata cod :*
//                    movement.runEncodersLateral(movement.cmToTicks(-160), 0.3, 4); // Deplasare spre culoar
//                    rotate(0.25, -1, -45); // Orientare cu spatele la depou
//                }
//                else if (R == 1) {
//                    movement.runEncodersLateral(movement.cmToTicks(23), 0.5, 3); // Deplasare cu centrul robotului in fata cubului
//                    movement.runEncoders(movement.cmToTicks(10), 0.3, 2);
//                    movement.runEncoders(movement.cmToTicks(-16), 0.3, 2); // Miscare cub si revenire la pozitia din TensorFlow
//                    movement.runEncodersLateral(movement.cmToTicks(-200), 0.5, 4); // Deplasare spre culoar
//                    rotate(0.25, -1, -45); // Orientare cu spatele la depou
//                }
//                else {
//                    movement.runEncodersLateral(movement.cmToTicks(-75), 0.2, 8); // Deplasare cu centrul robotului in fata cubului
//                    movement.runEncoders(movement.cmToTicks(10), 0.3, 2);
//                    movement.runEncoders(movement.cmToTicks(-16), 0.3, 2);
//                    movement.runEncodersLateral(movement.cmToTicks(-120), 0.3, 4); // Deplasare spre culoar
//                    rotate(0.25, -1, -45); // Orientare cu spatele la depou
//                }
//                afterMineral();


        }
//
    }
//    }

    public void gyroInit() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        fa = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        sa = Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));
        ta = Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle));
    }

//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//    }
//
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}