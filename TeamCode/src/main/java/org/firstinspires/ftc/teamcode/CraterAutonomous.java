package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;


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

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //End of Vuforia / Tensorflow methods
    //Start of motor declarations and custom methods

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor dreaptaFata = null;
    DcMotor dreaptaSpate = null;
    DcMotor stangaFata = null;
    DcMotor stangaSpate = null;
    //DcMotor motorColectare = null;
    Servo marker = null;
    Servo sensorServo = null;
    Servo leftServo = null;
    Servo rightServo = null;
    Servo servoBob = null;
    NormalizedColorSensor colorSensorLeft = null;
    NormalizedColorSensor colorSensorCenter = null;
    DistanceSensor distanceDreapta = null;
    DistanceSensor distanceStanga = null;
    DcMotor leftLandingMotor = null;
    DcMotor rightLandingMotor = null;

    //CRServo servoColectare = null;
    //DcMotor motorColectare = null;
    //ModernRoboticsI2cColorSensor colorSensor = null;

    public void stopDriving()
    {
        stangaFata.setPower(0);
        stangaSpate.setPower(0);
        dreaptaFata.setPower(0);
        dreaptaSpate.setPower(0);
    }


    public int L,C;
    public void runEncoders (int distance, double power, double timeoutS) {
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(-distance);
        dreaptaFata.setTargetPosition(distance);
        stangaSpate.setTargetPosition(-distance);
        dreaptaSpate.setTargetPosition(distance);

        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);


        while ( (stangaFata.isBusy() && dreaptaFata.isBusy() && stangaSpate.isBusy() && dreaptaSpate.isBusy()) && opModeIsActive() && (runtime.seconds() < timeoutS))
        {
            //Wait for the motors to run
        }

        stopDriving();
    }


    public void runLateral (int direction, int distance, double power, double timeoutS) {   ///Direction = 1 => stanga / Direction = -1 => dreapta
        ///Mers stanga:
        if (direction == 1) {
            while (distanceStanga.getDistance(DistanceUnit.CM) > distance && runtime.seconds() < timeoutS) {
                stangaFata.setPower(power);
                stangaSpate.setPower(-power);
                dreaptaFata.setPower(power);
                dreaptaSpate.setPower(-power);
            }

            stangaFata.setPower(0);
            stangaSpate.setPower(0);
            dreaptaFata.setPower(0);
            dreaptaSpate.setPower(0);
        }

        ///Mers dreapta
        else if (direction == -1) {
            while (distanceDreapta.getDistance(DistanceUnit.CM) > distance && runtime.seconds() < timeoutS) {
                stangaFata.setPower(power);
                stangaSpate.setPower(-power);
                dreaptaFata.setPower(-power);
                dreaptaSpate.setPower(power);
            }

            stangaFata.setPower(0);
            stangaSpate.setPower(0);
            dreaptaFata.setPower(0);
            dreaptaSpate.setPower(0);
        }
    }

    public void runEncodersLateral (int distance, double power, double timeoutS) {
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaFata.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
        stangaSpate.setTargetPosition(-distance);
        dreaptaSpate.setTargetPosition(distance);

        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);


        while ( (stangaFata.isBusy() && dreaptaFata.isBusy() && stangaSpate.isBusy() && dreaptaSpate.isBusy()) && opModeIsActive() && (runtime.seconds() < timeoutS))
        {
            //Wait for the motors to run
        }

        stopDriving();
    }


    void rotate(double power, int direction, int degrees) {
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = power * direction;
        gyroInit();
        if (direction == 1 && fa < degrees) {
            while (fa <= degrees) {
                gyroInit();
                stangaFata.setPower(power);
                stangaSpate.setPower(power);
                dreaptaFata.setPower(power);
                dreaptaSpate.setPower(power);
                telemetry.addData("heading", fa);
                telemetry.update();
            }

            stopDriving();
        }

        else if (direction == -1 && fa > degrees){
            while (fa >= degrees) {
                gyroInit();
                stangaFata.setPower(power);
                stangaSpate.setPower(power);
                dreaptaFata.setPower(power);
                dreaptaSpate.setPower(power);
                telemetry.addData("heading", fa);
                telemetry.update();
            }

            stopDriving();
        }
    }

    public void leftGold()
    {
        runEncoders(625, 0.25, 15);
        rotate(0.25,1, 40);
        runEncoders(1550, 0.25, 30);
        telemetry.addLine();
        telemetry.addData("Position", "left");
        telemetry.update();
    }


    public void centerGold()
    {
        runEncoders(650,0.05,15);
        runEncoders(2300,0.15,15);
        telemetry.addLine();
        telemetry.addData("Position", "center");
        telemetry.update();
    }


    public void rightGold()
    {
        runEncoders(625, 0.25, 15);
        rotate(0.25,-1, -40);
        runEncoders(1727, 0.25, 30);
        telemetry.addLine();
        telemetry.addData("Position", "right");
        telemetry.update();
    }


    public void afterMineral(int goldPos)
    {
        /* Daca minerala a fost in stanga, robotul mi se invarte 45 la dreapta, analog pt celalat if*/
        if(goldPos==0) //left
        {
            rotate(0.25, -1, -40);
            runEncoders(1850,0.25,15);
        }

        if(goldPos==1) //right
        {
            rotate(0.25, 1, 35);
            runEncoders(1617,0.25,15);
            rotate(0.25, -1, -45);
        }

        /*Dau drumul la marker*/
        marker.setPosition(0);
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

        leftServo.setPosition(1);
        rightServo.setPosition(0);

        while (leftLandingMotor.isBusy() && rightLandingMotor.isBusy() && opModeIsActive() && runtime.seconds() < timeout) {
            // Wait for the motors to run
        }

        leftLandingMotor.setPower(0);
        rightLandingMotor.setPower(0);
    }

    int goldPos = -1;


    @Override
    public void runOpMode() {

        // Color senzor
        float[] hsvValuesL = new float[3];
        final float valuesL[] = hsvValuesL;

        float[] hsvValuesC = new float[3];
        final float valuesC[] = hsvValuesC;

        float[] hsvValuesR = new float[3];
        final float valuesR[] = hsvValuesR;



        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        stangaSpate = hardwareMap.get(DcMotor.class,"stangaSpate");
        stangaFata = hardwareMap.get(DcMotor.class,"stangaFata");
        dreaptaFata = hardwareMap.get(DcMotor.class,"dreaptaFata");
        dreaptaSpate = hardwareMap.get(DcMotor.class,"dreaptaSpate");
        sensorServo = hardwareMap.get(Servo.class, "servoSensorLeft" );
        colorSensorCenter = hardwareMap.get(NormalizedColorSensor.class, "colorCenter" );
        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorLeft" );
        distanceDreapta = hardwareMap.get(DistanceSensor.class, "distanceDreapta");
        distanceStanga = hardwareMap.get(DistanceSensor.class, "distanceStanga");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        servoBob = hardwareMap.get(Servo.class, "Bob");
        marker = hardwareMap.get(Servo.class, "marker");
        leftLandingMotor = hardwareMap.get(DcMotor.class, "leftLandingMotor");
        rightLandingMotor = hardwareMap.get(DcMotor.class, "rightLandingMotor");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //colorSensor.enableLed(true);

        /** Wait for the game to begin */
        // telemetry.addData("heading", fa);
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLandingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLandingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        marker.setPosition(1);
        sensorServo.setPosition(0.1);
        leftServo.setPosition(1);
        rightServo.setPosition(0);
        servoBob.setPosition(1);
        gyroInit();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
//
//
//            if (tfod != null) {
//                tfod.activate();
//            }
//
//
//
//            runtime.reset();
//
//            while (goldPos == -1 && tfod != null && runtime.seconds() < 5) {
//                telemetry.addData("in","while" );
//                telemetry.update();
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if (updatedRecognitions.size() <= 3) {
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
//                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                goldPos = 0;
//                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                goldPos = 1;
//                            } else {
//                                goldPos = 2;
//                            }
//                        }
//                    }
//
//                }
//            }
//
//            tfod.shutdown();

            land(0.3, 500, 5);

            if(goldPos != -1) {
                if (goldPos == 0) {
                    telemetry.addData("Gold Mineral Position", "Left");
                    leftGold();
                    afterMineral(goldPos);

                } else if (goldPos == 1) {
                    telemetry.addData("Gold Mineral Position", "Right");
                    rightGold();
                    afterMineral(goldPos);

                } else if (goldPos == 2){
                    telemetry.addData("Gold Mineral Position", "Center");
                    centerGold();
                    afterMineral(goldPos);
                }
                telemetry.addData("position", goldPos);
                telemetry.update();

                sleep(3000);
            }


            else {

                runEncoders(2650, 0.15, 10);
                sensorServo.setPosition(1);
                runtime.reset();

                while(runtime.seconds() < 5) {
                    NormalizedRGBA colorsC = colorSensorCenter.getNormalizedColors();
                    NormalizedRGBA colorsL = colorSensorLeft.getNormalizedColors();


                    Color.colorToHSV(colorsC.toColor(), hsvValuesC);
                    Color.colorToHSV(colorsL.toColor(), hsvValuesL);


                    L=hsvValuesL[0]<70&&hsvValuesL[2]<0.02 ? (hsvValuesL[0]<10 ? -1 : 1) : 0; // 1 == cub, 0==bila, -1==teren
                    C=hsvValuesC[0]<70&&hsvValuesC[2]<0.02 ? (hsvValuesC[0]<10 ? -1 : 1) : 0;


                    telemetry.addData("rangeDreapta", String.format("%.01f cm", distanceDreapta.getDistance(DistanceUnit.CM)));
                    telemetry.addData("rangeStanga", String.format("%.01f cm", distanceStanga.getDistance(DistanceUnit.CM)));
                    telemetry.addData("L=", L);
                    telemetry.addData("C=", C);


                    if (L == 1) {
                        telemetry.addData("Cube: ", "Left");
                    }
                    else if (C == 1) {
                        telemetry.addData("Cube: ", "Center");
                    }
                    else {
                        telemetry.addData("Cube: ", "Right");
                    }


                    telemetry.update();
                }

                sensorServo.setPosition(0.1);

                if (L == 1) {
                    runLateral(1,50, 0.15, 3);
                    runEncoders(351, 0.15, 6);
                    runEncoders(-500, 0.15, 6);
                    runLateral(1,7,0.15,4);
                    rotate(0.15,1,135);
                    runEncodersLateral(3000,0.2,9);
                    marker.setPosition(1);
                    sleep(1500);
                    marker.setPosition(0.1);
                    runEncodersLateral(1000,0.2,5);
                    rotate(0.15,1, );

                }
                else if (C == 1) {
                    runEncoders(2500,0.15,6);
                    rotate(0.15,1,45);
                    marker.setPosition(0);
                    sleep(2500);
                    marker.setPosition(1);
                    runEncodersLateral(1250, 0.25, 10);
                    rotate(0.2,-1,180);
                    runEncoders(-1,0.25,3);
                }
                else {
                    runLateral(-1,50,0.15,3);
                    runEncoders(800,0.2,6);
                    runLateral(1, 40, 0.15, 4);
                    runEncoders(600, 0.15, 4);
                    rotate(0.15,1,45);
                    runEncoders(1000,0.2,4);
                    marker.setPosition(0);
                    sleep(2500);
                    marker.setPosition(1);
                    runEncodersLateral(1250, 0.25, 10);
                    rotate(0.2,-1,135);
                    runEncoders(-1,0.25,3);
                }


            }

        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void gyroInit() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        fa = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        sa = Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));
        ta = Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle));
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}