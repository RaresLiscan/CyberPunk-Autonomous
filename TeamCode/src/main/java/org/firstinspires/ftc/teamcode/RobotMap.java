package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class RobotMap
{
    HardwareMap hwMap = null;
    public static final String VUFORIA_KEY = "AVkOf0j/////AAABmRjwq1ZdP0O/htcXkMim08CHBQt3z5YM6hHnfFqlJNDQbZf/M093kM6IX5wdvKvZox6Skid1Hw1FVuIr1PLvCtHY+q771YzcambEV+cAkbH/rJ3Z+0dbdiPAH6QycOPOWJqNT38H5uW8O2iXiT5IsUnlqph2E2Vl30s8ICcLl6+4TtLskwZlsUKr5QmqJROFmzMo/BCEBqmxb1njxKmjolTZcKiBGdAHKvI+Xh4rzXzJr4MO3mrDeHyLi/QIHTx5R5u6vpfQNavMEWKXA+pMCarpR/MLPwPI7anUwfRAKXaBz08it+5Fu2c1iXh/hmwLvujZVgRWQcHyAd4k7eT49cBsZvLvrhdgxcbux1YX92Oq";
    public VuforiaLocalizer vuforia;

    // Movement
    public DcMotor dreaptaFata  = null;
    public DcMotor dreaptaSpate = null;
    public DcMotor stangaFata   = null;
    public DcMotor stangaSpate  = null;

    // Collect box
    public CRServo servoCutie = null;
    public DcMotor miscareCutie = null;

    // Lander pivot / lift
    public DcMotor bratStanga  = null;
    public DcMotor bratDreapta = null;
    public Servo   servoLock   = null;
    public Servo   servoBob    = null;
    public Servo   servoCarlig = null;

    public Servo servoSenzor = null;
    public Servo servoMarker = null;

    // Motor care actioneaza sistemul cu ata ce extinde bratul
    public DcMotor extindereBrat = null;

    //Senzori
    NormalizedColorSensor colorSensorLeft = null;
    NormalizedColorSensor colorSensorRight = null;

    // Constructor
    public RobotMap() {}


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    public void init(HardwareMap aMap)
    {
        hwMap = aMap;

        initVuforia();

        // Hardware mapping
        dreaptaFata       = hwMap.get(DcMotor.class, "dreaptaFata");
        dreaptaSpate      = hwMap.get(DcMotor.class, "dreaptaSpate");
        stangaFata        = hwMap.get(DcMotor.class, "stangaFata");
        stangaSpate       = hwMap.get(DcMotor.class, "stangaSpate");
        servoLock         = hwMap.get(Servo.class, "servoLock");
        servoBob          = hwMap.get(Servo.class, "servoBob");
        servoCutie        = hwMap.get(CRServo.class, "servoCutie");
        miscareCutie      = hwMap.get(DcMotor.class, "miscareCutie");
        bratStanga        = hwMap.get(DcMotor.class, "bratStanga");
        bratDreapta       = hwMap.get(DcMotor.class, "bratDreapta");
        servoCarlig       = hwMap.get(Servo.class, "servoCarlig");
        servoSenzor       = hwMap.get(Servo.class, "servoSenzor");
        servoMarker       = hwMap.get(Servo.class, "servoMarker");
        extindereBrat     = hwMap.get(DcMotor.class, "extindereBrat");
        colorSensorRight = hwMap.get(NormalizedColorSensor.class, "colorCenter" );
        colorSensorLeft   = hwMap.get(NormalizedColorSensor.class, "colorLeft" );


        // Default initializations
        servoLock.setPosition(0);
        servoBob.setPosition(0);
    }
}