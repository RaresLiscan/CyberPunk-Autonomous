package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo   servoCarlig = null;

    public Servo servoMarker = null;

    // Motor care actioneaza sistemul cu ata ce extinde bratul
    public DcMotor extindereBrat = null;

    //Senzori
    NormalizedColorSensor colorSensorLeft = null;
    NormalizedColorSensor colorSensorRight = null;

    // Constructor
    public RobotMap() {}

    public void init(HardwareMap aMap)
    {
        hwMap = aMap;

        // Hardware mapping
        dreaptaFata       = hwMap.get(DcMotor.class, "dreaptaFata");
        dreaptaSpate      = hwMap.get(DcMotor.class, "dreaptaSpate");
        stangaFata        = hwMap.get(DcMotor.class, "stangaFata");
        stangaSpate       = hwMap.get(DcMotor.class, "stangaSpate");
        servoLock         = hwMap.get(Servo.class, "servoLock");
//        servoCutie        = hwMap.get(CRServo.class, "servoCutie");
//        miscareCutie      = hwMap.get(DcMotor.class, "miscareCutie");
        bratStanga        = hwMap.get(DcMotor.class, "bratStanga");
        bratDreapta       = hwMap.get(DcMotor.class, "bratDreapta");
        servoCarlig       = hwMap.get(Servo.class, "servoCarlig");
        servoMarker       = hwMap.get(Servo.class, "servoMarker");
        extindereBrat     = hwMap.get(DcMotor.class, "extindereBrat");
        colorSensorRight = hwMap.get(NormalizedColorSensor.class, "colorRight" );
        colorSensorLeft   = hwMap.get(NormalizedColorSensor.class, "colorLeft" );

        extindereBrat.setDirection(DcMotorSimple.Direction.REVERSE);


//        // Default initializations
        servoLock.setPosition(1);
        servoCarlig.setPosition(0);
        servoMarker.setPosition(1);



    }
}