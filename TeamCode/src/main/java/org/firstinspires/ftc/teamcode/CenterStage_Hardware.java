package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CenterStage_Hardware
{
    /* Public OpMode members. */
    //DEFINE MOTORS
//epic gamer testing 
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor slide;
    public Servo plug;
    public Servo plugArm;

    //Servo Numbers
    public final double plugDrop = 0.03;
    public final double plugOpen = 0.42;
    public final double armIn = 0.77; //change to 0.58 if any problems happen
    public final double armOut = 0.18;
    public final int cone5 = 560;
    public final int coneHome = 0;
    public final int lowJ = 1400;
    public final int medJ = 0;
    public final int highJ = 3300;



    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CenterStage_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        front_left = hwMap.get(DcMotor.class, "front_left");
        front_right = hwMap.get(DcMotor.class, "front_right");
        back_left = hwMap.get(DcMotor.class, "back_left");
        back_right = hwMap.get(DcMotor.class, "back_right");
        slide= hwMap.get(DcMotor.class, "slide");
        plug = hwMap.servo.get("plug");
        plugArm = hwMap.servo.get("plugArm");


    }
}