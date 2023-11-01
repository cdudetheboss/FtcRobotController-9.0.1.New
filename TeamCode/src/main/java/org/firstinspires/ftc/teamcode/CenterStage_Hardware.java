package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CenterStage_Hardware
{
    /* Public OpMode members. */
    //DEFINE MOTORS
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
    public final double armIn = 0.77; //change to 0.58 if any problems happen1
    public final int coneHome = 0;
    public final int lowJ = 1400;



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
        slide = hwMap.get(DcMotor.class, "slide");
        plug = hwMap.servo.get("plug");
        plugArm = hwMap.servo.get("plugArm");


    }
}