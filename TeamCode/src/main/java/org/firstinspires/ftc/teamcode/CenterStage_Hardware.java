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
    public DcMotor slides;
    public DcMotor slidesrot;
    public Servo collection;
    public Servo collectionDrop;
    // Servo&Motor Numbers

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
        collection = hwMap.get(Servo.class, "collection");
        collectionDrop = hwMap.get(Servo.class, "collectionDrop");
        slides = hwMap.get(DcMotor.class, "slides");
        slidesrot = hwMap.get(DcMotor.class, "slidesrot");



    }
}