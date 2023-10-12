package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CenterStage_Hardware;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="robot teleop (old)", group="14174")
//@Disabled
public class NormalBoringTeleop extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    private static final String VUFORIA_KEY =
            "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";
    // Declare OpMode members.

    CenterStage_Hardware robot = new CenterStage_Hardware();

    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor slide;
    public Servo plug;
    public Servo plugArm;

    //other motors

    //@Override
    public void runOpMode() //throws InterruptedException
    {

        BNO055IMU imu;
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);


        //Drive motors
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        slide = hardwareMap.get(DcMotor.class, "slide");
        plug = hardwareMap.get(Servo.class, "plug");
        plugArm = hardwareMap.get(Servo.class, "plugArm");

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);







        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        //Servo Speed

        //Servo Positions
        plug.setPosition(0.03);
        plugArm.setPosition(0.69);

        robot.init(hardwareMap);

        //Variables

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;
        double spool = -gamepad2.right_stick_y;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //waitForStart();
        runtime.reset();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();

        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //-----------------------------------------------------------------------------------------
            //-----------------------------------------------------------------------------------------
            // DRIVING CONTROLS
            //-----------------------------------------------------------------------------------------
            //-----------------------------------------------------------------------------------------


            if (Math.abs(gamepad1.left_stick_y) > 0.01) {
                if (gamepad1.left_stick_y > 0) {
                    forward = -(gamepad1.left_stick_y * gamepad1.left_stick_y);
                } else if (gamepad1.left_stick_y < 0) {
                    forward = (gamepad1.left_stick_y * gamepad1.left_stick_y);
                }
            } else {
                forward = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) > 0.01) {
                if (gamepad1.left_stick_x > 0) {
                    right = gamepad1.left_stick_x * gamepad1.left_stick_x;
                } else if (gamepad1.left_stick_x < 0) {
                    right = -(gamepad1.left_stick_x * gamepad1.left_stick_x);
                }
            } else {
                right = 0;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.01) {
                if (gamepad1.right_stick_x > 0) {
                    clockwise = gamepad1.right_stick_x * gamepad1.right_stick_x;
                } else if (gamepad1.right_stick_x < 0) {
                    clockwise = -(gamepad1.right_stick_x * gamepad1.right_stick_x);
                }
            } else {
                clockwise = 0;
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                if (gamepad2.right_stick_y > 0) {
                    spool = -(gamepad2.right_stick_y * gamepad2.right_stick_y);
                } else if (gamepad2.right_stick_y < 0) {
                    spool = (gamepad2.right_stick_y * gamepad2.right_stick_y);
                }
            } else {
                spool = 0;
            }
            if (gamepad1.right_bumper) {
                right = -right;
            }
           if((forward > 0) &! (gamepad1.left_trigger > 0.1)) {
               clockwise = Range.clip(clockwise + (-0.035 * forward) + (0.035 * right), -1, 1 );
           }
           if((forward < 0) &! (gamepad1.left_trigger > 0.1)) {
               clockwise = Range.clip(clockwise + (-0.025 * forward) + (0.035 * right), -1, 1 );
           }
            if (gamepad1.right_trigger > 0.1 ) {
                front_left.setPower(0.25 * Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(0.25 * Range.clip(forward - clockwise - right, -1, 1));
                back_left.setPower(0.25 * Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(0.25 * Range.clip(forward - clockwise + right, -1, 1));
            }else if (gamepad1.left_trigger > 0.1) {
                front_left.setPower(Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(Range.clip(forward - clockwise - right, -1, 1));
                back_left.setPower(Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(Range.clip(forward - clockwise + right, -1, 1));
            }
            else  {
                front_left.setPower(0.5 * Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(0.5 * Range.clip(forward - clockwise - right, -1, 1));
                back_left.setPower(0.5 * Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(0.5 * Range.clip(forward - clockwise + right, -1, 1));
            }
            if (spool !=0) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setPower(spool);
            } else  if (spool == 0 && gamepad2.right_trigger > 0.01){
                slide.setTargetPosition(slide.getCurrentPosition());
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setPower(0);
            }


            //ServoTest
            if(gamepad1.a) {
                plug.setPosition(0.42);
            }
            if (gamepad1.b) {
                plug.setPosition(0.03);
            }
            if(gamepad2.a) {
                plugArm.setPosition(0.69);
            }
            if(gamepad2.b) {
                plugArm.setPosition(0.1);
            }
            if(gamepad2.x) {
                plugArm.setPosition(plugArm.getPosition() - 0.01);
            }
            if(gamepad2.y) {
                plugArm.setPosition(plugArm.getPosition() + 0.01);
            }
            telemetry.addData("Servo Position", plug.getPosition());
            telemetry.addData("Servo Position", plugArm.getPosition());
            telemetry.addData("Spool Position", slide.getCurrentPosition());
            telemetry.addData("GamePad2Pos", gamepad2.left_stick_y);
            telemetry.addData("Status", "Running");
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.update();
        }

    }
}
//530 With 5 Cone; 400 With 4 Cone; 315 with 3 Cone; 200 with 2 Cone; 50 or under with 1 cone.

