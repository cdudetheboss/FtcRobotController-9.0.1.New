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

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

@TeleOp(name="ExpansionHubTest", group="14174")
//@Disabled
public class ExpansionHubTest extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    private static final String VUFORIA_KEY =
            "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";
    // Declare OpMode members.

    CenterStage_Hardware robot = new CenterStage_Hardware();

    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor slide;
    public Servo plug;
    public Servo plugArm;
    double tX = 0;
    double tY = 0;
    double tZ = 0;

    double rX = 0;
    double rY = 0;
    double rZ = 0;

    double done = 0;
    double step = 0;
    double startTime = getRuntime();
    double step1Time = startTime+10;
    //final double step2Time = startTime+20;
    //final double step3Time = startTime+30;
    double rectifiedRY = ((rY+360)%360)-180;
    double DaS = Math.abs(rectifiedRY);
    double targetLock = 0;

    final double kP = 0.2;
    final double kI = 0.0;
    final double kD = 0;
    final double iLimit = 1;
    double error = 0;

    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    double dt = 0;
    double errorRate = 0;

    //other motors

    //@Override
    public void runOpMode() //throws InterruptedException
    {

        BNO055IMU imu;
        Orientation angles;

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //Drive motors

        slide = hardwareMap.get(DcMotor.class, "slide");
        plug = hardwareMap.get(Servo.class, "plug");
        plugArm = hardwareMap.get(Servo.class, "plugArm");


        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);






        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo Positions
        plug.setPosition(0.5);
        plugArm.setPosition(0.72);

        robot.init(hardwareMap);
        //Variables


        double spool = -gamepad2.left_stick_y;
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

            if (Math.abs(gamepad2.left_stick_y) > 0.01) {
                if (gamepad2.left_stick_y > 0) {
                    spool = -(gamepad2.left_stick_y * gamepad2.left_stick_y);
                } else if (gamepad2.left_stick_y < 0) {
                    spool = (gamepad2.left_stick_y * gamepad2.left_stick_y);
                }
            } else {
                spool = 0;
            }
            if (gamepad1.right_trigger > 0.1) {

            } else {
                //0.8 is wheel offset because of build team

                slide.setPower(Range.clip(spool, -1, 1));
            }

            /*if (slide.getCurrentPosition()<  0 && spool > 0 && !(gamepad1.right_trigger > 0.01)) {
               slide.setPower(0);
            } else if (slide.getCurrentPosition() > 200 && spool < 0 && !(gamepad1.right_trigger > 0.01)){
                slide.setPower(0);
            } else {
                slide.setPower(Range.clip(spool, -1, 1));
            }*/

            telemetry.addData("Servo Position", plug.getPosition());
            telemetry.addData("Spool Position", slide.getCurrentPosition());
            telemetry.addData("GamePad2Pos", gamepad2.left_stick_y);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }
}
