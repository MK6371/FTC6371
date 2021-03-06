package org.firstinspires.ftc.teamcode.test;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HolonomicRobot;
import org.firstinspires.ftc.teamcode.legacycode.PIDControllerLegacy;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by 292486 on 2/24/2017.
 */

@Autonomous(name="Vision Move", group="vision")
public class VisionMoveTest extends LinearVisionOpMode {

    private HolonomicRobot robot = new HolonomicRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        robot.init(hardwareMap);

        /*
        *** Vision configuration/setup
         */


        this.setCamera(Cameras.SECONDARY);  //Front-facing camera
        this.setFrameSize(new Size(900, 900));  //Frame to analyze (bigger = slower but more accurate)

        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);   //Corrects screen rotation
        //enableExtension(Extensions.CAMERA_CONTROL);

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);   //Complex, default, realtime
        //beacon.setAnalysisBounds(new Rectangle(new Point(width/2, height/2), width-200, 200));  //Cropping. Use if we can approximate location

        beacon.setColorToleranceBlue(0);  // -1 ~ 1 (0 default)
        beacon.setColorToleranceRed(0);   // -1 ~ 1 (0 default)

        rotation.setIsUsingSecondaryCamera(true);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);    //Can set white balance values for better
        cameraControl.setAutoExposureCompensation();

        /*
        *** Vision configuration/setup end
         */

        /*
        *** Read parameters from phone configuration
         */

        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        int alliance = Integer.parseInt(sharedPreferences.getString("mk_color", "0"));
        long delayTime = Long.parseLong(sharedPreferences.getString("mk_delayTime", "5000"));
        String allianceColor = (alliance == 0) ? "Blue" : "Red";
        double lSpeed = Double.parseDouble(sharedPreferences.getString("mk_kp", "-0.5"));
        double rSpeed = Double.parseDouble(sharedPreferences.getString("mk_ki", "0.5"));
        boolean goForTwo = sharedPreferences.getBoolean("mk_shootBalls", false);

        /*******************************/
        waitForStart();
        /*******************************/

        /*
        *** Move diagonal to the white line
         */

        double light = 1.5; //priming the while loop
        robot.lightFloor.enableLed(true);
        sleep(1000);    //let the LED turn on
        if (alliance == 0) //if blue team
        {
            robot.move(-lSpeed, 0, 0, rSpeed);
        } else {
            robot.move(0, -lSpeed, rSpeed, 0);
        }
        while ((light > 1.2 && light < 1.9) && opModeIsActive()) {
            light = robot.lightFloor.getRawLightDetected();
            telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
            telemetry.update();
        }
        robot.stop();

        /*
        *** Recenter it so we are more accurate (DISABLED because rotation can throw off the "center" reading)
         */
        //Center is about {200, 625}
        /*
        while(Math.abs(200-beacon.getAnalysis.getCenter().x) > 10)
        {
            //move toward the center more
        }
         */

        /*
        *** Read and hit the beacon!
         */
        sleep(1000);    //Give it some time before reading
        //beacon.getAnalysis().getConfidence();
        boolean leftBlue = beacon.getAnalysis().isLeftBlue();
        telemetry.addData("Left Side is Blue: ", leftBlue);
        telemetry.update();

        if (((alliance == 0) && leftBlue) || (!(alliance == 0) && !leftBlue))    //We want to hit left
        {
            robot.move(-0.25, 0.25, -0.25, 0.25);
            sleep(300);
            robot.stop(); //Just go straight. We rainbow toward the left side of the button anyways
            sleep(200);
            robot.move(-0.25, -0.25, 0.25, 0.25);
            sleep(1750);
            robot.stop();
        } else {    //We want to hit right
            robot.move(0.25, -0.25, 0.25, -0.25);
            sleep(750);
            robot.stop();
            sleep(200);
            robot.move(-0.25, -0.25, 0.25, 0.25);
            sleep(1750);
            robot.stop();
        }

        /*
        *** Back up before resetting the beacon
         */
        robot.move(0.25, 0.25, -0.25, -0.25);
        sleep(500);
        robot.stop();

        /*
        *** Prepare control system for moving sideways (forward) to the next beacon
         */
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController controller = new PIDController(hardwareMap, robot.frontLeft, robot.frontRight);
        controller.setTuning(0.001, 0.00005, 0, 0);

        robot.move(0.25, -0.3, 0.3, -0.25);
        sleep(750);

        /*
        *** Prime the while loop, or else the previous value of "light" will mess it up!
         */
        light = 1.5;
        double correction = controller.getCorrection();
        while ((light > 1.2 && light < 1.8) && opModeIsActive()) {
            light = robot.lightFloor.getRawLightDetected();
            telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
            telemetry.update();
            robot.move(0.25 - correction, -0.3 - correction, 0.3 - correction, -0.25 - correction); //PID work pls
            correction = controller.getCorrection();
        }
        robot.stop();

        /*
        *** Analyse the second beacon
         */
        sleep(1000);    //Give it some time before reading
        //beacon.getAnalysis().getConfidence();
        leftBlue = beacon.getAnalysis().isLeftBlue();
        telemetry.addData("Left Side is Blue: ", leftBlue);
        telemetry.update();

        if (((alliance == 0) && leftBlue) || (!(alliance == 0) && !leftBlue))    //We want to hit left
        {
            robot.move(-0.25, 0.25, -0.25, 0.25);
            sleep(300);
            robot.stop();
            sleep(200);
            robot.move(-0.25, -0.25, 0.25, 0.25);
            sleep(2000);    // ***** We have to go a bit farther than the last one, since the robot rainbowed backwards even with PID. Hmmm.
            robot.stop();
        } else {    //We want to hit right
            robot.move(0.25, -0.25, 0.25, -0.25);
            sleep(750);
            robot.stop();
            sleep(200);
            robot.move(-0.25, -0.25, 0.25, 0.25);
            sleep(2000);
            robot.stop();
        }

        /*
        *** Backup before the beacon resets
         */
        robot.move(0.25, 0.25, -0.25, -0.25);
        sleep(500);
        robot.stop();

        /*
        *** Turn toward the beacon
         */
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(robot.frontLeft.getCurrentPosition() < 1900 && opModeIsActive())
        {
            robot.move(0.25, 0.25, 0.25, 0.25);
        }

        /*
        *** Move forward a bit so the shot is safer. Move more if we're offset by trying to hit the red side
         */
        robot.move(0.25, -0.25, 0.25, -0.25);
        if (((alliance == 0) && leftBlue) || (!(alliance == 0) && !leftBlue))    //We want to hit left
        {
            sleep(1250);
        } else {    //We want to hit right
            sleep(1750);
        }
        robot.stop();

        /*
        *** SHOOT!
         */

        robot.shooterRed.setPower(-.7);
        robot.shooterBlue.setPower(-.7);
        sleep(1500);
        robot.shooterRed.setPower(0);
        robot.shooterBlue.setPower(0);
        robot.intake.setPower(-0.5);
        sleep(4000);
        robot.intake.setPower(0);
        robot.shooterRed.setPower(-.7);
        robot.shooterBlue.setPower(-.7);
        sleep(1500);
        robot.shooterRed.setPower(0);
        robot.shooterBlue.setPower(0);
    }

}
