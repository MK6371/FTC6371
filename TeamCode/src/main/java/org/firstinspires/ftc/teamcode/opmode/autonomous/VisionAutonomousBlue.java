package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HolonomicRobot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by 292486 on 2/25/2017.
 */

@Autonomous(name="vision autonomous blue", group="vision")
public class VisionAutonomousBlue extends LinearVisionOpMode {

    private HolonomicRobot robot = new HolonomicRobot();
    private boolean blueAlliance = true;
    private String allianceColor;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        robot.init(hardwareMap);

        initializeVision();

        //**********************************//
        waitForStart();
        //**********************************//

        moveDiagonal();

        if(blueAlliance) //Correcting light sensor - phone camera offset on blue side
        {
            robot.move(0.25, -0.25, 0.25, -0.25);
            sleep(300);
            robot.stop();
        }

        sleep(1000);
        analyzeHitBeacon(0, 650, 1750); //How long to go if left (0ms), how long to go if right (850ms), how long to go forward

        resetEncoders();

        moveToNext();

        if(blueAlliance) //Correcting light sensor - phone camera offset on blue side
        {
            robot.move(0.25, -0.25, 0.25, -0.25);
            sleep(300);
            robot.stop();
        }

        sleep(750);
        boolean leftBlue = analyzeHitBeacon(0, 650, 2000);
        boolean farSide = (blueAlliance && leftBlue) || (!blueAlliance && leftBlue);    //We're farther away from the beacon

        resetEncoders();

        turnToVortex(farSide);

        shoot();
    }

    /****
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     *  *
     ****/

    private void initializeVision()
    {
        /*
        *** Vision configuration/setup
         */

        this.setCamera(Cameras.SECONDARY);  //Front-facing camera
        this.setFrameSize(new Size(900, 900));  //Frame to analyze (bigger = slower but more accurate)

        enableExtension(VisionOpMode.Extensions.BEACON);
        enableExtension(VisionOpMode.Extensions.ROTATION);   //Corrects screen rotation
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
    }

    private void moveDiagonal() throws InterruptedException
    {
        double light = 1.5; //Prime the loop
        robot.lightFloor.enableLed(true);
        sleep(750);    //let the LED turn on
        if(blueAlliance) //if blue team
        {
            robot.move(-0.5, 0, 0, 0.4);
        } else {
            robot.move(0, -0.5, 0.3, 0); // -.5, 0.3
        }
        while ((light > 1.2 && light < 1.9) && opModeIsActive()) {
            light = robot.lightFloor.getRawLightDetected();
            telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
            telemetry.update();
        }
        robot.stop();
    }

    private void moveToNext() throws InterruptedException{
        PIDController controller = new PIDController(hardwareMap, robot.frontLeft, robot.frontRight);
        controller.setTuning(0.0015, 0.00001, 0, 0);
        double correction = controller.getCorrection();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if(blueAlliance) {
            robot.move(-0.25, 0.32, -0.32, 0.25);
        } else {
            robot.move(0.25, -0.32, 0.32, -0.25);
        }
        while(timer.time() < 1500 && opModeIsActive())  //Get off the white tape
        {
            if(blueAlliance) {
                robot.move(-0.25 + correction, 0.32 + correction, -0.32 + correction, 0.25 + correction); //PID work pls
            } else {
                robot.move(0.25 - correction, -0.3 - correction, 0.3 - correction, -0.25 - correction);
            }
            correction = controller.getCorrection();
        }

        /*
        *** Prime the while loop, or else the previous value of "light" will mess it up!
         */
        double light = 1.5;
        while ((light > 1.2 && light < 1.9) && opModeIsActive()) {
            light = robot.lightFloor.getRawLightDetected();
            telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
            telemetry.addData("Correction: ", correction);
            telemetry.update();
            if(blueAlliance) {
                robot.move(-0.25 + correction, 0.32 + correction, -0.32 + correction, 0.25 + correction); //PID work pls
            } else {
                robot.move(0.25 - correction, -0.3 - correction, 0.3 - correction, -0.25 - correction);
            }
            correction = controller.getCorrection();
        }
        robot.stop();
    }

    private void turnToVortex(boolean farSide) throws InterruptedException {
        robot.move(0.25, 0.25, 0.25, 0.25);
        if(blueAlliance) {
            while (Math.abs(robot.frontLeft.getCurrentPosition()) < 450 && opModeIsActive()) {
            }
        } else {
            while(Math.abs(robot.frontLeft.getCurrentPosition()) < 1900 && opModeIsActive()) {

            }
        }

        /*
        *** Move forward a bit so the shot is safer. Move more if we're offset by trying to hit the red side
         */
        robot.move(0.25, -0.3, 0.3, -0.25);
        if(blueAlliance) {
            if (farSide)    //We need to go forward farther if we're on the far side
            {
                sleep(1000);
            } else {
                sleep(750);
            }
        } else {
            if(farSide)
            {
                sleep(1500);
            } else {
                sleep(1000);
            }
        }
        robot.stop();
    }

    private void shoot() throws InterruptedException {
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

    private boolean analyzeHitBeacon(long leftTime, long rightTime, long forwardTime) throws InterruptedException
    {
        if(!beacon.getAnalysis().isLeftKnown() || !beacon.getAnalysis().isRightKnown()) //Make sure we don't try hitting a beacon if one is not known
        {
            return true;
        }

        boolean leftBlue = beacon.getAnalysis().isLeftBlue();
        telemetry.addData("Left Side is Blue: ", leftBlue);
        telemetry.update();

        if ((blueAlliance && leftBlue) || (!blueAlliance && !leftBlue))    //We want to hit left
        {
            /*
            robot.move(-0.25, 0.25, -0.25, 0.25);
            sleep(leftTime);
            robot.stop();*/ //Just go straight. We rainbow toward the left side of the button anyways.
            //********************************************
            // It's not working for blue side, so we're not going to move to the left
            //********************************************
            sleep(200);
            robot.move(-0.25, -0.25, 0.25, 0.25);
            sleep(forwardTime);
            robot.stop();
        } else {    //We want to hit right
            robot.move(0.25, -0.25, 0.25, -0.25);
            sleep(rightTime);
            robot.stop();
            sleep(200);
            robot.move(-0.25, -0.25, 0.25, 0.25);
            sleep(forwardTime);
            robot.stop();
        }

        /*
        *** Back up before resetting the beacon
         */
        robot.move(0.25, 0.25, -0.25, -0.25);
        sleep(750);
        robot.stop();

        return leftBlue;
    }

    private void resetEncoders() throws InterruptedException
    {
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
