package org.firstinspires.ftc.teamcode.test;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Holonomic;
import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;
import org.firstinspires.ftc.teamcode.utils.Gyro;

/**
 * Created by 292486 on 1/25/2017.
 */

@Autonomous
public class BeaconLightTest extends LinearOpMode {

    HardwareHolonomic robot = new HardwareHolonomic();
    private static final double WHITE = 2.63; //TODO



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        hardwareMap.logDevices();

        int alliance = 0;
        String allianceColor;
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        alliance = Integer.parseInt(sharedPreferences.getString("mk_color", "0"));
        allianceColor = alliance==0 ? "Blue" : "Red";
        long delayTime = Long.parseLong(sharedPreferences.getString("mk_delayTime", "5000"));
        waitForStart();

        //robot.gyro.calibrate();

        sleep(delayTime);

        moveDiagonal(alliance == 0);

        /*
        robot.lightFloor.enableLed(true);
        sleep(1000);
        while((robot.lightFloor.getRawLightDetected() > 1.2 || robot.lightFloor.getRawLightDetected() < 1.9) && opModeIsActive());    //Raw light?? Light??
        {
            Holonomic.move(0, 0.5, -0.5, 0, robot);
            telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
            //telemetry.addData("Gyro: ", robot.gyro.heading);
            telemetry.update();
        }
        Holonomic.stop(robot);
        sleep(750);
        */
        sleep(1500);
        Holonomic.stop(robot);

       // moveToSonarVal(4);
        sleep(1000);

        readLeftSide(alliance == 0);
        sleep(5000);
    }

    private void moveDiagonal(boolean blueAlliance)
    {
        double correction = 0;
        double light = 1.5; //priming the while loop
        robot.lightFloor.enableLed(true);
        sleep(1500);
        if(blueAlliance)
        {
            Holonomic.move(-0.5, 0, 0, 0.5, robot);
            while((light > 1.2 && light < 1.9) && opModeIsActive()) {
                telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
                updateTelemetry(telemetry);
                light = robot.lightFloor.getRawLightDetected();
            }
            Holonomic.stop(robot);
        } else {
            //while(robot.lightFloor.getRawLightDetected() > 1.2 && robot.lightFloor.getRawLightDetected() < 1.9 && opModeIsActive());    //Raw light?? Light??
            //{
                Holonomic.move(0, -0.5, 0.5, 0, robot);
            while((light > 1.2 && light < 1.9) && opModeIsActive()) {
                telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
                updateTelemetry(telemetry);
                light = robot.lightFloor.getRawLightDetected();
            }
            //}
            Holonomic.stop(robot);
        }
    }

    private void moveToSonarVal(double distance)
    {
        double correction = 0;
        ElapsedTime forwardTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        PIDTest pidController = new PIDTest(hardwareMap, robot.backLeft, robot.frontLeft);
        pidController.setMaxCorrection(0.09);
        pidController.setTuning(0.0025, 0, 0, 0);
        pidController.resetPID();
        forwardTimer.reset();
        Holonomic.move(0, 0, 0.3, 0.3, robot);
        sleep(100);
        while((/*robot.sonar.getUltrasonicLevel() > distance && */forwardTimer.time() < 1000) && opModeIsActive())
        //****************************************
        {
            correction = pidController.getCorrection();
            Holonomic.move(-0.3 - correction, -0.3 - correction, 0.3 - correction, 0.3 - correction, robot);
            telemetry.addData("SONAR: ", robot.sonar.getUltrasonicLevel());
            telemetry.update();
        }
        Holonomic.stop(robot);
    }

    private void readLeftSide(boolean blueSide)
    {
        //robot.lightBeacon.enableLed(false);
        Holonomic.move(-0.2, 0.2, -0.2, 0.2, robot);
        sleep(150);
        Holonomic.stop(robot);
        boolean leftBlue = false;
        telemetry.addData("RGB: ", "%d %d %d", robot.sensorRgb.red(), robot.sensorRgb.green(), robot.sensorRgb.blue());
        if(Math.abs(robot.sensorRgb.blue() - robot.sensorRgb.red()) < 10)
        {
            telemetry.addData("BEACON: ", "too close");
        }
        if(robot.sensorRgb.blue() > robot.sensorRgb.red())
        {
            //Return BLUE;

            telemetry.addData("BEACON: ", "Blue");
            leftBlue = true;
        } else {
            //Return RED;
            leftBlue = false;
            telemetry.addData("BEACON: ", "Red");
        }
        telemetry.update();

        if((blueSide && leftBlue) || (!blueSide && !leftBlue))
        {
            Holonomic.move(-0.25, 0.25, -0.25, 0.25, robot);
            sleep(350);
            Holonomic.stop(robot);
            sleep(200);
            Holonomic.move(-0.25, -0.25, 0.25, 0.25, robot);
            sleep(1000);
            Holonomic.stop(robot);
        } else {
            Holonomic.move(0.25, -0.25, 0.25, -0.25, robot);
            sleep(550);
            Holonomic.stop(robot);
            sleep(200);
            Holonomic.move(-0.25, -0.25, 0.25, 0.25, robot);
            sleep(1000);
            Holonomic.stop(robot);
        }

        Holonomic.move(0.3, 0.3, -0.25, -0.25, robot);
        sleep(1500);
        Holonomic.move(0.25, 0.25, -0.25, -0.25, robot);
        sleep(1000);
        Holonomic.stop(robot);

        if(blueSide) //Blue [Change spin direction]
        {
            Holonomic.move(-0.5, -0.5, -0.5, -0.5, robot);
        } else {
            Holonomic.move(0.5, 0.5, 0.5, 0.5, robot);
        }
        sleep(1000);
        Holonomic.stop(robot);
    }

    /*
    private static final double gkp = 0.005;    //TODO
    private void moveDiagonal(boolean blueAlliance)
    {
        double correction = 0;
        robot.lightFloor.enableLed(true);
        sleep(1500);
        if(blueAlliance)
        {
            while(robot.lightFloor.getRawLightDetected() > 1.2 && robot.lightFloor.getRawLightDetected() < 1.9 && opModeIsActive());    //Raw light?? Light??
            {
                if(robot.gyro.heading > 1)
                {
                    correction = robot.gyro.heading * gkp;
                } else {
                    correction = 0;
                }
                Holonomic.move(0.25 + correction, 0, 0, -0.25 + correction, robot);
                robot.gyro.updateHeading();
                telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
                telemetry.addData("Gyro: ", robot.gyro.heading);
                telemetry.update();
            }
            Holonomic.stop(robot);
        } else {
            //TODO: everything except the opposite direction b/c RED
            while(robot.lightFloor.getRawLightDetected() > 1.2 && robot.lightFloor.getRawLightDetected() < 1.9 && opModeIsActive());    //Raw light?? Light??
            {
                if(robot.gyro.heading > 1)
                {
                    correction = robot.gyro.heading * gkp;
                } else {
                    correction = 0;
                }
                Holonomic.move(0, 0.5 + correction, -0.5 + correction, 0, robot);
                robot.gyro.updateHeading();
                telemetry.addData("Light: ", robot.lightFloor.getRawLightDetected());
                telemetry.addData("Gyro: ", robot.gyro.heading);
                updateTelemetry(telemetry);
            }
            Holonomic.stop(robot);
        }
    }

    private void moveToSonarVal(double distance)
    {
        double correction = 0;
        while(robot.sonar.getUltrasonicLevel() > distance && opModeIsActive())
        //****************************************
        {
            if(robot.gyro.heading > 1)
            {
                correction = robot.gyro.heading * gkp;
            } else {
                correction = 0;
            }
            Holonomic.move(0.1 + correction, 0.1 + correction, -0.1 + correction, -0.1 + correction, robot); //"Forward" is actually left
            robot.gyro.updateHeading();
        }
        Holonomic.stop(robot);
    }

    private void readLeftSide()
    {
        robot.lightBeacon.enableLed(false);
        if(Math.abs(robot.lightBeacon.getLightDetected() - 32) < 1)
        {
            //Return BLUE;
            telemetry.addData("BEACON: ", "Blue");
        } else {
            //Return RED;
            telemetry.addData("BEACON: ", "Red");
        }
        telemetry.update();
    }*/
}
