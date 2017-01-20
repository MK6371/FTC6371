package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jojotastic777 on 11/28/16.
 */


@Autonomous(name="Forward", group="Autonomous Drive")
public class HolonomicAutonBeacon extends LinearOpMode{

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor outTake;
    public DcMotor inTake;
    public LightSensor beaconLightSensor;
    public LightSensor lineLightSensor;
    //public Servo //buttonServo;

    private HardwareMap hMap;
    private ElapsedTime period = new ElapsedTime();

    public boolean onLine;

    public void moveX(double power, long time) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveY(double power, long time) throws InterruptedException {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void move(double x, double y, double time) throws InterruptedException {
        frontLeft.setPower(Range.clip(x + y, 1, 1));
        frontRight.setPower(Range.clip(x - y, 1, 1));
        backLeft.setPower(Range.clip(-x + y, 1, 1));
        backRight.setPower(Range.clip(-x - y, 1, 1));
        sleep(Math.round(time));
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setMotors(double x, double y) {
        frontLeft.setPower(Range.clip(x + y, 1, 1));
        frontRight.setPower(Range.clip(x - y, 1, 1));
        backLeft.setPower(Range.clip(-x + y, 1, 1));
        backRight.setPower(Range.clip(-x - y, 1, 1));
    }

    public void turn(double power, long time) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void fire() throws InterruptedException {
        outTake.setPower(1);
        sleep(1500);
        inTake.setPower(1);
        sleep(1000);
        inTake.setPower(0);
        outTake.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hMap = hardwareMap;

        frontLeft = hMap.dcMotor.get("front_left");
        frontRight = hMap.dcMotor.get("front_right");
        backLeft = hMap.dcMotor.get("back_left");
        backRight = hMap.dcMotor.get("back_right");
        outTake = hMap.dcMotor.get("outTake");
        inTake = hMap.dcMotor.get("inTake");
        beaconLightSensor = hMap.lightSensor.get("beaconLightSensor");
        lineLightSensor = hMap.lightSensor.get("lineLightSensor");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        onLine = false;

        //buttonServo.setPosition(0.5);

        //sleep(10000);
        setMotors(0.3,0.3);
        while (!onLine) {
            if(beaconLightSensor.getLightDetected()> 0.9) onLine = false;
        }
        setMotors(0,0);
        move(-0.3,0,0.3);
        if(beaconLightSensor.getLightDetected()>=0.34 && beaconLightSensor.getLightDetected()<=0.36) {
            //move(-0.3,0,0.5);
            move(1,1,0.5);
        } else if (beaconLightSensor.getLightDetected()>=0.28 && beaconLightSensor.getLightDetected()<=0.3) {
            move(0.3,0,0.6);
            move(1,1,0.5);
        }

    }


}
