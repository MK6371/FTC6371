package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jojotastic777 on 11/28/16.
 */


@Autonomous(name="Beaconless Red", group="Autonomous Drive")
public class HolonomicAuton extends LinearOpMode{

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor outTake;
    public DcMotor inTake;
    public Servo buttonServo;

    private HardwareMap hMap;
    private ElapsedTime period = new ElapsedTime();

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
        outTake = hMap.dcMotor.get("shooter");
        inTake = hMap.dcMotor.get("intake");
        buttonServo = hMap.servo.get("buttonServo");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buttonServo.setPosition(0.5);

        move(0.9,0,1);
        move(-0.9,0,0.5);
        fire();
        move(0,-0.9,2);
        move(-1,0,1);

    }


}
