package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 292486 on 9/20/2016.
 */
public class HardwareHolonomic {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor shooter;
    public DcMotor intake;
    //public Servo servo;

    private HardwareMap hMap;
    private ElapsedTime period = new ElapsedTime();

    public HardwareHolonomic(){

    }

    public void init(HardwareMap map){
        hMap = map;

        frontLeft = hMap.dcMotor.get("front_left");
        frontRight = hMap.dcMotor.get("front_right");
        backLeft = hMap.dcMotor.get("back_left");
        backRight = hMap.dcMotor.get("back_right");
        shooter = hMap.dcMotor.get("shooter");
        intake = hMap.dcMotor.get("intake");
        //servo = hMap.servo.get("servo");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     * [Taken from HardwarePushbot. What we can use it for is not immediately apparent]
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
