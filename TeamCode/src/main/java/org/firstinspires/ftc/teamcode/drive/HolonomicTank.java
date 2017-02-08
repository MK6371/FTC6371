package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;
import org.firstinspires.ftc.teamcode.test.PIDTest;

/**
 * Created by 292486 on 1/12/2017.
 */

public class HolonomicTank {

    private static double threshold = 0.1;

    public static void tank(double l, double r, HardwareHolonomic robot)
    {
        move(-l, r, -l, r, robot);

    }

    public static void move(double fl, double fr, double bl, double br, HardwareHolonomic robot){
        robot.frontLeft.setPower((Math.abs(fl) > threshold ? fl : 0));
        robot.frontRight.setPower((Math.abs(fr) > threshold ? fr : 0));
        robot.backLeft.setPower((Math.abs(bl) > threshold ? bl : 0));
        robot.backRight.setPower((Math.abs(br) > threshold ? br : 0));
    }

    public static void tankStrafe(double trigL, double trigR, HardwareHolonomic robot)
    {
        move(trigR - trigL, trigR - trigL, -trigR + trigL, -trigR + trigL, robot);

    }
}
