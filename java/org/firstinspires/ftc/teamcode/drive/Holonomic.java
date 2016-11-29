package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;

import java.sql.Array;
import java.util.Arrays;

/**
 * Created by 292486 on 10/13/2016.
 */

public class Holonomic {

    public static void arcade(double y, double x, double l, double r, HardwareHolonomic robot) {
        double flSpeed = -y - x + l - r;
        double frSpeed = y - x + l - r;
        double blSpeed = -y + x + l - r;
        double brSpeed = y + x + l - r;

        double[] speeds = {flSpeed, frSpeed, blSpeed, brSpeed};
        Arrays.sort(speeds);
        if(speeds[3] > 1){
            for(double speed : speeds){
                speed /= speeds[3];
            }
        }

        move(flSpeed, frSpeed, blSpeed, brSpeed, robot);
    }

    public static void arcade(double y, double x, HardwareHolonomic robot) {
        move(Range.clip(-y - x, -1, 1), Range.clip(y - x, -1, 1), Range.clip(-y + x, -1, 1), Range.clip(y + x, -1, 1), robot);
    }

    public static void dpad(Gamepad gamepad, HardwareHolonomic robot) {

    }

    public static void move(double fl, double fr, double bl, double br, HardwareHolonomic robot){
        robot.frontLeft.setPower(fl);
        robot.frontRight.setPower(fr);
        robot.backLeft.setPower(bl);
        robot.backRight.setPower(br);
    }

    //Probably not very useful with holonomic
    public static void move(double speed, HardwareHolonomic robot) {
        move(speed, speed, speed, speed, robot);
    }

    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT, FORWARDR, FORWARDL, BACKWARDR, BACKWARDL;
    }
    /*
    Forward: -1, 1, -1, 1
    Backward: 1, -1, 1, -1 (-Forward)
    Left: -1, 1, -1, -1
    Right: 1, -1, 1, 1 (-Left)
    ForwardR: -1, 0, 0, 1
    ForwardL: 0, 1, -1, 0
    BackwardR: 0, -1, 1, 0 (-ForwardL)
    BackwardL: 1, 0, 0, -1 (-ForwardR)
    */
    public static void move(Direction dir, double speed, HardwareHolonomic robot) {
        switch(dir)
        {
            case FORWARD:
                move(-speed, speed, -speed, speed, robot);
                break;
            case BACKWARD:
                move(speed, -speed, speed, -speed, robot);
                break;
            case LEFT:
                move(-speed, speed, -speed, -speed, robot);
                break;
            case RIGHT:
                move(speed, -speed, speed, speed, robot);
                break;
            case FORWARDR:
                move(-speed, 0, 0, speed, robot);
                break;
            case FORWARDL:
                move(0, speed, -speed, 0, robot);
                break;
            case BACKWARDR:
                move(0, -speed, speed, 0, robot);
                break;
            case BACKWARDL:
                move(speed, 0, 0, -speed, robot);
                break;
            default:
        }
    }

    public static void stop(HardwareHolonomic robot) {
        move(0, 0, 0, 0, robot);
    }
}
