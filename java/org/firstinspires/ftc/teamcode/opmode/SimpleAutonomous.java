package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Holonomic;
import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;

/**
 * Created by 292486 on 11/15/2016.
 */

@Autonomous
public class SimpleAutonomous extends LinearOpMode {

    HardwareHolonomic robot = new HardwareHolonomic();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        //Shoot();
        Holonomic.move(Holonomic.Direction.FORWARD, 0.5, robot);
        sleep(5000);
        Holonomic.stop(robot);
    }

}
