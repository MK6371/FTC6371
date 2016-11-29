package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 292486 on 10/26/2016.
 */

public class PIDTest {

    private HardwareMap map;
    private DcMotor leftMotor;  //Get the front left and right motors when instantiating a controller
    private DcMotor rightMotor;

    private ElapsedTime timeController;

    private double error, lastError, integral, derivative;  //Controller variables
    private double kp, ki, kd, kf;  //Tuning variables/coefficients
    private double correction, maxIntegral, maxCorrection ,tolerance;  //Limits and output
    private double lastTime, deltaTime;

    public PIDTest(HardwareMap map){
        this.map = map;
        this.leftMotor = map.dcMotor.get("front_left");
        this.rightMotor = map.dcMotor.get("front_right");

        maxIntegral = 500;
        maxCorrection = .5;
        tolerance = 100;

        timeController = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);  //Keep track of time in miliseconds. When instantiated, timeController.time() is 0
    }

    public void setTuning(double kp, double ki, double kd, double kf){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    /*
    Proportional: main correction (make changes based on an error)
    Integral: eliminates steady state error/offset and contributes to effect of proportional term
    Derivative: reacts to changes... less overshoot and faster settling time
     */
    public double getCorrection(){
        deltaTime = timeController.time() - lastTime;    //Find how much time has passed since last one. In nanoseconds
        error = leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition();   //Difference in encoders is the process variable. Setpoint is 0 (we want encoders to be equal), but we don't need to type that
        if(error < tolerance) error = 0;

        integral = (1/deltaTime)*(integral + error);    //Assume time interval is near 0, so the integral is sum of errors times delta(time)
        if(integral > maxIntegral) integral = maxIntegral;
        else if(integral < -maxIntegral) integral = -maxIntegral;

        derivative = deltaTime *(error - lastError);    //Assume time interval is near 0, so the derivative is delta(error)/delta(time)
        //To predict how much the error will change in the future, dy = e'(t)dt where dy=how much it will change & dt=how much time later (ex// dt=10 means 10 miliseconds later)
        //We assume the change in time from the last correction is equal to the next change in time
        //Low pass filter (for what?): Esubt = r(Esub(t-1) + derivative)

        correction = (kp*error) + (ki*integral) - (kd*derivative);

        lastError = error;
        lastTime += deltaTime;  //Should we set it to timeController.time() or add the difference? Because .time() may have changed at this point
        return correction;
    }

}
