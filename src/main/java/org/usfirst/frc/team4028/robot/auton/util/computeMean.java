package org.usfirst.frc.team4028.robot.auton.util;

public class computeMean {

    public computeMean(){

    }


    public double mean(double[] listOfVals){
        double sum = 0.;
        for (Double duble : listOfVals){
            sum += duble;
        }
        return (sum)/(listOfVals.length);
    }

}