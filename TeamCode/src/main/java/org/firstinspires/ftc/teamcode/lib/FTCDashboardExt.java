package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public final class FTCDashboardExt{
    public static void init(Telemetry telemetry){
        FTCDashboardExt.telemetry = telemetry;
    }

    private static Telemetry telemetry = null;

    public static double inputValue;
    public static boolean updated;

    /**
     * Wait for user to update value and return it
     * @return the value
     */
    public static double waitForValue(String varname){
        telemetry.addData("Waiting on:", varname);
        //noinspection StatementWithEmptyBody,WhileLoopSpinsOnField
        while (!updated); //this fucking sucks
        double temp = inputValue;
        updated = false;
        inputValue = 0;
        return temp;
    }
}