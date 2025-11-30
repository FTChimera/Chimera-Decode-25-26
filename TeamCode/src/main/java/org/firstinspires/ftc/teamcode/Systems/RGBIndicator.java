package org.firstinspires.ftc.teamcode.Systems;


import com.qualcomm.robotcore.hardware.Servo;

public class RGBIndicator {
    public enum Color {
        BLACK,
        RED,
        ORANGE,
        GOLD,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }
    /*
    Black – 0
    Red – 0.279
    Orange – 0.333
    Gold – 0.357
    Yellow – 0.388
    Sage – 0.444
    Green – 0.500
    Azure – 0.555
    Blue – 0.611
    Indigo – 0.666
    Violet – 0.722
    White – 1
    */
    public static double BLACK_PWM = 0.0;
    public static double RED_PWM = 0.279;
    public static double ORANGE_PWM = 0.333;
    public static double GOLD_PWM = 0.357;
    public static double YELLOW_PWM = 0.388;
    public static double SAGE_PWM = 0.444;
    public static double GREEN_PWM = 0.500;
    public static double AZURE_PWM = 0.555;
    public static double BLUE_PWM = 0.611;
    public static double INDIGO_PWM = 0.666;
    public static double VIOLET_PWM = 0.722;
    public static double WHITE_PWM = 1.0;

    private Servo rgb;
    public RGBIndicator(Servo indicator){this.rgb = indicator;rgb.setPosition(BLACK_PWM);}
    public void setCustomPWM(double val) {rgb.setPosition(val);}
    public void setColor() {}

}
