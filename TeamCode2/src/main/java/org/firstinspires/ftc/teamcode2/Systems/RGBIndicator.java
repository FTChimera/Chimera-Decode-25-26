package org.firstinspires.ftc.teamcode2.Systems;

import com.qualcomm.robotcore.hardware.Servo;
@SuppressWarnings("SpellCheckingInspection")
/*
* GOBILDA PRODUCT INSIGHT: https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1
*/
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
    public void setColor(Color col) {
        if (col==Color.BLACK) rgb.setPosition(BLACK_PWM);
        if (col==Color.RED) rgb.setPosition(RED_PWM);
        if (col==Color.ORANGE) rgb.setPosition(ORANGE_PWM);
        if (col==Color.GOLD) rgb.setPosition(GOLD_PWM);
        if (col==Color.YELLOW) rgb.setPosition(YELLOW_PWM);
        if (col==Color.SAGE) rgb.setPosition(SAGE_PWM);
        if (col==Color.GREEN) rgb.setPosition(GREEN_PWM);
        if (col==Color.AZURE) rgb.setPosition(AZURE_PWM);
        if (col==Color.BLUE) rgb.setPosition(BLUE_PWM);
        if (col==Color.INDIGO) rgb.setPosition(INDIGO_PWM);
        if (col==Color.VIOLET) rgb.setPosition(VIOLET_PWM);
        if (col==Color.WHITE) rgb.setPosition(WHITE_PWM);
    }
    public void updateUsingLL(LimelightSystem ll) {
        if (ll.isDisconnected) {
            this.setColor(RGBIndicator.Color.BLACK); // Limelight not looking at target
        }
        else if (ll.getLLScore() < 1.5) {
            // GREEN
            this.setColor(RGBIndicator.Color.GREEN);
        } else if (ll.getLLScore() < 5) {
            // YELLOW
            this.setColor(RGBIndicator.Color.YELLOW);
        } else if (ll.getLLScore() < 12.5) {
            // ORANGE
            this.setColor(RGBIndicator.Color.ORANGE);
        } else {
            // OFF
            this.setColor(RGBIndicator.Color.BLACK);
        }
    }

}