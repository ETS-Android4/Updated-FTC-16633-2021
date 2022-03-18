package org.firstinspires.ftc.teamcode.util.helpers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Printer {
    private final Telemetry t;

    // Constructor
    public Printer(Telemetry telemetry) {
        t = telemetry;
    }

    // Function for easy telemetry printing without update
    public void add(String message) {
        t.addLine(message);
    }

    // Function for easy telemetry printing with update
    public void print(String message) {
        t.addLine(message);
        t.update();
    }

    // To print multiple lines
    public void printLines(ArrayList<String> str) {
        for (int i = 0; i < str.size(); i++) {
            add(str.get(i));
        }
        t.update();
    }

    // To print line with caption and update
    public void printInfo(String caption, String message) {
        addInfo(caption, message);
        t.update();
    }

    // Optional function if passed in integer
    public void printInfo(String caption, double val) {
        addInfo(caption, val);
        t.update();
    }

    // To add caption data
    public void addInfo(String caption, String message) {
        t.addData(caption, message);
    }

    // Optional function if passed in integer
    public void addInfo(String caption, double val) {
        t.addData(caption, String.valueOf(val));
    }

    // To manually update
    public void load() {
        t.update();
    }
}
