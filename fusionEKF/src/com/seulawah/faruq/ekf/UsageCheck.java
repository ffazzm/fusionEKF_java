package com.seulawah.faruq.ekf;

import org.apache.commons.math3.linear.RealVector;

import java.io.*;

import org.apache.commons.lang3.StringUtils;

public class UsageCheck {
    public void check_arguments(int argc, String[] argv) {
        String usage_instructions = "Usage Instruction";
        usage_instructions += argv[0];
        usage_instructions += " path/to/input.txt output.txt";

        Boolean has_valid_args = false;

        if (argc == 1) {
            System.err.println(usage_instructions);
        }   else if (argc == 2) {
            System.err.println("Please include an output file.");
            System.err.println(usage_instructions);
        } else if (argc == 3) {
            has_valid_args = true;
        } else if (argc > 4) {
            System.err.println("Too many arguments");
            System.err.println(usage_instructions);
        }

        if (!has_valid_args) {
            System.exit(1);
        }
    }

    public void check_files(BufferedReader in_file, String in_name, BufferedWriter out_file, String out_name) {
        try {
            FileReader fr_in = new FileReader("in_name");
            in_file = new BufferedReader(fr_in);
        } catch (IOException e) {
            System.err.println(e + in_name);
        }

        try {
            FileWriter fr_out = new FileWriter("out_name");
            out_file = new BufferedWriter(fr_out);
        } catch (IOException e) {
            System.err.println(e + out_name);
        }

        System.out.println("in_file out_file successfully added");
    }

    public void print_EKF_data(final RealVector RMSE, final RealVector estimations, final RealVector ground_truths, final RealVector all_sensor_data) {
        System.out.println("--------------------------------------------------------------------");
        System.out.println(StringUtils.rightPad("RMSE: ", 20, "") + " | " + RMSE.getEntry(0) + " | " + RMSE.getEntry(1) + " | " + RMSE.getEntry(2) + " | " + RMSE.getEntry(3));
        System.out.println("--------------------------------------------------------------------");
    }
}