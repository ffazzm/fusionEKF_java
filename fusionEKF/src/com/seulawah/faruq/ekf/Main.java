package com.seulawah.faruq.ekf;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.io.*;
import java.util.*;

public class Main {
    public static void main(String[] args) throws IOException{

        String in_filename = "obj_pose-laser-radar-synthetic-input.txt";
        String out_filename = "out_file.txt";

        BufferedReader in_file;
        FileReader fr_in = new FileReader(in_filename);
        in_file = new BufferedReader(fr_in);

        BufferedWriter out_file;
        FileWriter fr_out = new FileWriter(out_filename);
        out_file = new BufferedWriter(fr_out);

        Vector<DataPoint> all_sensor_data = new Vector<>(9);
        Vector<DataPoint> all_truth_data = new Vector<>(9);

        String sensor_id;

        String line;

        while ((line = in_file.readLine()) != null){
            String[] linevec = line.split("\\s+");
            DataPoint sensor_data = new DataPoint();
            DataPoint truth_data = new DataPoint();

            sensor_id = linevec[0];

            if (sensor_id.compareTo("R") == 0) {
                double val1 = Double.parseDouble(linevec[1]);
                double val2 = Double.parseDouble(linevec[2]);
                double val3 = Double.parseDouble(linevec[3]);
                long timestamp = Long.parseLong(linevec[4]);

                RealVector radar_vec = new ArrayRealVector(new double[] {val1, val2, val3});
                sensor_data.set(timestamp, DataPointType.RADAR, radar_vec);

                double x = Double.parseDouble(linevec[5]);
                double y = Double.parseDouble(linevec[6]);
                double vx = Double.parseDouble(linevec[7]);
                double vy = Double.parseDouble(linevec[8]);

                RealVector truth_vec = new ArrayRealVector(new double[] {x, y, vx, vy});
                truth_data.set(timestamp, DataPointType.STATE, truth_vec);

            }
            else if (sensor_id.compareTo("L") == 0) {
                double val1 = Double.parseDouble(linevec[1]);
                double val2 = Double.parseDouble(linevec[2]);
                long timestamp = Long.parseLong(linevec[3]);

                RealVector lidar_vec = new ArrayRealVector(new double[] {val1, val2});
                sensor_data.set(timestamp, DataPointType.LIDAR, lidar_vec);

                double x = Double.parseDouble(linevec[4]);
                double y = Double.parseDouble(linevec[5]);
                double vx = Double.parseDouble(linevec[6]);
                double vy = Double.parseDouble(linevec[7]);

                RealVector truth_vec = new ArrayRealVector(new double[] {x, y, vx, vy});
                truth_data.set(timestamp, DataPointType.STATE, truth_vec);

            }
            all_sensor_data.add(sensor_data);
            all_truth_data.add(truth_data);

        }

        EKF ekf = new EKF();

        Vector<RealVector> estimations = new Vector<>();
        Vector<RealVector> ground_truths = new Vector<>();

        for (int k = 0; k < all_sensor_data.size(); ++k) {
            ekf.process(all_sensor_data.get(k));

            RealVector prediction = ekf.get();
            RealVector measurement = all_sensor_data.get(k).get_state();
            RealVector truth = all_truth_data.get(k).get();

            System.out.println("iterasi ke: " + (k+1));
            System.out.println("pred: " + prediction.getEntry(0) + " | " + prediction.getEntry(1) + " | " + prediction.getEntry(2) + " | " + prediction.getEntry(3));
            System.out.println("gt: " + truth.getEntry(0) + " | " + truth.getEntry(1) + " | " + truth.getEntry(2) + " | " + truth.getEntry(3));

            String write_this =
                    prediction.getEntry(0) + "\t" + prediction.getEntry(1) + "\t" + prediction.getEntry(2) + "\t" + prediction.getEntry(3)  + "\t" +
                    measurement.getEntry(0) + "\t" + measurement.getEntry(1) + "\t" +
                    truth.getEntry(0) + "\t" + truth.getEntry(1) + "\t" + truth.getEntry(2) + "\t" + truth.getEntry(3);

            out_file.write(write_this);
            out_file.newLine();

            estimations.add(prediction);
            ground_truths.add(truth);

            Tools tools = new Tools();
            RealVector RMSE = tools.calculate_RMSE(estimations, ground_truths);

            System.out.println("RMSE: " + RMSE);
            System.out.println("__________________________________________________________________");
        }

        in_file.close();
        out_file.close();

        System.out.println("Complete");
    }

}