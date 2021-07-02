package com.seulawah.faruq.ekf;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Vector;

public class Tools {
    public RealVector convert_cartesian_to_polar(final RealVector v){
        final double THRESH = 0.0001;
        RealVector polar_vec;

        final double px = v.getEntry(0);
        final double py = v.getEntry(1);
        final double vx = v.getEntry(2);
        final double vy = v.getEntry(3);

        final double rho = Math.sqrt(px*px + py*py);
        final double phi = Math.atan2(py, px);
        final double drho = rho > THRESH ? ( px * vx + py * vy ) / rho : 0.0;

        polar_vec = new ArrayRealVector(new double[] {rho, phi, drho});
        return polar_vec;
    }

    public RealVector convert_polar_to_cartesian(final RealVector v){
        RealVector cartesian_vec;

        final double rho = v.getEntry(0);
        final double phi = v.getEntry(1);
        final double drho = v.getEntry(2);

        final double px = rho * Math.cos(phi);
        final double py = rho * Math.sin(phi);
        final double vx = drho * Math.cos(phi);
        final double vy = drho * Math.sin(phi);

        cartesian_vec = new ArrayRealVector(new double[] {px, py, vx, vy});
        return cartesian_vec;
    }

    public RealMatrix calculate_Jacobian(final RealVector v){
        final double THRESH = 0.0001;
        RealMatrix H;

        H = new Array2DRowRealMatrix(new double[][] {
                {0,0,0,0},
                {0,0,0,0},
                {0,0,0,0}
        });

        final double px = v.getEntry(0);
        final double py = v.getEntry(1);
        final double vx = v.getEntry(2);
        final double vy = v.getEntry(3);

        final double d_squared = px * px + py * py;
        final double d = Math.sqrt(d_squared);
        final double d_cubed = d_squared * d;

        if (d >= THRESH){
            final  double h11 = px / d;
            final double h12 = py / d;
            final  double h21 = -py / d_squared;
            final  double h22 = px / d_squared;
            final  double h31 = px * (vx * py - vy * px) /d_cubed;
            final  double h32 = px * (vy * px - vx * py) /d_cubed;

            H = new Array2DRowRealMatrix(new double[][] {
                    {h11,h12,0,0},
                    {h21,h22,0,0},
                    {h31,h32,h11,h12}
            });
        }
        return H;
    }

    public RealVector calculate_RMSE(final Vector<RealVector> estimations, final Vector<RealVector> ground_truths){
        RealVector rmse= new ArrayRealVector(4);

        for(int k = 0; k < estimations.size(); k++){
            RealVector diff = estimations.get(k).subtract(ground_truths.get(k));
            diff = diff.ebeMultiply(diff);
            rmse = rmse.add(diff);
        }

        rmse = rmse.mapDivide(estimations.size());
        double[] rmse_array = rmse.toArray();
        for(int k = 0; k < rmse_array.length; k++){
            rmse_array[k] = Math.sqrt(rmse_array[k]);
        }

        rmse = new ArrayRealVector(rmse_array);
        return rmse;
    }

}
