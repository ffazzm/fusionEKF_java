package com.seulawah.faruq.ekf;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import static org.apache.commons.math3.linear.MatrixUtils.inverse;

public class KalmanFilter {

    private int n;
    private RealVector x;
    private RealMatrix P;
    private RealMatrix F;
    private RealMatrix Q;
    private RealMatrix I;

    public void start(final int nin, final RealVector xin, final RealMatrix Pin, final RealMatrix Fin, final RealMatrix Qin){
        this.n = nin;
        this.x = xin;
        this.P = Pin;
        this.F = Fin;
        this.Q = Qin;
        this.I = MatrixUtils.createRealIdentityMatrix(n);
    }

    public void setQ(final RealMatrix Qin){
        this.Q = Qin;
    }

    public void updateF(final double dt){
        this.F.setEntry(0,2, dt);
        this.F.setEntry(1,3, dt);
    }

    public final RealVector get(){
        return this.x;
    }

    public void predict(){
        this.x = F.operate(x);
        this.P = F.multiply(P).multiply(this.F.transpose()).add(this.Q);
    }

    public void update(final RealVector z, final RealMatrix H, final RealVector Hx, final RealMatrix R){
        final RealMatrix Pht = this.P.multiply(H.transpose());
        final RealMatrix S = H.multiply(Pht).add(R);
        final RealMatrix K = Pht.multiply(inverse(S));
        RealVector y = z.subtract(Hx);

        if (y.getDimension() == 3) {
            y.setEntry(1, Math.atan2(Math.sin(y.getEntry(1)), Math.cos(y.getEntry(1))));
            this.x = this.x.add(K.operate(y));
            this.P = (this.I.subtract(K.multiply(H)).multiply(this.P));
        }
    }



}
