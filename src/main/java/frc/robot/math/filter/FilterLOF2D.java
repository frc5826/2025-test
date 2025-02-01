package frc.robot.math.filter;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.LinkedList;
import java.util.List;

public class FilterLOF2D {

    int k;

    public FilterLOF2D(int k){
        this.k = k - 1; // Lists start at index 0 and I don't feel like dealing with that later
    }

    public double LOF(Pose2d A, List<Pose2d> L){
        if (L.size() <= k + 1){
            return -1;
        }
        double lof = 0;
        for(Pose2d B : Nk(A, L)){
            lof += lrd(B, L);
        }
        return lof / (L.size() * lrd(A, L));
    }

    public double lrd(Pose2d A, List<Pose2d> L){
        double lrd = 0;
        for(Pose2d B : Nk(A, L)){
            lrd += rdist(A, B, L);
        }
        lrd = lrd / L.size();
        return 1/lrd;
    }


    public double rdist(Pose2d A, Pose2d B, List<Pose2d> L){
        return Math.max(kdist(B, L), dist(A, B));
    }

    public LinkedList<Pose2d> Nk(Pose2d A, List<Pose2d> L){
        LinkedList<Pose2d> klist = new LinkedList<>();
        double kdist = kdist(A, L);
        for(Pose2d B : L){
            if(A != B && dist(A, B) <= kdist){
                klist.add(B);
            }
        }
        return klist;
    }

    public double kdist(Pose2d A, List<Pose2d> L){
        LinkedList<Double> distlist = new LinkedList<>();
        for(Pose2d B : L){
            if(A != B){
                distlist.add(dist(A, B));
            }
        }
        distlist.sort(null); // Same as Collections.sort();
        return distlist.get(k);
    }

    public double dist(Pose2d A, Pose2d B){
        return /*Math.sqrt(*/Math.pow(A.getX() - B.getX(), 2) + Math.pow(A.getY() - B.getY(), 2)/*)*/;
    }

    public void adjk(int adj){
        k = Math.max(k + adj, 1);
    }

    public int getK() {
        return k;
    }
}
