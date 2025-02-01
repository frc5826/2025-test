package frc.robot.localization;

import edu.wpi.first.math.Pair;

import java.util.Arrays;
import java.util.Optional;

public class FilterBuffer {

    private FilterSnapshot[] snapshots;
    private int index;
    private double period;

    public FilterBuffer(int length, double period) {
        snapshots = new FilterSnapshot[length];
        this.period = period;
    }

    public void addSnapshot(FilterSnapshot snapshot) {
        snapshots[Math.floorMod(index++, snapshots.length)] = snapshot;
    }

    public void addSnapshot(FilterSnapshot snapshot, int index) {
        snapshots[Math.floorMod((this.index - index), snapshots.length)] = snapshot;
    }

    public Optional<Pair<FilterSnapshot, Integer>> getSnapshot(double time) {
        if(index < snapshots.length) {
            return  Optional.empty();
        }
        System.out.println("camera time: " + time);
        System.out.println("snapshot time: " + snapshots[Math.floorMod(index, snapshots.length)].time());

        int guess = (int) Math.ceil((snapshots[Math.floorMod(index, snapshots.length)].time() - time) / period);

        if (guess > snapshots.length) {
            return Optional.empty();
        }

        while (guess++ != index) {
            if (snapshots[Math.floorMod((index - guess), snapshots.length)].time() < time) {
                return Optional.of(Pair.of(snapshots[Math.floorMod(guess, snapshots.length)], index - guess));
            }
        }

        return Optional.empty();
    }

    public FilterSnapshot[] getReplay(int index) {
        index %= snapshots.length;
        FilterSnapshot[] replay = new FilterSnapshot[index-1];

        for (int i = 0; i+1 < index; i++) {
            replay[i] = snapshots[Math.floorMod((this.index - index + i+1), snapshots.length)];
        }

        return replay;
    }


}
