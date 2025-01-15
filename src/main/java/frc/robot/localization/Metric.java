package frc.robot.localization;

public record Metric(long time, MoveMetric moveMetric, MeasureMetric measureMetric, TruthMetric truthMetric) {
}
