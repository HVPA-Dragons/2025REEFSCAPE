package frc.robot.utils;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;

/**
 * The {@code DoubleTransformer} class is a utility class for manipulating
 * joystick input values. It implements the {@code DoubleSupplier} interface,
 * allowing it to be used as a supplier of double values.
 */
public class DoubleTransformer implements DoubleSupplier {
    private final DoubleSupplier supplier;

    /**
     * Constructs a new {@code DoubleTransformer} instance with the specified
     * {@code DoubleSupplier}.
     * 
     * @param supplier The underlying {@code DoubleSupplier} providing joystick
     *                 input values.
     */
    private DoubleTransformer(DoubleSupplier supplier) {
        this.supplier = supplier;
    }

    /**
     * Constructs a new {@code DoubleTransformer} instance with the specified
     * {@code DoubleSupplier}.
     * 
     * @return this {@code DoubleTransformer}
     */
    public static DoubleTransformer of(DoubleSupplier supplier) {
        return new DoubleTransformer(supplier);
    }

    /**
     * Returns a new {@code DoubleTransformer} that negates the joystick input
     * values.
     *
     * @return A new {@code DoubleTransformer} with negated values.
     */
    public DoubleTransformer negate() {
        return map((x) -> -x);
    }

    /**
     * Returns a new {@code DoubleTransformer} that squares the joystick input
     * values.
     *
     * @return A new {@code DoubleTransformer} with squared values.
     */
    public DoubleTransformer square() {
        return map((x) -> x * x);
    }

    /**
     * Returns a new {@code DoubleTransformer} that squares the joystick input
     * values
     * while preserving the sign.
     *
     * @return A new {@code DoubleTransformer} with signed squared values.
     */
    public DoubleTransformer signedSquare() {
        return map((x) -> x < 0 ? -x * x : x * x);
    }

    /**
     * Returns a new {@code DoubleTransformer} that applies a deadzone to the
     * joystick input values.
     *
     * @param deadzone The deadzone threshold for input values.
     * @return A new {@code DoubleTransformer} with deadzone applied.
     */
    public DoubleTransformer deadzone(double deadzone) {
        return map((x) -> Math.abs(x) < deadzone ? 0 : x);
    }

    /**
     * Returns a new {@code DoubleTransformer} that applies a deadzone to the
     * joystick input values. Uses a default value.
     *
     * @return A new {@code DoubleTransformer} with deadzone applied.
     */
    public DoubleTransformer deadzone() {
        return deadzone(0.05);
    }

    /**
     * Returns a new {@code DoubleTransformer} that maps the joystick input values
     * from one range to another.
     *
     * @param inputStart  The start of the input range.
     * @param inputEnd    The end of the input range.
     * @param outputStart The start of the output range.
     * @param outputEnd   The end of the output range.
     * @return A new {@code DoubleTransformer} with mapped values.
     */
    public DoubleTransformer mapRange(double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return map((x) -> MathUtil.interpolate(outputStart, outputEnd,
                MathUtil.inverseInterpolate(inputStart, inputEnd, x)));
    }

    /**
     * Returns a new {@code DoubleTransformer} that applies a custom mapping
     * function to the joystick input values.
     *
     * @param f The mapping function to apply.
     * @return A new {@code DoubleTransformer} with the applied mapping function.
     */
    public DoubleTransformer map(DoubleUnaryOperator f) {
        return new DoubleTransformer(() -> f.applyAsDouble(supplier.getAsDouble()));
    }

    /**
     * Gets the current joystick input value.
     *
     * @return The current joystick input value.
     */
    @Override
    public double getAsDouble() {
        return supplier.getAsDouble();
    }
}
