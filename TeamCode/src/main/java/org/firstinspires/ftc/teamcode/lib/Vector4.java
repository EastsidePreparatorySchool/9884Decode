package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.NonNull;

import java.util.Iterator;
import java.util.function.BiFunction;
import java.util.stream.DoubleStream;

/**
 * A class that represents a 4-dimensional vector (w, x, y, z) with basic vector operations
 * such as addition, subtraction, multiplication, and division.
 * Implements {@link Iterable} for iterating over the vector components and supports cloning.
 */
public final class Vector4 implements Iterable<Double>, Cloneable {

    // Fields
    /** The w component of the vector */
    public double w;
    /** The x component of the vector */
    public double x;
    /** The y component of the vector */
    public double y;
    /** The z component of the vector */
    public double z;

    // Getters and Setters
    /**
     * Returns the w component of the vector.
     *
     * @return the w component
     */
    public double getW() {
        return w;
    }

    /**
     * Returns the x component of the vector.
     *
     * @return the x component
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y component of the vector.
     *
     * @return the y component
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the z component of the vector.
     *
     * @return the z component
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets the w component of the vector.
     *
     * @param w the new value for the w component
     * @return the updated w value
     */
    public double setW(double w) {
        this.w = w;
        return w;
    }

    /**
     * Sets the x component of the vector.
     *
     * @param x the new value for the x component
     * @return the updated x value
     */
    public double setX(double x) {
        this.x = x;
        return x;
    }

    /**
     * Sets the y component of the vector.
     *
     * @param y the new value for the y component
     * @return the updated y value
     */
    public double setY(double y) {
        this.y = y;
        return y;
    }

    /**
     * Sets the z component of the vector.
     *
     * @param z the new value for the z component
     * @return the updated z value
     */
    public double setZ(double z) {
        this.z = z;
        return z;
    }

    /**
     * Gets the vector component at a given index (0 for w, 1 for x, 2 for y, 3 for z).
     *
     * @param i the index of the component
     * @return the value of the component at the given index
     * @throws IndexOutOfBoundsException if the index is outside the range [0, 3]
     */
    public double get(int i) {
        switch (i) {
            case 0: return w;
            case 1: return x;
            case 2: return y;
            case 3: return z;
            default: throw new IndexOutOfBoundsException();
        }
    }

    /**
     * Sets the vector component at a given index.
     *
     * @param i the index of the component to set
     * @param v the new value for the component
     * @throws IndexOutOfBoundsException if the index is outside the range [0, 3]
     */
    public void set(int i, double v) {
        switch (i) {
            case 0: w = v; break;
            case 1: x = v; break;
            case 2: y = v; break;
            case 3: z = v; break;
            default: throw new IndexOutOfBoundsException();
        }
    }

    // Constructors
    /**
     * Creates a new Vector4 instance with the specified components.
     *
     * @param _w the w component
     * @param _x the x component
     * @param _y the y component
     * @param _z the z component
     */
    public Vector4(double _w, double _x, double _y, double _z) {
        w = _w;
        x = _x;
        y = _y;
        z = _z;
    }

    /**
     * Creates a new Vector4 instance as a copy of another Vector4.
     *
     * @param v the vector to copy
     */
    public Vector4(@NonNull Vector4 v) {
        w = v.w;
        x = v.x;
        y = v.y;
        z = v.z;
    }

    // Instance generators
    /**
     * Creates a new Vector4 instance with the specified components.
     *
     * @param w the w component
     * @param x the x component
     * @param y the y component
     * @param z the z component
     * @return a new Vector4 instance
     */
    public static Vector4 of(double w, double x, double y, double z) {
        return new Vector4(w, x, y, z);
    }

    /**
     * Creates a new Vector4 instance by copying another Vector4.
     *
     * @param template the vector to copy
     * @return a new Vector4 instance
     */
    public static Vector4 of(Vector4 template) {
        return template.easyClone();
    }

    /**
     * Combines two Vector4 instances using a custom binary function for each component.
     *
     * @param lhs the left-hand side vector
     * @param rhs the right-hand side vector
     * @param func the function to combine each component
     * @return a new Vector4 with combined components
     */
    public static Vector4 of(Vector4 lhs, Vector4 rhs, BiFunction<Double, Double, Double> func) {
        return combine(lhs, rhs, func);
    }

    /**
     * Creates a new Vector4 where all components are set to the specified value.
     *
     * @param value the value for all components
     * @return a new Vector4 instance with all components equal to the given value
     */
    public static Vector4 repeat(double value) {
        return of(value, value, value, value);
    }

    /**
     * Combines two Vector4 instances using a binary function for each component.
     *
     * @param lhs the left-hand side vector
     * @param rhs the right-hand side vector
     * @param func the function to apply to each pair of components
     * @return a new Vector4 with combined components
     */
    public static Vector4 combine(Vector4 rhs, Vector4 lhs, BiFunction<Double, Double, Double> func) {
        return new Vector4(
                func.apply(lhs.w, rhs.w),
                func.apply(lhs.x, rhs.x),
                func.apply(lhs.y, rhs.y),
                func.apply(lhs.z, rhs.z)
        );
    }

    /**
     * Combines a Vector4 with a scalar using a binary function for each component.
     *
     * @param v the vector to combine
     * @param value the scalar value
     * @param func the function to apply to each component and the scalar
     * @return a new Vector4 with combined components
     */
    public static Vector4 combine(Vector4 v, double value, BiFunction<Double, Double, Double> func) {
        return new Vector4(
                func.apply(v.w, value),
                func.apply(v.x, value),
                func.apply(v.y, value),
                func.apply(v.z, value)
        );
    }

    /**
     * Adds two Vector4 instances.
     *
     * @param lhs the left-hand side vector
     * @param rhs the right-hand side vector
     * @return a new Vector4 that is the sum of the two vectors
     */
    public static Vector4 add(Vector4 lhs, Vector4 rhs) {
        return combine(lhs, rhs, Double::sum);
    }

    /**
     * Subtracts the right-hand side vector from the left-hand side vector.
     *
     * @param lhs the left-hand side vector
     * @param rhs the right-hand side vector
     * @return a new Vector4 that is the difference of the two vectors
     */
    public static Vector4 sub(Vector4 lhs, Vector4 rhs) {
        return combine(lhs, rhs, (a, b) -> a - b);
    }

    /**
     * Multiplies two Vector4 instances component-wise.
     *
     * @param lhs the left-hand side vector
     * @param rhs the right-hand side vector
     * @return a new Vector4 that is the product of the two vectors
     */
    public static Vector4 mult(Vector4 lhs, Vector4 rhs) {
        return combine(lhs, rhs, (a, b) -> a * b);
    }

    /**
     * Divides the left-hand side vector by the right-hand side vector component-wise.
     *
     * @param lhs the left-hand side vector
     * @param rhs the right-hand side vector
     * @return a new Vector4 that is the quotient of the two vectors
     */
    public static Vector4 div(Vector4 lhs, Vector4 rhs) {
        return combine(lhs, rhs, (a, b) -> a / b);
    }

    /**
     * Multiplies a Vector4 by a scalar.
     *
     * @param v the vector
     * @param value the scalar value
     * @return a new Vector4 that is the result of scalar multiplication
     */
    public static Vector4 mult(Vector4 v, double value) {
        return combine(v, value, (a, b) -> a * b);
    }

    /**
     * Divides a Vector4 by a scalar.
     *
     * @param v the vector
     * @param value the scalar value
     * @return a new Vector4 that is the result of scalar division
     */
    public static Vector4 div(Vector4 v, double value) {
        return combine(v, value, (a, b) -> a / b);
    }

    /**
     * Negates a Vector4 (multiplies all components by -1).
     *
     * @param v the vector to negate
     * @return a new Vector4 with negated components
     */
    public static Vector4 neg(Vector4 v) {
        return mult(v, -1);
    }

    // Binary mutators
    /**
     * Applies a binary function to each component of this vector and another vector,
     * modifying this vector.
     *
     * @param other the other vector
     * @param func the function to apply to each pair of components
     * @return this vector after applying the operation
     */
    public Vector4 apply(Vector4 other, BiFunction<Double, Double, Double> func) {
        w = func.apply(w, other.w);
        x = func.apply(x, other.x);
        y = func.apply(y, other.y);
        z = func.apply(z, other.z);
        return this;
    }

    /**
     * Applies a binary function to each component of this vector and a scalar,
     * modifying this vector.
     *
     * @param scalar the scalar value
     * @param func the function to apply to each component and the scalar
     * @return this vector after applying the operation
     */
    public Vector4 apply(double scalar, BiFunction<Double, Double, Double> func) {
        w = func.apply(w, scalar);
        x = func.apply(x, scalar);
        y = func.apply(y, scalar);
        z = func.apply(z, scalar);
        return this;
    }

    // Vector operations that modify this instance
    /**
     * Adds another vector to this vector.
     *
     * @param other the vector to add
     * @return this vector after the addition
     */
    public Vector4 add(Vector4 other) {
        return apply(other, Double::sum);
    }

    /**
     * Subtracts another vector from this vector.
     *
     * @param other the vector to subtract
     * @return this vector after the subtraction
     */
    public Vector4 sub(Vector4 other) {
        return apply(other, (a, b) -> a - b);
    }

    /**
     * Multiplies this vector by another vector component-wise.
     *
     * @param other the vector to multiply with
     * @return this vector after the multiplication
     */
    public Vector4 mult(Vector4 other) {
        return apply(other, (a, b) -> a * b);
    }

    /**
     * Divides this vector by another vector component-wise.
     *
     * @param other the vector to divide by
     * @return this vector after the division
     */
    public Vector4 div(Vector4 other) {
        return apply(other, (a, b) -> a / b);
    }

    /**
     * Multiplies this vector by a scalar.
     *
     * @param scalar the scalar value
     * @return this vector after the scalar multiplication
     */
    public Vector4 mult(double scalar) {
        return apply(scalar, (a, b) -> a * b);
    }

    /**
     * Divides this vector by a scalar.
     *
     * @param scalar the scalar value
     * @return this vector after the scalar division
     */
    public Vector4 div(double scalar) {
        return apply(scalar, (a, b) -> a / b);
    }

    // Cloneable interface
    /**
     * Creates a shallow copy of this Vector4 instance.
     *
     * @return a new Vector4 that is a copy of this instance
     */
    @NonNull
    public Vector4 easyClone() {
        return new Vector4(this);
    }

    /**
     * Clones this Vector4 instance.
     *
     * @return a clone of this instance
     */
    @NonNull
    @Override
    public Object clone() {
        return easyClone();
    }

    /**
     * Creates a shallow copy of the specified Vector4.
     *
     * @param template the Vector4 to copy
     * @return a new Vector4 that is a copy of the specified vector
     */
    public static Vector4 clone(Vector4 template) {
        return new Vector4(template);
    }

    // Iterable interface
    /**
     * Returns an iterator over the components of the vector (w, x, y, z).
     *
     * @return an iterator over the components
     */
    @NonNull
    @Override
    public Iterator<Double> iterator() {
        return new Vector4Iterator();
    }

    /**
     * Returns a string representation of the vector.
     *
     * @return a string representing the vector
     */
    @Override
    public String toString() {
        return "Vector4{" +
                "w=" + w +
                ", x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }

    // Iterator class for Vector4
    /**
     * An iterator over the components of the Vector4.
     */
    private final class Vector4Iterator implements Iterator<Double> {
        int index = 0;

        @Override
        public boolean hasNext() {
            return index < 4;
        }

        @Override
        public Double next() {
            return get(index++);
        }
    }

    // Streams
    /**
     * Returns a stream of the components of the vector.
     *
     * @return a DoubleStream of the vector components
     */
    public DoubleStream stream() {
        return DoubleStream.of(w, x, y, z);
    }
}