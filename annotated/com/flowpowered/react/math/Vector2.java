/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.math;

import units.qual.Dimensionless;
import com.flowpowered.react.ReactDefaults;

/**
 *
 */
public class Vector2 {
    /**
     * X_AXIS, represents the x axis in the vector. Value of 0
     */
    public static final @Dimensionless int X_AXIS = ((@Dimensionless int) (0));
    /**
     * Y_AXIS, represents the y axis in the vector. Value of 1
     */
    public static final @Dimensionless int Y_AXIS = ((@Dimensionless int) (1));
    private @Dimensionless float x;
    private @Dimensionless float y;

    /**
     * Default constructor. All values are 0.0F
     */
    public Vector2() {
        this(((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Copy constructor
     *
     * @param vector to copy
     */
    public Vector2(@Dimensionless Vector2 vector) {
        this(vector.getX(), vector.getY());
    }

    /**
     * Constructor with arguments.
     *
     * @param x value
     * @param y value
     */
    public Vector2(float x, float y) {
        setAllValues(x, y);
    }

    /**
     * Sets the x value of the vector
     *
     * @param x value to set
     */
    public void setX(@Dimensionless Vector2 this, float x) {
        this.x = x;
    }

    /**
     * Sets the y value of the vector
     *
     * @param y value to set
     */
    public void setY(@Dimensionless Vector2 this, float y) {
        this.y = y;
    }

    /**
     * Sets all values of the vector
     *
     * @param x value to set
     * @param y value to set
     */
    public final void setAllValues(@Dimensionless Vector2 this, @Dimensionless float x, @Dimensionless float y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Sets the values of this vector2 to those of the provided vector2.
     *
     * @param vector The vector2 to copy the values from
     */
    public Vector2 set(@Dimensionless Vector2 this, Vector2 vector) {
        setAllValues(vector.getX(), vector.getY());
        return this;
    }

    /**
     * Gets the x value of the vector
     *
     * @return {@link float} x value
     */
    public float getX(@Dimensionless Vector2 this) {
        return x;
    }

    /**
     * Gets the y value of the vector
     *
     * @return {@link float} y value
     */
    public float getY(@Dimensionless Vector2 this) {
        return y;
    }

    /**
     * Sets the x, y and z values to zero.
     */
    public void setToZero(@Dimensionless Vector2 this) {
        setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Return the length of the vector
     *
     * @return {@link float} length of the vector
     */
    public @Dimensionless float length(@Dimensionless Vector2 this) {
        return (@Dimensionless float) Math.sqrt(x * x + y * y);
    }

    /**
     * Return the square of the length of the vector
     *
     * @return {@link float} square length of the vector
     */
    public @Dimensionless float lengthSquare(@Dimensionless Vector2 this) {
        return x * x + y * y;
    }

    /**
     * Return the axis with the minimal value
     *
     * @return {@link int} axis with minimal value
     */
    public @Dimensionless int getMinAxis(@Dimensionless Vector2 this) {
        return x < y ? X_AXIS : Y_AXIS;
    }

    /**
     * Return the axis with the maximum value
     *
     * @return {@link int} axis with maximum value
     */
    public @Dimensionless int getMaxAxis(@Dimensionless Vector2 this) {
        return x < y ? Y_AXIS : X_AXIS;
    }

    /**
     * True if the vector is unit, otherwise false
     *
     * @return true if the vector is unit, otherwise false
     */
    public @Dimensionless boolean isUnit(@Dimensionless Vector2 this) {
        return Mathematics.approxEquals(lengthSquare(), ((@Dimensionless int) (1)));
    }

    /**
     * True if the vector is the zero vector
     *
     * @return true if the vector is the zero vector
     */
    public @Dimensionless boolean isZero(@Dimensionless Vector2 this) {
        return Mathematics.approxEquals(lengthSquare(), ((@Dimensionless int) (0)));
    }

    /**
     * Return the corresponding unit vector. Creates a new vector.
     *
     * @return new unit {@link Vector2} corresponding to this vector
     */
    public @Dimensionless Vector2 getUnit(@Dimensionless Vector2 this) {
        final @Dimensionless float lengthVector = length();
        if (lengthVector <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        final @Dimensionless float lengthInv = ((@Dimensionless int) (1)) / lengthVector;
        return new @Dimensionless Vector2(x * lengthInv, y * lengthInv);
    }

    /**
     * Return an orthogonal vector of this vector
     *
     * @return an orthogonal {@link Vector2} of the current vector
     */
    public @Dimensionless Vector2 getOneUnitOrthogonalVector(@Dimensionless Vector2 this) {
        final @Dimensionless float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        return new @Dimensionless Vector2(-y / l, x / l);
    }

    /**
     * Normalizes the vector. Doesn't create a new vector.
     *
     * @return This vector after normalization
     */
    public @Dimensionless Vector2 normalize(@Dimensionless Vector2 this) {
        final @Dimensionless float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        x /= l;
        y /= l;
        return this;
    }

    /**
     * Return the corresponding absolute value vector. Creates a new vector.
     *
     * @return new {@link Vector2} absolute value vector
     */
    public @Dimensionless Vector2 getAbsoluteVector(@Dimensionless Vector2 this) {
        return new @Dimensionless Vector2(
                Math.abs(x),
                Math.abs(y));
    }

    /**
     * Scalar product of two vectors
     *
     * @param vector to compute scalar product with
     * @return {@link float} scalar product
     */
    public @Dimensionless float dot(@Dimensionless Vector2 this, @Dimensionless Vector2 vector) {
        return x * vector.getX() + y * vector.getY();
    }

    /**
     * Adds a vector2 to this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to add to this one
     * @return this vector, after addition is finished
     */
    public Vector2 add(@Dimensionless Vector2 this, Vector2 vector) {
        x += vector.getX();
        y += vector.getY();
        return this;
    }

    /**
     * Negates the components of this vector, then returns the result. Does not create a new vector.
     *
     * @return this vector, after negation is finished
     */
    public @Dimensionless Vector2 negate(@Dimensionless Vector2 this) {
        setAllValues(-x, -y);
        return this;
    }

    /**
     * Subtracts a vector2 from this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to subtract from this one
     * @return the difference of this vector and the other vector
     */
    public @Dimensionless Vector2 subtract(@Dimensionless Vector2 this, @Dimensionless Vector2 vector) {
        x -= vector.getX();
        y -= vector.getY();
        return this;
    }

    /**
     * Multiplies this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after multiplication is finished
     */
    public Vector2 multiply(@Dimensionless Vector2 this, float value) {
        x *= value;
        y *= value;
        return this;
    }

    /**
     * Divides this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after division is finished
     */
    public @Dimensionless Vector2 divide(@Dimensionless Vector2 this, @Dimensionless float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot divide by zero");
        }
        x /= value;
        y /= value;
        return this;
    }

    /**
     * Gets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link Vector2#X_AXIS}<br> {@link Vector2#Y_AXIS}
     *
     * @param axis to get; {@link Vector2#X_AXIS} OR {@link Vector2#Y_AXIS}
     * @return The value of the axis
     */
    public @Dimensionless float get(@Dimensionless Vector2 this, @Dimensionless int axis) {
        switch (axis) {
            case X_AXIS:
                return x;
            case Y_AXIS:
                return y;
        }
        throw new @Dimensionless UnsupportedOperationException("Must specify 0 or 1 as an axis. (Vector2.X_AXIS, Vector2.Y_AXIS)");
    }

    /**
     * Sets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link Vector2#X_AXIS}<br> {@link Vector2#Y_AXIS}
     *
     * @param axis to set; {@link Vector2#X_AXIS} OR {@link Vector2#Y_AXIS}
     * @param value The value for the axis
     */
    public void set(@Dimensionless Vector2 this, @Dimensionless int axis, @Dimensionless float value) {
        switch (axis) {
            case X_AXIS:
                x = value;
                return;
            case Y_AXIS:
                y = value;
                return;
        }
        throw new @Dimensionless UnsupportedOperationException("Must specify 0, or 1 as an axis. (Vector2.X_AXIS, Vector2.Y_AXIS)");
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Vector2 this) {
        final @Dimensionless int prime = ((@Dimensionless int) (31));
        @Dimensionless
        int result = ((@Dimensionless int) (1));
        result = prime * result + Float.floatToIntBits(x);
        result = prime * result + Float.floatToIntBits(y);
        return result;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Vector2 this, @Dimensionless Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector2)) {
            return false;
        }
        @Dimensionless
        Vector2 other = (@Dimensionless Vector2) obj;
        if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        return true;
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Vector2 this) {
        return "(" + x + ", " + y + ")";
    }

    /**
     * Adds a vector2 to another vector2. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the sum of the two vectors
     */
    public static @Dimensionless Vector2 add(@Dimensionless Vector2 vector1, @Dimensionless Vector2 vector2) {
        return new @Dimensionless Vector2(
                vector1.getX() + vector2.getX(),
                vector1.getY() + vector2.getY());
    }

    /**
     * Negates the components of this vector. Creates a new vector.
     *
     * @param vector the vector to negate
     * @return the negative vector for this vector
     */
    public static Vector2 negate(Vector2 vector) {
        return new @Dimensionless Vector2(
                -vector.getX(),
                -vector.getY());
    }

    /**
     * Subtracts a vector2 from another vector2. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the difference of the two vectors
     */
    public static Vector2 subtract(Vector2 vector1, Vector2 vector2) {
        return new @Dimensionless Vector2(
                vector1.getX() - vector2.getX(),
                vector1.getY() - vector2.getY());
    }

    /**
     * Multiplies the value by a specified vector. Creates a new vector.
     *
     * @param value the value
     * @param vector the vector
     * @return the product of the value and the vector
     */
    public static Vector2 multiply(float value, Vector2 vector) {
        return multiply(vector, value);
    }

    /**
     * Multiplies the vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the product of the vector and the value
     */
    public static @Dimensionless Vector2 multiply(@Dimensionless Vector2 vector, @Dimensionless float value) {
        return new @Dimensionless Vector2(
                vector.getX() * value,
                vector.getY() * value);
    }

    /**
     * Divides this vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the quotient (vector2) of the vector and the value
     */
    public static @Dimensionless Vector2 divide(@Dimensionless Vector2 vector, @Dimensionless float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot divide by zero");
        }
        return new @Dimensionless Vector2(
                vector.getX() / value,
                vector.getY() / value);
    }
}
