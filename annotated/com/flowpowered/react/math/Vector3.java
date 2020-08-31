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
 * Represents a 3D vector in space.
 */
public class Vector3 {
    /**
     * X_AXIS, represents the x axis in the vector. Value of 0
     */
    public static final @Dimensionless int X_AXIS = ((@Dimensionless int) (0));
    /**
     * Y_AXIS, represents the y axis in the vector. Value of 1
     */
    public static final @Dimensionless int Y_AXIS = ((@Dimensionless int) (1));
    /**
     * Z_AXIS, represents the z axis in the vector. Value of 2
     */
    public static final @Dimensionless int Z_AXIS = ((@Dimensionless int) (2));
    private @Dimensionless float x;
    private @Dimensionless float y;
    private @Dimensionless float z;

    /**
     * Default constructor. All values are 0.0F
     */
    public Vector3() {
        this(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Copy constructor
     *
     * @param vector to copy
     */
    public Vector3(@Dimensionless Vector3 vector) {
        this(vector.getX(), vector.getY(), vector.getZ());
    }

    /**
     * Constructor with arguments.
     *
     * @param x value
     * @param y value
     * @param z value
     */
    public Vector3(float x, float y, float z) {
        setAllValues(x, y, z);
    }

    /**
     * Sets the x value of the vector
     *
     * @param x value to set
     */
    public void setX(@Dimensionless Vector3 this, @Dimensionless float x) {
        this.x = x;
    }

    /**
     * Sets the y value of the vector
     *
     * @param y value to set
     */
    public void setY(@Dimensionless Vector3 this, @Dimensionless float y) {
        this.y = y;
    }

    /**
     * Sets the z value of the vector
     *
     * @param z value to set
     */
    public void setZ(@Dimensionless Vector3 this, @Dimensionless float z) {
        this.z = z;
    }

    /**
     * Sets all values of the vector
     *
     * @param x value to set
     * @param y value to set
     * @param z value to set
     */
    public final void setAllValues(@Dimensionless Vector3 this, @Dimensionless float x, @Dimensionless float y, @Dimensionless float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Sets the values of this vector3 to those of the provided vector3.
     *
     * @param vector The vector3 to copy the values from
     */
    public Vector3 set(@Dimensionless Vector3 this, Vector3 vector) {
        setAllValues(vector.getX(), vector.getY(), vector.getZ());
        return this;
    }

    /**
     * Gets the x value of the vector
     *
     * @return {@link float} x value
     */
    public @Dimensionless float getX(@Dimensionless Vector3 this) {
        return x;
    }

    /**
     * Gets the y value of the vector
     *
     * @return {@link float} y value
     */
    public @Dimensionless float getY(@Dimensionless Vector3 this) {
        return y;
    }

    /**
     * Gets the z value of the vector
     *
     * @return {@link float} z value
     */
    public @Dimensionless float getZ(@Dimensionless Vector3 this) {
        return z;
    }

    /**
     * Sets the x, y and z values to zero.
     */
    public void setToZero(@Dimensionless Vector3 this) {
        setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Return the length of the vector
     *
     * @return {@link float} length of the vector
     */
    public @Dimensionless float length(@Dimensionless Vector3 this) {
        return (@Dimensionless float) Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * Return the square of the length of the vector
     *
     * @return {@link float} square length of the vector
     */
    public @Dimensionless float lengthSquare(@Dimensionless Vector3 this) {
        return x * x + y * y + z * z;
    }

    /**
     * Return the axis with the minimal value
     *
     * @return {@link int} axis with minimal value
     */
    public @Dimensionless int getMinAxis(@Dimensionless Vector3 this) {
        return x < y ? (x < z ? X_AXIS : Z_AXIS) : (y < z ? Y_AXIS : Z_AXIS);
    }

    /**
     * Return the axis with the maximum value
     *
     * @return {@link int} axis with maximum value
     */
    public @Dimensionless int getMaxAxis(@Dimensionless Vector3 this) {
        return x < y ? (y < z ? Z_AXIS : Y_AXIS) : (x < z ? Z_AXIS : X_AXIS);
    }

    /**
     * True if the vector is unit, otherwise false
     *
     * @return true if the vector is unit, otherwise false
     */
    public @Dimensionless boolean isUnit(@Dimensionless Vector3 this) {
        return Mathematics.approxEquals(lengthSquare(), ((@Dimensionless int) (1)));
    }

    /**
     * True if the vector is the zero vector
     *
     * @return true if the vector is the zero vector
     */
    public @Dimensionless boolean isZero(@Dimensionless Vector3 this) {
        return Mathematics.approxEquals(lengthSquare(), ((@Dimensionless int) (0)));
    }

    /**
     * Return the corresponding unit vector. Creates a new vector.
     *
     * @return new unit {@link Vector3} corresponding to this vector
     */
    public @Dimensionless Vector3 getUnit(@Dimensionless Vector3 this) {
        final @Dimensionless float lengthVector = length();
        if (lengthVector <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        final @Dimensionless float lengthInv = ((@Dimensionless int) (1)) / lengthVector;
        return new @Dimensionless Vector3(x * lengthInv, y * lengthInv, z * lengthInv);
    }

    /**
     * Return an orthogonal vector of this vector
     *
     * @return an orthogonal {@link Vector3} of the current vector
     */
    public @Dimensionless Vector3 getOneUnitOrthogonalVector(@Dimensionless Vector3 this) {
        if (length() <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        final @Dimensionless Vector3 vectorAbs = new @Dimensionless Vector3(Math.abs(x), Math.abs(y), Math.abs(z));
        final @Dimensionless int minElement = vectorAbs.getMinAxis();
        if (minElement == ((@Dimensionless int) (0))) {
            return new @Dimensionless Vector3(((@Dimensionless int) (0)), -z, y).divide((@Dimensionless float) Math.sqrt(y * y + z * z));
        } else if (minElement == ((@Dimensionless int) (1))) {
            return new @Dimensionless Vector3(-z, ((@Dimensionless int) (0)), x).divide((@Dimensionless float) Math.sqrt(x * x + z * z));
        } else {
            return new @Dimensionless Vector3(-y, x, ((@Dimensionless int) (0))).divide((@Dimensionless float) Math.sqrt(x * x + y * y));
        }
    }

    /**
     * Normalizes the vector. Doesn't create a new vector.
     *
     * @return This vector after normalization
     */
    public @Dimensionless Vector3 normalize(@Dimensionless Vector3 this) {
        final @Dimensionless float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        x /= l;
        y /= l;
        z /= l;
        return this;
    }

    /**
     * Return the corresponding absolute value vector. Creates a new vector.
     *
     * @return new {@link Vector3} absolute value vector
     */
    public @Dimensionless Vector3 getAbsoluteVector(@Dimensionless Vector3 this) {
        return new @Dimensionless Vector3(
                Math.abs(x),
                Math.abs(y),
                Math.abs(z));
    }

    /**
     * Scalar product of two vectors
     *
     * @param vector to compute scalar product with
     * @return {@link float} scalar product
     */
    public @Dimensionless float dot(@Dimensionless Vector3 this, @Dimensionless Vector3 vector) {
        return x * vector.getX() + y * vector.getY() + z * vector.getZ();
    }

    /**
     * Crosses a vector3 with this vector. Creates a new vector.
     *
     * @param vector to compute the cross product with
     * @return a new vector, result of the cross product
     */
    public @Dimensionless Vector3 cross(@Dimensionless Vector3 this, @Dimensionless Vector3 vector) {
        return new @Dimensionless Vector3(
                y * vector.getZ() - z * vector.getY(),
                z * vector.getX() - x * vector.getZ(),
                x * vector.getY() - y * vector.getX());
    }

    /**
     * Adds a vector3 to this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to add to this one
     * @return this vector, after addition is finished
     */
    public @Dimensionless Vector3 add(@Dimensionless Vector3 this, @Dimensionless Vector3 vector) {
        x += vector.getX();
        y += vector.getY();
        z += vector.getZ();
        return this;
    }

    /**
     * Negates the components of this vector, then returns the result. Does not create a new vector.
     *
     * @return this vector, after negation is finished
     */
    public @Dimensionless Vector3 negate(@Dimensionless Vector3 this) {
        setAllValues(-x, -y, -z);
        return this;
    }

    /**
     * Subtracts a vector3 from this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to subtract from this one
     * @return the difference of this vector and the other vector
     */
    public @Dimensionless Vector3 subtract(@Dimensionless Vector3 this, @Dimensionless Vector3 vector) {
        x -= vector.getX();
        y -= vector.getY();
        z -= vector.getZ();
        return this;
    }

    /**
     * Multiplies this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after multiplication is finished
     */
    public @Dimensionless Vector3 multiply(@Dimensionless Vector3 this, @Dimensionless float value) {
        x *= value;
        y *= value;
        z *= value;
        return this;
    }

    /**
     * Divides this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after division is finished
     */
    public @Dimensionless Vector3 divide(@Dimensionless Vector3 this, @Dimensionless float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot divide by zero");
        }
        x /= value;
        y /= value;
        z /= value;
        return this;
    }

    /**
     * Gets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link Vector3#X_AXIS}<br> {@link Vector3#Y_AXIS}<br> {@link
     * Vector3#Z_AXIS}<br>
     *
     * @param axis to get; {@link Vector3#X_AXIS} OR {@link Vector3#Y_AXIS} OR {@link Vector3#Z_AXIS}
     * @return {@link float} value of the axis
     */
    public @Dimensionless float get(@Dimensionless Vector3 this, @Dimensionless int axis) {
        switch (axis) {
            case X_AXIS:
                return x;
            case Y_AXIS:
                return y;
            case Z_AXIS:
                return z;
        }
        throw new @Dimensionless UnsupportedOperationException("Must specify 0, 1, or 2 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS");
    }

    /**
     * Sets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link Vector3#X_AXIS}<br> {@link Vector3#Y_AXIS}<br> {@link
     * Vector3#Z_AXIS}<br>
     *
     * @param axis to set; {@link Vector3#X_AXIS} OR {@link Vector3#Y_AXIS} OR {@link Vector3#Z_AXIS}
     * @param value {@link float} value for the axis
     */
    public void set(@Dimensionless Vector3 this, @Dimensionless int axis, @Dimensionless float value) {
        switch (axis) {
            case X_AXIS:
                x = value;
                return;
            case Y_AXIS:
                y = value;
                return;
            case Z_AXIS:
                z = value;
                return;
        }
        throw new @Dimensionless UnsupportedOperationException("Must specify 0, 1, or 2 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS");
    }

    @Override
    public int hashCode(@Dimensionless Vector3 this) {
        final @Dimensionless int prime = ((@Dimensionless int) (31));
        @Dimensionless
        int result = ((@Dimensionless int) (1));
        result = prime * result + Float.floatToIntBits(x);
        result = prime * result + Float.floatToIntBits(y);
        result = prime * result + Float.floatToIntBits(z);
        return result;
    }

    @Override
    public boolean equals(@Dimensionless Vector3 this, Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector3)) {
            return false;
        }
        @Dimensionless
        Vector3 other = (@Dimensionless Vector3) obj;
        if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z)) {
            return false;
        }
        return true;
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Vector3 this) {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    /**
     * Adds a vector3 to another vector3. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the sum of the two vectors
     */
    public static Vector3 add(Vector3 vector1, Vector3 vector2) {
        return new @Dimensionless Vector3(
                vector1.getX() + vector2.getX(),
                vector1.getY() + vector2.getY(),
                vector1.getZ() + vector2.getZ());
    }

    /**
     * Negates the components of this vector. Creates a new vector.
     *
     * @param vector the vector to negate
     * @return the negative vector for this vector
     */
    public static Vector3 negate(Vector3 vector) {
        return new @Dimensionless Vector3(
                -vector.getX(),
                -vector.getY(),
                -vector.getZ());
    }

    /**
     * Subtracts a vector3 from another vector3. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the difference of the two vectors
     */
    public static @Dimensionless Vector3 subtract(@Dimensionless Vector3 vector1, @Dimensionless Vector3 vector2) {
        return new @Dimensionless Vector3(
                vector1.getX() - vector2.getX(),
                vector1.getY() - vector2.getY(),
                vector1.getZ() - vector2.getZ());
    }

    /**
     * Multiplies the value by a specified vector. Creates a new vector.
     *
     * @param value the value
     * @param vector the vector
     * @return the product of the value and the vector
     */
    public static @Dimensionless Vector3 multiply(@Dimensionless float value, @Dimensionless Vector3 vector) {
        return multiply(vector, value);
    }

    /**
     * Multiplies the vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the product of the vector and the value
     */
    public static Vector3 multiply(Vector3 vector, float value) {
        return new @Dimensionless Vector3(
                vector.getX() * value,
                vector.getY() * value,
                vector.getZ() * value);
    }

    /**
     * Divides this vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the quotient (vector3) of the vector and the value
     */
    public static @Dimensionless Vector3 divide(@Dimensionless Vector3 vector, @Dimensionless float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot divide by zero");
        }
        return new @Dimensionless Vector3(
                vector.getX() / value,
                vector.getY() / value,
                vector.getZ() / value);
    }
}
