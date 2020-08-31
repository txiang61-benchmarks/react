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
 * Represents a 4D vector in space.
 */
public class Vector4 {
    /**
     * X_AXIS, represents the x axis in the vector. Value of 0.
     */
    public static final @Dimensionless int X_AXIS = ((@Dimensionless int) (0));
    /**
     * Y_AXIS, represents the y axis in the vector. Value of 1.
     */
    public static final @Dimensionless int Y_AXIS = ((@Dimensionless int) (1));
    /**
     * Z_AXIS, represents the z axis in the vector. Value of 2.
     */
    public static final @Dimensionless int Z_AXIS = ((@Dimensionless int) (2));
    /**
     * W_AXIS, represents the w axis in the vector. Value of 3.
     */
    public static final @Dimensionless int W_AXIS = ((@Dimensionless int) (3));
    private @Dimensionless float x;
    private @Dimensionless float y;
    private @Dimensionless float z;
    private @Dimensionless float w;

    /**
     * Default constructor. All values are 0.
     */
    public Vector4() {
        this(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Copy constructor
     *
     * @param vector to copy
     */
    public Vector4(@Dimensionless Vector4 vector) {
        this(vector.getX(), vector.getY(), vector.getZ(), vector.getW());
    }

    /**
     * Constructor with arguments.
     *
     * @param x value
     * @param y value
     * @param z value
     * @param w value
     */
    public Vector4(float x, float y, float z, float w) {
        setAllValues(x, y, z, w);
    }

    /**
     * Sets the x value of the vector
     *
     * @param x value to set
     */
    public void setX(@Dimensionless Vector4 this, @Dimensionless float x) {
        this.x = x;
    }

    /**
     * Sets the y value of the vector
     *
     * @param y value to set
     */
    public void setY(@Dimensionless Vector4 this, @Dimensionless float y) {
        this.y = y;
    }

    /**
     * Sets the z value of the vector
     *
     * @param z value to set
     */
    public void setZ(@Dimensionless Vector4 this, @Dimensionless float z) {
        this.z = z;
    }

    /**
     * Sets the w value of the vector
     *
     * @param w value to set
     */
    public void setW(@Dimensionless Vector4 this, @Dimensionless float w) {
        this.w = w;
    }

    /**
     * Sets all values of the vector
     *
     * @param x value to set
     * @param y value to set
     * @param z value to set
     * @param w value to set
     */
    public final void setAllValues(@Dimensionless Vector4 this, float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Sets the values of this vector4 to those of the provided vector4.
     *
     * @param vector The vector4 to copy the values from
     */
    public @Dimensionless Vector4 set(@Dimensionless Vector4 this, @Dimensionless Vector4 vector) {
        setAllValues(vector.getX(), vector.getY(), vector.getZ(), vector.getW());
        return this;
    }

    /**
     * Gets the x value of the vector
     *
     * @return {@link float} x value
     */
    public float getX(@Dimensionless Vector4 this) {
        return x;
    }

    /**
     * Gets the y value of the vector
     *
     * @return {@link float} y value
     */
    public float getY(@Dimensionless Vector4 this) {
        return y;
    }

    /**
     * Gets the z value of the vector
     *
     * @return {@link float} z value
     */
    public float getZ(@Dimensionless Vector4 this) {
        return z;
    }

    /**
     * Gets the w value of the vector
     *
     * @return {@link float} w value
     */
    public float getW(@Dimensionless Vector4 this) {
        return w;
    }

    /**
     * Sets the x, y, z and w values to zero.
     */
    public void setToZero(@Dimensionless Vector4 this) {
        setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Return the length of the vector
     *
     * @return {@link float} length of the vector
     */
    public @Dimensionless float length(@Dimensionless Vector4 this) {
        return (@Dimensionless float) Math.sqrt(x * x + y * y + z * z + w * w);
    }

    /**
     * Return the square of the length of the vector
     *
     * @return {@link float} square length of the vector
     */
    public @Dimensionless float lengthSquare(@Dimensionless Vector4 this) {
        return x * x + y * y + z * z + w * w;
    }

    /**
     * Return the axis with the minimal value
     *
     * @return {@link int} axis with minimal value
     */
    public @Dimensionless int getMinAxis(@Dimensionless Vector4 this) {
        @Dimensionless
        float value = x;
        @Dimensionless
        int axis = ((@Dimensionless int) (0));
        if (y < value) {
            value = y;
            axis = ((@Dimensionless int) (1));
        }
        if (z < value) {
            value = z;
            axis = ((@Dimensionless int) (2));
        }
        if (w < value) {
            axis = ((@Dimensionless int) (3));
        }
        return axis;
    }

    /**
     * Return the axis with the maximum value
     *
     * @return {@link int} axis with maximum value
     */
    public @Dimensionless int getMaxAxis(@Dimensionless Vector4 this) {
        @Dimensionless
        float value = x;
        @Dimensionless
        int axis = ((@Dimensionless int) (0));
        if (y > value) {
            value = y;
            axis = ((@Dimensionless int) (1));
        }
        if (z > value) {
            value = z;
            axis = ((@Dimensionless int) (2));
        }
        if (w > value) {
            axis = ((@Dimensionless int) (3));
        }
        return axis;
    }

    /**
     * True if the vector is unit, otherwise false
     *
     * @return true if the vector is unit, otherwise false
     */
    public @Dimensionless boolean isUnit(@Dimensionless Vector4 this) {
        return Mathematics.approxEquals(lengthSquare(), ((@Dimensionless int) (1)));
    }

    /**
     * True if the vector is the zero vector
     *
     * @return true if the vector is the zero vector
     */
    public @Dimensionless boolean isZero(@Dimensionless Vector4 this) {
        return Mathematics.approxEquals(lengthSquare(), ((@Dimensionless int) (0)));
    }

    /**
     * Return the corresponding unit vector. Creates a new vector.
     *
     * @return new unit {@link com.flowpowered.react.math.Vector4} corresponding to this vector
     */
    public @Dimensionless Vector4 getUnit(@Dimensionless Vector4 this) {
        final @Dimensionless float lengthVector = length();
        if (lengthVector <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        final @Dimensionless float lengthInv = ((@Dimensionless int) (1)) / lengthVector;
        return new @Dimensionless Vector4(x * lengthInv, y * lengthInv, z * lengthInv, w * lengthInv);
    }

    /**
     * Normalizes the vector. Doesn't create a new vector.
     *
     * @return This vector after normalization
     */
    public @Dimensionless Vector4 normalize(@Dimensionless Vector4 this) {
        final @Dimensionless float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero vector");
        }
        x /= l;
        y /= l;
        z /= l;
        w /= l;
        return this;
    }

    /**
     * Return the corresponding absolute value vector. Creates a new vector.
     *
     * @return new {@link com.flowpowered.react.math.Vector4} absolute value vector
     */
    public @Dimensionless Vector4 getAbsoluteVector(@Dimensionless Vector4 this) {
        return new @Dimensionless Vector4(
                Math.abs(x),
                Math.abs(y),
                Math.abs(z),
                Math.abs(w));
    }

    /**
     * Scalar product of two vectors
     *
     * @param vector to compute scalar product with
     * @return {@link float} scalar product
     */
    public @Dimensionless float dot(@Dimensionless Vector4 this, @Dimensionless Vector4 vector) {
        return x * vector.getX() + y * vector.getY() + z * vector.getZ() + w * vector.getW();
    }

    /**
     * Adds a vector3 to this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to add to this one
     * @return this vector, after addition is finished
     */
    public @Dimensionless Vector4 add(@Dimensionless Vector4 this, @Dimensionless Vector4 vector) {
        x += vector.getX();
        y += vector.getY();
        z += vector.getZ();
        w += vector.getW();
        return this;
    }

    /**
     * Negates the components of this vector, then returns the result. Does not create a new vector.
     *
     * @return this vector, after negation is finished
     */
    public @Dimensionless Vector4 negate(@Dimensionless Vector4 this) {
        setAllValues(-x, -y, -z, -w);
        return this;
    }

    /**
     * Subtracts a vector3 from this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to subtract from this one
     * @return the difference of this vector and the other vector
     */
    public @Dimensionless Vector4 subtract(@Dimensionless Vector4 this, @Dimensionless Vector4 vector) {
        x -= vector.getX();
        y -= vector.getY();
        z -= vector.getZ();
        w -= vector.getW();
        return this;
    }

    /**
     * Multiplies this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after multiplication is finished
     */
    public @Dimensionless Vector4 multiply(@Dimensionless Vector4 this, @Dimensionless float value) {
        x *= value;
        y *= value;
        z *= value;
        w *= value;
        return this;
    }

    /**
     * Divides this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after division is finished
     */
    public @Dimensionless Vector4 divide(@Dimensionless Vector4 this, @Dimensionless float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot divide by zero");
        }
        x /= value;
        y /= value;
        z /= value;
        w /= value;
        return this;
    }

    /**
     * Gets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link #X_AXIS}<br> {@link #Y_AXIS}<br> {@link #Z_AXIS}<br> {@link #W_AXIS}
     *
     * @param axis to set; {@link #X_AXIS} OR {@link #Y_AXIS} OR {@link #Z_AXIS} OR {@link #W_AXIS}
     * @return {@link float} value of the axis
     */
    public float get(@Dimensionless Vector4 this, int axis) {
        switch (axis) {
            case X_AXIS:
                return x;
            case Y_AXIS:
                return y;
            case Z_AXIS:
                return z;
            case W_AXIS:
                return w;
        }
        throw new @Dimensionless UnsupportedOperationException("Must specify 0, 1, 2 or 3 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS or Vector3.W_AXIS)");
    }

    /**
     * Sets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link #X_AXIS}<br> {@link #Y_AXIS}<br> {@link #Z_AXIS}<br> {@link #W_AXIS}
     *
     * @param axis to set; {@link #X_AXIS} OR {@link #Y_AXIS} OR {@link #Z_AXIS} OR {@link #W_AXIS}
     * @param value {@link float} value for the axis
     */
    public void set(@Dimensionless Vector4 this, int axis, float value) {
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
            case W_AXIS:
                w = value;
                return;
        }
        throw new @Dimensionless UnsupportedOperationException("Must specify 0, 1, 2 or 3 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS or Vector3.W_AXIS)");
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Vector4 this) {
        final @Dimensionless int prime = ((@Dimensionless int) (31));
        @Dimensionless
        int result = ((@Dimensionless int) (1));
        result = prime * result + Float.floatToIntBits(x);
        result = prime * result + Float.floatToIntBits(y);
        result = prime * result + Float.floatToIntBits(z);
        result = prime * result + Float.floatToIntBits(w);
        return result;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Vector4 this, @Dimensionless Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector4)) {
            return false;
        }
        @Dimensionless
        Vector4 other = (@Dimensionless Vector4) obj;
        if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z)) {
            return false;
        }
        if (Float.floatToIntBits(w) != Float.floatToIntBits(other.w)) {
            return false;
        }
        return true;
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Vector4 this) {
        return "(" + x + ", " + y + ", " + z + ", " + w + ")";
    }

    /**
     * Adds a vector4 to another vector4. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the sum of the two vectors
     */
    public static @Dimensionless Vector4 add(@Dimensionless Vector4 vector1, @Dimensionless Vector4 vector2) {
        return new @Dimensionless Vector4(
                vector1.getX() + vector2.getX(),
                vector1.getY() + vector2.getY(),
                vector1.getZ() + vector2.getZ(),
                vector1.getW() + vector2.getW());
    }

    /**
     * Negates the components of this vector. Creates a new vector.
     *
     * @param vector the vector to negate
     * @return the negative vector for this vector
     */
    public static @Dimensionless Vector4 negate(@Dimensionless Vector4 vector) {
        return new @Dimensionless Vector4(
                -vector.getX(),
                -vector.getY(),
                -vector.getZ(),
                -vector.getW());
    }

    /**
     * Subtracts a vector4 from another vector4. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the difference of the two vectors
     */
    public static @Dimensionless Vector4 subtract(@Dimensionless Vector4 vector1, @Dimensionless Vector4 vector2) {
        return new @Dimensionless Vector4(
                vector1.getX() - vector2.getX(),
                vector1.getY() - vector2.getY(),
                vector1.getZ() - vector2.getZ(),
                vector1.getW() - vector2.getW());
    }

    /**
     * Multiplies the value by a specified vector. Creates a new vector.
     *
     * @param value the value
     * @param vector the vector
     * @return the product of the value and the vector
     */
    public static @Dimensionless Vector4 multiply(@Dimensionless float value, @Dimensionless Vector4 vector) {
        return multiply(vector, value);
    }

    /**
     * Multiplies the vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the product of the vector and the value
     */
    public static @Dimensionless Vector4 multiply(@Dimensionless Vector4 vector, @Dimensionless float value) {
        return new @Dimensionless Vector4(
                vector.getX() * value,
                vector.getY() * value,
                vector.getZ() * value,
                vector.getW() * value);
    }

    /**
     * Divides this vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the quotient (vector4) of the vector and the value
     */
    public static @Dimensionless Vector4 divide(@Dimensionless Vector4 vector, @Dimensionless float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot divide by zero");
        }
        return new @Dimensionless Vector4(
                vector.getX() / value,
                vector.getY() / value,
                vector.getZ() / value,
                vector.getW() / value);
    }
}
