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
import units.qual.rad;
import units.qual.UnitsRep;
import com.flowpowered.react.ReactDefaults;

/**
 * Represents a quaternion. The notation q = (x*i, y*j, z*k, w) is used to represent the quaternion.
 */
public class Quaternion {
    private @Dimensionless float x;
    private @Dimensionless float y;
    private @Dimensionless float z;
    private @Dimensionless float w;

    /**
     * Default constructor. All values are zero.
     */
    public Quaternion() {
        this(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Constructs a new quaternion from the w component as a float and the x, y and z component as a vector.
     *
     * @param w The w component
     * @param v The vector for the x, y and z component
     */
    public Quaternion(@Dimensionless float w, @Dimensionless Vector3 v) {
        this(v.getX(), v.getY(), v.getZ(), w);
    }

    /**
     * Copy constructor.
     *
     * @param quaternion The quaternion to copy
     */
    public Quaternion(@Dimensionless Quaternion quaternion) {
        this(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());
    }

    /**
     * Constructs a new quaternion with the desired values for each component.
     *
     * @param x The value of the x component
     * @param y The value of the y component
     * @param z The value of the z component
     * @param w The value of the w component
     */
    public Quaternion(@Dimensionless float x, @Dimensionless float y, @Dimensionless float z, @Dimensionless float w) {
        setAllValues(x, y, z, w);
    }

    /**
     * Constructs a quaternion from the rotation element of the matrix.
     *
     * @param matrix The matrix to get the rotation element from
     */
    public Quaternion(Matrix3x3 matrix) {
        final @Dimensionless float trace = matrix.getTrace();
        if (trace < ((@Dimensionless int) (0))) {
            if (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) > matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0)))) {
                if (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) > matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1)))) {
                    final @Dimensionless float r = (@Dimensionless float) Math.sqrt(matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + ((@Dimensionless int) (1)));
                    final @Dimensionless float s = ((@Dimensionless float) (0.5f)) / r;
                    x = (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2)))) * s;
                    y = (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1)))) * s;
                    z = ((@Dimensionless float) (0.5f)) * r;
                    w = (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1)))) * s;
                } else {
                    final @Dimensionless float r = (@Dimensionless float) Math.sqrt(matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) + ((@Dimensionless int) (1)));
                    final @Dimensionless float s = ((@Dimensionless float) (0.5f)) / r;
                    x = (matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))) * s;
                    y = ((@Dimensionless float) (0.5f)) * r;
                    z = (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1)))) * s;
                    w = (matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0)))) * s;
                }
            } else if (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) > matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0)))) {
                final @Dimensionless float r = (@Dimensionless float) Math.sqrt(matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) + ((@Dimensionless int) (1)));
                final @Dimensionless float s = ((@Dimensionless float) (0.5f)) / r;
                x = (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) + matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2)))) * s;
                y = (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2))) + matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1)))) * s;
                z = ((@Dimensionless float) (0.5f)) * r;
                w = (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1)))) * s;
            } else {
                final @Dimensionless float r = (@Dimensionless float) Math.sqrt(matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (2))) + ((@Dimensionless int) (1)));
                final @Dimensionless float s = ((@Dimensionless float) (0.5f)) / r;
                x = ((@Dimensionless float) (0.5f)) * r;
                y = (matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1))) + matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0)))) * s;
                z = (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2)))) * s;
                w = (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))) * s;
            }
        } else {
            final @Dimensionless float r = (@Dimensionless float) Math.sqrt(trace + ((@Dimensionless int) (1)));
            final @Dimensionless float s = ((@Dimensionless float) (0.5f)) / r;
            x = (matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (1))) - matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (2)))) * s;
            y = (matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (2))) - matrix.get(((@Dimensionless int) (2)), ((@Dimensionless int) (0)))) * s;
            z = (matrix.get(((@Dimensionless int) (1)), ((@Dimensionless int) (0))) - matrix.get(((@Dimensionless int) (0)), ((@Dimensionless int) (1)))) * s;
            w = ((@Dimensionless float) (0.5f)) * r;
        }
    }

    /**
     * Gets the x component of the quaternion.
     *
     * @return The x component of the quaternion
     */
    public @Dimensionless float getX(@Dimensionless Quaternion this) {
        return x;
    }

    /**
     * Gets the y component of the quaternion.
     *
     * @return The y component of the quaternion
     */
    public @Dimensionless float getY(@Dimensionless Quaternion this) {
        return y;
    }

    /**
     * Gets the z component of the quaternion.
     *
     * @return The z component of the quaternion
     */
    public @Dimensionless float getZ(@Dimensionless Quaternion this) {
        return z;
    }

    /**
     * Gets the w component of the quaternion.
     *
     * @return The w component of the quaternion
     */
    public @Dimensionless float getW(@Dimensionless Quaternion this) {
        return w;
    }

    /**
     * Sets the x component of the quaternion to the desired value.
     *
     * @param x The desired value for the x component
     */
    public void setX(@Dimensionless Quaternion this, @Dimensionless float x) {
        this.x = x;
    }

    /**
     * Sets the y component of the quaternion to the desired value.
     *
     * @param y The desired value for the y component
     */
    public void setY(@Dimensionless Quaternion this, @Dimensionless float y) {
        this.y = y;
    }

    /**
     * Sets the z component of the quaternion to the desired value.
     *
     * @param z The desired value for the z component
     */
    public void setZ(@Dimensionless Quaternion this, @Dimensionless float z) {
        this.z = z;
    }

    /**
     * Sets the w component of the quaternion to the desired value.
     *
     * @param w The desired value for the w component
     */
    public void setW(@Dimensionless Quaternion this, @Dimensionless float w) {
        this.w = w;
    }

    /**
     * Sets the values of this quaternion to those of the provided quaternion.
     *
     * @param quaternion The quaternion to copy the values from
     */
    public Quaternion set(@Dimensionless Quaternion this, Quaternion quaternion) {
        setAllValues(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());
        return this;
    }

    /**
     * Sets the x, y, z and w values of this quaternion to the desired ones.
     *
     * @param x The desired x value
     * @param y The desired y value
     * @param z The desired z value
     * @param w The desired w value
     */
    public final void setAllValues(@Dimensionless Quaternion this, @Dimensionless float x, @Dimensionless float y, @Dimensionless float z, @Dimensionless float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Sets the x, y, z and w values of this quaternion to zero.
     */
    public void setToZero(@Dimensionless Quaternion this) {
        setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    /**
     * Sets to the identity quaternion.
     */
    public void setToIdentity(@Dimensionless Quaternion this) {
        setAllValues(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Adds a quaternion to this quaternion, then returns the result. Does not create a new quaternion.
     *
     * @param quaternion to add to this one
     * @return this quaternion, after addition is finished
     */
    public @Dimensionless Quaternion add(@Dimensionless Quaternion this, @Dimensionless Quaternion quaternion) {
        x += quaternion.getX();
        y += quaternion.getY();
        z += quaternion.getZ();
        w += quaternion.getW();
        return this;
    }

    /**
     * Subtracts a quaternion from this quaternion, then returns the result. Does not create a new quaternion.
     *
     * @param quaternion to subtract from this one
     * @return the difference of this quaternion and the other quaternion
     */
    public @Dimensionless Quaternion subtract(@Dimensionless Quaternion this, @Dimensionless Quaternion quaternion) {
        x -= quaternion.getX();
        y -= quaternion.getY();
        z -= quaternion.getZ();
        w -= quaternion.getW();
        return this;
    }

    /**
     * Returns the x, y and z values of this quaternion as a vector3.
     *
     * @return The x, y and z values of this quaternion as a vector3.
     */
    public @Dimensionless Vector3 getVectorV(@Dimensionless Quaternion this) {
        return new @Dimensionless Vector3(x, y, z);
    }

    /**
     * Returns the square of the length of the quaternion.
     *
     * @return The length squared
     */
    public @Dimensionless float lengthSquare(@Dimensionless Quaternion this) {
        return x * x + y * y + z * z + w * w;
    }

    /**
     * Returns the length of this quaternion.
     *
     * @return The length
     */
    public @Dimensionless float length(@Dimensionless Quaternion this) {
        return (@Dimensionless float) Math.sqrt(x * x + y * y + z * z + w * w);
    }

    /**
     * Normalizes this quaternion. Doesn't create a new quaternion.
     */
    public void normalize(@Dimensionless Quaternion this) {
        final @Dimensionless float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero quaternion");
        }
        x /= l;
        y /= l;
        z /= l;
        w /= l;
    }

    public void inverse(@Dimensionless Quaternion this) {
        final @Dimensionless float lengthSquareQuaternion = lengthSquare();
        if (lengthSquareQuaternion <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero quaternion");
        }
        x /= -lengthSquareQuaternion;
        y /= -lengthSquareQuaternion;
        z /= -lengthSquareQuaternion;
        w /= lengthSquareQuaternion;
    }

    /**
     * Gets the unit length quaternion for this quaternion. Both have the same orientation. Creates a new quaternion.
     *
     * @return The unit length quaternion for this one
     */
    public @Dimensionless Quaternion getUnit(@Dimensionless Quaternion this) {
        final @Dimensionless float lengthQuaternion = length();
        if (lengthQuaternion <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero quaternion");
        }
        return new @Dimensionless Quaternion(
                x / lengthQuaternion,
                y / lengthQuaternion,
                z / lengthQuaternion,
                w / lengthQuaternion);
    }

    /**
     * Returns the conjugate of this quaternion. Creates a new quaternion.
     *
     * @return The conjugate of this quaternion
     */
    public @Dimensionless Quaternion getConjugate(@Dimensionless Quaternion this) {
        return new @Dimensionless Quaternion(-x, -y, -z, w);
    }

    /**
     * Returns the inverse of this quaternion. Creates a new quaternion.
     *
     * @return The inverse of this quaternion
     */
    public Quaternion getInverse(@Dimensionless Quaternion this) {
        final @Dimensionless float lengthSquareQuaternion = lengthSquare();
        if (lengthSquareQuaternion <= ReactDefaults.MACHINE_EPSILON) {
            throw new @Dimensionless IllegalArgumentException("Cannot normalize the zero quaternion");
        }
        return new @Dimensionless Quaternion(
                -x / lengthSquareQuaternion,
                -y / lengthSquareQuaternion,
                -z / lengthSquareQuaternion,
                w / lengthSquareQuaternion);
    }

    /**
     * Returns the dot product of this quaternion and the provided one.
     *
     * @param quaternion The quaternion to calculate the dot product with
     * @return The dot product of this and the other quaternion
     */
    public @Dimensionless float dot(@Dimensionless Quaternion this, @Dimensionless Quaternion quaternion) {
        return x * quaternion.getX() + y * quaternion.getY() + z * quaternion.getZ() + w * quaternion.getW();
    }

    /**
     * Gets the rotation represented as an angle around an axis. The angle part is returned by the method, while the axis, represented as a vector, will be stored in the passed vector parameter.
     *
     * @param axis The vector in which to store the axis component values
     * @return The angle as a float
     */
    public @Dimensionless float getRotationAngleAxis(@Dimensionless Quaternion this, @Dimensionless Vector3 axis) {
        final @Dimensionless Quaternion quaternion;
        if (length() == ((@Dimensionless int) (1))) {
            quaternion = this;
        } else {
            quaternion = getUnit();
        }
        final @Dimensionless Vector3 rotationAxis = new @Dimensionless Vector3(quaternion.getX(), quaternion.getY(), quaternion.getZ()).getUnit();
        axis.setAllValues(rotationAxis.getX(), rotationAxis.getY(), rotationAxis.getZ());
        return (@rad float) Math.acos(quaternion.getW()) * ((@UnitsRep(top=false, bot=false, prefixExponent=0, baseUnitComponents={@units.qual.BUC(unit="C", exponent=0), @units.qual.BUC(unit="K", exponent=0), @units.qual.BUC(unit="deg", exponent=0), @units.qual.BUC(unit="g", exponent=0), @units.qual.BUC(unit="hr", exponent=0), @units.qual.BUC(unit="m", exponent=0), @units.qual.BUC(unit="mol", exponent=0), @units.qual.BUC(unit="rad", exponent=-1), @units.qual.BUC(unit="s", exponent=0)}) int) (2));
    }

    /**
     * Gets the 3x3 rotation matrix for this quaternion.
     *
     * @return The rotation matrix3x3
     */
    public Matrix3x3 getMatrix(@Dimensionless Quaternion this) {
        final @Dimensionless float nQ = x * x + y * y + z * z + w * w;
        final @Dimensionless float s;
        if (nQ > ((@Dimensionless double) (0.0))) {
            s = ((@Dimensionless int) (2)) / nQ;
        } else {
            s = ((@Dimensionless int) (0));
        }
        final @Dimensionless float xs = x * s;
        final @Dimensionless float ys = y * s;
        final @Dimensionless float zs = z * s;
        final @Dimensionless float wxs = w * xs;
        final @Dimensionless float wys = w * ys;
        final @Dimensionless float wzs = w * zs;
        final @Dimensionless float xxs = x * xs;
        final @Dimensionless float xys = x * ys;
        final @Dimensionless float xzs = x * zs;
        final @Dimensionless float yys = y * ys;
        final @Dimensionless float yzs = y * zs;
        final @Dimensionless float zzs = z * zs;
        return new @Dimensionless Matrix3x3(
                ((@Dimensionless int) (1)) - yys - zzs, xys - wzs, xzs + wys,
                xys + wzs, ((@Dimensionless int) (1)) - xxs - zzs, yzs - wxs,
                xzs - wys, yzs + wxs, ((@Dimensionless int) (1)) - xxs - yys);
    }

    @Override
    public int hashCode(@Dimensionless Quaternion this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (47)) * hash + Float.floatToIntBits(x);
        hash = ((@Dimensionless int) (47)) * hash + Float.floatToIntBits(y);
        hash = ((@Dimensionless int) (47)) * hash + Float.floatToIntBits(z);
        hash = ((@Dimensionless int) (47)) * hash + Float.floatToIntBits(w);
        return hash;
    }

    @Override
    public boolean equals(@Dimensionless Quaternion this, Object obj) {
        if (!(obj instanceof Quaternion)) {
            return false;
        }
        final @Dimensionless Quaternion other = (@Dimensionless Quaternion) obj;
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
    public @Dimensionless String toString(@Dimensionless Quaternion this) {
        return "(" + w + ", " + x + ", " + y + ", " + z + ")";
    }

    /**
     * Returns a new identity quaternion.
     *
     * @return A new identity quaternion
     */
    public static Quaternion identity() {
        return new @Dimensionless Quaternion(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (1)));
    }

    /**
     * Adds the first and second quaternion. Creates a new quaternion.
     *
     * @param quaternion1 The first quaternion
     * @param quaternion2 The second quaternion
     * @return The result of the addition of the first and second quaternion
     */
    public static @Dimensionless Quaternion add(@Dimensionless Quaternion quaternion1, @Dimensionless Quaternion quaternion2) {
        return new @Dimensionless Quaternion(
                quaternion1.getX() + quaternion2.getX(),
                quaternion1.getY() + quaternion2.getY(),
                quaternion1.getZ() + quaternion2.getZ(),
                quaternion1.getW() + quaternion2.getW());
    }

    /**
     * Subtracts the first and second quaternion. Creates a new quaternion.
     *
     * @param quaternion1 The first quaternion
     * @param quaternion2 The second quaternion
     * @return The result of the subtraction of the first and second quaternion
     */
    public static @Dimensionless Quaternion subtract(@Dimensionless Quaternion quaternion1, @Dimensionless Quaternion quaternion2) {
        return new @Dimensionless Quaternion(
                quaternion1.getX() - quaternion2.getX(),
                quaternion1.getY() - quaternion2.getY(),
                quaternion1.getZ() - quaternion2.getZ(),
                quaternion1.getW() - quaternion2.getW());
    }

    /**
     * Multiplies the quaternion by the float value. Creates a new quaternion.
     *
     * @param quaternion The quaternion to multiply
     * @param value The value to multiply the quaternion by
     * @return The result of the multiplication of the quaternion by the float
     */
    public static @Dimensionless Quaternion multiply(@Dimensionless Quaternion quaternion, @Dimensionless float value) {
        return new @Dimensionless Quaternion(
                quaternion.getX() * value,
                quaternion.getY() * value,
                quaternion.getZ() * value,
                quaternion.getW() * value);
    }

    /**
     * Multiplies the first and second quaternion. Creates a new quaternion.
     *
     * @param quaternion1 The first quaternion
     * @param quaternion2 The second quaternion
     * @return The result of the multiplication of the first and second quaternion
     */
    public static Quaternion multiply(Quaternion quaternion1, Quaternion quaternion2) {
        return new @Dimensionless Quaternion(
                quaternion1.getW() * quaternion2.getW() - quaternion1.getVectorV().dot(quaternion2.getVectorV()),
                quaternion2.getVectorV().multiply(quaternion1.getW()).add(quaternion1.getVectorV().multiply(quaternion2.getW())).
                        add(quaternion1.getVectorV().cross(quaternion2.getVectorV()))
        );
    }

    /**
     * Multiplies the quaternion by the vector3. Creates a new quaternion.
     *
     * @param quaternion The quaternion to multiply
     * @param vector The vector to multiply the quaternion by
     * @return The result of the multiplication of the quaternion by the vector
     */
    public static @Dimensionless Vector3 multiply(@Dimensionless Quaternion quaternion, @Dimensionless Vector3 vector) {
        final @Dimensionless Quaternion p = new @Dimensionless Quaternion(vector.getX(), vector.getY(), vector.getZ(), ((@Dimensionless int) (0)));
        return multiply(multiply(quaternion, p), quaternion.getConjugate()).getVectorV();
    }

    /**
     * Interpolates a quaternion between two others using spherical linear interpolation.
     *
     * @param quaternion1 The first quaternion
     * @param quaternion2 The second quaternion
     * @param percent The percent for the interpolation, between 0 and 1 inclusively
     * @return The interpolated quaternion
     */
    public static Quaternion slerp(Quaternion quaternion1, Quaternion quaternion2, float percent) {
        if (percent < ((@Dimensionless int) (0)) && percent > ((@Dimensionless int) (1))) {
            throw new @Dimensionless IllegalArgumentException("\"percent\" must be greater than zero and smaller than one");
        }
        final @Dimensionless float invert;
        @Dimensionless
        float cosineTheta = quaternion1.dot(quaternion2);
        if (cosineTheta < ((@Dimensionless int) (0))) {
            cosineTheta = -cosineTheta;
            invert = ((@Dimensionless int) (-1));
        } else {
            invert = ((@Dimensionless int) (1));
        }
        final @Dimensionless float epsilon = ((@Dimensionless float) (0.00001f));
        if (((@Dimensionless int) (1)) - cosineTheta < epsilon) {
            return add(multiply(quaternion1, (((@Dimensionless int) (1)) - percent)), multiply(quaternion2, (percent * invert)));
        }
        final @rad float theta = (@rad float) Math.acos(cosineTheta);
        final @Dimensionless float sineTheta = (@Dimensionless float) Math.sin(theta);
        final @Dimensionless float coeff1 = (@Dimensionless float) Math.sin((((@Dimensionless int) (1)) - percent) * theta) / sineTheta;
        final @Dimensionless float coeff2 = (@Dimensionless float) Math.sin(percent * theta) / sineTheta * invert;
        return add(multiply(quaternion1, coeff1), multiply(quaternion2, coeff2));
    }
}
