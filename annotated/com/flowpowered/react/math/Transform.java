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

/**
 * Represents a position and an orientation in 3D. It can also be seen as representing a translation and a rotation.
 */
@Dimensionless
public class Transform {
    private final @Dimensionless Vector3 mPosition = new @Dimensionless Vector3();
    private final @Dimensionless Quaternion mOrientation = new @Dimensionless Quaternion();

    /**
     * Default constructor. Position will be the zero vector and the rotation, quaternion identity.
     */
    public Transform() {
        this(new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0))), Quaternion.identity());
    }

    /**
     * Constructs a new transform from the position as a vector3 and the orientation as a 3x3 matrix.
     *
     * @param position The position
     * @param orientation The orientation
     */
    public Transform(@Dimensionless Vector3 position, @Dimensionless Matrix3x3 orientation) {
        this(position, new @Dimensionless Quaternion(orientation));
    }

    /**
     * Constructs a new transform from the position as a vector3 and the orientation as a quaternion.
     *
     * @param position The position
     * @param orientation The orientation
     */
    public Transform(@Dimensionless Vector3 position, @Dimensionless Quaternion orientation) {
        this.mPosition.set(position);
        this.mOrientation.set(orientation);
    }

    /**
     * Copy constructor.
     *
     * @param transform The transform to copy
     */
    public Transform(@Dimensionless Transform transform) {
        this(transform.getPosition(), transform.getOrientation());
    }

    /**
     * Gets the position component of this transform as a vector3.
     *
     * @return The position
     */
    public @Dimensionless Vector3 getPosition(@Dimensionless Transform this) {
        return mPosition;
    }

    /**
     * Gets the orientation component of this transform as a quaternion.
     *
     * @return The orientation
     */
    public @Dimensionless Quaternion getOrientation(@Dimensionless Transform this) {
        return mOrientation;
    }

    /**
     * Sets the position component of this transform to the desired vector3.
     *
     * @param position The position to set
     */
    public void setPosition(@Dimensionless Transform this, @Dimensionless Vector3 position) {
        mPosition.set(position);
    }

    /**
     * Sets the orientation component of this transform to the desired quaternion.
     *
     * @param orientation The position to set
     */
    public void setOrientation(@Dimensionless Transform this, @Dimensionless Quaternion orientation) {
        mOrientation.set(orientation);
    }

    /**
     * Sets the position and orientation of this transform to those of the provided transform.
     *
     * @param transform The transform to copy the position and orientation from
     */
    public @Dimensionless Transform set(@Dimensionless Transform this, @Dimensionless Transform transform) {
        mPosition.set(transform.getPosition());
        mOrientation.set(transform.getOrientation());
        return this;
    }

    /**
     * Sets this transform to identity. The vector is set to the zero vector, and the quaternion, to the identity quaternion.
     */
    public void setToIdentity(@Dimensionless Transform this) {
        mPosition.set(new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0))));
        mOrientation.set(Quaternion.identity());
    }

    /**
     * Inverses the rotation and position of this transform and returns it as a new one.
     *
     * @return The transform which is the inverse of this one
     */
    public @Dimensionless Transform getInverse(@Dimensionless Transform this) {
        final @Dimensionless Quaternion invQuaternion = mOrientation.getInverse();
        final @Dimensionless Matrix3x3 invMatrix = invQuaternion.getMatrix();
        return new @Dimensionless Transform(Matrix3x3.multiply(invMatrix, Vector3.negate(mPosition)), invQuaternion);
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Transform this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (3));
        hash = ((@Dimensionless int) (11)) * hash + mPosition.hashCode();
        hash = ((@Dimensionless int) (11)) * hash + mOrientation.hashCode();
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Transform this, @Dimensionless Object obj) {
        if (!(obj instanceof Transform)) {
            return false;
        }
        final @Dimensionless Transform other = (@Dimensionless Transform) obj;
        if (mPosition != other.mPosition && !mPosition.equals(other.mPosition)) {
            return false;
        }
        if (mOrientation != other.mOrientation && !mOrientation.equals(other.mOrientation)) {
            return false;
        }
        return true;
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Transform this) {
        return "Transform{position= " + mPosition + ", orientation= " + mOrientation + "}";
    }

    /**
     * Returns a new identity transform. That is, a transform with the position as the zero vector and the orientation as the identity quaternion.
     *
     * @return A new identity transform
     */
    public static @Dimensionless Transform identity() {
        return new @Dimensionless Transform(new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0))), Quaternion.identity());
    }

    /**
     * Multiplies the transform by a vector3 and returns the result as a new vector3.
     *
     * @param transform The transform
     * @param vector The vector
     * @return The result of the multiplication of the transform by a vector3 as a new vector3
     */
    public static @Dimensionless Vector3 multiply(@Dimensionless Transform transform, @Dimensionless Vector3 vector) {
        return Vector3.add(Matrix3x3.multiply(transform.getOrientation().getMatrix(), vector), transform.getPosition());
    }

    /**
     * Multiplies the first transform by the second one and returns the result as a new transform.
     *
     * @param transform1 The first transform
     * @param transform2 The second transform
     * @return The result of the multiplication of the two transforms as a new transform
     */
    public static @Dimensionless Transform multiply(@Dimensionless Transform transform1, @Dimensionless Transform transform2) {
        return new @Dimensionless Transform(
                Vector3.add(transform1.getPosition(), Matrix3x3.multiply(transform1.getOrientation().getMatrix(), transform2.getPosition())),
                Quaternion.multiply(transform1.getOrientation(), transform2.getOrientation()));
    }

    /**
     * Interpolates a transform between two other.
     *
     * @param transform1 The first transform
     * @param transform2 The second transform
     * @param percent The percent for the interpolation, between 0 and 1 inclusively
     * @return The interpolated transform
     */
    public static @Dimensionless Transform interpolateTransforms(@Dimensionless Transform transform1, @Dimensionless Transform transform2, @Dimensionless float percent) {
        final @Dimensionless Vector3 interPosition = Vector3.add(
                Vector3.multiply(transform1.getPosition(), (((@Dimensionless int) (1)) - percent)),
                Vector3.multiply(transform2.getPosition(), percent));
        final @Dimensionless Quaternion interOrientation = Quaternion.slerp(transform1.getOrientation(), transform2.getOrientation(), percent);
        return new @Dimensionless Transform(interPosition, interOrientation);
    }
}
