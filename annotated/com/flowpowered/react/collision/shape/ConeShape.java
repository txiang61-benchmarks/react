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
package com.flowpowered.react.collision.shape;

import units.qual.Dimensionless;
import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a cone collision shape centered at the origin and aligned with the Y axis. The cone is defined by its height and by the radius of its base. The center of the cone is at the half of the
 * height. The "transform" of the corresponding rigid body gives an orientation and a position to the cone. This collision shape uses an extra margin distance around it for collision detection
 * purpose. The default margin is 4cm (if your units are meters, which is recommended). In case, you want to simulate small objects (smaller than the margin distance), you might want to reduce the
 * margin by specifying your own margin distance using the "margin" parameter in the constructor of the cone shape. Otherwise, it is recommended to use the default margin distance by not using the
 * "margin" parameter in the constructor.
 */
public class ConeShape extends CollisionShape {
    private final @Dimensionless float mRadius;
    private final @Dimensionless float mHalfHeight;
    private final @Dimensionless float mSinTheta;

    /**
     * Constructs a new cone shape from the radius of the base and the height.
     *
     * @param radius The radius of the base
     * @param height The height
     */
    public ConeShape(@Dimensionless float radius, @Dimensionless float height) {
        this(radius, height, ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a new cone shape from the radius of the base and the height and the AABB margin.
     *
     * @param radius The radius of the base
     * @param height The height
     * @param margin The margin
     */
    public ConeShape(@Dimensionless float radius, @Dimensionless float height, @Dimensionless float margin) {
        super(CollisionShapeType.CONE, margin);
        mRadius = radius;
        mHalfHeight = height * ((@Dimensionless float) (0.5f));
        if (mRadius <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Radius must be greater than zero");
        }
        if (mHalfHeight <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Height must be greater than zero");
        }
        if (margin <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Margin must be greater than 0");
        }
        mSinTheta = mRadius / (@Dimensionless float) Math.sqrt(mRadius * mRadius + height * height);
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public ConeShape(@Dimensionless ConeShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
        mSinTheta = shape.mSinTheta;
    }

    /**
     * Gets the radius of the base.
     *
     * @return The radius
     */
    public float getRadius(@Dimensionless ConeShape this) {
        return mRadius;
    }

    /**
     * Gets the height of the cone.
     *
     * @return The height
     */
    public float getHeight(@Dimensionless ConeShape this) {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless ConeShape this, @Dimensionless Vector3 direction) {
        final @Dimensionless Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);
        final @Dimensionless Vector3 unitVec;
        if (direction.lengthSquare() > ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            unitVec = direction.getUnit();
        } else {
            unitVec = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (-1)), ((@Dimensionless int) (0)));
        }
        supportPoint.add(Vector3.multiply(unitVec, mMargin));
        return supportPoint;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless ConeShape this, @Dimensionless Vector3 direction) {
        final @Dimensionless Vector3 v = direction;
        final @Dimensionless float sinThetaTimesLengthV = mSinTheta * v.length();
        final @Dimensionless Vector3 supportPoint;
        if (v.getY() > sinThetaTimesLengthV) {
            supportPoint = new @Dimensionless Vector3(((@Dimensionless int) (0)), mHalfHeight, ((@Dimensionless int) (0)));
        } else {
            final @Dimensionless float projectedLength = (@Dimensionless float) Math.sqrt(v.getX() * v.getX() + v.getZ() * v.getZ());
            if (projectedLength > ReactDefaults.MACHINE_EPSILON) {
                final @Dimensionless float d = mRadius / projectedLength;
                supportPoint = new @Dimensionless Vector3(v.getX() * d, -mHalfHeight, v.getZ() * d);
            } else {
                supportPoint = new @Dimensionless Vector3(((@Dimensionless int) (0)), -mHalfHeight, ((@Dimensionless int) (0)));
            }
        }
        return supportPoint;
    }

    @Override
    public void getLocalBounds(@Dimensionless ConeShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {
        max.setX(mRadius + mMargin);
        max.setY(mHalfHeight + mMargin);
        max.setZ(max.getX());
        min.setX(-max.getX());
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    @Override
    public void computeLocalInertiaTensor(@Dimensionless ConeShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        final @Dimensionless float rSquare = mRadius * mRadius;
        final @Dimensionless float diagXZ = ((@Dimensionless float) (0.15f)) * mass * (rSquare + mHalfHeight);
        tensor.setAllValues(
                diagXZ, ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless float) (0.3f)) * mass * rSquare, ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), diagXZ);
    }

    @Override
    public @Dimensionless ConeShape clone(@Dimensionless ConeShape this) {
        return new @Dimensionless ConeShape(this);
    }

    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless ConeShape this, @Dimensionless CollisionShape otherCollisionShape) {
        final @Dimensionless ConeShape otherShape = (@Dimensionless ConeShape) otherCollisionShape;
        return mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight;
    }
}
