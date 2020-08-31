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
 * Represents a capsule collision shape that is defined around the Y axis. A capsule shape can be seen as the convex hull of two spheres. The capsule shape is defined by its radius (radius of the two
 * spheres of the capsule) and its height (distance between the centers of the two spheres). This collision shape does not have an explicit object margin distance. The margin is implicitly the radius
 * and height of the shape. Therefore, there is no need to specify an object margin for a capsule shape.
 */
public class CapsuleShape extends CollisionShape {
    private final @Dimensionless float mRadius;
    private final @Dimensionless float mHalfHeight;

    /**
     * Constructs a new capsule shape from its radius and height.
     *
     * @param radius The radius
     * @param height The height
     */
    public CapsuleShape(@Dimensionless float radius, @Dimensionless float height) {
        super(CollisionShapeType.CAPSULE, radius);
        mRadius = radius;
        mHalfHeight = height * ((@Dimensionless float) (0.5f));
        if (radius <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Radius must be greater than zero");
        }
        if (height <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Height must be greater than zero");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public CapsuleShape(@Dimensionless CapsuleShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
    }

    /**
     * Returns the radius of the spherical ends of the capsule.
     *
     * @return The radius of the capsule
     */
    public float getRadius(@Dimensionless CapsuleShape this) {
        return mRadius;
    }

    /**
     * Returns the distance between the middle of the two hemispheres.
     *
     * @return The height of the capsule
     */
    public float getHeight(@Dimensionless CapsuleShape this) {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless CapsuleShape this, @Dimensionless Vector3 direction) {
        if (direction.lengthSquare() >= ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            final @Dimensionless Vector3 unitDirection = direction.getUnit();
            final @Dimensionless Vector3 centerTopSphere = new @Dimensionless Vector3(((@Dimensionless int) (0)), mHalfHeight, ((@Dimensionless int) (0)));
            final @Dimensionless Vector3 topSpherePoint = Vector3.add(centerTopSphere, Vector3.multiply(unitDirection, mRadius));
            final @Dimensionless float dotProductTop = topSpherePoint.dot(direction);
            final @Dimensionless Vector3 centerBottomSphere = new @Dimensionless Vector3(((@Dimensionless int) (0)), -mHalfHeight, ((@Dimensionless int) (0)));
            final @Dimensionless Vector3 bottomSpherePoint = Vector3.add(centerBottomSphere, Vector3.multiply(unitDirection, mRadius));
            final @Dimensionless float dotProductBottom = bottomSpherePoint.dot(direction);
            if (dotProductTop > dotProductBottom) {
                return topSpherePoint;
            } else {
                return bottomSpherePoint;
            }
        }
        return new @Dimensionless Vector3(((@Dimensionless int) (0)), mRadius, ((@Dimensionless int) (0)));
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless CapsuleShape this, @Dimensionless Vector3 direction) {
        if (direction.getY() > ((@Dimensionless int) (0))) {
            return new @Dimensionless Vector3(((@Dimensionless int) (0)), mHalfHeight, ((@Dimensionless int) (0)));
        } else {
            return new @Dimensionless Vector3(((@Dimensionless int) (0)), -mHalfHeight, ((@Dimensionless int) (0)));
        }
    }

    @Override
    public void getLocalBounds(@Dimensionless CapsuleShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {
        max.setX(mRadius);
        max.setY(mHalfHeight + mRadius);
        max.setZ(mRadius);
        min.setX(-mRadius);
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    @Override
    public void computeLocalInertiaTensor(@Dimensionless CapsuleShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        final @Dimensionless float height = mHalfHeight + mHalfHeight;
        final @Dimensionless float radiusSquare = mRadius * mRadius;
        final @Dimensionless float heightSquare = height * height;
        final @Dimensionless float radiusSquareDouble = radiusSquare + radiusSquare;
        final @Dimensionless float factor1 = ((@Dimensionless int) (2)) * mRadius / (((@Dimensionless int) (4)) * mRadius + ((@Dimensionless int) (3)) * height);
        final @Dimensionless float factor2 = ((@Dimensionless int) (3)) * height / (((@Dimensionless int) (4)) * mRadius + ((@Dimensionless int) (3)) * height);
        final @Dimensionless float sum1 = ((@Dimensionless float) (0.4f)) * radiusSquareDouble;
        final @Dimensionless float sum2 = ((@Dimensionless float) (0.75f)) * height * mRadius + ((@Dimensionless float) (0.5f)) * heightSquare;
        final @Dimensionless float sum3 = ((@Dimensionless float) (0.25f)) * radiusSquare + ((@Dimensionless float) (1f)) / ((@Dimensionless int) (12)) * heightSquare;
        final @Dimensionless float IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
        final @Dimensionless float Iyy = factor1 * mass * sum1 + factor2 * mass * ((@Dimensionless float) (0.25f)) * radiusSquareDouble;
        tensor.setAllValues(
                IxxAndzz, ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), Iyy, ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), IxxAndzz);
    }

    @Override
    public @Dimensionless CollisionShape clone(@Dimensionless CapsuleShape this) {
        return new @Dimensionless CapsuleShape(this);
    }

    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless CapsuleShape this, @Dimensionless CollisionShape otherCollisionShape) {
        final @Dimensionless CapsuleShape otherShape = (@Dimensionless CapsuleShape) otherCollisionShape;
        return mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight;
    }
}
