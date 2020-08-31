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
 * Represents a cylinder collision shape around the Y axis and centered at the origin. The cylinder is defined by its height and the radius of its base. The "transform" of the corresponding rigid body
 * gives an orientation and a position to the cylinder. This collision shape uses an extra margin distance around it for collision detection purpose. The default margin is 4cm (if your units are
 * meters, which is recommended). In case, you want to simulate small objects (smaller than the margin distance), you might want to reduce the margin by specifying your own margin distance using the
 * "margin" parameter in the constructor of the cylinder shape. Otherwise, it is recommended to use the default margin distance by not using the "margin" parameter in the constructor.
 */
public class CylinderShape extends CollisionShape {
    private final @Dimensionless float mRadius;
    private final @Dimensionless float mHalfHeight;

    /**
     * Constructs a new cylinder from the radius of the base and the height.
     *
     * @param radius The radius of the base
     * @param height The height
     */
    public CylinderShape(@Dimensionless float radius, @Dimensionless float height) {
        this(radius, height, ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a new cylinder from the radius of the base and the height and the AABB margin.
     *
     * @param radius The radius of the base
     * @param height The height
     * @param margin The margin
     */
    public CylinderShape(@Dimensionless float radius, @Dimensionless float height, @Dimensionless float margin) {
        super(CollisionShapeType.CYLINDER, margin);
        mRadius = radius;
        mHalfHeight = height / ((@Dimensionless int) (2));
        if (mRadius <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Radius must be greater than zero");
        }
        if (height <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Height must be greater than zero");
        }
        if (margin <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Margin must be greater than 0");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public CylinderShape(@Dimensionless CylinderShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
    }

    /**
     * Gets the radius of the base.
     *
     * @return The radius
     */
    public float getRadius(@Dimensionless CylinderShape this) {
        return mRadius;
    }

    /**
     * Gets the height of the cylinder.
     *
     * @return The height
     */
    public float getHeight(@Dimensionless CylinderShape this) {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless CylinderShape this, @Dimensionless Vector3 direction) {
        final @Dimensionless Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);
        final @Dimensionless Vector3 unitVec;
        if (direction.lengthSquare() > ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            unitVec = direction.getUnit();
        } else {
            unitVec = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (1)), ((@Dimensionless int) (0)));
        }
        supportPoint.add(Vector3.multiply(unitVec, mMargin));
        return supportPoint;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless CylinderShape this, @Dimensionless Vector3 direction) {
        final @Dimensionless Vector3 supportPoint = new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
        final @Dimensionless float uDotv = direction.getY();
        final @Dimensionless Vector3 w = new @Dimensionless Vector3(direction.getX(), ((@Dimensionless int) (0)), direction.getZ());
        final @Dimensionless float lengthW = (@Dimensionless float) Math.sqrt(direction.getX() * direction.getX() + direction.getZ() * direction.getZ());
        if (lengthW > ReactDefaults.MACHINE_EPSILON) {
            if (uDotv < ((@Dimensionless double) (0.0))) {
                supportPoint.setY(-mHalfHeight);
            } else {
                supportPoint.setY(mHalfHeight);
            }
            supportPoint.add(Vector3.multiply(mRadius / lengthW, w));
        } else {
            if (uDotv < ((@Dimensionless double) (0.0))) {
                supportPoint.setY(-mHalfHeight);
            } else {
                supportPoint.setY(mHalfHeight);
            }
        }
        return supportPoint;
    }

    @Override
    public void getLocalBounds(@Dimensionless CylinderShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {
        max.setX(mRadius + mMargin);
        max.setY(mHalfHeight + mMargin);
        max.setZ(max.getX());
        min.setX(-max.getX());
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    @Override
    public void computeLocalInertiaTensor(@Dimensionless CylinderShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        final @Dimensionless float height = ((@Dimensionless int) (2)) * mHalfHeight;
        final @Dimensionless float diag = (((@Dimensionless float) (1f)) / ((@Dimensionless int) (12))) * mass * (((@Dimensionless int) (3)) * mRadius * mRadius + height * height);
        tensor.setAllValues(
                diag, ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless float) (0.5f)) * mass * mRadius * mRadius, ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), diag);
    }

    @Override
    public @Dimensionless CylinderShape clone(@Dimensionless CylinderShape this) {
        return new @Dimensionless CylinderShape(this);
    }

    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless CylinderShape this, @Dimensionless CollisionShape otherCollisionShape) {
        final @Dimensionless CylinderShape otherShape = (@Dimensionless CylinderShape) otherCollisionShape;
        return mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight;
    }
}
