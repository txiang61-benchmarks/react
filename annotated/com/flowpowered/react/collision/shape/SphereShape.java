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
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a sphere collision shape that is centered at the origin and defined by its radius. This collision shape does not have an explicit object margin distance. The margin is implicitly the
 * radius of the sphere. Therefore, there is no need to specify an object margin for a sphere shape.
 */
public class SphereShape extends CollisionShape {
    private final @Dimensionless float mRadius;

    /**
     * Constructs a new sphere from the radius.
     *
     * @param radius The radius
     */
    public SphereShape(float radius) {
        super(CollisionShapeType.SPHERE, radius);
        mRadius = radius;
        if (radius <= ((@Dimensionless int) (0))) {
            throw new @Dimensionless IllegalArgumentException("Radius must be greater than zero");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public SphereShape(@Dimensionless SphereShape shape) {
        super(shape);
        mRadius = shape.mRadius;
    }

    /**
     * Gets the radius.
     *
     * @return The radius
     */
    public float getRadius(@Dimensionless SphereShape this) {
        return mRadius;
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless SphereShape this, @Dimensionless Vector3 direction) {
        if (direction.lengthSquare() >= ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            return Vector3.multiply(mMargin, direction.getUnit());
        }
        return new @Dimensionless Vector3(((@Dimensionless int) (0)), mMargin, ((@Dimensionless int) (0)));
    }

    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless SphereShape this, @Dimensionless Vector3 direction) {
        return new @Dimensionless Vector3(((@Dimensionless int) (0)), ((@Dimensionless int) (0)), ((@Dimensionless int) (0)));
    }

    @Override
    public void getLocalBounds(@Dimensionless SphereShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {
        max.setX(mRadius);
        max.setY(mRadius);
        max.setZ(mRadius);
        min.setX(-mRadius);
        min.setY(min.getX());
        min.setZ(min.getX());
    }

    @Override
    public void computeLocalInertiaTensor(@Dimensionless SphereShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        final @Dimensionless float diag = ((@Dimensionless float) (0.4f)) * mass * mRadius * mRadius;
        tensor.setAllValues(
                diag, ((@Dimensionless int) (0)), ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), diag, ((@Dimensionless int) (0)),
                ((@Dimensionless int) (0)), ((@Dimensionless int) (0)), diag);
    }

    @Override
    public void updateAABB(@Dimensionless SphereShape this, @Dimensionless AABB aabb, @Dimensionless Transform transform) {
        final @Dimensionless Vector3 extents = new @Dimensionless Vector3(mRadius, mRadius, mRadius);
        aabb.setMin(Vector3.subtract(transform.getPosition(), extents));
        aabb.setMax(Vector3.add(transform.getPosition(), extents));
    }

    @Override
    public @Dimensionless SphereShape clone(@Dimensionless SphereShape this) {
        return new @Dimensionless SphereShape(this);
    }

    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless SphereShape this, @Dimensionless CollisionShape otherCollisionShape) {
        final @Dimensionless SphereShape otherShape = (@Dimensionless SphereShape) otherCollisionShape;
        return mRadius == otherShape.mRadius;
    }
}
