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
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents the collision shape associated with a body that is used during the narrow-phase collision detection.
 */
public abstract class CollisionShape {
    protected final @Dimensionless CollisionShapeType mType;
    private @Dimensionless int mNbSimilarCreatedShapes;
    protected final @Dimensionless float mMargin;

    /**
     * Constructs a new collision shape from its type.
     *
     * @param type The type of the collision shape
     */
    protected CollisionShape(@Dimensionless CollisionShapeType type, @Dimensionless float margin) {
        mType = type;
        mNbSimilarCreatedShapes = ((@Dimensionless int) (0));
        mMargin = margin;
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    protected CollisionShape(@Dimensionless CollisionShape shape) {
        mType = shape.mType;
        mNbSimilarCreatedShapes = shape.mNbSimilarCreatedShapes;
        mMargin = shape.mMargin;
    }

    /**
     * Gets the type of collision shape associated to this shape.
     *
     * @return The collision shape type
     */
    public @Dimensionless CollisionShapeType getType(@Dimensionless CollisionShape this) {
        return mType;
    }

    /**
     * Gets the margin distance around the shape.
     *
     * @return The margin for the shape
     */
    public @Dimensionless float getMargin(@Dimensionless CollisionShape this) {
        return mMargin;
    }

    /**
     * Gets a local support point in a given direction with the object margin.
     *
     * @param direction The desired direction
     * @return The local support point as a vector3
     */
    public abstract Vector3 getLocalSupportPointWithMargin(@Dimensionless CollisionShape this, Vector3 direction);

    /**
     * Gets a local support point in a given direction without the object margin.
     *
     * @param direction The desired direction
     * @return The local support point as a vector3
     */
    public abstract @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless CollisionShape this, @Dimensionless Vector3 direction);

    /**
     * Gets the local extents in x,y and z direction.
     *
     * @param min Where to store the minimum point of the bounds
     * @param max Where to store the maximum point of the bounds
     */
    public abstract void getLocalBounds(@Dimensionless CollisionShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max);

    /**
     * Computes the local inertia tensor of the collision shape for the mass. Stores the results in the passed matrix3x3.
     *
     * @param tensor The matrix3x3 in which the tensor should be stored
     * @param mass The mass of the shape
     */
    public abstract void computeLocalInertiaTensor(@Dimensionless CollisionShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass);

    /**
     * Allocates and returns a copy of the object.
     *
     * @return A copy of the objects
     */
    @Override
    public abstract @Dimensionless CollisionShape clone(@Dimensionless CollisionShape this);

    /**
     * Tests equality between two collision shapes of the same type (same derived classes).
     *
     * @param otherCollisionShape The shape to test for equality
     */
    public abstract @Dimensionless boolean isEqualTo(@Dimensionless CollisionShape this, @Dimensionless CollisionShape otherCollisionShape);

    /**
     * Update the AABB of a body using its collision shape.
     *
     * @param aabb The AABB to update
     * @param transform The AABB's transform
     */
    public void updateAABB(@Dimensionless CollisionShape this, @Dimensionless AABB aabb, @Dimensionless Transform transform) {
        final @Dimensionless Vector3 minBounds = new @Dimensionless Vector3();
        final @Dimensionless Vector3 maxBounds = new @Dimensionless Vector3();
        getLocalBounds(minBounds, maxBounds);
        final @Dimensionless Matrix3x3 worldAxis = transform.getOrientation().getMatrix().getAbsoluteMatrix();
        final @Dimensionless Vector3 worldMinBounds = new @Dimensionless Vector3(
                worldAxis.getColumn(((@Dimensionless int) (0))).dot(minBounds),
                worldAxis.getColumn(((@Dimensionless int) (1))).dot(minBounds),
                worldAxis.getColumn(((@Dimensionless int) (2))).dot(minBounds));
        final @Dimensionless Vector3 worldMaxBounds = new @Dimensionless Vector3(
                worldAxis.getColumn(((@Dimensionless int) (0))).dot(maxBounds),
                worldAxis.getColumn(((@Dimensionless int) (1))).dot(maxBounds),
                worldAxis.getColumn(((@Dimensionless int) (2))).dot(maxBounds));
        final @Dimensionless Vector3 minCoordinates = Vector3.add(transform.getPosition(), worldMinBounds);
        final @Dimensionless Vector3 maxCoordinates = Vector3.add(transform.getPosition(), worldMaxBounds);
        aabb.setMin(minCoordinates);
        aabb.setMax(maxCoordinates);
    }

    /**
     * Returns the number of similar created shapes.
     *
     * @return The number of similar created shapes
     */
    public @Dimensionless int getNbSimilarCreatedShapes(@Dimensionless CollisionShape this) {
        return mNbSimilarCreatedShapes;
    }

    /**
     * Increments the number of similar allocated collision shapes.
     */
    public void incrementNbSimilarCreatedShapes(@Dimensionless CollisionShape this) {
        mNbSimilarCreatedShapes++;
    }

    /**
     * Decrements the number of similar allocated collision shapes.
     */
    public void decrementNbSimilarCreatedShapes(@Dimensionless CollisionShape this) {
        mNbSimilarCreatedShapes--;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless CollisionShape this, @Dimensionless Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof CollisionShape)) {
            return false;
        }
        final @Dimensionless CollisionShape that = (@Dimensionless CollisionShape) o;
        return mMargin == that.mMargin && mType == that.mType && that.isEqualTo(this);
    }

    @Override
    protected void finalize(@Dimensionless CollisionShape this) throws Throwable {
        try {
            if (mNbSimilarCreatedShapes != ((@Dimensionless int) (0))) {
                // Thrown exceptions are ignored, so we need to print instead
                System.err.println("The number of similar created shapes should be 0, is " + mNbSimilarCreatedShapes + " instead");
            }
        } finally {
            super.finalize();
        }
    }

    /**
     * An enumeration of the possible collision shape (box, sphere, cone and cylinder).
     */
    @Dimensionless
    public static enum CollisionShapeType {
        @Dimensionless BOX,
        @Dimensionless SPHERE,
        @Dimensionless CONE,
        @Dimensionless CYLINDER,
        @Dimensionless CAPSULE,
        @Dimensionless CONVEX_MESH
    public CollisionShapeType(CollisionShape.@Dimensionless CollisionShapeType this) { super(); }
    }
}
