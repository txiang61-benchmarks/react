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
package com.flowpowered.react.engine;

import units.qual.Dimensionless;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.constraint.ContactPoint;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a pair of two bodies that are overlapping during the broad-phase collision detection. It is created when the two bodies start to overlap and is destroyed when they do not overlap
 * anymore. This class contains a contact manifold that stores all the contact points between the two bodies.
 */
public class OverlappingPair {
    private final @Dimensionless CollisionBody mBody1;
    private final @Dimensionless CollisionBody mBody2;
    private final @Dimensionless ContactManifold mContactManifold;
    private final @Dimensionless Vector3 mCachedSeparatingAxis;

    /**
     * Constructs a new overlapping pair from the first and second body.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public OverlappingPair(CollisionBody body1, CollisionBody body2) {
        mBody1 = body1;
        mBody2 = body2;
        mContactManifold = new @Dimensionless ContactManifold(body1, body2);
        mCachedSeparatingAxis = new @Dimensionless Vector3(((@Dimensionless int) (1)), ((@Dimensionless int) (1)), ((@Dimensionless int) (1)));
    }

    /**
     * Gets the first body.
     *
     * @return The first body
     */
    public CollisionBody getFirstBody(@Dimensionless OverlappingPair this) {
        return mBody1;
    }

    /**
     * Gets the second body.
     *
     * @return The second body
     */
    public CollisionBody getSecondBody(@Dimensionless OverlappingPair this) {
        return mBody2;
    }

    /**
     * Adds a contact point to the contact manifold.
     *
     * @param contact The contact point to add
     */
    public void addContact(@Dimensionless OverlappingPair this, ContactPoint contact) {
        mContactManifold.addContactPoint(contact);
    }

    /**
     * Updates the contact manifold.
     */
    public void update(@Dimensionless OverlappingPair this) {
        mContactManifold.update(mBody1.getTransform(), mBody2.getTransform());
    }

    /**
     * Gets the cached separating axis.
     *
     * @return The cached separating axis
     */
    public @Dimensionless Vector3 getCachedSeparatingAxis(@Dimensionless OverlappingPair this) {
        return mCachedSeparatingAxis;
    }

    /**
     * Sets the cached separating axis.
     *
     * @param axis The separating axis to set
     */
    public void setCachedSeparatingAxis(@Dimensionless OverlappingPair this, @Dimensionless Vector3 axis) {
        mCachedSeparatingAxis.set(axis);
    }

    /**
     * Gets the number of contact points in the contact manifold.
     *
     * @return The number of contact points
     */
    public int getNbContactPoints(@Dimensionless OverlappingPair this) {
        return mContactManifold.getNbContactPoints();
    }

    /**
     * Gets the contact manifold.
     *
     * @return The contact manifold
     */
    public ContactManifold getContactManifold(@Dimensionless OverlappingPair this) {
        return mContactManifold;
    }
}
