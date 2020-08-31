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
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.constraint.Joint;

/**
 * An island represent an isolated group of awake bodies that are connected with each other by some constraints (contacts or joints).
 */
public class Island {
    private @Dimensionless RigidBody @Dimensionless [] mBodies;
    private @Dimensionless ContactManifold @Dimensionless [] mContactManifolds;
    private @Dimensionless Joint @Dimensionless [] mJoints;
    private @Dimensionless int mNbBodies;
    private @Dimensionless int mNbContactManifolds;
    private @Dimensionless int mNbJoints;

    /**
     * Constructs a new island from the maximum number of bodies, the maximum number of contact manifolds and the maximum number of joints.
     *
     * @param nbMaxBodies The maximum number of bodies
     * @param nbMaxContactManifolds The maximum number of contact manifolds
     * @param nbMaxJoints The maximum number of joints
     */
    public Island(@Dimensionless int nbMaxBodies, @Dimensionless int nbMaxContactManifolds, @Dimensionless int nbMaxJoints) {
        mNbBodies = ((@Dimensionless int) (0));
        mNbContactManifolds = ((@Dimensionless int) (0));
        mNbJoints = ((@Dimensionless int) (0));
        mBodies = new @Dimensionless RigidBody @Dimensionless [nbMaxBodies];
        mContactManifolds = new @Dimensionless ContactManifold @Dimensionless [nbMaxContactManifolds];
        mJoints = new @Dimensionless Joint @Dimensionless [nbMaxJoints];
    }

    /**
     * Adds a body into the island
     *
     * @param body The body
     */
    public void addBody(@Dimensionless Island this, @Dimensionless RigidBody body) {
        if (body.isSleeping()) {
            throw new @Dimensionless IllegalArgumentException("Body to add is sleeping");
        }
        mBodies[mNbBodies] = body;
        mNbBodies++;
    }

    /**
     * Adds a contact manifold into the island
     *
     * @param contactManifold The contact manifold
     */
    public void addContactManifold(@Dimensionless Island this, @Dimensionless ContactManifold contactManifold) {
        mContactManifolds[mNbContactManifolds] = contactManifold;
        mNbContactManifolds++;
    }

    /**
     * Adds a joint into the island.
     *
     * @param joint The joint
     */
    public void addJoint(@Dimensionless Island this, @Dimensionless Joint joint) {
        mJoints[mNbJoints] = joint;
        mNbJoints++;
    }

    /**
     * Returns the number of bodies in the island.
     *
     * @return The number of bodies
     */
    public int getNbBodies(@Dimensionless Island this) {
        return mNbBodies;
    }

    /**
     * Returns the number of contact manifolds in the island.
     *
     * @return The number of contact manifolds
     */
    public @Dimensionless int getNbContactManifolds(@Dimensionless Island this) {
        return mNbContactManifolds;
    }

    /**
     * Returns the number of joints in the island.
     *
     * @return The number of joints
     */
    public int getNbJoints(@Dimensionless Island this) {
        return mNbJoints;
    }

    /**
     * Returns the array of bodies.
     *
     * @return The array of bodies
     */
    public @Dimensionless RigidBody @Dimensionless [] getBodies(@Dimensionless Island this) {
        return mBodies;
    }

    /**
     * Returns the array of contact manifolds.
     *
     * @return The array of contact manifold
     */
    public @Dimensionless ContactManifold @Dimensionless [] getContactManifolds(@Dimensionless Island this) {
        return mContactManifolds;
    }

    /**
     * Returns the array of joints.
     *
     * @return The array of joints
     */
    public Joint[] getJoints(@Dimensionless Island this) {
        return mJoints;
    }
}
